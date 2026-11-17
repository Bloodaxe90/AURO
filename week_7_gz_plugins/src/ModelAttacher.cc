// Copyright 2026 Pedro Ribeiro
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \brief A world system that, on request, welds a named link of one model to a named link of
/// another with a fixed joint, removes that joint again, and reports the poses of all models.
///
/// It stands in for what gazebo_ros_link_attacher's /attach and /detach services and
/// gazebo_ros's /model_states topic did in Gazebo Classic. Any number of pairs can be attached
/// at once, but a link can only be the child of one attachment at a time.
///
/// ## Services
///
/// The default names start with /world/<world>/model_attacher/.
///
/// - attach: request gz.msgs.StringMsg_V, whose `data` is [parent model, parent link, child
///   model, child link]; reply gz.msgs.Boolean, true on success. The joint keeps the links at
///   their current relative pose: nothing is moved. Fails if a model or link does not exist,
///   either model is static (a fixed joint to a static body would pin the other in place), both
///   links are in the same model, or the child link is already attached to something.
/// - detach: the same request and reply; removes the joint of that exact pair. Fails if they are
///   not attached.
/// - poses: request gz.msgs.Empty; reply gz.msgs.Pose_V with the world pose of every top-level
///   model, each named after its model, with the simulation time it is from in `header.stamp`.
///   Answered at once, from the last simulation step; fails before the first one. It lets a
///   client follow the models by asking, rather than by subscribing to the world's pose topics:
///   in Python, a gz-transport request does not get its reply while the same process has a
///   subscription.
///
/// attach and detach are carried out in the next PreUpdate, which gz-sim also runs while paused;
/// if that does not happen within 2 s (e.g. the server is stalled) the call fails and the
/// request is dropped. The reason for a failure is written to the Gazebo log.
///
/// ## System parameters
///
/// - `<attach_service>`, `<detach_service>`, `<poses_service>` (optional): service names.
///
/// ## Example
///
///   <plugin filename="gz_model_attacher" name="week_7::ModelAttacher"/>
///
///   gz service -s /world/week_7/model_attacher/attach --reqtype gz.msgs.StringMsg_V
///     --reptype gz.msgs.Boolean --timeout 3000
///     --req 'data: ["robot1", "base_link", "barrel_0", "body"]'

#include <gz/common/Console.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace week_7
{
using namespace gz;
using namespace gz::sim;

class ModelAttacher
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate,
    public ISystemReset
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    const World world(_entity);
    if (!world.Valid(_ecm))
    {
      gzerr << "[ModelAttacher] must be attached to the world. Not loading.\n";
      return;
    }
    this->world = _entity;

    const std::string prefix = "/world/" + world.Name(_ecm).value_or("default") +
                               "/model_attacher/";
    const auto attachSrv = _sdf->Get<std::string>("attach_service", prefix + "attach").first;
    const auto detachSrv = _sdf->Get<std::string>("detach_service", prefix + "detach").first;
    const auto posesSrv = _sdf->Get<std::string>("poses_service", prefix + "poses").first;

    if (!this->node.Advertise(attachSrv, &ModelAttacher::OnAttach, this) ||
        !this->node.Advertise(detachSrv, &ModelAttacher::OnDetach, this) ||
        !this->node.Advertise(posesSrv, &ModelAttacher::OnPoses, this))
    {
      gzerr << "[ModelAttacher] failed to advertise [" << attachSrv << "], [" << detachSrv
            << "] or [" << posesSrv << "]. Not loading.\n";
      return;
    }
    gzmsg << "[ModelAttacher] services [" << attachSrv << "], [" << detachSrv << "], ["
          << posesSrv << "]\n";
    this->valid = true;
  }

  void PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm) override
  {
    if (!this->valid)
      return;

    // Either link may have been removed from the world while attached.
    for (auto it = this->attachments.begin(); it != this->attachments.end();)
    {
      if (_ecm.HasEntity(it->parentLink) && _ecm.HasEntity(it->childLink))
      {
        ++it;
        continue;
      }
      gzwarn << "[ModelAttacher] a link of [" << it->Describe() << "] disappeared; detaching.\n";
      if (_ecm.HasEntity(it->joint))
        _ecm.RequestRemoveEntity(it->joint);
      it = this->attachments.erase(it);
    }

    std::deque<std::shared_ptr<Request>> requests;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      requests.swap(this->pending);
    }
    for (auto &req : requests)
      req->result.set_value(req->attach ? this->Attach(_ecm, req->names)
                                        : this->Detach(_ecm, req->names));
  }

  void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override
  {
    if (!this->valid)
      return;

    msgs::Pose_V poses;
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    poses.mutable_header()->mutable_stamp()->set_sec(sec.count());
    poses.mutable_header()->mutable_stamp()->set_nsec(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime - sec).count());
    _ecm.Each<components::Model, components::Name, components::ParentEntity>(
      [&](const Entity &_e, const components::Model *, const components::Name *_name,
          const components::ParentEntity *_parent) -> bool
      {
        if (_parent->Data() == this->world)
        {
          msgs::Pose *pose = poses.add_pose();
          msgs::Set(pose, worldPose(_e, _ecm));
          pose->set_name(_name->Data());
          pose->set_id(_e);
        }
        return true;
      });

    std::lock_guard<std::mutex> lock(this->posesMutex);
    this->lastPoses = std::move(poses);
    this->havePoses = true;
  }

  void Reset(const UpdateInfo &, EntityComponentManager &_ecm) override
  {
    for (const auto &attachment : this->attachments)
    {
      if (_ecm.HasEntity(attachment.joint))
        _ecm.RequestRemoveEntity(attachment.joint);
    }
    this->attachments.clear();
    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto &req : this->pending)
      req->result.set_value(false);
    this->pending.clear();
  }

private:
  /// [parent model, parent link, child model, child link]
  using Names = std::vector<std::string>;

  struct Request
  {
    bool attach;
    Names names;
    std::promise<bool> result;
  };

  struct Attachment
  {
    Names names;
    Entity parentLink;
    Entity childLink;
    Entity joint;

    std::string Describe() const
    {
      return names[0] + "::" + names[1] + " -> " + names[2] + "::" + names[3];
    }
  };

  bool OnAttach(const msgs::StringMsg_V &_req, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(true, _req));
    return true;
  }

  bool OnDetach(const msgs::StringMsg_V &_req, msgs::Boolean &_rep)
  {
    _rep.set_data(this->Submit(false, _req));
    return true;
  }

  bool OnPoses(const msgs::Empty &, msgs::Pose_V &_rep)
  {
    std::lock_guard<std::mutex> lock(this->posesMutex);
    if (!this->havePoses)
      return false;
    _rep = this->lastPoses;
    return true;
  }

  /// Runs on a transport thread: queue the request for the next PreUpdate and wait for it.
  bool Submit(bool _attach, const msgs::StringMsg_V &_msg)
  {
    const char *action = _attach ? "attach" : "detach";
    if (_msg.data_size() != 4)
    {
      gzwarn << "[ModelAttacher] " << action << " refused: expected 4 names (parent model, "
             << "parent link, child model, child link), got " << _msg.data_size() << ".\n";
      return false;
    }

    auto req = std::make_shared<Request>();
    req->attach = _attach;
    req->names.assign(_msg.data().begin(), _msg.data().end());
    auto result = req->result.get_future();
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->pending.push_back(req);
    }
    if (result.wait_for(kRequestTimeout) == std::future_status::ready)
      return result.get();

    // Timed out: withdraw the request so it is not carried out after we have replied.
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      auto it = std::find(this->pending.begin(), this->pending.end(), req);
      if (it != this->pending.end())
      {
        this->pending.erase(it);
        gzwarn << "[ModelAttacher] " << action
               << " timed out: no simulation update within 2 s.\n";
        return false;
      }
    }
    return result.get();  // PreUpdate took it just before we withdrew it
  }

  /// The link named _link of the top-level model named _model, or kNullEntity with a warning.
  Entity FindLink(const EntityComponentManager &_ecm, const std::string &_model,
                  const std::string &_link, const char *_action) const
  {
    const Entity model = _ecm.EntityByComponents(
      components::Name(_model), components::Model(), components::ParentEntity(this->world));
    if (model == kNullEntity)
    {
      gzwarn << "[ModelAttacher] " << _action << " refused: no model [" << _model << "].\n";
      return kNullEntity;
    }
    const auto *isStatic = _ecm.Component<components::Static>(model);
    if (isStatic && isStatic->Data())
    {
      gzwarn << "[ModelAttacher] " << _action << " refused: model [" << _model
             << "] is static.\n";
      return kNullEntity;
    }
    const Entity link = Model(model).LinkByName(_ecm, _link);
    if (link == kNullEntity)
    {
      gzwarn << "[ModelAttacher] " << _action << " refused: model [" << _model
             << "] has no link [" << _link << "].\n";
    }
    return link;
  }

  bool Attach(EntityComponentManager &_ecm, const Names &_names)
  {
    const Entity parent = this->FindLink(_ecm, _names[0], _names[1], "attach");
    const Entity child = this->FindLink(_ecm, _names[2], _names[3], "attach");
    if (parent == kNullEntity || child == kNullEntity)
      return false;
    if (_names[0] == _names[2])
    {
      gzwarn << "[ModelAttacher] attach refused: both links are in model [" << _names[0]
             << "].\n";
      return false;
    }
    for (const auto &attachment : this->attachments)
    {
      if (attachment.childLink == child)
      {
        gzwarn << "[ModelAttacher] attach refused: [" << _names[2] << "::" << _names[3]
               << "] is already attached (" << attachment.Describe() << ").\n";
        return false;
      }
    }

    const Entity joint = _ecm.CreateEntity();
    _ecm.CreateComponent(joint, components::DetachableJoint({parent, child, "fixed"}));
    this->attachments.push_back({_names, parent, child, joint});
    gzmsg << "[ModelAttacher] attached [" << this->attachments.back().Describe() << "].\n";
    return true;
  }

  bool Detach(EntityComponentManager &_ecm, const Names &_names)
  {
    auto it = std::find_if(this->attachments.begin(), this->attachments.end(),
                           [&](const Attachment &_a) { return _a.names == _names; });
    if (it == this->attachments.end())
    {
      gzwarn << "[ModelAttacher] detach refused: [" << _names[0] << "::" << _names[1]
             << " -> " << _names[2] << "::" << _names[3] << "] is not attached.\n";
      return false;
    }
    if (_ecm.HasEntity(it->joint))
      _ecm.RequestRemoveEntity(it->joint);
    gzmsg << "[ModelAttacher] detached [" << it->Describe() << "].\n";
    this->attachments.erase(it);
    return true;
  }

  static constexpr std::chrono::seconds kRequestTimeout{2};

  Entity world{kNullEntity};
  bool valid{false};
  std::vector<Attachment> attachments;  // only touched in PreUpdate and Reset

  transport::Node node;

  std::mutex mutex;
  std::deque<std::shared_ptr<Request>> pending;

  std::mutex posesMutex;
  msgs::Pose_V lastPoses;
  bool havePoses{false};
};
}  // namespace week_7

GZ_ADD_PLUGIN(week_7::ModelAttacher,
              gz::sim::System,
              week_7::ModelAttacher::ISystemConfigure,
              week_7::ModelAttacher::ISystemPreUpdate,
              week_7::ModelAttacher::ISystemPostUpdate,
              week_7::ModelAttacher::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(week_7::ModelAttacher, "week_7::ModelAttacher")
