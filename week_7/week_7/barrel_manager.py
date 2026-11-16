"""
Spawn barrels in the arena, and let the robots pick them up and offload them.

Serves
  /pick_up_item (auro_interfaces/ItemRequest): attach to the robot named robot_id the nearest
      barrel right behind it, within 0.45 m of its origin and 15 degrees either side of straight
      behind, if the robot is not already holding one. The barrel is then dragged along behind it.
  /offload_item (auro_interfaces/ItemRequest): detach the barrel the robot holds, leaving it where
      it is.
The responses' success says whether it worked, and message why not.

The barrels are models named barrel_<n>, spawned around a cluster centre when this node starts.
Robots are the models whose names start with robot, as week_7.launch.py names them.

Everything Gazebo does here is through gz-transport services: the world's create service spawns
the barrels, and the week_7::ModelAttacher system in worlds/week_7.world (package
week_7_gz_plugins) attaches and detaches them, and reports where all the models are. Poses are
asked for when needed rather than subscribed to: in Python, gz-transport requests do not get their
replies while the process has a subscription.

Author: Pedro Ribeiro. Ported from Gazebo Classic (AURO2025 week_5) for week 7.
"""

import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from auro_interfaces.srv import ItemRequest
from geometry_msgs.msg import Point, Pose, Quaternion
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.empty_pb2 import Empty
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.stringmsg_v_pb2 import StringMsg_V
from gz.transport import Node as GzNode
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf_transformations import quaternion_matrix

REQUEST_TIMEOUT = 3000  # ms, for Gazebo's services; ModelAttacher's give up themselves after 2 s
ROBOT_LINK = 'base_link'  # of the TurtleBot3, that barrels are attached to
BARREL_LINK = 'body'  # of models/barrel.sdf
PICK_UP_DISTANCE = 0.45  # m, from the robot's origin to the barrel's
PICK_UP_ANGLE = np.deg2rad(30)  # total width of the sector behind the robot


class Barrel():

    def __init__(self, x, y, cluster_id):
        self.x = x
        self.y = y
        self.cluster_id = cluster_id

    def __repr__(self):
        return f'Barrel(x={self.x}, y={self.y})'


class Cluster():

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f'({self.x}, {self.y})'


class Robot():

    def __init__(self, robot_id=''):
        self.robot_id = robot_id
        self.item_held = None

    def __repr__(self):
        return f'({self.robot_id}, {self.item_held})'


def pose_from_gz(gz_pose):
    """Convert a gz.msgs.Pose into a geometry_msgs Pose."""
    p, q = gz_pose.position, gz_pose.orientation
    return Pose(position=Point(x=p.x, y=p.y, z=p.z),
                orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w))


def transform_points(points_world, robot_pose):
    """
    Return points_world (an Nx3 numpy array of points in the world frame) in the robot's frame.

    robot_pose is the geometry_msgs Pose of the robot in the world frame.
    """
    # Extract translation
    t = np.array([robot_pose.position.x,
                  robot_pose.position.y,
                  robot_pose.position.z])

    # Extract rotation (as matrix)
    q = [robot_pose.orientation.x,
         robot_pose.orientation.y,
         robot_pose.orientation.z,
         robot_pose.orientation.w]

    # Convert quaternion to 3x3 rotation matrix, returning a 4x4 homogeneous transformation
    # matrix, following which we take the top-left 3x3 submatrix.
    R = quaternion_matrix(q)[:3, :3]

    # Apply transformation to all points:
    # First, subtract from points_world the translation matrix 't', with numpy we have the
    # subtraction row by row. Then, take the transpose and multiply by R.T to rotate into robot
    # frame, finally transpose back.
    return (R.T @ (points_world - t).T).T


def filter_points_behind(points, max_dist=2.0, alpha=np.deg2rad(60)):
    """
    Filter 2D points within a distance threshold and an angular sector behind the robot (+X ahead).

    points is an Nx2 numpy array of points in the robot's frame, max_dist the maximum distance
    from its origin, and alpha the total angular width (radians) of the "behind" sector. Returns
    (mask, filtered_points): mask is True where a point meets both conditions, and
    filtered_points is the subset of points that are behind and close.
    """
    # Compute polar coordinates
    dist = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])  # radians in [-pi, pi]

    # Distance condition
    mask_dist = dist <= max_dist

    # Angular sector centered at pi (behind the robot)
    lower = np.pi - alpha / 2.0
    upper = -np.pi + alpha / 2.0  # wraps around negative side

    # Angles near +pi or -pi are "behind"
    mask_angle = (angles >= lower) | (angles <= upper)

    # Combine conditions
    mask = mask_dist & mask_angle

    return mask, points[mask]


class BarrelManager(Node):

    def __init__(self):
        super().__init__('barrel_manager')

        # ROS2 parameters for node
        self.declare_parameter('random_seed', 0)
        self.declare_parameter('world', 'week_7')
        random.seed(self.get_parameter('random_seed').value)
        world = self.get_parameter('world').value

        # Gazebo services
        self.gz = GzNode()
        self.create_service_name = f'/world/{world}/create'
        attacher = f'/world/{world}/model_attacher'
        self.attach_service = f'{attacher}/attach'
        self.detach_service = f'{attacher}/detach'
        self.poses_service = f'{attacher}/poses'

        # Load barrel model
        self.load_and_configure_models()

        # Barrels
        self.barrels = {}

        # Central locations of barrel clusters
        self.clusters = {}
        self.clusters[0] = Cluster(0.5, 1)

        # Number of items
        self.item_counter = 0

        # Internal tracking of robots and their held items
        self.robots = {}

        # Barrels to spawn, which the control loop does once Gazebo is up
        self.generate_barrels_in_cluster(0, 2)
        self.to_spawn = list(self.barrels)

        # Provide services to collect item or offload item
        self.pick_up_service = self.create_service(ItemRequest, '/pick_up_item',
                                                   self.pick_up_item)
        self.offload_service = self.create_service(ItemRequest, '/offload_item',
                                                   self.offload_item)

        self.timer = self.create_timer(0.5, self.control_loop)

    def load_and_configure_models(self):
        barrel_model_path = os.path.join(get_package_share_directory('week_7'), 'models',
                                         'barrel.sdf')
        with open(barrel_model_path) as f:
            self.barrel_model = f.read()
        self.get_logger().info(f'Loaded barrel model: {barrel_model_path}')

    def add_barrel(self, x, y, cluster_id):
        barrel = Barrel(x, y, cluster_id)
        self.barrels['barrel_' + str(self.item_counter)] = barrel
        self.item_counter += 1
        self.get_logger().info(f'Added {barrel}')

    def get_barrels(self):
        return self.barrels

    def generate_barrel_position(self, cluster_id):

        while True:
            radius = random.uniform(0.25, 0.75)
            angle = math.radians(random.uniform(0, 360))

            x = round(self.clusters[cluster_id].x + radius * math.cos(angle), 2)
            y = round(self.clusters[cluster_id].y + radius * math.sin(angle), 2)

            for b in self.barrels.values():
                if b.cluster_id == cluster_id and math.dist((b.x, b.y), (x, y)) < 0.25:
                    break
            else:
                return x, y

    def generate_barrels_in_cluster(self, cluster_id, num_barrels):
        """Generate barrels within a cluster."""
        for _ in range(num_barrels):
            x, y = self.generate_barrel_position(cluster_id)
            self.add_barrel(x, y, cluster_id)

    def model_poses(self):
        """Return {model name: geometry_msgs Pose} of every model in the world, or None."""
        ok, msg = self.gz.request(self.poses_service, Empty(), Empty, Pose_V, REQUEST_TIMEOUT)
        if not ok:
            return None
        return {p.name: pose_from_gz(p) for p in msg.pose}

    def gz_attacher_request(self, service, robot_id, barrel_name):
        """Ask ModelAttacher to attach or detach the barrel to or from the robot's base link."""
        request = StringMsg_V(data=[robot_id, ROBOT_LINK, barrel_name, BARREL_LINK])
        ok, reply = self.gz.request(service, request, StringMsg_V, Boolean, REQUEST_TIMEOUT)
        return ok and reply.data

    # ROS2 callbacks
    def pick_up_item(self, request, response):
        """
        Make the robot whose 'robot_id' is passed in the request pick up an item behind it.

        The 'success' field of response is set to True if pick up succeeded, and otherwise is set
        to False. The 'message' field provides further details.
        """
        self.get_logger().info(
            f"Incoming request on pick_up_item from robot_id '{request.robot_id}'")

        response.success = False

        # Need to check whether the robot is already holding an item.
        robot = self.robots.get(request.robot_id)
        if robot is not None and robot.item_held is not None:
            response.message = f"Robot '{request.robot_id}' is already holding an item"
            return response

        poses = self.model_poses()
        if poses is None:
            response.message = 'Unable to get the poses of the models from Gazebo'
            return response

        if not request.robot_id.startswith('robot') or request.robot_id not in poses:
            response.message = f"Unable to find robot_id '{request.robot_id}'"
            return response

        # Barrels in the world that no robot holds
        held = {r.item_held for r in self.robots.values()}
        barrel_names = np.array([n for n in poses if n.startswith('barrel_') and n not in held])
        if len(barrel_names) == 0:
            response.message = 'No barrels available to pick up'
            return response

        # We do have the robot's position and some barrels in the world, so transform the barrel
        # positions into the robot's frame of reference. We ignore the 'z' coordinate.
        barrel_positions = np.array([[poses[n].position.x, poses[n].position.y, 0]
                                     for n in barrel_names])
        points_robot = transform_points(barrel_positions, poses[request.robot_id])

        mask, points_behind = filter_points_behind(points_robot[:, :2],
                                                   max_dist=PICK_UP_DISTANCE,
                                                   alpha=PICK_UP_ANGLE)
        final_names = barrel_names[mask]

        if len(final_names) == 0:
            response.message = f"No barrels behind robot '{request.robot_id}' within range"
            return response

        # Pick the nearest barrel behind the robot, and attach it there.
        distances = np.hypot(points_behind[:, 0], points_behind[:, 1])
        picked_barrel_name = str(final_names[np.argmin(distances)])

        if not self.gz_attacher_request(self.attach_service, request.robot_id,
                                        picked_barrel_name):
            response.message = (f"Failed to attach barrel '{picked_barrel_name}' to robot "
                                f"'{request.robot_id}'")
            return response

        self.robots.setdefault(request.robot_id, Robot(request.robot_id)).item_held = \
            picked_barrel_name
        self.get_logger().info(f"Robot '{request.robot_id}' picked up barrel "
                               f"'{picked_barrel_name}'")
        response.success = True
        response.message = (f"Robot '{request.robot_id}' picked up barrel "
                            f"'{picked_barrel_name}' successfully")
        return response

    def offload_item(self, request, response):
        """
        Make the robot whose 'robot_id' is passed in the request offload its held item.

        The 'success' field of response is set to True if offload succeeded, and otherwise is set
        to False. The 'message' field provides further details.
        """
        self.get_logger().info(
            f"Incoming request on offload_item from robot_id '{request.robot_id}'")

        response.success = False

        if request.robot_id not in self.robots:
            response.message = f"Unable to find robot_id '{request.robot_id}'"
            return response

        item_held = self.robots[request.robot_id].item_held
        if item_held is None:
            response.message = f"Robot '{request.robot_id}' is not holding any item"
            return response

        if not self.gz_attacher_request(self.detach_service, request.robot_id, item_held):
            response.message = (f"Failed to detach item '{item_held}' from robot "
                                f"'{request.robot_id}'")
            return response

        self.get_logger().info(f"Robot '{request.robot_id}' offloaded item '{item_held}'")
        self.robots[request.robot_id].item_held = None
        response.success = True
        response.message = f"Robot '{request.robot_id}' offloaded item successfully"
        return response

    def control_loop(self):
        """Spawn the barrels in Gazebo, once it is up; then stop."""
        poses = self.model_poses()
        if poses is None:
            self.get_logger().info('Waiting for Gazebo to spawn barrels',
                                   throttle_duration_sec=5.0)
            return

        for name in list(self.to_spawn):
            # Spawned already, if the reply to an earlier request was lost
            if name in poses or self.spawn_item(name, self.barrels[name].x,
                                                self.barrels[name].y):
                self.get_logger().info(f'Spawned barrel {name} at x: {self.barrels[name].x}, '
                                       f'y: {self.barrels[name].y}')
                self.to_spawn.remove(name)

        if not self.to_spawn:
            self.timer.cancel()

    def spawn_item(self, name, x, y, z=0.0):
        """Spawn a barrel in Gazebo, returning whether it worked."""
        request = EntityFactory(sdf=self.barrel_model, name=name, allow_renaming=False)
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        ok, reply = self.gz.request(self.create_service_name, request, EntityFactory, Boolean,
                                    REQUEST_TIMEOUT)
        if not (ok and reply.data):
            self.get_logger().warn(f'Failed to spawn barrel {name}; will retry')
            return False
        return True


def main(args=None):
    rclpy.init(args=args)

    node = BarrelManager()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
