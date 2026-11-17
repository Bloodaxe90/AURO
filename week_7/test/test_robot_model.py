import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
from week_7.robot_model import namespace_robot_model, TOPIC_ELEMENTS


def load_model():
    return ET.parse(os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models',
                                 'turtlebot3_waffle_pi', 'model.sdf'))


def topics(tree):
    return [e.text for tag in TOPIC_ELEMENTS for e in tree.getroot().iter(tag)]


def test_every_topic_is_moved_under_the_namespace():
    tree = load_model()
    original = topics(tree)
    namespace_robot_model(tree, '/robot2/')
    assert topics(tree) == ['/robot2/' + t.lstrip('/') for t in original]
    # the DiffDrive's tf_topic is absolute (/tf) in the original
    assert '/robot2/tf' in topics(tree)
    assert '/robot2/cmd_vel' in topics(tree)


def test_robots_do_not_share_topics_and_keep_their_frames():
    one, two = load_model(), load_model()
    namespace_robot_model(one, 'robot1')
    namespace_robot_model(two, 'robot2')
    assert not set(topics(one)) & set(topics(two))
    assert one.getroot().find('model').get('name') == 'robot1'
    frame = ".//plugin[@name='gz::sim::systems::DiffDrive']/frame_id"
    assert one.getroot().find(frame).text == load_model().getroot().find(frame).text


def test_a_namespace_is_required():
    with pytest.raises(ValueError):
        namespace_robot_model(load_model(), '/')
