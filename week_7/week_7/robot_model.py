"""Changes to a TurtleBot3's model.sdf, made at launch, so that several can share one world."""

import xml.etree.ElementTree as ET

# The elements of the TurtleBot3's model.sdf that name a Gazebo topic: those of its sensors and
# of its DiffDrive and JointStatePublisher systems.
TOPIC_ELEMENTS = ('topic', 'camera_info_topic', 'odom_topic', 'tf_topic')


def namespace_robot_model(tree: ET.ElementTree, namespace: str) -> None:
    """
    Move every Gazebo topic of the model in tree under /<namespace>/, in place.

    Gazebo topics are global, so two robots spawned from the same model would otherwise publish
    on, and listen to, the same topics: both would follow every cmd_vel, and their odometry and
    scans would be mixed together. The model is also renamed, although ros_gz_sim's create names
    the spawned model anyway.

    Frame names (odom, base_footprint, base_scan, ...) are left alone: each robot has its own TF
    tree, on /<namespace>/tf, so they cannot clash.
    """
    namespace = namespace.strip('/')
    if not namespace:
        raise ValueError('a robot needs a namespace')
    root = tree.getroot()
    for tag in TOPIC_ELEMENTS:
        for element in root.iter(tag):
            element.text = f'/{namespace}/{element.text.strip().lstrip("/")}'
    root.find('model').set('name', namespace)
