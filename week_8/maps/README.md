Put maps here, saved while `launch.py nav2:=slam` runs with

    ros2 run nav2_map_server map_saver_cli -f <name> --ros-args -p save_map_timeout:=10.0

Each is a `<name>.yaml` and a `<name>.pgm`. The timeout is for the map to arrive: it is 2 s by
default, which can be too short, and then the saver fails with `Failed to spin map subscription`.
After `colcon build`, use a map with

    ros2 launch week_8 launch.py nav2:=package://week_8/maps/<name>.yaml
