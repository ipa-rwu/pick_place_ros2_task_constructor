from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.substitutions import FindExecutable

import yaml
from yaml import SafeLoader

document = """
    object_name: object
    object_pose:
        header:
            stamp:
                sec: 0
            frame_id: world
        pose:
            position:
                x: 0.5
                y: 0.5
                z: 0.0
            orientation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0
    place_pose:
        header:
            stamp:
                sec: 0
            frame_id: world
        pose:
            position:
                x: 0.5
                y: -0.4
                z: 0.0
            orientation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0
    """


def generate_launch_description():
    ld = LaunchDescription()

    req = yaml.load(document, Loader=SafeLoader)

    cmd_str = 'service call /pick_place_service pick_place_msgs/srv/PickPlace "{}"'.format(
        str(req))

    ld.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " {}".format(cmd_str)
            ]],
            shell=True
        )
    )
    return ld
