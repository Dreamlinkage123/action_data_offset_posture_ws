"""启动 CASBOT 手臂位姿标定 Web 服务。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    host = LaunchConfiguration("host").perform(context)
    port = int(LaunchConfiguration("port").perform(context))
    urdf = LaunchConfiguration("robot_urdf_path").perform(context)
    js_topic = LaunchConfiguration("joint_states_topic").perform(context)
    joint_cmd_time_ref = float(LaunchConfiguration("joint_cmd_time_ref").perform(context))
    return [
        Node(
            package="casbot_arm_calibration_web",
            executable="arm_calibration_web",
            name="casbot_arm_calibration_web",
            output="screen",
            parameters=[
                {
                    "host": host,
                    "port": port,
                    "robot_urdf_path": urdf,
                    "joint_states_topic": js_topic,
                    "joint_cmd_time_ref": joint_cmd_time_ref,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "host",
                default_value="0.0.0.0",
                description="HTTP 监听地址",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="6090",
                description="HTTP 端口",
            ),
            DeclareLaunchArgument(
                "robot_urdf_path",
                default_value="",
                description=(
                    "URDF 路径；留空则使用包内 share/urdf/ 下默认文件。"
                    "可为绝对路径，或相对该 urdf 目录的文件名"
                ),
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="sensor_msgs/JointState 话题",
            ),
            DeclareLaunchArgument(
                "joint_cmd_time_ref",
                default_value="1.0",
                description="示教按钮发布 UpperJointData 时的 time_ref（秒）",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
