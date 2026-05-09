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
    joint_cmd_vel_scale = float(LaunchConfiguration("joint_cmd_vel_scale").perform(context))
    calibration_initial_ee_joint_resample_sec = float(
        LaunchConfiguration("calibration_initial_ee_joint_resample_sec").perform(context)
    )
    action_play_topic = LaunchConfiguration("action_play_topic").perform(context)
    band_csv_action_timeout_sec = float(
        LaunchConfiguration("band_csv_action_timeout_sec").perform(context)
    )
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
                    "joint_cmd_vel_scale": joint_cmd_vel_scale,
                    "calibration_initial_ee_joint_resample_sec": calibration_initial_ee_joint_resample_sec,
                    "action_play_topic": action_play_topic,
                    "band_csv_action_timeout_sec": band_csv_action_timeout_sec,
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
                description="UpperJointData 发布时的 time_ref（秒）；须与现场其它 joint_cmd 源一致",
            ),
            DeclareLaunchArgument(
                "joint_cmd_vel_scale",
                default_value="1.0",
                description="UpperJointData 发布时的 vel_scale；终端常用 0.5 时请改为 0.5",
            ),
            DeclareLaunchArgument(
                "calibration_initial_ee_joint_resample_sec",
                default_value="3.0",
                description="开始标定末帧播完后等待本秒数（默认 3s 关节稳定），再将界面当前末端位姿写入初值",
            ),
            DeclareLaunchArgument(
                "action_play_topic",
                default_value="/motion/action/play",
                description="乐队 CSV 面板顺序播放使用的 ActionPlay 名称（crb_ros_msg/action/ActionPlay）",
            ),
            DeclareLaunchArgument(
                "band_csv_action_timeout_sec",
                default_value="7200.0",
                description="单个 CSV 片段 ActionPlay 等待结果的最大时间（秒）",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
