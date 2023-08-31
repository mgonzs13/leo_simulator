from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    leo_description_share = get_package_share_directory("leo_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value=[leo_description_share,
                               "/urdf/leo_sim.urdf.xacro"],
                description="Absolute path to robot urdf.xacro file",
            ),
            DeclareLaunchArgument(
                name="fixed",
                default_value="false",
                description='Set to "true" to spawn the robot fixed to the world',
            ),
            DeclareLaunchArgument(
                name="robot_ns", default_value="/", description="Namespace of the robot"
            ),
            DeclareLaunchArgument(
                name="model_name",
                default_value="leo",
                description="The name of the spawned model in Gazebo",
            ),
            DeclareLaunchArgument(
                name="initial_pose_x",
                default_value="0.0",
                description="Initial pose x"
            ),
            DeclareLaunchArgument(
                name="initial_pose_y",
                default_value="0.0",
                description="Initial pose y"
            ),
            DeclareLaunchArgument(
                name="initial_pose_z",
                default_value="0.0",
                description="Initial pose z"
            ),
            DeclareLaunchArgument(
                name="initial_pose_yaw",
                default_value="0.0",
                description="Initial pose yaw"
            ),
            PushRosNamespace(LaunchConfiguration("robot_ns")),
            Node(
                name="robot_state_publisher",
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                LaunchConfiguration("model"),
                                " fixed:=",
                                LaunchConfiguration("fixed"),
                            ]
                        )
                    }
                ],
            ),
            Node(
                name="spawn_entity",
                package="gazebo_ros",
                executable="spawn_entity.py",
                # fmt: off
                arguments=[
                    "-topic", "robot_description",
                    "-entity", LaunchConfiguration("model_name"),
                    "-timeout", "120",
                    "-x", LaunchConfiguration("initial_pose_x"),
                    "-y", LaunchConfiguration("initial_pose_y"),
                    "-z", LaunchConfiguration("initial_pose_z"),
                    "-Y", LaunchConfiguration("initial_pose_yaw")
                ],
                parameters=[{"use_sim_time": True}]
                # fmt: on
            ),
            Node(
                name="joint_state_broadcaster_spawner",
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                name="diff_drive_controller_spawner",
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
            ),
        ]
    )
