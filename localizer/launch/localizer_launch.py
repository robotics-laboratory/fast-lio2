import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_fastlio = LaunchConfiguration("start_fastlio")
    start_foxglove_proxy = LaunchConfiguration("start_foxglove_proxy")
    pcd_path = LaunchConfiguration("pcd_path")
    map_pcd_path = LaunchConfiguration("map_pcd_path")
    map_downsample_leaf_size = LaunchConfiguration("map_downsample_leaf_size")

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("localizer"), "rviz", "localizer.rviz"]
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Whether to launch RViz2",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time from /clock",
            ),
            DeclareLaunchArgument(
                "start_fastlio",
                default_value="true",
                description="Whether to launch fast-lio2 together with localizer",
            ),
            DeclareLaunchArgument(
                "start_foxglove_proxy",
                default_value="false",
                description="Whether to launch Foxglove relocalization proxy",
            ),
            DeclareLaunchArgument(
                "pcd_path", default_value="/truck/data/maps/atrium-v1.pcd"
            ),
            DeclareLaunchArgument(
                "map_pcd_path", default_value="/truck/data/maps/atrium-v1.pcd"
            ),
            DeclareLaunchArgument("map_downsample_leaf_size", default_value="0.3"),
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                condition=IfCondition(start_fastlio),
                parameters=[
                    {
                        "config_path": lio_config_path,
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {
                        "config_path": localizer_config_path,
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="localizer",
                executable="foxglove_localization_proxy.py",
                name="initialpose_to_relocalize",
                output="screen",
                condition=IfCondition(start_foxglove_proxy),
                parameters=[
                    {
                        "pcd_path": pcd_path,
                        "map_pcd_path": map_pcd_path,
                        "map_downsample_leaf_size": ParameterValue(
                            map_downsample_leaf_size, value_type=float
                        ),
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="localizer",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
                arguments=["-d", rviz_cfg],
            )
        ]
    )
