# launch 启动脚本

```python
import launch
import launch_ros


def generate_launch_description():
    action_node_turtlesim_node = launch_ros.actions.Node(
        package="turtlesim", executable="turtlesim_node", output="screen"
    )
    action_node_partol_client = launch_ros.actions.Node(
        package="demo_cpp_service", executable="partol_client", output="log"
    )
    action_node_turtle_control = launch_ros.actions.Node(
        package="demo_cpp_service", executable="turtle_control", output="log"
    )

    return launch.LaunchDescription(
        [
            action_node_turtlesim_node,
            action_node_partol_client,
            action_node_turtle_control,
        ]
    )
```

```bash
ros2 launch demo_cpp_service launch.py
```

```python
def generate_launch_description():
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument(
        "background_g", default_value="150"
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        package="turtlesim",
        executable="turtlesim_node",
        output="screen",
        parameters=[
            {
                "background_g": launch.substitutions.LaunchConfiguration(
                    "background_g", default_value="150"
                )
            }
        ],
    )
```

```python
multisim_launch_path = [
        get_package_share_directory("multisim_launch"),
        "launch",
        "multisim.launch.py",
    ]
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )
    # 2
    action_log_info = launch.actions.LogInfo(str(multisim_launch_path))
    # 3
    action_topic_list = launch.actions.ExecuteProcess(
        cmd=["ros2", "topic", "list"],
    )
    # 4
    action_group = launch.actions.GroupAction(
        [
            launch.actions.TimerAction(
                period=2.0,
                actions=[action_include_launch],
            ),
            launch.actions.TimerAction(
                period=4.0,
                actions=[action_topic_list],
            ),
        ]
    )
```
