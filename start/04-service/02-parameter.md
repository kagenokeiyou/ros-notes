# 参数

```bash
ros2 service list -t | grep parameter
```

<details>

```plain
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
```

</details>

```bash
ros2 param list
```

<details>

```plain
/turtlesim:
  background_b
  background_g
  background_r
  holonomic
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time
```

</details>

```bash
ros2 param describe /turtlesim background_r
```

<details>

```plain
Parameter name: background_r
  Type: integer
  Description: Red channel of the background color
  Constraints:
    Min value: 0
    Max value: 255
    Step: 1
```

</details>

```bash
ros2 param get /turtlesim background_r
```

```plain
Integer value is: 69
```

```bash
ros2 param set /turtlesim background_r 255
```

![turtlesim_set_bgr](/assets/turtlesim_set_bgr.png)

```bash
ros2 param dump /turtlesim > turtlesim_param.yaml
cat turtlesim_param.yaml
```

<details>

```yaml
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 255
    holonomic: false
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    start_type_description_service: true
    use_sim_time: false
```

</details>

```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_param.yaml
```

```bash
rqt
```

`Plugin > Configuration > Dynamic Reconfigure`

![rqt_dyn_config](/assets/rqt_dyn_config.png)
