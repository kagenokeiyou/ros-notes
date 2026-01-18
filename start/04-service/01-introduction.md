# 服务通信介绍

```bash
ros2 run turtlesim turtlesim_node
```

```bash
ros2 service list -t
```

<details>

```plain
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
```

</details>

```bash
ros2 interface show turtlesim/srv/Spawn
```

<details>

```plain
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

</details>

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0}"
```

![turtlesim_call_spawn](/assets/turtlesim_call_spawn.png)

```bash
rqt
```

`Plugin > Services > Service Caller`

![rqt_service_caller](/assets/rqt_service_caller.png)

![rqt_call_spawn](/assets/rqt_call_spawn.png)
