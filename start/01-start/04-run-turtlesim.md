# 运行 turtlesim

## 运行 turtlesim_node

```bash
ros2 run turtlesim turtlesim_node
```

![turtlesim_node](/assets/turtlesim_node.png)

## 运行 turtle_teleop_key

在另一个终端窗口运行

```bash
ros2 run turtlesim turtle_teleop_key
```

在这个终端可以控制海龟移动

![turtle_teleop_key](/assets/turtle_teleop_key.png)

## 使用 rqt 查看关系

```bash
rqt
```

选择 `Plugins > Introspection > Node Graph`

![rqt_turtlesim](/assets/rqt_turtlesim.png)
