# 节点（Node）

[[toc]]

## 节点

节点是 ROS 系统中通信的主要入口点。它可以用来创建 ROS 实体，如发布者、订阅者、服务等。

### `rclpy.node.Node`

```python
class Node: ...
```

#### `__init__`

```python
def __init__(
    self,
    node_name: str,
    *,
    context: Context | None = None,
    cli_args: List[str] | None = None,
    namespace: str | None = None,
    use_global_arguments: bool = True,
    enable_rosout: bool = True,
    rosout_qos_profile: rclpy.qos.QoSProfile
    | int = rclpy.qos.qos_profile_rosout_default,
    start_parameter_services: bool = True,
    parameter_overrides: List[Parameter[Any]] | None = None,
    allow_undeclared_parameters: bool = False,
    automatically_declare_parameters_from_overrides: bool = False,
    enable_logger_service: bool = False,
): ...
```

- `node_name`: 节点的名称。
- `context`: 要关联的上下文，如果为 `None`，则使用全局上下文。
- `cli_args`: 仅用于此节点的命令行参数字符串列表。这些参数用于提取节点使用的重映射以及其他 ROS 特定设置，以及用户定义的非 ROS 参数。
- `namespace`: 相对主题和服务名称将被添加前缀的命名空间。
- `use_global_arguments`: 节点是否使用进程范围的命令行参数。
- `enable_rosout`: 节点是否启用 rosout 日志。
- `rosout_qos_profile`: 应用于 rosout 发布者的 QoSProfile 或历史深度。如果提供了历史深度，则 QoS 历史设置为 KEEP_LAST，QoS 历史深度设置为参数的值，所有其他 QoS 设置都设置为默认值。
- `start_parameter_services`: 节点是否创建参数服务。
- `parameter_overrides`: 对节点上声明的参数的初始值进行覆盖的列表。
- `allow_undeclared_parameters`: 节点是否允许未声明的参数。
- `automatically_declare_parameters_from_overrides`: 自动从覆盖中声明参数。
- `enable_logger_service`: 创建服务以允许外部节点获取和设置此节点的日志级别。否则，日志级别仅本地管理。也就是说，日志级别不能远程更改。

#### 属性

- `clients: Iterator[Client[Any, Any]]`: 获取在此节点上创建的客户端。
- `context: Context`: 获取与节点关联的上下文。
- `executor: Executor | None`: 获取节点已添加到其中的执行器，否则返回 `None`。
- `guards: Iterator[GuardCondition]`: 获取在此节点上创建的守卫。
- `publishers: Iterator[Publisher[Any]]`: 获取在此节点上创建的发布者。
- `services: Iterator[Service[Any, Any]]`: 获取在此节点上创建的服务。
- `subscriptions: Iterator[Subscription[Any]]`: 获取在此节点上创建的订阅。
- `timers: Iterator[Timer]`: 获取在此节点上创建的计时器。
- `waitables: Iterator[Waitable[Any]]`: 获取在此节点上创建的等待对象。
