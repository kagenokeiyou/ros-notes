# 使用命令行工具进行内省

ROS 2 包含一套用于内省 ROS 2 系统的命令行工具。

## 用法

工具的主要入口点是命令 `ros2` ，它本身具有各种子命令，用于检查和操作节点、主题、服务等内容。

要查看所有可用的子命令，请运行：

```bash
ros2 --help
```

可用的子命令示例包括：

- action ：检查/交互 ROS 动作
- bag : 录制/播放 rosbag
- component : 管理组件容器
- daemon : 检查/配置 ROS 2 守护进程
- doctor : 检查 ROS 设置以发现潜在问题
- interface : 显示 ROS 接口信息
- launch : 运行/内省一个 launch 文件
- lifecycle : 内省/管理具有管理生命周期的节点
- multicast : 多播调试命令
- node : 查看 ROS 节点
- param : 查看配置节点上的参数
- pkg : 查看 ROS 包
- plugin : 查看 ROS 插件
- run : 运行 ROS 节点
- security : 配置安全设置
- service : 检视/调用 ROS 服务
- test : 运行 ROS 启动测试
- topic : 检查/发布 ROS 主题
- trace : 追踪工具，用于获取 ROS 节点执行信息（仅在 Linux 上可用）
- wtf : doctor 的别名

## 示例

使用命令行工具生成典型的说话者-监听者示例时，可以使用 topic 子命令在主题上发布和回显消息。

在一个终端中发布消息：

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: Hello world"
```

```ansi
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hello world')

publishing #2: std_msgs.msg.String(data='Hello world')
```

在另一个终端中回显接收到的消息：

```bash
ros2 topic echo /chatter
```

```ansi
data: Hello world

data: Hello world
```

## ROS 2 守护进程：后台发现服务

ROS 2 使用分布式发现过程使节点能够相互连接。由于该过程故意不使用集中式发现机制，因此 ROS 节点发现 ROS 图中所有其他参与者可能需要时间。为此，ROS 2 运行一个后台守护进程，该进程维护 ROS 图的信息，以提供对查询（如节点名称列表）的更快响应。

当您首次使用 `ros2 node list` 、 `ros2 topic list` 或其他自省命令等命令行工具时，ROS 2 守护进程会自动启动。如果守护进程未运行，这些工具将在执行请求命令之前在后台实例化一个新的守护进程。

守护进程使用本地主机网络接口（127.0.0.1）进行通信，并将 `ROS_DOMAIN_ID` 环境变量用作端口号偏移量。这意味着如果您想控制特定的守护进程实例（例如，使用 `ros2 daemon stop` ），您必须确保您的 `ROS_DOMAIN_ID` 与该守护进程使用的域 ID 匹配。不同的 `ROS_DOMAIN_ID` 值会导致在不同的端口上运行不同的守护进程实例。

你可以运行 `ros2 daemon --help` 来获取更多与守护进程交互的选项，包括启动、停止或检查守护进程状态的命令。
