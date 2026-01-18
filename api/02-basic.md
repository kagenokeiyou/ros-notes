# 基础

## rcl 上下文

rcl 上下文是 ROS 2 客户端库（rclcpp / rclpy）的运行上下文，用于管理 ROS 初始化、关闭、资源归属和中间件状态。

### `rclpy.context.Context`

```python
class Context: ...
```

#### `__init__`

```python
def __init__(self): ...
```

创建未初始化的上下文。

#### `init`

```python
def init(
    self,
    args: List[str] | None = None,
    *,
    initialize_logging: bool = True,
    domain_id: int | None = None,
) -> None: ...
```

初始化上下文。

- `args`: 命令行参数列表。
- `initialize_logging`: 是否初始化整个进程的日志记录。默认为初始化日志记录。
- `domain_id`: 用于此上下文的哪个域 ID。如果为 None（默认值），则使用域 ID 0。

#### `shutdown`

```python
def shutdown(self) -> None: ...
```

关闭此上下文。

#### `try_shutdown`

```python
def try_shutdown(self) -> None: ...
```

关闭此上下文，如果尚未关闭。

#### `ok`

```python
def ok(self) -> bool: ...
```

检查上下文是否尚未关闭。

#### `on_shutdown`

```python
def on_shutdown(self, callback: Callable[[], None]) -> None: ...
```

添加一个在上下文关闭时被调用的回调。

#### `track_node`

```python
def track_node(self, node: Node) -> None: ...
```

跟踪与此上下文关联的节点。当上下文被销毁时，它将销毁它跟踪的所有节点。

#### `untrack_node`

```python
def untrack_node(self, node: Node) -> None: ...
```

停止跟踪与此上下文关联的节点。如果一个节点在上下文之前被销毁，我们不再需要跟踪它以进行上下文的销毁，因此在此处将其移除。

#### `get_domain_id`

```python
def get_domain_id(self) -> int: ...
```

获取上下文的域 ID。

#### `destroy`

```python
def destroy(self) -> None: ...
```

销毁上下文。

### `rclcpp::Context`

```cpp
class Context : public std::enable_shared_from_this<Context>
```

#### `Context`

```cpp
Context()
```

默认构造函数，上下文未初始化。

#### `init`

```cpp
virtual void init(
    int argc, char const* const* argv,
    const rclcpp::InitOptions& init_options = rclcpp::InitOptions())
```

初始化上下文。此函数是线程安全的。

#### `get_init_options`

```cpp
const rclcpp::InitOptions &get_init_options() const;
rclcpp::InitOptions get_init_options();
```

返回初始化过程中使用的初始化选项。

#### `shutdown`

```cpp
virtual bool shutdown(const std::string &reason)
```

关闭上下文。此函数是线程安全的。

当上下文关闭时，会按以下顺序发生几件事情：

1. 获取一个锁以防止与 `init()`、`on_shutdown()` 等操作发生竞态条件。
2. 如果上下文尚未初始化，则返回 `false`。
3. 在内部 `rcl_context_t` 实例上调用 `rcl_shutdown()`。
4. 设置关闭原因。
5. 每个 `on_shutdown` 回调按添加顺序被调用。
6. 中断阻塞的 `sleep_for()` 调用，因此它们因关闭而提前返回。
7. 中断阻塞的执行器和等待集。

#### `is_valid`

```cpp
bool is_valid() const
```

如果上下文有效则返回 `true`，否则返回 `false`。上下文有效是指它已被初始化但尚未关闭。此函数是线程安全的。

#### `on_shutdown`

```cpp
virtual OnShutdownCallback on_shutdown(OnShutdownCallback callback)
```

添加一个在上下文关闭时被调用的回调。

#### `add_on_shutdown_callback`

```cpp
virtual OnShutdownCallbackHandle add_on_shutdown_callback(
    OnShutdownCallback callback)
```

添加一个在上下文关闭时被调用的回调。

#### `remove_on_shutdown_callback`

```cpp
virtual bool remove_on_shutdown_callback(
    const OnShutdownCallbackHandle& callback_handle)
```

移除已注册的回调。如果找到并移除了回调返回 `true`，否则返回 `false`。
