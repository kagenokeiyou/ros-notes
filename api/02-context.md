# 上下文（Context）

[[toc]]

## 上下文

上下文是 ROS 2 客户端库（rclcpp / rclpy）的运行上下文，用于管理 ROS 初始化、关闭、资源归属和中间件状态。

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

停止跟踪与此上下文关联的节点。

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
class Context : public std::enable_shared_from_this<Context>;
```

#### `Context`

```cpp
Context();
```

默认构造函数，上下文未初始化。

#### `init`

```cpp
virtual void init(
    int argc, char const* const* argv,
    const rclcpp::InitOptions& init_options = rclcpp::InitOptions());
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
virtual bool shutdown(const std::string &reason);
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

#### `shutdown_reason`

```cpp
std::string shutdown_reason() const;
```

返回关闭原因，如果没有关闭则返回空字符串。此函数是线程安全的。

#### `is_valid`

```cpp
bool is_valid() const;
```

如果上下文有效则返回 `true`，否则返回 `false`。上下文有效是指它已被初始化但尚未关闭。此函数是线程安全的。

#### `on_shutdown`

```cpp
virtual OnShutdownCallback on_shutdown(OnShutdownCallback callback);
```

添加一个在上下文关闭时被调用的回调。

#### `add_on_shutdown_callback`

```cpp
virtual OnShutdownCallbackHandle add_on_shutdown_callback(
    OnShutdownCallback callback);
```

添加一个在上下文关闭时被调用的回调。

#### `remove_on_shutdown_callback`

```cpp
virtual bool remove_on_shutdown_callback(
    const OnShutdownCallbackHandle& callback_handle);
```

移除已注册的回调。如果找到并移除了回调返回 `true`，否则返回 `false`。

#### `get_on_shutdown_callbacks`

```cpp
std::vector<OnShutdownCallback> get_on_shutdown_callbacks() const;
```

返回关机回调。

#### `add_pre_shutdown_callback`

```cpp
virtual PreShutdownCallbackHandle add_pre_shutdown_callback(
    PreShutdownCallback callback);
```

添加一个预关闭回调，在调用此上下文的关闭之前执行。

#### `remove_pre_shutdown_callback`

```cpp
virtual bool remove_pre_shutdown_callback(
    const PreShutdownCallbackHandle &callback_handle);
```

移除一个已注册的预关闭回调。

#### `get_pre_shutdown_callbacks`

```cpp
std::vector<PreShutdownCallback> get_pre_shutdown_callbacks() const;
```

返回预关机回调。

#### `get_domain_id`

```cpp
size_t get_domain_id() const;
```

返回实际域 ID。

#### `sleep_for`

```cpp
bool sleep_for(const std::chrono::nanoseconds &nanoseconds);
```

休眠指定时间段或直到调用 `shutdown()`。如果没有超时，即被中断，则返回 `true`。

此函数可以在以下情况下提前中断：

- 当前上下文被 `shutdown()`。
- 当前上下文被销毁（导致关闭）。
- 当前上下文具有 `shutdown_on_signal=true` 且发生 `SIGINT` / `SIGTERM` 信号（导致关闭）。
- `interrupt_all_sleep_for()` 被调用。

#### `interrupt_all_sleep_for`

```cpp
void interrupt_all_sleep_for();
```

中断任何阻塞的 `sleep_for()` 调用，使它们立即返回 `true`。

## 全局上下文

`rclpy` 和 `rclcpp` 模块都包含一个全局上下文单例。

### `rclpy.get_default_context`

```python
def get_default_context(*, shutting_down: bool = False) -> Context: ...
```

返回全局上下文单例。

### `rclcpp::contexts::get_global_default_context`

```cpp
DefaultContext::SharedPtr rclcpp::contexts::get_global_default_context()
```

返回全局上下文单例。

## 基础函数

### `rclpy`

#### `init`

```python
def init(
    *,
    args: List[str] | None = None,
    context: Context | None = None,
    domain_id: int | None = None,
    signal_handler_options: rpyutils.import_c_library.SignalHandlerOptions
    | None = None,
) -> InitContextManager: ...
```

初始化给定的上下文。

- `args`: 命令行参数列表。
- `context`: 要初始化的上下文。如果为 `None` ，则使用全局上下文。
- `domain_id`: 要使用的域 ID。如果为 `None` ，则使用默认域 ID。
- `signal_handler_options`: 指示要安装哪些信号处理器。如果为 `None`，则在初始化全局上下文时安装 `SIGINT` 和 `SIGTERM`。

返回值是上下文管理器，可使用 `with` 语句。

```python
with rclpy.init() as context:
    ...
```

#### `shutdown`

```python
def shutdown(
    *,
    context: Context | None = None,
    uninstall_handlers: bool | None = None,
) -> None: ...
```

关闭给定的上下文。这也会关闭全局执行器。

- `context`: 要关闭的上下文。如果为 `None` ，则使用全局上下文。
- `uninstall_handlers`: 如果为 `True` 或 `None`，则在关闭全局上下文时将卸载信号处理器。如果为 `False`，则不会卸载信号处理器。

#### `try_shutdown`

```python
def try_shutdown(
    *,
    context: Context | None = None,
    uninstall_handlers: bool | None = None,
) -> None: ...
```

关闭给定的上下文，如果尚未关闭。

#### `ok`

```python
def ok(*, context: Context | None = None) -> bool: ...
```

检查给定的上下文是否尚未关闭。如果为 `None`，则使用全局上下文。

### `rclcpp`

#### `init`

```cpp
void rclcpp::init(
    int argc, char const* const* argv,
    const InitOptions& init_options = InitOptions(),
    SignalHandlerOptions signal_handler_options = SignalHandlerOptions::All)
```

初始化全局上下文。

- `argc`: 命令行参数数量。
- `argv`: 命令行参数数组。
- `init_options`: 要应用的初始化选项。
- `signal_handler_options`: 指示应安装哪些信号处理器的选项。

#### `shutdown`

```cpp
bool rclcpp::shutdown(
    rclcpp::Context::SharedPtr context = nullptr,
    const std::string& reason = "user called rclcpp::shutdown()")
```

关闭给定的上下文。如果为 `nullptr`，则使用全局上下文。如果使用全局上下文，则信号处理程序也将被卸载。如果关闭操作成功则为 `true`，如果上下文已经关闭则为 `false`

#### `ok`

```cpp
bool rclcpp::ok(rclcpp::Context::SharedPtr context = nullptr)
```

检查给定的上下文是否尚未关闭。如果为 `nullptr`，则使用全局上下文。
