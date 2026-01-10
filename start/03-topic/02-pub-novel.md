# 发布小说

```bash
ros2 pkg create python_topic --build-type ament_python --dependencies rclpy example_interfaces --license MIT
```

## example_interfaces

```bash
ros2 node list | grep example_interfaces
```

<details>

```plain
    example_interfaces/msg/Bool
    example_interfaces/msg/Byte
    example_interfaces/msg/ByteMultiArray
    example_interfaces/msg/Char
    example_interfaces/msg/Empty
    example_interfaces/msg/Float32
    example_interfaces/msg/Float32MultiArray
    example_interfaces/msg/Float64
    example_interfaces/msg/Float64MultiArray
    example_interfaces/msg/Int16
    example_interfaces/msg/Int16MultiArray
    example_interfaces/msg/Int32
    example_interfaces/msg/Int32MultiArray
    example_interfaces/msg/Int64
    example_interfaces/msg/Int64MultiArray
    example_interfaces/msg/Int8
    example_interfaces/msg/Int8MultiArray
    example_interfaces/msg/MultiArrayDimension
    example_interfaces/msg/MultiArrayLayout
    example_interfaces/msg/String
    example_interfaces/msg/UInt16
    example_interfaces/msg/UInt16MultiArray
    example_interfaces/msg/UInt32
    example_interfaces/msg/UInt32MultiArray
    example_interfaces/msg/UInt64
    example_interfaces/msg/UInt64MultiArray
    example_interfaces/msg/UInt8
    example_interfaces/msg/UInt8MultiArray
    example_interfaces/msg/WString
    example_interfaces/srv/AddTwoInts
    example_interfaces/srv/SetBool
    example_interfaces/srv/Trigger
    example_interfaces/action/Fibonacci
```

</details>

```bash
ros2 interface show example_interfaces/msg/String
```

```plain
# This is an example message of using a primitive datatype, string.
# If you want to test with this that's fine, but if you are deploying
# it into a system you should create a semantically meaningful message type.
# If you want to embed it in another message, use the primitive data type instead.
string data
```

## 编写脚本

::: code-group

```python [novel_pub.py]
from queue import Queue

import rclpy
import requests
from example_interfaces.msg import String
from rclpy.node import Node


class NovelPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name} started")
        self.queue = Queue()
        self.publisher = self.create_publisher(String, "novel", 10)
        self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        if not self.queue.empty():
            msg = String()
            msg.data = self.queue.get()
            self.publisher.publish(msg)
            self.get_logger().info(f"published: {msg.data}")

    def download(self, url):
        resp = requests.get(url)
        for line in resp.content.decode().splitlines():
            self.queue.put(line)


def main():
    rclpy.init()
    novel_pub = NovelPub("novel_pub")
    novel_pub.download("http://127.0.0.1:8080/novel.txt")
    rclpy.spin(novel_pub)
    rclpy.shutdown()
```

```python [setup.py]
...
    entry_points={
        'console_scripts': [
            "novel_pub = python_topic.novel_pub:main" # [!code ++]
        ],
    },
...
```

:::

## 运行

确保有一个 `novel.txt`，然后启动文件服务器

```bash
python3 -m http.server 8080
```

构建

```bash
colcon build
```

运行

```bash
source install/setup.bash
ros2 run python_topic novel_pub
```

接收

```bash
ros2 topic echo /novel
```
