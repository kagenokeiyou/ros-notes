# 订阅小说

```python
from queue import Queue

import rclpy
from example_interfaces.msg import String
from rclpy import Node


class NovelSub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name} started")
        self.queue = Queue()
        self.subscriber = self.create_subscription(
            String, "novel", self.novel_callback, 10
        )

    def novel_callback(self, msg):
        self.queue.put(msg.data)
        self.get_logger().info(f"received: {msg.data}")


def main():
    rclpy.init()
    novel_sub = NovelSub("novel_sub")
    rclpy.shutdown()
```
