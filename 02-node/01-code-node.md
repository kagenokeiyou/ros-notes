# 编写第一个节点

::: code-group

```python
import rclpy
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("python_node")
    node.get_logger().info("hello python node")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

```

```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    RCLCPP_INFO(node->get_logger(), "hello cpp node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

:::
