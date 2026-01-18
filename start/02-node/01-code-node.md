# 编写第一个节点

## 使用 Python

编写 `python_node.py`

::: code-group

```python [python_node.py]
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

:::

运行

```bash
python3 python_node.py
```

## 使用 C++

编写 `CMakeLists.txt` 与 `cpp_node.cpp`

::: code-group

```cpp [cpp_node.cpp]
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cpp_node");
  RCLCPP_INFO(node->get_logger(), "hello cpp node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

```cmake [CMakeLists.txt]
cmake_minimum_required(VERSION 3.20)
project(cpp_node)
add_executable(cpp_node node.cpp)

find_package(rclcpp REQUIRED)
target_link_libraries(cpp_node rclcpp::rclcpp)
```

:::

编译

```bash
cmake -B build
```

运行

```bash
./build/cpp_node
```

## 查看节点

```bash
ros2 node list
```
