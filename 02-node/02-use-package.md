# 使用功能包

## 使用 Python

```bash
ros2 pkg create python_pkg --build-type ament_python --license MIT
```

进入 `python_pkg` 文件夹，可以看到有个同名的 python 包文件夹 `python_pkg`（含有 `__init__.py`），在其中创建 `python_node.py`。

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
```

:::

随后在 `setup.py` 和 `package.xml` 中配置

::: code-group

```python [setup.py]
from setuptools import find_packages, setup

package_name = "python_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="keiyou",
    maintainer_email="keiyou@todo.todo",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["python_node = python_pkg.python_node:main"], # [!code ++]
    },
)
```

```xml [package.xml]
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>python_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="keiyou@todo.todo">keiyou</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend> <!-- [!code ++] -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

:::

构建

```bash
colcon build
```

构建生成了 `build`、`install`、`log` 文件夹

运行环境配置脚本

```bash
source install/setup.bash
```

运行

```bash
ros2 run python_pkg python_node
```

## 使用 C++

```bash
ros2 pkg create cpp_pkg --build-type ament_cmake --license MIT
```

在 `cpp_pkg` 的 `src` 中编写 `cpp_node.cpp`，然后编辑 `cpp_pkg` 下的 `CMakeLists.txt`

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
cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # [!code ++]

add_executable(cpp_node src/cpp_node.cpp) # [!code ++]
ament_target_dependencies(cpp_node rclcpp) # [!code ++]

install(TARGETS cpp_node # [!code ++]
  DESTINATION lib/${PROJECT_NAME} # [!code ++]
) # [!code ++]

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

:::

构建

```bash
colcon build
```

运行环境配置脚本

```bash
source install/setup.bash
```

运行

```bash
ros2 run cpp_pkg cpp_node
```
