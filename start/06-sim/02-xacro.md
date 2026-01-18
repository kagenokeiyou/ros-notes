# Xacro

Xacro 是 URDF 的“宏语言增强版”，本质上仍然生成 URDF。

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_arm">

  <!-- 参数 -->
  <xacro:property name="base_size" value="0.3"/>
  <xacro:property name="link_radius" value="0.05"/>
  <xacro:property name="link1_length" value="0.4"/>

  <!-- 底座 -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_size} ${base_size} 0.1"/>
      </geometry>
      <origin xyz="0 0 0.05"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
  </link>

  <!-- 第一段 link 的宏 -->
  <xacro:macro name="cylinder_link" params="name length color">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${link_radius}"/>
        </geometry>
        <origin xyz="0 0 ${length/2}"/>
        <material name="${color}">
          <color rgba="0.2 0.2 0.8 1.0"/>
        </material>
      </visual>
    </link>
  </xacro:macro>

  <!-- 使用宏 -->
  <xacro:cylinder_link
      name="link1"
      length="${link1_length}"
      color="blue"/>

  <!-- 关节 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

转 URDF

```bash
ros2 run xacro xacro simple_arm.xacro > simple_arm.urdf
```

直接在 launch 中使用

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    robot_description = Command([
        'xacro ',
        '/path/to/simple_arm.xacro'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        )
    ])
```

启动后在 RViz2 中查看
