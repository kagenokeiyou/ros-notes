# URDF

URDF（Unified Robot Description Format，统一机器人描述格式）是 ROS 中用来描述机器人结构、外形和物理属性的一种 XML 格式文件。

[[toc]]

## 文件结构

### XML 头部

```xml
<?xml version="1.0" encoding="UTF-8"?>
```

### `<robot>` 根标签

```xml
<robot name="robot_name">
  <link></link>
  <joint></joint>
</robot>
```

### `<link>`

定义机器人的一个刚体

- `name`（必须）

子标签：`<inertial>`、`<visual>`、`<collision>`

```xml
<link name="link_name">
  <visual></visual>
  <collision></collision>
  <inertial></inertial>
</link>
```

#### `<inertial>`

（可选）刚体的质量、其质心的位置以及其中心惯性特性。

子标签：`<origin>`、`<mass>`、`<inertia>`

##### `<origin>`

（可选）位置与旋转。

- `xyz`（可选：默认为零向量）表示 x、y、z 偏移。所有位置均以米为单位。
- `rpy`（可选：默认为零向量）表示绕 x、y、z 轴的旋转。所有角度均以弧度为单位。

##### `<mass>`

（可选）刚体的质量。

- `value`：该元素表示刚体的质量。以千克为单位。

##### `<inertia>`

（可选）惯性张量 $I = \begin{bmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{xy} & I_{yy} & I_{yz} \\ I_{xz} & I_{yz} & I_{zz} \end{bmatrix}$

- `ixx`：x 轴的转动惯量
- `iyy`：y 轴的转动惯量
- `izz`：z 轴的转动惯量
- `ixy`：x–y 轴的惯量耦合
- `ixz`：x–z 轴的惯量耦合
- `iyz`：y–z 轴的惯量耦合

#### `<visual>`

（可选）刚体的视觉属性。此元素指定用于可视化的对象的形状（盒子、圆柱体等）。注意：同一刚体可以有多个 `<visual>` 标签实例，它们定义的几何形状的并集形成链接的视觉表示。

- `name`（可选）

子标签：`<origin>`、`<geometry>`、`<material>`

##### `<origin>`

（可选）位置与旋转。

- `xyz`（可选：默认为零向量）表示 x、y、z 偏移。所有位置均以米为单位。
- `rpy`（可选：默认为零向量）表示绕 x、y、z 轴的旋转。所有角度均以弧度为单位。

##### `<geometry>`

（必需）视觉对象的形状。

子标签可以是以下之一：

- `<box>`
- `<cylinder>`
- `<sphere>`
- `<shape>`
- `<mesh>`

##### `<material>`

（可选）视觉元素的材质。允许在 `<link>` 对象外部指定材质元素，在顶级`<robot>` 元素中。然后，您可以从链接元素中通过名称引用材质。

- `name`：材质名称。

子标签：

- `<color>`
- `<texture>`

#### `<collision>`

刚体的碰撞属性。请注意，这可能与链接的视觉属性不同，例如，通常使用更简单的碰撞模型来减少计算时间。注意：同一链接可以有多个 `<collision>` 标签实例。它们定义的几何形状的并集构成了链接的碰撞表示。

- `name`（可选）

子标签：`<origin>`、`<geometry>`

##### `<origin>`

（可选）位置与旋转。

- `xyz`（可选：默认为零向量）表示 x、y、z 偏移。所有位置均以米为单位。
- `rpy`（可选：默认为零向量）表示绕 x、y、z 轴的旋转。所有角度均以弧度为单位。

##### `<geometry>`

碰撞形状。

### `<joint>`

定义机器人的关节

- `name`（必须）
- `type`（必须）
  - `revolute`：旋转关节（绕单轴旋转）
  - `continuous`：连续旋转关节（无限旋转）
  - `prismatic`：平移关节（沿单轴移动）
  - `fixed`：固定关节（无运动）
  - `floating`：浮动关节（六自由度）
  - `planar`：平面关节（二维运动）

子标签：`<origin>`、`<parent>`、`<child>`、`<axis>`、`<calibration>`、`<dynamics>`、`<limit>`、`<mimic>`、`<safety_controller>`

```xml
<joint name="joint_name" type="joint_type">
  <origin></origin>
  <parent></parent>
  <child></child>
  <axis></axis>
  <calibration></calibration>
  <dynamics></dynamics>
  <limit></limit>
  <mimic></mimic>
  <safety_controller></safety_controller>
</joint>
```

#### `<origin>`

（可选）位置与旋转。

- `xyz`（可选：默认为零向量）表示 x、y、z 偏移。所有位置均以米为单位。
- `rpy`（可选：默认为零向量）表示绕 x、y、z 轴的旋转。所有角度均以弧度为单位。

#### `<parent>`

（必须）父刚体。

- `link`（必须）父刚体名称。

#### `<child>`

（必须）子刚体。

- `link`（必须）子刚体名称。

#### `<axis>`

（可选：默认为 `1 0 0`）在关节框架中指定的关节轴。这是旋转关节的旋转轴，滑动关节的平移轴，以及平面关节的法线。轴在关节参考框架中指定。固定关节和浮动关节不使用轴字段。

- `xyz`（必须）表示向量的 x、y、z 分量。向量应该是归一化的。

#### `<calibration>`

（可选）关节的参考位置，用于校准关节的绝对位置。

- `rising`（可选）当关节向正方向移动时，此参考位置将触发上升沿。
- `falling`（可选）当关节向正方向移动时，此参考位置将触发下降沿。

#### `<dynamics>`

（可选）一个指定关节物理属性的元素。这些值用于指定关节的建模属性，尤其是在仿真中非常有用。

- `damping`（可选，默认为 0）关节的物理阻尼值（对于棱柱关节为每米牛顿-秒 `N∙s/m`，对于旋转关节为每弧度牛顿-米-秒 `N∙m∙s/rad`。
- `friction`（可选，默认为 0）关节的物理静摩擦值（对于棱柱关节为牛顿 `N`，对于旋转关节为牛顿-米 `N·m`）。

#### `<limit>`

（仅适用于旋转和棱柱关节）限制。

- `lower`（可选，默认为 0）指定关节下限的属性（旋转关节为弧度 `rad`，滑动关节为米 `m`）。如果关节是连续的，则省略。
- `upper`（可选，默认为 0）指定关节上限的属性（旋转关节为弧度 `rad`，滑动关节为米 `m`）。如果关节是连续的，则省略。
- `effort`（必需）用于强制执行最大关节力的属性。
- `velocity`（必需）用于强制执行最大关节速度（对于旋转关节为每秒弧度 `rad/s`，对于滑动关节为每秒米 `m/s`）的属性。

#### `<mimic>`

（可选）用于指定定义的关节模仿另一个现有关节。该关节的值可以计算为 `值 = 乘数 * 其他关节值 + 偏移量`。

- `joint`（必须）指定要模拟的关节名称。
- `multiplier`（可选）指定上述公式中的乘法因子。
- `offset`（可选，默认为 0）指定上述公式中要添加的偏移量（旋转关节为弧度 `rad`，滑动关节为米 `m`）。

#### `<safety_controller>`

（可选） 用于指定关节的安全控制器，该控制器将检查关节的绝对位置，并确保其值在给定范围内。

- `soft_lower_limit`（可选，默认为 0）一个指定关节安全控制器开始限制关节位置的下限边界属性。此限制需要大于关节下限。
- `soft_upper_limit`（可选，默认为 0）一个指定关节安全控制器开始限制关节位置的上限边界属性。此限制需要小于关节上限。
- `k_position`（可选，默认为 0）一个指定位置和速度限制之间关系的属性。
- `k_velocity`（必需）一个指定力和速度限制之间关系的属性。

## 在 RViz 中显示

```sh
sudo apt install ros-$ROS_DISTRO-joint-state-publisher
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
```

编写 launch 脚本（记得修改 `setup.py` / `CMakeLists.txt`）

```python

```
