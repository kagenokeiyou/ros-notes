# 消息（Message）

消息是 ROS 2 节点在网络上向其他 ROS 节点发送数据的方式，无需响应。例如，如果 ROS 2 节点从传感器读取温度数据，它可以使用 `Temperature` 消息在 ROS 2 网络上发布这些数据。ROS 2 网络上的其他节点可以订阅这些数据并接收 `Temperature` 消息。

消息在 ROS 包的 `msg/` 目录中的 `.msg` 文件中描述和定义。 `.msg` 文件由两部分组成：字段和常量。

## 字段

每个字段由类型和名称组成，两者之间用空格分隔，例如：

```rosmsg
int32 my_int
string my_string
```

## 字段类型

字段类型可以是：

- 一个内置类型
- 独立定义的消息描述的名称，例如 `geometry_msgs/PoseStamped`

目前支持的内置类型：

| Type      | C++              | Python  | DDS                  |
| --------- | ---------------- | ------- | -------------------- |
| `bool`    | `bool`           | `bool`  | `boolean`            |
| `byte`    | `uint8_t`        | `bytes` | `octet`              |
| `char`    | `char`           | `int`   | `char`               |
| `float32` | `float`          | `float` | `float`              |
| `float64` | `double`         | `float` | `double`             |
| `int8`    | `int8_t`         | `int`   | `octet`              |
| `uint8`   | `uint8_t`        | `int`   | `octet`              |
| `int16`   | `int16_t`        | `int`   | `short`              |
| `uint16`  | `uint16_t`       | `int`   | `unsigned short`     |
| `int32`   | `int32_t`        | `int`   | `long`               |
| `uint32`  | `uint32_t`       | `int`   | `unsigned long`      |
| `int64`   | `int64_t`        | `int`   | `long long`          |
| `uint64`  | `uint64_t`       | `int`   | `unsigned long long` |
| `string`  | `std::string`    | `str`   | `string`             |
| `wstring` | `std::u16string` | `str`   | `wstring`            |

每个内置类型都可以用来定义数组：

| Type                      | C++                  | Python | DDS              |
| ------------------------- | -------------------- | ------ | ---------------- |
| `static array`            | `std::array<T, N>`   | `list` | `T[N]`           |
| `unbounded dynamic array` | `std::vector`        | `list` | `sequence`       |
| `bounded dynamic array`   | `custom_class<T, N>` | `list` | `sequence<T, N>` |
| `bounded string`          | `std::string`        | `str`  | `string`         |

Python 中所有比 ROS 定义更宽松的类型通过软件强制执行 ROS 在范围和长度方面的约束。

使用数组和有界类型的消息定义示例：

```rosmsg
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```

## 字段名称

字段名称必须是小写字母数字字符，使用下划线分隔单词。它们必须以字母开头，且不能以下划线结尾或包含两个连续的下划线。

## 字段默认值

默认值可以设置在消息类型中的任何字段。目前不支持字符串数组和复杂数据类型（即不在上述内置类型表中出现的类型；这适用于所有嵌套消息）。

例如：

```rosmsg
uint8 x 42
int16 y -2000
string full_name "John Doe"
int32[] samples [-200, -100, 0, 100, 200]
```

> [!Note]
> 字符串值必须使用单 `'` 或双 `"` 引号定义  
> 当前字符串值没有转义

## 常量

每个常量定义类似于带默认值的字段描述，但这个值不能通过程序更改。这个值赋值通过使用等号 `=` 表示，例如：

```rosmsg
int32 X=123
int32 Y=-123
string FOO="foo"
string EXAMPLE='bar'
```

> [!Note]
>
> 常量名称必须为大写
