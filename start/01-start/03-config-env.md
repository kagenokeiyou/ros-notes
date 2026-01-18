# 配置环境

## setup.bash

运行 `setup.bash` 后才可使用 ROS 命令，以 humble 版本为例，在 `~/.bashrc` 中添加以下内容：

```bash
source /opt/ros/humble/setup.bash
```

## 检查环境变量

```bash
printenv | grep -i ROS
```

```ansi
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

正常情况下，加载 `setup.bash` 过后，以上环境变量应当被正常设置。

## ROS_DOMAIN_ID {#ros-domain-id}

[关于 ROS_DOMAIN_ID](/concepts/02-intermediate/01-domain-id.md)

一般将 `ROS_DOMAIN_ID` 设置为 0，在 `~/.bashrc` 中添加以下内容：

```bash
export ROS_DOMAIN_ID=0
```
