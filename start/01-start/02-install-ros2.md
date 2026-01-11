# 安装 ROS2

## 前置条件

一个 Ubuntu 系统的物理机或虚拟机。

在 Windows 上可以使用 VirtualBox 或 VMware 安装虚拟机，也可以使用 WSL2。

我选用的是 Ubuntu 的 WSL2。

## 检查 locale

确保有一个支持 UTF-8 的 locale，使用 `locale` 命令检查当前环境，使用 `locale -a` 命令查看所有支持的 locale。

如果不支持 UTF-8，需安装 `locales` 并使用 `locale-gen` 命令生成，使用 `update-locale` 命令更新。

```bash
sudo apt update
sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
```

然后在 `~/.bashrc` 中添加 `LANG` 环境变量。

```bash
export LANG=en_US.UTF-8
```

## 一键安装

使用鱼香的一键安装脚本：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

## 手动安装

如不使用一键安装，请按照以下步骤手动进行安装。

### 启用必要的仓库

需要将 ROS 2 apt 仓库添加到系统中。

首先确保 Ubuntu Universe 仓库已启用。

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

ros-apt-source 软件包提供密钥和 apt 源配置，用于各种 ROS 仓库。

安装 ros2-apt-source 软件包将配置系统中的 ROS 2 仓库。当该软件包的新版本发布到 ROS 仓库时，仓库配置的更新将自动进行。

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 安装开发工具（可选）

如果你打算构建 ROS 软件包或进行其他开发，也可以安装开发工具。

```bash
sudo apt update && sudo apt install ros-dev-tools
```

### 安装 ROS2

更新 apt 并升级软件包。

```bash
sudo apt update && sudo apt upgrade
```

> [!TIP]
> 以下以 jazzy 版本为例，可自行替换为其他版本。

- 桌面安装（推荐）：包含 ROS、RViz、示例程序和教程。

```bash
sudo apt install ros-jazzy-desktop
```

- ROS-Base 安装（精简版）：通信库、消息包、命令行工具。不包含 GUI 工具。

```bash
sudo apt install ros-jazzy-ros-base
```
