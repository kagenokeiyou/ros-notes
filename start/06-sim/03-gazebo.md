# Gazebo

Gazebo 是一个 机器人三维物理仿真器，在 ROS / ROS 2 生态中用于 在虚拟世界中真实地模拟机器人、传感器和环境。

## 安装

Gazebo Classic

```bash
sudo apt install gazebo
```

## 下载模型

```bash
git clone --depth 1 https://github.com/osrf/gazebo_models.git ~/.gazebo/models
rm -rf ~/.gazebo/models/.git
```

## 使用 WSL2 遇到的问题

WSL2 启动 Gazebo 黑屏，这个问题硬控了我好几天😅

中间偶尔正常了一次又不行了，反复重装配置都不行，给我整无语了😅

最后看了一堆 Github Issue，终于发现了是网络问题，因为我挂着代理又开了虚拟网卡，Gazebo 内部通过 WSL2 获取到了一个谜之 IP😅

我把虚拟网卡关掉瞬间就好了，TM 的怎么会有这种 bug😅

666，无语了😅😅😅

如果遇到类似问题，直接配置以下环境变量：

```bash
export GAZEBO_IP=127.0.0.1
export IGN_IP=127.0.0.1
```

这样即使挂了虚拟网卡也不会有问题。

累了，毁灭吧🫠🫠🫠
