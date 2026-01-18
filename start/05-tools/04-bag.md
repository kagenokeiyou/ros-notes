# Bag

ros2 bag 是 ROS 2 中用于录制、回放话题数据的工具

```bash
ros2 bag record   # 录制
ros2 bag play     # 回放
ros2 bag info     # 查看信息
```

录制指定话题

```bash
ros2 bag record /scan /odom /imu
```

指定 bag 名称

```bash
ros2 bag record -o my_bag /scan /odom
```
