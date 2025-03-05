# Fast-Tracker 🚁

Welcome to **Fast-Tracker**! This project enables **drone tracking** of pedestrians in a **Gazebo simulation** environment. It combines real-time pedestrian movement control, detection algorithms, and drone tracking systems.

## 项目描述 📝

在Gazebo环境下控制行人运动，实现无人机的追踪。通过实时获取行人的位置信息，控制无人机追踪其运动路径，完成目标追踪任务。

## 实现逻辑 🔧

### 1. 启动仿真 🚀
首先，启动仿真环境并启动追踪系统：
```bash
roslaunch px4 tracker.launch
```
确保 **Gazebo** 环境已启动并且仿真目标与无人机已配置。

### 2. 启动行人控制脚本 🧑‍🦯
接下来，启动行人控制脚本来模拟行人运动：
```bash
rosrun gazebo_ex keyboard.py
```
这将通过键盘控制行人在 **Gazebo** 中的移动，支持前进、后退、左转、右转等控制。

### 3. 启动检测模块 🧐
最后，启动目标检测模块，进行行人检测与无人机追踪：
```bash
roslaunch d2p d2p
roslaunch d2p start
```
检测模块会识别行人并为无人机提供追踪目标的位置信息。

## 项目结构 🏗️

```
├── px4
│   └── tracker.launch        # 启动PX4仿真
├── gazebo_ex
│   └── keyboard.py           # 控制行人运动的脚本
├── d2p
│   ├── d2p.launch            # 启动检测模块
│   └── start.launch          # 启动检测进程
└── README.md                 # 项目说明文件
```

## 依赖项 📦

确保以下 ROS 包已正确安装和配置：

1. **px4** — 用于无人机仿真和控制。
2. **gazebo** — 用于仿真环境。
3. **d2p** — 目标检测与追踪模块。

## 使用方法 🖥️

1. 确保你已启动并连接到 **ROS Master**。
2. 启动仿真并控制行人运动，进行目标检测和无人机追踪。

## 项目开发计划 📅

- [ ] 训练无人机的目标检测模型 🤖
- [ ] 优化追踪算法，以提高稳定性和精度 📊
- [ ] 将系统集成到完整的无人机自主飞行方案中 ✈️

## 联系我们 📞

如有任何问题或建议，请随时联系我们！🚀
