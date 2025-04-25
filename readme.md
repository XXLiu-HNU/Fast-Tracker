# Fast-Tracker 🚁

Welcome to **Fast-Tracker**! This project enables **drone tracking** of pedestrians in a **Gazebo simulation** environment. It combines real-time pedestrian movement control, detection algorithms, and drone tracking systems.

## 项目描述 📝

在Gazebo环境下控制行人运动，实现无人机的追踪。通过实时获取行人的位置信息，控制无人机追踪其运动路径，完成目标追踪任务。

项目环境依赖于XTDrone，请依照[XTDroen](https://github.com/robin-shaun/XTDrone) 介绍完成环境的配置

追踪部分为 [Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker)

目标检测部分参考了 [onboard-detect](https://github.com/Zhefan-Xu/onboard_detector)

**Update 25.4.25：该项目目前实现效果一般，仅作为复现 [Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker) 的一个示例，如果有更好的实现效果欢迎PR！**

## 实现逻辑 🔧

### 1. 启动仿真 🚀
首先，启动仿真环境并启动追踪系统：
```bash
roslaunch px4 tracker.launch
```
**NOTE1:** 确保 **Gazebo** 环境已启动并且仿真目标与无人机已配置。
注意，`launch`文件和对应的`world`文件在`files/`文件下，请移动到PX4 仿真对应的文件目录下

**NOTE2:** 为了使用Gazebo插件控制行人运动，请在`.bashrc`
文件中增加
`export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${your_path}/gazebo_custom_plugin/catkin_ws/devel/lib`

这个插件参考 [gazebo_custom_plugin](https://github.com/red0orange/gazebo_custom_plugin)


### 2. 启动行人控制脚本 🧑‍🦯
接下来，启动行人控制脚本来模拟行人运动：
```bash
rosrun gazebo_extra_plugins keyboard_actor.py 
```
这将通过键盘`i,j,k,l`控制行人在 **Gazebo** 中的移动，支持前进、后退、左转、右转等控制。

### 3. 启动检测模块
启动目标检测模块，进行行人检测：
```bash
roslaunch d2p d2p.launch
```
检测模块会识别行人并为无人机提供追踪目标的位置信息。

### 4. 启动追踪模块
注意对照launch文件修改话题名称
```bash
roslaunch planning simulation1.launch 
```

### 4. 启动控制模块、
使用px4ctrl跟踪规划结果，同时请修改话题名称
```bash
roslaunch px4ctrl singl_run.launch 
```

## 依赖项 📦

确保以下 ROS 包已正确安装和配置：

1. **XTDrone** — 用于无人机仿真
2. **d2p** — 目标检测与追踪模块。

## 项目开发计划 📅

- [ ] 训练无人机的目标检测模型 🤖
- [ ] 优化追踪算法，以提高稳定性和精度 📊
- [ ] 将系统集成到完整的无人机自主飞行方案中 ✈️