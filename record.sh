#!/bin/bash

# 定义要录制的话题
TOPICS="/yolo_detect /target_ekf_odom /iris_0/mavros/vision_odom/odom   /yolo_detector/detected_image"

# 定义输出bag文件的名称
BAG_NAME="my_recording_$(date +%Y%m%d_%H%M%S).bag"

# 开始录制
echo "开始录制话题: $TOPICS"
rosbag record $TOPICS -O $BAG_NAME

# 录制完成后提示
echo "录制完成，bag文件保存为: $BAG_NAME"
