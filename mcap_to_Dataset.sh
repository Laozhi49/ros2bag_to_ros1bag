
#!/bin/bash

# PATH Parameters
MCAP_DIR=~/ros2bag_to_ros1bag/ros2_bags_mcap/rosbag2_stereo_wheeltec_imu_pool1/rosbag2_stereo_wheeltec_imu_pool1.mcap
OUT=~/ros2bag_to_ros1bag/Dataset/rosbag2_stereo_wheeltec_imu_pool1_Dataset

# TOPIC NAME Parameters
IMU_TOPIC=/imu/data_raw
LEFT_TOPIC=/camera/left
RIGHT_TOPIC=/camera/right

# source ROS2
source /opt/ros/galactic/setup.bash
# mcap to Dataset
python3 scripts/mcap_to_csv_Dataset.py --bag "$MCAP_DIR" --out "$OUT" --imu_topic "$IMU_TOPIC" --left_topic "$LEFT_TOPIC" --right_topic "$RIGHT_TOPIC"

echo "mcap to Dataset finish!"


