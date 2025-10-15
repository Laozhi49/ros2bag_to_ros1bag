
#!/bin/bash

# PATH Parameters
DB3_DIR=~/ros2bag_to_ros1bag/ros2_bags_db3/rosbag2_stereo_icm_indoor/rosbag2_stereo_icm_indoor_0.db3
OUT=~/ros2bag_to_ros1bag/Dataset/rosbag2_stereo_icm_indoor_Dataset

# TOPIC NAME Parameters
IMU_TOPIC=/imu_data
LEFT_TOPIC=/camera/left
RIGHT_TOPIC=/camera/right

# source ROS2
source /opt/ros/galactic/setup.bash
# db3 to Dataset
python3 scripts/db3_to_csv_Dataset.py --bag "$DB3_DIR" --out "$OUT" --imu_topic "$IMU_TOPIC" --left_topic "$LEFT_TOPIC" --right_topic "$RIGHT_TOPIC"

echo "db3 to Dataset finish!"


