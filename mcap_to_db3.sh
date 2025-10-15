
#!/bin/bash

# PATH Parameters
MCAP_DIR=~/ros2bag_to_ros1bag/ros2_bags_mcap/rosbag2_stereo_wheeltec_imu_pool1/rosbag2_stereo_wheeltec_imu_pool1.mcap
CSV_DIR=~/ros2bag_to_ros1bag/export_csv/rosbag2_stereo_wheeltec_imu_pool1_csv
OUT=~/ros2bag_to_ros1bag/ros2_bags_db3/rosbag2_stereo_wheeltec_imu_pool1

# TOPIC NAME Parameters
IMU_TOPIC=/imu/data_raw
LEFT_TOPIC=/camera/left
RIGHT_TOPIC=/camera/right

# source ROS2
source /opt/ros/galactic/setup.bash
# mcap to csv
python3 scripts/mcap_to_csv.py --bag "$MCAP_DIR" --out "$CSV_DIR" --imu_topic "$IMU_TOPIC" --left_topic "$LEFT_TOPIC" --right_topic "$RIGHT_TOPIC"

echo "mcap to csv finish!"

# csv to bag
python3 scripts/csv_to_db3.py --csv_dir "$CSV_DIR" --out "$OUT" --imu_topic "$IMU_TOPIC" --left_topic "$LEFT_TOPIC" --right_topic "$RIGHT_TOPIC"

echo "csv to db3 finish!!"
