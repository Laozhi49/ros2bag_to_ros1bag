
#!/bin/bash

# PATH Parameters
DB3_DIR=~/ros2bag_to_ros1bag/ros2_bags_db3/stereo_icm_calibr/stereo_icm_calibr.db3
CSV_DIR=~/ros2bag_to_ros1bag/export_csv/stereo_icm_calibr_csv
OUT=~/ros2bag_to_ros1bag/ros1_bags/stereo_icm_calibr.bag

# TOPIC NAME Parameters
IMU_TOPIC=/imu_data
LEFT_TOPIC=/camera/left
RIGHT_TOPIC=/camera/right

# source ROS2
source /opt/ros/galactic/setup.bash
# db3 to csv
python3 scripts/db3_to_csv.py --bag "$DB3_DIR" --out "$CSV_DIR" --imu_topic "$IMU_TOPIC" --left_topic "$LEFT_TOPIC" --right_topic "$RIGHT_TOPIC"

echo "db3 to csv finish!"

# source ROS1
source /opt/ros/noetic/setup.bash
# csv to bag
python3 scripts/csv_to_bag.py --csv_dir "$CSV_DIR" --out "$OUT" --imu_topic "$IMU_TOPIC" --left_topic "$LEFT_TOPIC" --right_topic "$RIGHT_TOPIC"

echo "csv to bag finish!!"