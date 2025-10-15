#!/usr/bin/env python3
import os
import argparse
import json
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import cv2
import numpy as np


def save_image(msg, out_dir, name):
    if msg.encoding == "mono8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    elif msg.encoding in ("bgr8", "rgb8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    os.makedirs(out_dir, exist_ok=True)
    cv2.imwrite(os.path.join(out_dir, f"{name}.png"), img)


def main():
    parser = argparse.ArgumentParser(description="Export ROS2 bag data to CSV and images")

    # --- 默认参数（你可以改成自己的路径） ---
    default_bag = os.path.expanduser("~/ros2bag_to_ros1bag/ros2_bags_db3/stereo_icm_calibr/stereo_icm_calibr.db3")
    default_out = os.path.expanduser("~/ros2bag_to_ros1bag/export_csv/stereo_icm_calibr_csv")

    parser.add_argument("--bag", default=default_bag, help="Path to ROS2 bag (.db3)")
    parser.add_argument("--out", default=default_out, help="Output directory")
    parser.add_argument("--imu_topic", default="/imu_data", help="IMU topic name")
    parser.add_argument("--left_topic", default="/camera/left", help="Left camera topic name")
    parser.add_argument("--right_topic", default="/camera/right", help="Right camera topic name")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    os.makedirs(os.path.join(args.out, "left"), exist_ok=True)
    os.makedirs(os.path.join(args.out, "right"), exist_ok=True)

    # 打开 ROS2 db3 bag
    storage_options = StorageOptions(uri=args.bag, storage_id="sqlite3")
    converter_options = ConverterOptions("cdr", "cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    imu_type = get_message("sensor_msgs/msg/Imu")
    img_type = get_message("sensor_msgs/msg/Image")

    imu_csv = open(os.path.join(args.out, "imu.csv"), "w")
    imu_csv.write("sec,nsec,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n")

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == args.imu_topic:   # 根据实际 topic 修改
            msg = deserialize_message(data, imu_type)
            imu_csv.write(f"{msg.header.stamp.sec},{msg.header.stamp.nanosec},"
                          f"{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z},"
                          f"{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},"
                          f"{msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w}\n")
        elif topic == args.left_topic:
            msg = deserialize_message(data, img_type)
            save_image(msg, os.path.join(args.out, "left"), f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}")
        elif topic == args.right_topic:
            msg = deserialize_message(data, img_type)
            save_image(msg, os.path.join(args.out, "right"), f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}")

    imu_csv.close()
    print(f"导出完成，保存在 {args.out}")


if __name__ == "__main__":
    main()

