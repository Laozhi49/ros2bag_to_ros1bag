#!/usr/bin/env python3
import os
import argparse
import csv
import cv2
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rosidl_runtime_py.utilities import get_message

from sensor_msgs.msg import Imu, Image


def create_writer(output_bag_path):
    storage_options = StorageOptions(uri=output_bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("cdr", "cdr")
    writer = SequentialWriter()
    writer.open(storage_options, converter_options)
    return writer


def write_imu(writer, csv_file, topic="/imu_data"):
    # 注册 IMU topic
    writer.create_topic(
        TopicMetadata(
            name=topic,
            type="sensor_msgs/msg/Imu",
            serialization_format="cdr"
        )
    )

    with open(csv_file, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            msg = Imu()
            msg.header.stamp.sec = int(row["sec"])
            msg.header.stamp.nanosec = int(row["nsec"])
            msg.linear_acceleration.x = float(row["ax"])
            msg.linear_acceleration.y = float(row["ay"])
            msg.linear_acceleration.z = float(row["az"])
            msg.angular_velocity.x = float(row["gx"])
            msg.angular_velocity.y = float(row["gy"])
            msg.angular_velocity.z = float(row["gz"])
            msg.orientation.x = float(row["qx"])
            msg.orientation.y = float(row["qy"])
            msg.orientation.z = float(row["qz"])
            msg.orientation.w = float(row["qw"])

            timestamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            writer.write(topic, serialize_message(msg), timestamp)


def write_images(writer, img_dir, topic="/camera/left"):
    # 注册 Image topic
    writer.create_topic(
        TopicMetadata(
            name=topic,
            type="sensor_msgs/msg/Image",
            serialization_format="cdr"
        )
    )

    for fname in sorted(os.listdir(img_dir)):
        if not fname.endswith(".png"):
            continue
        sec, nsec = map(int, fname.replace(".png", "").split("_"))
        img = cv2.imread(os.path.join(img_dir, fname), cv2.IMREAD_UNCHANGED)

        msg = Image()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.height, msg.width = img.shape[:2]
        if len(img.shape) == 2:
            msg.encoding = "mono8"
        else:
            msg.encoding = "bgr8"
        msg.step = msg.width * (img.shape[2] if len(img.shape) == 3 else 1)
        msg.data = img.tobytes()

        timestamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        writer.write(topic, serialize_message(msg), timestamp)


def main():
    parser = argparse.ArgumentParser(description="Export CSV and images data to ROS2 db3 bag")

    # --- 默认参数（你可以改成自己的路径） ---
    default_csv_dir = os.path.expanduser("~/ros2bag_to_ros1bag/export_csv/stereo_icm_calibr_csv")
    default_out = os.path.expanduser("~/ros2bag_to_ros1bag/ros2_bags_db3/rosbag2_stereo_icm_calibr")

    parser.add_argument("--csv_dir", default=default_csv_dir, help="Path to CSV and images data")
    parser.add_argument("--out", default=default_out, help="Output directory")
    parser.add_argument("--imu_topic", default="/imu_data", help="IMU topic name")
    parser.add_argument("--left_topic", default="/camera/left", help="Left camera topic name")
    parser.add_argument("--right_topic", default="/camera/right", help="Right camera topic name")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    writer = create_writer(args.out)

    # 写 IMU 数据
    write_imu(writer, os.path.join(args.csv_dir, "imu.csv"), args.imu_topic)

    # 写左右相机数据
    write_images(writer, os.path.join(args.csv_dir, "left"), args.left_topic)
    write_images(writer, os.path.join(args.csv_dir, "right"), args.right_topic)

    print(f"Bag 写入完成: {args.out}")


if __name__ == "__main__":
    rclpy.init()
    main()
    rclpy.shutdown()

