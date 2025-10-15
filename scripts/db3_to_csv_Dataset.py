#!/usr/bin/env python3
import os
import argparse
import cv2
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def save_image(msg, out_dir, timestamp_ns):
    """保存ROS图像为PNG文件"""
    if msg.encoding == "mono8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    elif msg.encoding in ("bgr8", "rgb8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")
    os.makedirs(out_dir, exist_ok=True)
    cv2.imwrite(os.path.join(out_dir, f"{timestamp_ns}.png"), img)


def main():
    parser = argparse.ArgumentParser(description="Export ROS2 bag data to CSV and images")

    # --- 默认参数（你可以改成自己的路径） ---
    default_bag = os.path.expanduser("~/ros2bag_to_ros1bag/ros2_bags_mcap/rosbag2_data_icm_200hz_indoor/rosbag2_data_icm_200hz_indoor.mcap")
    default_out = os.path.expanduser("~/ros2bag_to_ros1bag/Dataset/icm_200hz_indoor_Dataset")

    parser.add_argument("--bag", default=default_bag, help="Path to ROS2 bag (.mcap)")
    parser.add_argument("--out", default=default_out, help="Output directory")
    parser.add_argument("--imu_topic", default="/imu_data", help="IMU topic name")
    parser.add_argument("--left_topic", default="/camera/left", help="Left camera topic name")
    parser.add_argument("--right_topic", default="/camera/right", help="Right camera topic name")
    args = parser.parse_args()

    # ====== EuRoC 文件夹结构 ======
    cam0_path = os.path.join(args.out, "mav0/cam0/data")
    cam1_path = os.path.join(args.out, "mav0/cam1/data")
    imu_path = os.path.join(args.out, "mav0/imu0")
    os.makedirs(cam0_path, exist_ok=True)
    os.makedirs(cam1_path, exist_ok=True)
    os.makedirs(imu_path, exist_ok=True)

    # ====== 打开 ROS2 bag ======
    storage_options = StorageOptions(uri=args.bag, storage_id="sqlite3")
    converter_options = ConverterOptions("cdr", "cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    imu_type = get_message("sensor_msgs/msg/Imu")
    img_type = get_message("sensor_msgs/msg/Image")

    # ====== 输出文件 ======
    imu_csv_path = os.path.join(imu_path, "data.csv")
    imu_csv = open(imu_csv_path, "w")
    imu_csv.write("#timestamp [ns],omega_x,omega_y,omega_z,acc_x,acc_y,acc_z\n")

    timestamps = set()

    # ====== 读取 bag 内容 ======
    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == args.imu_topic:
            msg = deserialize_message(data, imu_type)
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            imu_csv.write(
                f"{timestamp_ns},{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},"
                f"{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}\n"
            )

        elif topic == args.left_topic:
            msg = deserialize_message(data, img_type)
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            save_image(msg, cam0_path, timestamp_ns)
            timestamps.add(timestamp_ns)

        elif topic == args.right_topic:
            msg = deserialize_message(data, img_type)
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            save_image(msg, cam1_path, timestamp_ns)
            timestamps.add(timestamp_ns)

    imu_csv.close()

    # ====== 生成 timestamp.txt ======
    timestamps_file = os.path.join(args.out, "timestamp.txt")
    with open(timestamps_file, "w") as f:
        for ts in sorted(timestamps):
            f.write(f"{ts}\n")

    print("✅ 导出完成！")
    print(f"📂 数据集路径: {args.out}")
    print(f"📄 IMU数据: {imu_csv_path}")
    print(f"🕓 时间戳文件: {timestamps_file}")
    print("✅ 结构如下：")
    print(f"""
{args.out}/
 └── mav0/
      ├── cam0/data/*.png
      ├── cam1/data/*.png
      └── imu0/data.csv
 timestamp.txt
""")


if __name__ == "__main__":
    main()

