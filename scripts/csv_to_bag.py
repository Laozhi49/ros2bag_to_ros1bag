#!/usr/bin/env python3
import os
import argparse
import csv
import cv2
import rospy
import rosbag
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

def main():
    parser = argparse.ArgumentParser(description="Export CSV and images data to ROS1 bag")

    # --- 默认参数（你可以改成自己的路径） ---
    default_csv_dir = os.path.expanduser("~/ros2bag_to_ros1bag/export_csv/stereo_icm_calibr_csv")
    default_out = os.path.expanduser("~/ros2bag_to_ros1bag/ros1_bags/stereo_icm_calibr.bag")

    parser.add_argument("--csv_dir", default=default_csv_dir, help="Path to CSV and images data")
    parser.add_argument("--out", default=default_out, help="Output directory")
    parser.add_argument("--imu_topic", default="/imu_data", help="IMU topic name")
    parser.add_argument("--left_topic", default="/camera/left", help="Left camera topic name")
    parser.add_argument("--right_topic", default="/camera/right", help="Right camera topic name")
    args = parser.parse_args()

    bag = rosbag.Bag(args.out, "w")
    bridge = CvBridge()

    # 处理 IMU
    with open(os.path.join(args.csv_dir, "imu.csv")) as f:
        reader = csv.DictReader(f)
        for row in reader:
            h = Header()
            h.stamp = rospy.Time(int(row["sec"]), int(row["nsec"]))
            h.frame_id = "imu_link"

            msg = Imu()
            msg.header = h
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
            bag.write(args.imu_topic, msg, h.stamp)

    # 处理左相机
    left_dir = os.path.join(args.csv_dir, "left")
    for fname in sorted(os.listdir(left_dir)):
        sec, nsec = fname.replace(".png", "").split("_")
        stamp = rospy.Time(int(sec), int(nsec))
        cv_img = cv2.imread(os.path.join(left_dir, fname), cv2.IMREAD_COLOR)
        img_msg = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "camera_left"
        bag.write(args.left_topic, img_msg, stamp)

    # 处理右相机
    right_dir = os.path.join(args.csv_dir, "right")
    for fname in sorted(os.listdir(right_dir)):
        sec, nsec = fname.replace(".png", "").split("_")
        stamp = rospy.Time(int(sec), int(nsec))
        cv_img = cv2.imread(os.path.join(right_dir, fname), cv2.IMREAD_COLOR)
        img_msg = bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "camera_right"
        bag.write(args.right_topic, img_msg, stamp)

    bag.close()
    print(f"已生成 ROS1 bag: {args.out}")

if __name__ == "__main__":
    main()

