#!/usr/bin/env python

import math
import threading

from YbImuLib import YbImuSerial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray

class ybimu_driver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.robot = None

    def init_topic(self):
        port_list = ["/dev/myimu", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2","/dev/ttyUSB3"]
        for port in port_list:
            try:
                self.robot = YbImuSerial(port)
                self.get_logger().info("Open Ybimu Port OK:%s" % port)
                break
            except:
                pass
        if self.robot is None:
            self.get_logger().error("---------Fail To Open Ybimu Serial------------")
            return
        self.robot.create_receive_threading()

        self.imuPublisher = self.create_publisher(Imu, "imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "imu/mag", 100)
        self.baroPublisher = self.create_publisher(Float32MultiArray, "baro", 100)
        self.eulerPublisher = self.create_publisher(Float32MultiArray, "euler", 100)

        self.timer = self.create_timer(0.1, self.pub_data)

    def pub_data(self):
        if self.robot is None:
            return

        time_stamp = self.get_clock().now()
        imu = Imu()
        mag = MagneticField()
        baro = Float32MultiArray()
        euler = Float32MultiArray()

        [ax, ay, az] = self.robot.get_accelerometer_data()
        [gx, gy, gz] = self.robot.get_gyroscope_data()
        [mx, my, mz] = self.robot.get_magnetometer_data()
        [height, temperature, pressure, pressure_contrast] = self.robot.get_baro_data()

        [roll, pitch, yaw] = self.robot.get_imu_attitude_data(True)

        roll_rad = roll * (math.pi / 180.0)
        pitch_rad = pitch * (math.pi / 180.0)
        yaw_rad = -yaw * (math.pi / 180.0)

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        imu.orientation.w = cr * cp * cy + sr * sp * sy
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy

        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = "imu_link"

        imu.linear_acceleration.x = ax * 9.80665
        imu.linear_acceleration.y = ay * 9.80665
        imu.linear_acceleration.z = az * 9.80665

        imu.angular_velocity.x = gx * (math.pi / 180.0)
        imu.angular_velocity.y = gy * (math.pi / 180.0)
        imu.angular_velocity.z = gz * (math.pi / 180.0)

        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = "imu_link"
        # Y-axis sign inverted per IMU mounting orientation
        mag.magnetic_field.x = mx * 1.0
        mag.magnetic_field.y = -my * 1.0
        mag.magnetic_field.z = mz * 1.0

        baro.data = [height, temperature, pressure, pressure_contrast]
        euler.data = [roll, pitch, yaw]

        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.baroPublisher.publish(baro)
        self.eulerPublisher.publish(euler)

    def ready(self):
        return self.robot is not None

def main(args=None):
    rclpy.init(args=args)
    node = ybimu_driver("ybimu_node")
    node.init_topic()
    if not node.ready():
        node.get_logger().error("IMU not ready, shutting down")
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
