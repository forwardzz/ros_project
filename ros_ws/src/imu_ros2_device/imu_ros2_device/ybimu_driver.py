#!/usr/bin/env python

import math

from YbImuLib import YbImuSerial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray


DEFAULT_SERIAL_PORT = "/dev/serial/by-id/usb-ATK_ATK-HS-V4-CMSIS-DAP_ATK_20210914-if00"
FALLBACK_SERIAL_PORTS = [
    "/dev/myimu",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyACM2",
    "/dev/ttyACM3",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyUSB2",
    "/dev/ttyUSB3",
]


class ybimu_driver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.robot = None
        self.serial_port = None
        self.samples_seen = 0
        self.received_nonzero_data = False
        self.zero_data_warning_sent = False
        self.declare_parameter("serial_port", DEFAULT_SERIAL_PORT)
        self.declare_parameter("fallback_serial_ports", FALLBACK_SERIAL_PORTS)

    def init_topic(self):
        port_list = self._candidate_ports()
        open_errors = []
        for port in port_list:
            try:
                self.robot = YbImuSerial(port)
                self.serial_port = port
                self.get_logger().info("Open Ybimu Port OK:%s" % port)
                break
            except Exception as exc:
                open_errors.append("%s: %s" % (port, exc))
        if self.robot is None:
            self.get_logger().error(
                "Fail to open Ybimu serial. Tried: %s" % "; ".join(open_errors)
            )
            return
        self.robot.create_receive_threading()

        self.imuPublisher = self.create_publisher(Imu, "imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "imu/mag", 100)
        self.baroPublisher = self.create_publisher(Float32MultiArray, "baro", 100)
        self.eulerPublisher = self.create_publisher(Float32MultiArray, "euler", 100)

        self.timer = self.create_timer(0.1, self.pub_data)

    def _candidate_ports(self):
        configured_port = self.get_parameter("serial_port").value
        fallback_ports = list(self.get_parameter("fallback_serial_ports").value)
        ports = []
        if configured_port:
            ports.append(configured_port)
        ports.extend(fallback_ports)
        return list(dict.fromkeys(ports))

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

        self._track_receive_health([ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw])

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
        # 协方差：加速度测量噪声 (m/s^2)^2
        imu.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                               0.0, 0.01, 0.0,
                                               0.0, 0.0, 0.01]

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        # 协方差：角速度测量噪声 (rad/s)^2
        imu.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                            0.0, 0.001, 0.0,
                                            0.0, 0.0, 0.001]
        # 协方差：姿态四元数噪声
        imu.orientation_covariance = [0.0025, 0.0, 0.0,
                                       0.0, 0.0025, 0.0,
                                       0.0, 0.0, 0.0025]

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

    def _track_receive_health(self, values):
        self.samples_seen += 1
        if any(abs(value) > 1e-6 for value in values):
            self.received_nonzero_data = True
            return
        if self.samples_seen >= 20 and not self.zero_data_warning_sent:
            self.zero_data_warning_sent = True
            self.get_logger().warning(
                "IMU serial port is open but received only zero data from %s. "
                "Check wiring, baudrate, and auto-report settings." % self.serial_port
            )

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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
