import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__("thermal_camera_node")

        self.declare_parameter("frame_topic", "/thermal_frame")
        self.declare_parameter("width", 32)
        self.declare_parameter("height", 24)
        self.declare_parameter("timer_period", 0.35)
        self.declare_parameter("i2c_frequency", 400000)

        self.frame_topic = self.get_parameter("frame_topic").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.i2c_frequency = int(self.get_parameter("i2c_frequency").value)

        self.publisher = self.create_publisher(Float32MultiArray, self.frame_topic, 10)
        self.frame_buffer = [0.0] * (self.width * self.height)
        self.sensor_ready = False
        self.sensor = None

        self._init_sensor()
        self.timer = self.create_timer(self.timer_period, self._publish_frame)

    def _init_sensor(self):
        try:
            import board
            import busio
            import adafruit_mlx90640

            i2c = busio.I2C(board.SCL, board.SDA, frequency=self.i2c_frequency)
            self.sensor = adafruit_mlx90640.MLX90640(i2c)
            self.sensor.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
            self.sensor_ready = True
            self.get_logger().info("MLX90640 thermal camera initialized")
        except Exception as exc:
            self.sensor_ready = False
            self.get_logger().error(f"Failed to initialize MLX90640: {exc}")

    def _publish_frame(self):
        if not self.sensor_ready or self.sensor is None or not rclpy.ok():
            return

        try:
            self.sensor.getFrame(self.frame_buffer)
        except Exception as exc:
            self.get_logger().warn(f"Thermal frame read failed: {exc}")
            return

        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="height", size=self.height, stride=self.width * self.height),
            MultiArrayDimension(label="width", size=self.width, stride=self.width),
        ]
        msg.data = [self._sanitize_temperature(value) for value in self.frame_buffer]
        try:
            self.publisher.publish(msg)
        except Exception:
            # ROS context may already be shutting down.
            return

    @staticmethod
    def _sanitize_temperature(value):
        if math.isnan(value) or math.isinf(value):
            return 0.0
        return float(value)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
