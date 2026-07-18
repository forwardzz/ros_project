import os
import time

import rclpy
from rclpy.node import Node

from robot_monitor_interfaces.msg import GasData

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


class GasSensorNode(Node):
    def __init__(self):
        super().__init__("gas_sensor_node")

        self.declare_parameter(
            "serial_port",
            "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0",
        )
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("poll_period", 0.5)
        self.declare_parameter(
            "fallback_ports",
            [
                "/dev/gas_sensor",
                "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0",
                "/dev/serial/by-path/platform-xhci-hcd.1-usbv2-0:2:1.0-port0",
                "/dev/ttyUSB1",
            ],
        )

        self.baudrate = int(self.get_parameter("baudrate").value)
        self.poll_period = float(self.get_parameter("poll_period").value)
        self.serial_port = str(self.get_parameter("serial_port").value).strip()
        self.fallback_ports = list(self.get_parameter("fallback_ports").value)
        self.send_frame = bytes([0x01, 0x03, 0x00, 0x01, 0x00, 0x04, 0x15, 0xC9])
        self.serial_handle = None
        self.active_port = None
        self.last_open_attempt = 0.0
        self.last_port_warn = 0.0
        self.last_short_frame_warn = 0.0
        self.last_read_error = 0.0
        self.short_frame_count = 0

        self.publisher = self.create_publisher(GasData, "/gas_data", 10)
        self.timer = self.create_timer(self.poll_period, self._poll_sensor)

        if serial is None:
            self.get_logger().error("pyserial is not available, gas sensor node disabled")
        else:
            self._ensure_serial_open(force=True)

    def _candidate_ports(self):
        ports = []
        if self.serial_port:
            ports.append(self.serial_port)
        for port in self.fallback_ports:
            if port not in ports:
                ports.append(port)
        return ports

    def _open_serial(self, port):
        kwargs = {
            "port": port,
            "baudrate": self.baudrate,
            "bytesize": serial.EIGHTBITS,
            "parity": serial.PARITY_NONE,
            "stopbits": serial.STOPBITS_ONE,
            "timeout": 1,
        }
        try:
            return serial.Serial(exclusive=True, **kwargs)
        except TypeError:
            return serial.Serial(**kwargs)

    def _warn_interval(self, attr_name, message, interval):
        now = time.time()
        last = getattr(self, attr_name)
        if now - last >= interval:
            self.get_logger().warn(message)
            setattr(self, attr_name, now)

    def _ensure_serial_open(self, force=False):
        if serial is None:
            return False
        if self.serial_handle is not None and self.serial_handle.is_open:
            return True

        now = time.time()
        if not force and now - self.last_open_attempt < 2.0:
            return False
        self.last_open_attempt = now

        if self.serial_handle is not None:
            try:
                self.serial_handle.close()
            except Exception:
                pass
            self.serial_handle = None

        for port in self._candidate_ports():
            if not port or not os.path.exists(port):
                continue
            try:
                handle = self._open_serial(port)
            except Exception as exc:
                self._warn_interval(
                    "last_port_warn",
                    f"Failed to open gas sensor port {port}: {exc}",
                    5.0,
                )
                continue

            self.serial_handle = handle
            self.active_port = port
            self.short_frame_count = 0
            self.get_logger().info(f"Gas sensor connected on {port}")
            return True

        self._warn_interval(
            "last_port_warn",
            "Gas sensor serial port not available yet",
            10.0,
        )
        return False

    def _poll_sensor(self):
        if not self._ensure_serial_open():
            return

        try:
            self.serial_handle.reset_input_buffer()
            self.serial_handle.write(self.send_frame)
            time.sleep(0.1)
            response = self.serial_handle.read(13)
            if len(response) != 13:
                self.short_frame_count += 1
                self._warn_interval(
                    "last_short_frame_warn",
                    (
                        f"Gas sensor short frame on {self.active_port}: {len(response)} bytes "
                        f"(count={self.short_frame_count})"
                    ),
                    5.0,
                )
                return

            self.short_frame_count = 0
            msg = GasData()
            msg.co_concentration = float((response[3] << 8) + response[4])
            msg.hydrogen_concentration = float((response[5] << 8) + response[6])
            msg.voc_concentration = float((response[7] << 8) + response[8])
            msg.smoke_concentration = 40.0 if (response[10] & 0x08) else 0.0
            now_msg = self.get_clock().now().to_msg()
            msg.stamp = float(now_msg.sec) + float(now_msg.nanosec) / 1e9
            self.publisher.publish(msg)
        except Exception as exc:
            self._warn_interval(
                "last_read_error",
                f"Gas sensor read failed on {self.active_port}: {exc}",
                5.0,
            )
            try:
                self.serial_handle.close()
            except Exception:
                pass
            self.serial_handle = None

    def destroy_node(self):
        if self.serial_handle is not None:
            try:
                self.serial_handle.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GasSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
