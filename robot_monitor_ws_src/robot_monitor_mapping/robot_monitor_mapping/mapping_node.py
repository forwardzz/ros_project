import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time
import serial
import struct
import threading

class RPLIDARA1Driver:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.is_connected = False
        
    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.is_connected = True
            self._start_scan()
            return True
        except Exception as e:
            print(f"RPLIDAR A1 connection failed: {e}")
            return False
            
    def _start_scan(self):
        cmd = bytes([0xA2, 0x20])
        self.serial.write(cmd)
        
    def get_scan_data(self):
        if not self.is_connected:
            return None
            
        try:
            scan_points = []
            response = self.serial.read(5)
            if len(response) == 5:
                start_bit = response[0]
                quality = response[1]
                angle_info = (response[3] << 8) | response[2]
                distance = (response[5] << 8) | response[4] if len(response) > 5 else 0
                
                if distance > 0:
                    angle = (angle_info >> 1) / 64.0
                    distance_mm = distance / 4.0
                    scan_points.append((angle, distance_mm / 1000.0))
                    
            return scan_points
        except Exception as e:
            print(f"Error reading RPLIDAR data: {e}")
            return None
            
    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.is_connected = False


class IMUDriver:
    def __init__(self, port="/dev/ttyUSB1", baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.is_connected = False
        
    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.is_connected = True
            return True
        except Exception as e:
            print(f"IMU connection failed: {e}")
            return False
            
    def read_imu_data(self):
        if not self.is_connected:
            return None
            
        try:
            data = self.serial.read(11)
            if len(data) == 11:
                roll = struct.unpack('<h', data[1:3])[0] / 100.0
                pitch = struct.unpack('<h', data[3:5])[0] / 100.0
                yaw = struct.unpack('<h', data[5:7])[0] / 100.0
                
                gyro_x = struct.unpack('<h', data[7:9])[0] / 100.0
                gyro_y = struct.unpack('<h', data[9:11])[0] / 100.0
                
                return {
                    'roll': roll,
                    'pitch': pitch, 
                    'yaw': yaw,
                    'gyro_x': gyro_x,
                    'gyro_y': gyro_y
                }
        except Exception as e:
            print(f"Error reading IMU data: {e}")
            return None
            
    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.is_connected = False


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        self.declare_parameter('lidar_port', '/dev/ttyUSB0')
        self.declare_parameter('lidar_baudrate', 115200)
        self.declare_parameter('imu_port', '/dev/ttyUSB1')
        self.declare_parameter('imu_baudrate', 9600)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('save_map_path', '/home/user/maps/')
        
        self.map_resolution = self.get_parameter('map_resolution').value
        self.save_map_path = self.get_parameter('save_map_path').value
        
        self.lidar_driver = RPLIDARA1Driver(
            self.get_parameter('lidar_port').value,
            self.get_parameter('lidar_baudrate').value
        )
        
        self.imu_driver = IMUDriver(
            self.get_parameter('imu_port').value,
            self.get_parameter('imu_baudrate').value
        )
        
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.scan_data = []
        self.imu_data = None
        self.map_data = None
        
        self._init_hardware()
        
        self.lidar_timer = self.create_timer(0.1, self.publish_laser_scan)
        self.imu_timer = self.create_timer(0.01, self.publish_imu_data)
        self.map_timer = self.create_timer(1.0, self.update_and_publish_map)
        
        self.get_logger().info('Mapping node started successfully')
        
    def _init_hardware(self):
        lidar_connected = self.lidar_driver.connect()
        imu_connected = self.imu_driver.connect()
        
        if lidar_connected:
            self.get_logger().info('RPLIDAR A1 connected successfully')
        else:
            self.get_logger().warn('RPLIDAR A1 connection failed, using simulation mode')
            
        if imu_connected:
            self.get_logger().info('IMU connected successfully')
        else:
            self.get_logger().warn('IMU connection failed, using simulation mode')
            
    def publish_laser_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser'
        scan_msg.angle_min = -3.14159
        scan_msg.angle_max = 3.14159
        scan_msg.angle_increment = 0.0174533
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.15
        scan_msg.range_max = 12.0
        
        if self.lidar_driver.is_connected:
            points = self.lidar_driver.get_scan_data()
            if points:
                scan_msg.ranges = [p[1] for p in points if p[1] > 0.15 and p[1] < 12.0]
                scan_msg.intensities = [p[0] for p in points if p[1] > 0.15 and p[1] < 12.0]
            else:
                scan_msg.ranges = [0.0] * 360
                scan_msg.intensities = [0.0] * 360
        else:
            ranges = np.random.uniform(0.5, 10.0, 360).tolist()
            scan_msg.ranges = ranges
            scan_msg.intensities = [0.0] * 360
            
        self.laser_pub.publish(scan_msg)
        
    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        if self.imu_driver.is_connected:
            data = self.imu_driver.read_imu_data()
            if data:
                q = self._euler_to_quaternion(data['roll'], data['pitch'], data['yaw'])
                imu_msg.orientation.x = q[0]
                imu_msg.orientation.y = q[1]
                imu_msg.orientation.z = q[2]
                imu_msg.orientation.w = q[3]
            else:
                imu_msg.orientation.w = 1.0
        else:
            imu_msg.orientation.w = 1.0
            
        self.imu_pub.publish(imu_msg)
        
    def _euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return [qx, qy, qz, qw]
        
    def update_and_publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = 200
        map_msg.info.height = 200
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        map_data = [-1] * (200 * 200)
        
        for i in range(200):
            for j in range(200):
                dist = np.sqrt((i - 100) ** 2 + (j - 100) ** 2)
                if dist < 10:
                    map_data[i * 200 + j] = 0
                elif dist < 50 and np.random.random() < 0.1:
                    map_data[i * 200 + j] = 100
                else:
                    map_data[i * 200 + j] = -1
                    
        map_msg.data = map_data
        self.map_pub.publish(map_msg)
        
        self._publish_tf()
        
    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)
        
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'laser'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.1
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t3)
        
        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = 'base_link'
        t4.child_frame_id = 'imu_link'
        t4.transform.translation.x = 0.0
        t4.transform.translation.y = 0.0
        t4.transform.translation.z = 0.05
        t4.transform.rotation.x = 0.0
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t4)
        
    def destroy_node(self):
        self.lidar_driver.disconnect()
        self.imu_driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
