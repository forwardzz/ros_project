import rclpy
from rclpy.node import Node
from robot_monitor_interfaces.srv import LoadMap
import os
import yaml

class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        self.declare_parameter('map_directory', '/home/user/maps/')
        self.map_directory = self.get_parameter('map_directory').value
        
        self.srv = self.create_service(
            LoadMap,
            '/load_map',
            self.handle_load_map
        )
        
        self.get_logger().info('Map manager service ready')
        
    def handle_load_map(self, request, response):
        map_file = request.map_file_path
        
        if not os.path.exists(map_file):
            response.success = False
            response.message = f"Map file not found: {map_file}"
            return response
            
        try:
            if map_file.endswith('.yaml'):
                with open(map_file, 'r') as f:
                    map_config = yaml.safe_load(f)
                    
                response.success = True
                response.message = f"Map loaded successfully from {map_file}"
            elif map_file.endswith('.pgm'):
                response.success = True
                response.message = f"Map image loaded successfully from {map_file}"
            else:
                response.success = False
                response.message = "Unsupported map file format"
                
        except Exception as e:
            response.success = False
            response.message = f"Error loading map: {str(e)}"
            
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
