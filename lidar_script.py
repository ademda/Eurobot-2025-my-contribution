import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_custom_messages.msg import NavigatorNavigationMessage
from geometry_msgs.msg import Point


class LidarReader(Node):
    def __init__(self):
        super().__init__('Lidar_Reader')
        #self.declare_parameter('Enemy_X', 0.0)
       # self.declare_parameter('Enemy_Y',0.0  )
        # Publisher for obstacle detection
        self.pub = self.create_publisher(Bool, 'obstacle_detected', 10)

        # Define the QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.enemy_position_pub = self.create_publisher(Point, 'Enemy_Robot_Position', qos_profile)

        # Subscriber to navigation parameters
        self.nav_subscription = self.create_subscription(
            NavigatorNavigationMessage,
            'ros_can_interface/navigator_nav',
            self.nav_callback,
            10
            
        )
         # Subscriber to scan parameters
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Initialize navigation parameters
        self.nav_current_x = 1.2
        self.nav_current_y = 0.275
        self.nav_current_angle = 90.0
        self.robot_current_state = 0
        self.ObstacleDetected = False
        self.get_logger().info(f'Received nav_currentX: {self.nav_current_x}')
        self.get_logger().info(f'Received nav_currentY: {self.nav_current_y}')
        self.get_logger().info(f'Received nav_currentTetha: {self.nav_current_angle}')
    def nav_callback(self, msg):
        try:
            self.robot_current_state = msg.nav_current_state
            # Extract navigation parameters
            if self.robot_current_state != 0:
                self.nav_currentX = msg.nav_current_x / 1000
                self.nav_currentY = msg.nav_current_y / 1000
                self.nav_currentTetha = msg.nav_current_angle
                self.get_logger().info(f'Received nav_currentX: {self.nav_currentX}')
                self.get_logger().info(f'Received nav_currentY: {self.nav_currentY}')
                self.get_logger().info(f'Received nav_currentTetha: {self.nav_currentTetha}')
            else:
                self.nav_currentX = 1.2
                self.nav_currentY = 0.275
                self.nav_currentTetha = 90.0
        except IndexError as e:
            self.get_logger().error(f"Message format error: {e}")

    def scan_callback(self, scan_data):
        angle1 = 0
        min_value, min_index = self.min_range_index(scan_data.ranges, len(scan_data.ranges), scan_data)

        angle1 = (math.degrees(min_index * scan_data.angle_increment) - 5)
        msg = Point()
        if min_index == -1:
            ClosestObstacle_X_Map = 0
            ClosestObstacle_Y_Map = 0
        else:
            ClosestObstacle_X_LIDAR = min_value * math.cos(angle1 * math.pi / 180)
            ClosestObstacle_Y_LIDAR = min_value * math.sin(angle1 * math.pi / 180)
            ClosestObstacle_X_Map = (ClosestObstacle_X_LIDAR * math.cos(self.nav_current_angle * math.pi / 180)) + (-ClosestObstacle_Y_LIDAR * math.sin(self.nav_current_angle * math.pi / 180)) + self.nav_current_x
            ClosestObstacle_Y_Map = (ClosestObstacle_X_LIDAR * math.sin(self.nav_current_angle * math.pi / 180)) + (ClosestObstacle_Y_LIDAR * math.cos(self.nav_current_angle* math.pi / 180))+ self.nav_current_y

        msg.x = ClosestObstacle_X_Map*1000
        msg.y = ClosestObstacle_Y_Map*1000
        msg.z = 0.0 
        self.enemy_position_pub.publish(msg)
        self.get_logger().info(f"ClosestObstacle Map Coordinates: X={ClosestObstacle_X_Map}, Y={ClosestObstacle_Y_Map}")
        self.get_logger().info(f"valeur minimale: {min_value}")
        
        if (min_value > 0.2) and (min_value < 0.45) and (0 < ClosestObstacle_X_Map < 3) and (0 < ClosestObstacle_Y_Map < 2):
            self.ObstacleDetected = True
            self.get_logger().info('Obstacle Detected!')
            #self.get_logger().info(f"valeur minimale: {min_value}")
            self.pub.publish(Bool(data=True))
        else:
            self.ObstacleDetected = False
            self.get_logger().info('No Obstacle Detected')
            self.pub.publish(Bool(data=False))

    def min_range_index(self, ranges, count, scan_data):
        i = 0
        min = 13
        index_min = 0
        while ranges[i] == 0.0:
            i += 1
        while i < count - 2:
            range_val = ranges[i]
            angle = math.degrees(i * scan_data.angle_increment) - 5
            X_LIDAR = range_val * math.cos(angle * math.pi / 180)
            Y_LIDAR = range_val * math.sin(angle * math.pi / 180)
            X_Map = (X_LIDAR * math.cos(self.nav_current_angle * math.pi / 180)) + (-Y_LIDAR * math.sin(self.nav_current_angle * math.pi / 180)) + self.nav_current_x
            Y_Map = X_LIDAR * math.sin(self.nav_current_angle * math.pi / 180) + Y_LIDAR * math.cos(self.nav_current_angle * math.pi / 180) + self.nav_current_y

            if (0 < X_Map < 3) and (0 < Y_Map < 2) and (ranges[i] != 0.0):
                min = range_val
                index_min = i
                break
            i += 1

        return min, index_min

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()