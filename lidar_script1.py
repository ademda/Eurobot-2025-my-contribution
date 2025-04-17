import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_custom_messages.msg import NavigatorNavigationMessage, MasterNavigationMessage
import math

jack_state = True
enemy_position = Point(x=0.0, y=0.0, z=0.0)  # Initialize enemy_position with default values
current_direction = True
class LidarReader(Node):
    def __init__(self):
        super().__init__('Lidar_Reader')
        #self.declare_parameter('Enemy_X', 0.0)
#        self.declare_parameter('Enemy_Y', 0.0)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.current_direction_sub = self.create_subscription(Bool, 'current_direction', self.current_direction_callback, 10)
        self.init_position_sub = self.create_subscription(MasterNavigationMessage,'init_position',self.init_position_callback ,10)

        # Publisher for obstacle detection
        self.obstacle_enemy_pub = self.create_publisher(Bool, 'Obstacle_Enemy_Detected', qos_profile)

        # Publisher for enemy position
        self.enemy_position_pub = self.create_publisher(Point, 'Enemy_Robot_Position', qos_profile)
        self.enemy_position_sub = self.create_subscription(Point, 'Enemy_Robot_Position', self.enemy_position_callback, qos_profile) 
        self.get_logger().warn('Created enemy_robot_position_subscriber')

        self.jack_state_subscriber = self.create_subscription(
            Bool,'jack_signal',self.jack_signal_callback ,10
        )

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
        self.nav_current_y = 0.195 #0.315
        self.nav_current_angle = 90.0
        self.robot_current_state = 0
        self.ObstacleDetected = False
        self.counter = 0
        self.get_logger().info(f'Received nav_current_x: {self.nav_current_x}')
        self.get_logger().info(f'Received nav_current_y: {self.nav_current_y}')
        self.get_logger().info(f'Received nav_current_angle: {self.nav_current_angle}')

    def current_direction_callback(self, msg):
        global current_direction
        current_direction = msg.data
        if current_direction:
            self.get_logger().info('Current direction: Forward')
        else:
            self.get_logger().info('Current direction: Backward')

    def enemy_position_callback(self, msg):
        global enemy_position
        enemy_position.x = (msg.x) / 1000
        enemy_position.y = (msg.y) / 1000
        enemy_position.z = 0.0
        self.Obstacle_Detected_Checker()
    
    def init_position_callback(self, msg):
        if self.counter == 0:
            self.nav_current_x = msg.nav_target_x / 1000
            self.nav_current_y = msg.nav_target_y / 1000
            self.nav_current_angle = msg.nav_target_angle
            #self.get_logger().info(f'Initialized nav_current_x: {self.nav_current_x}')
            #self.get_logger().info(f'Initialized nav_current_y: {self.nav_current_y}')
            #self.get_logger().info(f'Initialized nav_current_angle: {self.nav_current_angle}')
            self.counter += 1

    def Obstacle_Detected_Checker(self):
        global enemy_position
        global current_direction

        dx = enemy_position.x - self.nav_current_x
        dy = enemy_position.y - self.nav_current_y
        self.get_logger().info(f'Enemy Position: x={enemy_position.x}, y={enemy_position.y}')
        self.get_logger().info(f'Robot Position: x={self.nav_current_x}, y={self.nav_current_y}, angle={self.nav_current_angle}')

        distance = math.hypot(dx, dy)

        angle_rad = math.radians(self.nav_current_angle)
        heading_x = math.cos(angle_rad)
        heading_y = math.sin(angle_rad)

        if distance == 0:
            is_in_front = True
            self.get_logger().warn('Enemy and robot are at the same position!')
        else:
            enemy_vec_x = dx / distance
            enemy_vec_y = dy / distance

            dot = enemy_vec_x * heading_x + enemy_vec_y * heading_y
            angle = math.degrees(math.acos(dot))  # angle between vectors
            self.get_logger().warn(f'Angle: {angle}')
            is_in_front = 0 <= angle <= 90

        if is_in_front:
            self.get_logger().info('in front!')
        else:
            self.get_logger().info('not in front!')

        in_range = 0.05 < distance < 0.45

        obstacle_detected = False
        self.get_logger().warn(f'current_direction: {current_direction}')
        self.get_logger().warn(f'distance: {distance}')
        
        if current_direction and is_in_front and in_range:
            obstacle_detected = True
        elif not current_direction and is_in_front and in_range:
            obstacle_detected = True

        if obstacle_detected:
            self.get_logger().info('Obstacle Detected!')
            self.obstacle_enemy_pub.publish(Bool(data=True))
        else:
            self.ObstacleDetected = False
            self.get_logger().info('No Obstacle Detected')
            self.obstacle_enemy_pub.publish(Bool(data=False))


    def jack_signal_callback(self, msg):
        global jack_state
        jack_state = msg.data

    def nav_callback(self, msg):
        try:
            self.robot_current_state = msg.nav_current_state
            # Extract navigation parameters
            if (self.robot_current_state != 0):
                self.nav_current_x = msg.nav_current_x / 1000
                self.nav_current_y = msg.nav_current_y / 1000
                self.nav_current_angle = msg.nav_current_angle
                #print("x from lidar",self.nav_current_x)
                #print("y from lidar",self.nav_current_y)
            else:
                self.nav_current_x = 1.2
                self.nav_current_y = 0.195 #0.325
                self.nav_current_angle = 90.0
            #self.nav_current_x = msg.nav_current_x / 1000
            #self.nav_current_y = msg.nav_current_y / 1000
            #self.nav_current_angle = msg.nav_current_angle    
            #print("x from lidar",self.nav_current_x)
            #print("y from lidar",self.nav_current_y)    
            #self.get_logger().info(f'Received nav_current_x: {self.nav_current_x}')
            #self.get_logger().info(f'Received nav_current_y: {self.nav_current_y}')
            #self.get_logger().info(f'Received nav_current_angle: {self.nav_current_angle}')
        except IndexError as e:
            self.get_logger().error(f"Message format error: {e}")

    def scan_callback(self, scan_data):
        global jack_state
        if jack_state == True:
            angle1 = 0
            min_value, min_index = self.min_range_index(scan_data.ranges, len(scan_data.ranges), scan_data)

            angle1 = (math.degrees(min_index * scan_data.angle_increment) - 5)
            msg = Point()

            if min_index == -1:
                ClosestObstacle_X_Map = 0.0
                ClosestObstacle_Y_Map = 0.0
            else:
                ClosestObstacle_X_LIDAR = min_value * math.cos(angle1 * math.pi / 180)
                ClosestObstacle_Y_LIDAR = min_value * math.sin(angle1 * math.pi / 180)
                ClosestObstacle_X_Map = (ClosestObstacle_X_LIDAR * math.cos(self.nav_current_angle * math.pi / 180)) + (-ClosestObstacle_Y_LIDAR * math.sin(self.nav_current_angle * math.pi / 180)) + self.nav_current_x
                ClosestObstacle_Y_Map = (ClosestObstacle_X_LIDAR * math.sin(self.nav_current_angle * math.pi / 180)) + (ClosestObstacle_Y_LIDAR * math.cos(self.nav_current_angle * math.pi / 180)) + self.nav_current_y

            msg.x = ClosestObstacle_X_Map*1000
            msg.y = ClosestObstacle_Y_Map*1000
            msg.z = 0.0 
            self.enemy_position_pub.publish(msg)

            self.get_logger().info(f"ClosestObstacle Map Coordinates: X={ClosestObstacle_X_Map}, Y={ClosestObstacle_Y_Map}")
            self.get_logger().info(f"valeur minimale: {min_value}")
            if (min_value > 0.15) and (min_value < 0.45) and (0 < ClosestObstacle_X_Map < 3) and (0 < ClosestObstacle_Y_Map < 2):#was 0.5 min value was 0.2
                self.ObstacleDetected = True
                self.get_logger().info('Obstacle Detected!')
                self.obstacle_enemy_pub.publish(Bool(data=True))
            else:
                self.ObstacleDetected = False
                self.get_logger().info('No Obstacle Detected')
                self.obstacle_enemy_pub.publish(Bool(data=False))

    def min_range_index(self, ranges, count, scan_data):
        i = 0
        index_min = 0
        anglemin = 0
        min = 13

        # Skip invalid range values (0.0) at the beginning of the data
        while ranges[i] == 0.0:
            i = i + 1

        a = False
        while (a == False) and i < count - 2:
            # Calculate the angle and corresponding X, Y coordinates in the LIDAR's frame
            angle = (math.degrees(i * scan_data.angle_increment) - 5)
            range_val = ranges[i]
            X_LIDAR = range_val * math.cos(angle * math.pi / 180)
            Y_LIDAR = range_val * math.sin(angle * math.pi / 180)

            # Convert LIDAR coordinates to map coordinates using the current navigation angle and position
            X_Map = (X_LIDAR * math.cos(self.nav_current_angle * math.pi / 180)) + (-Y_LIDAR * math.sin(self.nav_current_angle * math.pi / 180)) + self.nav_current_x
            Y_Map = X_LIDAR * math.sin(self.nav_current_angle * math.pi / 180) + Y_LIDAR * math.cos(self.nav_current_angle * math.pi / 180) + self.nav_current_y

            # If the point is within the map area and the range is not zero, update the minimum range
            if (0 < X_Map < 3) and (0 < Y_Map < 2) and (ranges[i] != 0.0):
                min = range_val
                index_min = i
                a = True  # Stop the loop once we find the first valid point
            else:
                i = i + 1

        # If no valid point was found in the range
        if not ((0 < X_Map < 3) and (0 < Y_Map < 2)):
            X_Map = 0
            Y_Map = 0
            min = math.sqrt(pow(self.nav_current_x, 2) + pow(self.nav_current_y, 2))  # Distance from the origin
            index_min = -1

        # Search for the closest valid range if a valid point was found
        for j in range(i + 1, count - 1):
            range_val1 = ranges[j]
            if (range_val1 < min) and (ranges[j] != 0.0):
                angle2 = (math.degrees(j * scan_data.angle_increment) - 5)
                X_LIDAR_j = range_val1 * math.cos(angle2 * math.pi / 180)
                Y_LIDAR_j = range_val1 * math.sin(angle2 * math.pi / 180)
                X_Map_j = (X_LIDAR_j * math.cos(self.nav_current_angle * math.pi / 180)) + (-Y_LIDAR_j * math.sin(self.nav_current_angle * math.pi / 180)) + self.nav_current_x
                Y_Map_j = X_LIDAR_j * math.sin(self.nav_current_angle * math.pi / 180) + Y_LIDAR_j * math.cos(self.nav_current_angle * math.pi / 180) + self.nav_current_y

                # If the new point is within the map bounds and closer than the previous minimum, update the minimum
                if (0 < X_Map_j < 3) and (0 < Y_Map_j < 2):
                    min = range_val1
                    anglemin = angle2
                    X_Map = X_Map_j
                    Y_Map = Y_Map_j
                    index_min = j

        return min, index_min


def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
