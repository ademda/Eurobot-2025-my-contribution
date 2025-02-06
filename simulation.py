import rclpy
from rclpy.node import Node
import pygame
import math
from ros_custom_messages.msg import MasterNavigationMessage, MasterStopSignal, NavigatorNavigationMessage  # Your custom message types
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from ros_custom_messages.srv import GetMapState  # Import the service definition
from ros_custom_messages.msg import ObstacleMsg, TaskMsg  #

# Map and robot dimensions in millimeters
MAP_LENGTH_MM = 3000  # Map length in millimeters (longer side)
MAP_WIDTH_MM = 2000   # Map width in millimeters (shorter side)
ROBOT_LENGTH_MM = 230  # Robot length in millimeters (longer side)
ROBOT_WIDTH_MM = 370   # Robot width in millimeters (shorter side)
ENEMY_ROBOT_RADUIS = 150
# Scaling factors to fit the map into a window
WINDOW_LENGTH_PX = 1080  # Window length (longer side) in pixels
WINDOW_WIDTH_PX = 720   # Window width (shorter side) in pixels
SCALE_X = WINDOW_LENGTH_PX / MAP_LENGTH_MM  # Scale factor for X
SCALE_Y = WINDOW_WIDTH_PX / MAP_WIDTH_MM  # Scale factor for Y
WHEEL_CENTER_DISTANCE = 30 
# Robot dimensions in pixels
ROBOT_LENGTH_PX = int(ROBOT_LENGTH_MM * SCALE_X)
ROBOT_WIDTH_PX = int(ROBOT_WIDTH_MM * SCALE_Y)
ENEMY_ROBOT_RADUIS_PX = int(ENEMY_ROBOT_RADUIS * SCALE_X)
class RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.sub = self.create_subscription(Point, 'Enemy_Robot_Position', self.enemy_position_callback, qos_profile) 

        # Subscribe to the navigation topic
        self.subscription = self.create_subscription(
            NavigatorNavigationMessage, 'ros_can_interface/navigator_nav', self.nav_callback, 10
        )
        
        # Subscribe to the stop signal topic
        self.stop_subscription = self.create_subscription(
            MasterStopSignal, 'ros_can_interface/stop_signal', self.stop_callback, 10
        )

        self.map_state_client = self.create_client(GetMapState, 'get_map_state_service')
        while not self.map_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for get_map_state_service...')

        # Timer to periodically request map state
        self.timer = self.create_timer(1.5, self.request_map_state)

        # Storage for tasks and obstacles
        self.tasks = []
        self.obstacles = []

        # Robot's position and orientation (initialized to 0)
        self.robot_x = 0.0  # X position in millimeters
        self.robot_y = 0.0  # Y position in millimeters
        self.robot_angle = 0.0  # Orientation angle in degrees

        # Flag to check if stop signal is received
        self.stop_signal_received = False

        self.get_logger().info("Robot simulation node initialized.")

    def request_map_state(self):
            """Send a request to the map state service."""
            request = GetMapState.Request()  # Create an empty request
            future = self.map_state_client.call_async(request)
            future.add_done_callback(self.handle_map_state_response)

    def handle_map_state_response(self, future):
        """Callback to handle the response from the map state service."""
        try:
            response = future.result()

            # Update tasks and obstacles
            self.tasks = response.tasks
            self.obstacles = response.obstacle_positions

            self.get_logger().info(f"Updated map state: {len(self.tasks)} tasks, {len(self.obstacles)} obstacles")
        except Exception as e:
            self.get_logger().error(f"Failed to get map state: {e}")

    def enemy_position_callback(self, msg):
        """Callback function to update the enemy robot's position."""
        self.enemy_x = msg.x
        self.enemy_y = msg.y
        self.get_logger().info(f"Enemy robot position: X={self.enemy_x}, Y={self.enemy_y}")

    def nav_callback(self, msg):
        """Callback function to update the robot's position and orientation immediately."""
        if not self.stop_signal_received:  # Only update position if stop signal not received
            # Instantly update robot's position and orientation from the message
            self.robot_x = msg.nav_current_x
            self.robot_y = msg.nav_current_y
            self.robot_angle = msg.nav_current_angle  # Target orientation
            self.get_logger().info(f"Teleported to: X={self.robot_x}, Y={self.robot_y}, Angle={self.robot_angle}")
            self.robot_x = self.robot_x - WHEEL_CENTER_DISTANCE*math.cos(math.radians(self.robot_angle))
            self.robot_y = self.robot_y - WHEEL_CENTER_DISTANCE*math.sin(math.radians(self.robot_angle))

    def stop_callback(self, msg):
        """Callback function to stop the robot."""
        self.stop_signal_received = True
        self.get_logger().info("Stop signal received. Robot will not update its position.")

def axis_conversion(y):
    """Converts Y-axis to match the top-left origin system of Pygame."""
    return abs(y - 2000)

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Initialize Pygame
    pygame.init()

    # Create ROS 2 node
    robot_simulation_node = RobotSimulation()

    # Set up Pygame display window
    screen = pygame.display.set_mode((WINDOW_LENGTH_PX, WINDOW_WIDTH_PX))

    # Load and scale the map image
    map_image = pygame.image.load("map.png")  # Load your map image
    map_image = pygame.transform.scale(map_image, (WINDOW_LENGTH_PX, WINDOW_WIDTH_PX))

    # Load the robot image and scale it
    robot_image = pygame.image.load("robot.png")  # Make sure 'robot.png' is in the working directory
    robot_image = pygame.transform.scale(robot_image, (ROBOT_LENGTH_PX, ROBOT_WIDTH_PX))
    
    enemy_surface = pygame.Surface((ENEMY_ROBOT_RADUIS_PX * 2, ENEMY_ROBOT_RADUIS_PX * 2), pygame.SRCALPHA)

# Draw an orange circle onto the surface
    pygame.draw.circle(
    enemy_surface, 
    (255, 165, 0, 200),  # RGBA: Orange with some transparency
    (ENEMY_ROBOT_RADUIS_PX, ENEMY_ROBOT_RADUIS_PX),  # Center of the circle
    ENEMY_ROBOT_RADUIS_PX  # Radius
)
    # Clock to control the frame rate
    clock = pygame.time.Clock()

    # Main loop
    running = True
    while running:
        # Handle Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Spin ROS 2 node to process callbacks
        rclpy.spin_once(robot_simulation_node, timeout_sec=0.01)

        # Draw the map background
        screen.blit(map_image, (0, 0))  # Draw the map at the top-left corner
        # Convert robot's position (mm) to pixels
        robot_x_px = int(robot_simulation_node.robot_x * SCALE_X)
        robot_y_px = int(axis_conversion(robot_simulation_node.robot_y) * SCALE_Y)

        # Rotate the robot image to match its angle
        rotated_robot = pygame.transform.rotate(robot_image, robot_simulation_node.robot_angle)

        # Get the rotated image rectangle and update position
        robot_rect = rotated_robot.get_rect(center=(robot_x_px, robot_y_px))
        if hasattr(robot_simulation_node, 'enemy_x') and hasattr(robot_simulation_node, 'enemy_y'):
            enemy_x_px = int(robot_simulation_node.enemy_x * SCALE_X)
            enemy_y_px = int(axis_conversion(robot_simulation_node.enemy_y) * SCALE_Y)

            # Get the rectangle for the enemy image and position it
            enemy_rect = enemy_surface.get_rect(center=(enemy_x_px, enemy_y_px))
            screen.blit(enemy_surface, enemy_rect)
        screen.blit(rotated_robot, robot_rect)
        # Draw tasks
        # Create surfaces for tasks and obstacles
        task_surface = pygame.Surface((20, 20), pygame.SRCALPHA)  # Transparent surface for tasks
        task_surface.fill((0, 255, 0, 128))  # Semi-transparent green (R, G, B, Alpha)

        obstacle_surface = pygame.Surface((20, 20), pygame.SRCALPHA)  # Transparent surface for obstacles
        obstacle_surface.fill((255, 0, 0, 128))  # Semi-transparent red (R, G, B, Alpha)

        # Draw tasks
        for task in robot_simulation_node.tasks:
            task_x_px = int(task.position.x * SCALE_X)
            task_y_px = int(axis_conversion(task.position.y) * SCALE_Y)

            # Draw the task surface centered at the task's position
            screen.blit(task_surface, (task_x_px - 10, task_y_px - 10))  # Offset by half the surface size to center

        # Draw obstacles
        for obstacle in robot_simulation_node.obstacles:
            obs_x_px = int(obstacle.x * SCALE_X)
            obs_y_px = int(axis_conversion(obstacle.y) * SCALE_Y)

            # Draw the obstacle surface centered at the obstacle's position
            screen.blit(obstacle_surface, (obs_x_px - 10, obs_y_px - 10))  # Offset by half the surface size to center


        # Update the display
        pygame.display.flip()

        # Control frame rate
        clock.tick(30)  # 30 frames per second

    # Shutdown Pygame and ROS 2
    pygame.quit()
    robot_simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
