import rclpy
from rclpy.node import Node
import pygame
from ros_custom_messages.msg import MasterNavigationMessage  # Your custom message type

# Map and robot dimensions in millimeters
MAP_LENGTH_MM = 3000  # Map length in millimeters (longer side)
MAP_WIDTH_MM = 2000   # Map width in millimeters (shorter side)
ROBOT_LENGTH_MM = 230  # Robot length in millimeters (longer side)
ROBOT_WIDTH_MM = 370   # Robot width in millimeters (shorter side)

# Scaling factors to fit the map into a window
WINDOW_LENGTH_PX = 1080  # Window length (longer side) in pixels
WINDOW_WIDTH_PX = 720   # Window width (shorter side) in pixels
SCALE_X = WINDOW_LENGTH_PX / MAP_LENGTH_MM  # Scale factor for X
SCALE_Y = WINDOW_WIDTH_PX / MAP_WIDTH_MM  # Scale factor for Y

# Robot dimensions in pixels
ROBOT_LENGTH_PX = int(ROBOT_LENGTH_MM * SCALE_X)
ROBOT_WIDTH_PX = int(ROBOT_WIDTH_MM * SCALE_Y)


class RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')

        # Subscribe to the navigation topic
        self.subscription = self.create_subscription(
            MasterNavigationMessage, 'ros_can_interface/master_nav', self.nav_callback, 10
        )

        # Robot's position and orientation (initialized to 0)
        self.robot_x = 0.0  # X position in millimeters
        self.robot_y = 0.0  # Y position in millimeters
        self.robot_angle = 0.0  # Orientation angle in degrees

        self.get_logger().info("Robot simulation node initialized.")

    def nav_callback(self, msg):
        """Callback function to update the robot's position and orientation."""
        # Update robot's position and orientation from the message
        self.robot_x = msg.nav_target_x
        self.robot_y = msg.nav_target_y
        self.robot_angle = msg.nav_target_angle

        # Log received data
        self.get_logger().info(f"Received nav_target: X={self.robot_x}, Y={self.robot_y}, Angle={self.robot_angle}")


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
        robot_y_px = int(robot_simulation_node.robot_y * SCALE_Y)

        # Rotate the robot image to match its angle
        rotated_robot = pygame.transform.rotate(robot_image, -robot_simulation_node.robot_angle)

        # Get the rotated image rectangle and update position
        robot_rect = rotated_robot.get_rect(center=(robot_x_px, robot_y_px))
        screen.blit(rotated_robot, robot_rect)

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
