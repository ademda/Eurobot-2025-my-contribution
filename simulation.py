import rclpy
from rclpy.node import Node
import pygame
import math
from ros_custom_messages.msg import MasterNavigationMessage, MasterStopSignal  # Your custom message types

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

# Robot movement speed (in pixels per frame)
SPEED = 4  # Change this value to control robot speed

# Rotation speed (degrees per frame)
ROTATION_SPEED = 1  # Speed at which robot rotates towards the target angle

class RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')

        # Subscribe to the navigation topic
        self.subscription = self.create_subscription(
            MasterNavigationMessage, 'ros_can_interface/master_nav', self.nav_callback, 10
        )
        
        # Subscribe to the stop signal topic
        self.stop_subscription = self.create_subscription(
            MasterStopSignal, 'ros_can_interface/stop_signal', self.stop_callback, 10
        )

        # Robot's position and orientation (initialized to 0)
        self.robot_x = 0.0  # X position in millimeters
        self.robot_y = 0.0  # Y position in millimeters
        self.robot_angle = 0.0  # Orientation angle in degrees

        # Target position (in mm)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_angle = 0.0

        # Flag to check if stop signal is received
        self.stop_signal_received = False

        self.get_logger().info("Robot simulation node initialized.")

    def nav_callback(self, msg):
        """Callback function to update the robot's target position and orientation."""
        if not self.stop_signal_received:  # Only update target if stop signal not received
            # Update robot's target position and orientation from the message
            self.target_x = msg.nav_target_x
            self.target_y = msg.nav_target_y
            self.target_angle = msg.nav_target_angle  # Target orientation
            self.get_logger().info(f"Received nav_target: X={self.target_x}, Y={self.target_y}, Angle={self.target_angle}")

    def stop_callback(self, msg):
        """Callback function to stop the robot."""
        self.stop_signal_received = True
        self.get_logger().info("Stop signal received. Robot will stop immediately.")

        # You can reset the robot's target position to its current position or any other logic
        self.target_x = self.robot_x
        self.target_y = self.robot_y
        self.target_angle = self.robot_angle

def move_towards_target(robot_x, robot_y, target_x, target_y, speed):
    """Move the robot slowly towards the target position."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.hypot(dx, dy)

    # If the target is close enough, don't move further
    if distance < speed:
        return target_x, target_y

    # Move the robot a little towards the target position
    move_x = (dx / distance) * speed
    move_y = (dy / distance) * speed
    robot_x += move_x
    robot_y += move_y

    return robot_x, robot_y

def calculate_angle_to_target(robot_x, robot_y, target_x, target_y):
    """Calculate the angle between the robot's position and the target position."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    angle = math.degrees(math.atan2(dy, dx))
    return angle

def rotate_towards_target(robot_angle, target_angle, rotation_speed):
    """Rotate the robot slowly towards the target orientation."""
    angle_diff = (target_angle - robot_angle + 180) % 360 - 180  # Normalize the angle difference to [-180, 180]
    # Rotate the robot gradually
    if abs(angle_diff) < rotation_speed:
        return target_angle  # If the difference is small enough, we can directly set the angle

    if angle_diff > 0:
        return robot_angle + rotation_speed
    else:
        return robot_angle - rotation_speed

def axis_conversion(y):
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

        # Step 1: Orient the robot towards the target position (x, y)
        if not robot_simulation_node.stop_signal_received:
            target_angle = calculate_angle_to_target(
                robot_simulation_node.robot_x, robot_simulation_node.robot_y, 
                robot_simulation_node.target_x, robot_simulation_node.target_y
            )

            distance_to_target = math.hypot(
                    robot_simulation_node.robot_x - robot_simulation_node.target_x, 
                    robot_simulation_node.robot_y - robot_simulation_node.target_y)
            
            # Rotate the robot to face the target position
            if distance_to_target >= SPEED:
                robot_simulation_node.robot_angle = rotate_towards_target(
                    robot_simulation_node.robot_angle, target_angle, ROTATION_SPEED
                )
            
            # Once the robot is facing the target, move towards it
            if abs(robot_simulation_node.robot_angle - target_angle) < 1:  # A small threshold for orientation
                robot_simulation_node.robot_x, robot_simulation_node.robot_y = move_towards_target(
                    robot_simulation_node.robot_x, robot_simulation_node.robot_y,
                    robot_simulation_node.target_x, robot_simulation_node.target_y, SPEED
                )

                # Use a small threshold to determine when to stop moving
                distance_to_target = math.hypot(
                    robot_simulation_node.robot_x - robot_simulation_node.target_x, 
                    robot_simulation_node.robot_y - robot_simulation_node.target_y
                )

                # Stop the robot when it's close enough to the target position

            if distance_to_target < SPEED:
                robot_simulation_node.robot_angle = rotate_towards_target(
                    robot_simulation_node.robot_angle, robot_simulation_node.target_angle, ROTATION_SPEED
                )    
                

        # Draw the map background
        screen.blit(map_image, (0, 0))  # Draw the map at the top-left corner

        # Convert robot's position (mm) to pixels
        robot_x_px = int(robot_simulation_node.robot_x * SCALE_X)
        robot_y_px = int(axis_conversion(robot_simulation_node.robot_y) * SCALE_Y)

        # Rotate the robot image to match its angle
        rotated_robot = pygame.transform.rotate(robot_image, robot_simulation_node.robot_angle)

        # Get the rotated image rectangle and update position
        robot_rect = rotated_robot.get_rect(center=(robot_x_px, robot_y_px))
        screen.blit(rotated_robot, robot_rect)

        # Draw a line between the robot's current position and target position
        target_x_px = int(robot_simulation_node.target_x * SCALE_X)
        target_y_px = int(axis_conversion(robot_simulation_node.target_y) * SCALE_Y)
        pygame.draw.line(screen, (255, 0, 0), (robot_x_px, robot_y_px), (target_x_px, target_y_px), 2)

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
