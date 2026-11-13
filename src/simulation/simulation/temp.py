import rclpy
from rclpy.node import Node
import pygame
import math
from ros_custom_messages.msg import MasterNavigationMessage, MasterStopSignal, NavigatorNavigationMessage, TaskMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from ros_custom_messages.srv import GetMapState  # Import the service definition

# Map and robot dimensions
MAP_LENGTH_MM = 3000
MAP_WIDTH_MM = 2000
ROBOT_LENGTH_MM = 230
ROBOT_WIDTH_MM = 370
ENEMY_ROBOT_RADIUS = 150

# Scaling factors
WINDOW_LENGTH_PX = 1080
WINDOW_WIDTH_PX = 720
SCALE_X = WINDOW_LENGTH_PX / MAP_LENGTH_MM
SCALE_Y = WINDOW_WIDTH_PX / MAP_WIDTH_MM
ROBOT_LENGTH_PX = int(ROBOT_LENGTH_MM * SCALE_X)
ROBOT_WIDTH_PX = int(ROBOT_WIDTH_MM * SCALE_Y)
ENEMY_ROBOT_RADIUS_PX = int(ENEMY_ROBOT_RADIUS * SCALE_X)

class RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub = self.create_subscription(Point, 'Enemy_Robot_Position', self.enemy_position_callback, qos_profile)
        self.subscription = self.create_subscription(NavigatorNavigationMessage, 'ros_can_interface/navigator_nav', self.nav_callback, 10)
        self.stop_subscription = self.create_subscription(MasterStopSignal, 'ros_can_interface/stop_signal', self.stop_callback, 10)
        
        # Map state service client
        self.map_state_client = self.create_client(GetMapState, 'get_map_state_service')
        while not self.map_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for get_map_state_service...')
        self.timer = self.create_timer(1.5, self.request_map_state)
        
        # True tasks places service client
        self.true_tasks_client = self.create_client(GetMapState, 'true_tasks_places_service')
        while not self.true_tasks_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for true_tasks_places_service...')
        self.timer_true_tasks = self.create_timer(2.0, self.request_true_tasks)
        
        self.tasks = []
        self.true_tasks = []
        self.obstacles = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_angle = 0.0
        self.stop_signal_received = False
        self.get_logger().info("Robot simulation node initialized.")
    
    def request_map_state(self):
        request = GetMapState.Request()
        future = self.map_state_client.call_async(request)
        future.add_done_callback(self.handle_map_state_response)
    
    def handle_map_state_response(self, future):
        try:
            response = future.result()
            self.tasks = response.tasks
            self.obstacles = response.obstacle_positions
            self.get_logger().info(f"Updated map state: {len(self.tasks)} tasks, {len(self.obstacles)} obstacles")
        except Exception as e:
            self.get_logger().error(f"Failed to get map state: {e}")
    
    def request_true_tasks(self):
        request = GetMapState.Request()
        future = self.true_tasks_client.call_async(request)
        future.add_done_callback(self.handle_true_tasks_response)
    
    def handle_true_tasks_response(self, future):
        try:
            response = future.result()
            self.true_tasks = response.tasks
            self.get_logger().info(f"Updated true tasks: {len(self.true_tasks)} tasks")
        except Exception as e:
            self.get_logger().error(f"Failed to get true tasks: {e}")
    
    def enemy_position_callback(self, msg):
        self.enemy_x = msg.x
        self.enemy_y = msg.y
    
    def nav_callback(self, msg):
        if not self.stop_signal_received:
            self.robot_x = msg.nav_current_x
            self.robot_y = msg.nav_current_y
            self.robot_angle = msg.nav_current_angle
    
    def stop_callback(self, msg):
        self.stop_signal_received = True

def axis_conversion(y):
    return abs(y - 2000)

def main(args=None):
    rclpy.init(args=args)
    pygame.init()
    robot_simulation_node = RobotSimulation()
    screen = pygame.display.set_mode((WINDOW_LENGTH_PX, WINDOW_WIDTH_PX))
    map_image = pygame.image.load("map.png")
    map_image = pygame.transform.scale(map_image, (WINDOW_LENGTH_PX, WINDOW_WIDTH_PX))
    robot_image = pygame.image.load("robot.png")
    robot_image = pygame.transform.scale(robot_image, (ROBOT_LENGTH_PX, ROBOT_WIDTH_PX))
    clock = pygame.time.Clock()
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        rclpy.spin_once(robot_simulation_node, timeout_sec=0.01)
        screen.blit(map_image, (0, 0))
        
        robot_x_px = int(robot_simulation_node.robot_x * SCALE_X)
        robot_y_px = int(axis_conversion(robot_simulation_node.robot_y) * SCALE_Y)
        rotated_robot = pygame.transform.rotate(robot_image, robot_simulation_node.robot_angle)
        robot_rect = rotated_robot.get_rect(center=(robot_x_px, robot_y_px))
        screen.blit(rotated_robot, robot_rect)
        
        task_surface = pygame.Surface((20, 20), pygame.SRCALPHA)
        task_surface.fill((0, 255, 0, 128))
        true_task_surface = pygame.Surface((20, 20), pygame.SRCALPHA)
        true_task_surface.fill((255, 165, 0, 255))
        obstacle_surface = pygame.Surface((20, 20), pygame.SRCALPHA)
        obstacle_surface.fill((255, 0, 0, 128))
        
        for task in robot_simulation_node.tasks:
            task_x_px = int(task.position.x * SCALE_X)
            task_y_px = int(axis_conversion(task.position.y) * SCALE_Y)
            screen.blit(task_surface, (task_x_px - 10, task_y_px - 10))
        
        for true_task in robot_simulation_node.true_tasks:
            true_task_x_px = int(true_task.position.x * SCALE_X)
            true_task_y_px = int(axis_conversion(true_task.position.y) * SCALE_Y)
            screen.blit(true_task_surface, (true_task_x_px - 10, true_task_y_px - 10))
        
        for obstacle in robot_simulation_node.obstacles:
            obs_x_px = int(obstacle.x * SCALE_X)
            obs_y_px = int(axis_conversion(obstacle.y) * SCALE_Y)
            screen.blit(obstacle_surface, (obs_x_px - 10, obs_y_px - 10))
        
        pygame.display.flip()
        clock.tick(30)
    
    pygame.quit()
    robot_simulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
