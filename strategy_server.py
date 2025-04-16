"""
Author: <Adem Daly>

Description:
    This module is responsible for : 
        - Deciding which is the next task that should be executed :
            either a NORMAL_NAV task or BUILDING_PICKUP task or BUILDING_DROPDOWN task 
            or BONDEROLE_POSE or RETURN_TO_FINISH_SPOT

Note: 
    - right now, the task decision is task based on tasks_list[], the strategy node will get the next task to   
    to do and the next task isn't generated staticly...but based on :
        -if the enemy robot did it or not          
    - should include generating path to task for the robot to follow based on rrt algorithm
    on: 
        - avaliable_tasks got from static_task_generator node
        - enemy_position
        - my position
        - map obstacles

Classes:
    - Strategy: includes the main logic of the strategy server(tasks decision and functions to get enemy position and map obstacles)         
    - Services: includes the services that the strategy server provides:
        -GetNextTaskPointService: service that provides the next task point to the StateMachine
    - Clients: includes the clients that the strategy server uses to communicate with other nodes
        -GetAvailableTasksClient: client that requests the available tasks from the static_task_generator node
        /should include the client that requests the map obstacles
    -Subscribers:
        -should include the subscriber that gets the enemy position     

Functions:
    - get_available_tasks: requests the available tasks from the static_task_generator node
    - get_task_from_id: returns the task object from the available_tasks list based on the task_id
    - get_next_task: returns the next task to be executed based on the id of the current task
    - convert_task_to_response_msg: converts a TaskMsg object to a GetNextTaskPoint.Response object for the state machine node
    - communication_instances_init: initializes the communication instances
"""

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_custom_messages.srv import GetMapState
from enum import Enum
from typing import List
from ros_custom_messages.msg import TaskMsg, ObstacleMsg, PathMsg  
from ros_custom_messages.srv import GetNextTaskPoint
from strategy.static_task_generator import Task, SpeedChoice, TaskType
from path_planning.pathplanner import PathPlanner
from geometry_msgs.msg import Pose2D, Point
from ros_custom_messages.msg import NavigatorNavigationMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

############# GLOBAL VARIABLES ################

publishers_instance = None
subscribers_instance = None
clients_instance = None
servers_instance = None
strategy_instance = None
path_planner_instance = None
robot_color = None
current_position = None
current_task = None
enemy_position = Pose2D(x=0.0,y=0.0,theta=0.0)

obstacle_positions = []
available_tasks = []
#tasks_list=[2, 5, 8, 9] #based on task id 2, 5, 8, 9

blue_prohibited_zones = [
   [0, 1550, 1950, 2000],   
    [2550, 650, 3000, 1100],     
   [550, 0, 1000, 150]  ,
    [1000, 0, 1450, 450]  ,
    [2550, 0, 3000, 150]  ,
    			    
]
#blue prohibited zones : zones taht blue shouldn't go to
yellow_prohibited_zones = [
    [1050, 1550, 3000, 2000],   
    [0, 650, 460, 1100],     
    [0, 0, 450, 150]  ,
    [1560, 0, 2000, 450]  ,
    [2005, 0, 2450, 150]  ,
    			    
]

##yellow prohibited zones : zones taht yellow shouldn't go to
################# CLASSES ##################

class Strategy(Node):
    global clients_instance

    def __init__(self):
        super().__init__('strategy_server')

    def initialize_pathplanner(self):
        global obstacle_positions
        global path_planner_instance
        global yellow_prohibited_zones
        global blue_prohibited_zones
        global robot_color
        if robot_color == "blue":
            path_planner_instance = PathPlanner(blue_prohibited_zones)
        elif robot_color == "yellow":
            path_planner_instance = PathPlanner(yellow_prohibited_zones)
        else:
            raise ValueError("Invalid robot color provided.")        


########## COMMUNICATION CLASSES ###########


class Subscribers():
    def __init__(self):
        self.strategy_navigator_receive_position_subscriber = Navigator_Receive_Position_Subscriber()
        self.strategy_enemy_robot_position_subscriber = Enemy_Robot_Position_Subscriber()
        self.strategy_my_robot_color = StrategyServerRobotColorSubscriber()

class Services():
    def __init__(self):
        self.get_next_task_point_service= GetNextTaskPointService()


class Clients():
    def __init__(self):
        self.get_map_state_client = GetMapStateClient()

class Publishers():
    def __init__(self):
        self.strategy_rrt_path_publisher = StrategyRRTPathPublisher()

class StrategyRRTPathPublisher(Node):
    def __init__(self):
        super().__init__('strategy_rrt_path_publisher')
        self.strategy_rrt_path_publisher = self.create_publisher(PathMsg, 'strategy_rrt_path', 10)
        self.get_logger().info('StrategyRRTPathPublisher node has been started.')
class Navigator_Receive_Position_Subscriber(Node):
    def __init__(self):
        super().__init__('strategy_navigator_receive_position_subscriber')
        self.navigator_receive_position_subscriber = self.create_subscription(NavigatorNavigationMessage, 'ros_can_interface/navigator_nav', self.navigator_receive_position_callback, 10)
        self.get_logger().info('StrategyNavigatorReceivePositionSubscriber node has been started.')

    def navigator_receive_position_callback(self ,nav_data: NavigatorNavigationMessage):
        global current_position
        global current_navigator_state
        #self.get_logger().info(f"Received: navigation data")    
        current_position = Pose2D(x=nav_data.nav_current_x, y=nav_data.nav_current_y, theta=nav_data.nav_current_angle)
        current_navigator_state = nav_data.nav_current_state

class StrategyServerRobotColorSubscriber(Node):
    def __init__(self):
        super().__init__('static_task_robot_color_subscriber')
        self.sub = self.create_subscription(String, 'robot_color', self.robot_color_callback, 10) 
        self.get_logger().warn('Created robot_color_subscriber:strateg_server node')
        self.counter = 0

    def robot_color_callback(self, msg):
        global robot_color
        global strategy_instance
        robot_color = msg.data
        #self.get_logger().info(f'Robot color: {robot_color}')
        if robot_color == "blue" and self.counter == 0:
            strategy_instance.initialize_pathplanner()
            self.counter += 1
        elif robot_color == "yellow" and self.counter == 0:
            strategy_instance.initialize_pathplanner()
            self.counter += 1
class Enemy_Robot_Position_Subscriber(Node):
    def __init__(self):
        super().__init__('strategy_enemy_robot_position_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.sub = self.create_subscription(Point, 'Enemy_Robot_Position', self.enemy_position_callback, qos_profile) 
        self.get_logger().warn('Created enemy_robot_position_subscriber')

    def enemy_position_callback(self, msg):
        global enemy_position
        enemy_position.x = msg.x
        enemy_position.y = msg.y
        enemy_position.theta = msg.z

class GetMapStateClient(Node):
    def __init__(self):
        super().__init__('get_map_state_client')
        self.client = self.create_client(GetMapState, 'get_map_state_service')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_available_tasks_client Service not available, waiting again...')
        self.get_logger().info('get_available_tasks_client Service is available, ready to send request!')

    def send_request(self):
        global available_tasks
        global obstacle_positions
        request = GetMapState.Request()  # Create a request object
        self.get_logger().info('Sending request for available tasks...')
        
        # Call the service and block until the response is complete
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)  # Wait until the service call is completed
        
        if self.future.done():
            try:
                response = self.future.result()  # Get the response from the service
                available_tasks = response.tasks
                obstacle_positions = response.obstacle_positions
                #self.get_logger().warn(f'Received tasks: {response.tasks}')
                #self.get_logger().warn(f'Received obstacle positions: {response.obstacle_positions}')

                #self.process_response(available_tasks, obstacle_positions) removed it for easier debugging
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

    def process_response(self, tasks, obstacles):
        if tasks:
            for idx, task in enumerate(tasks):
                """print(f"Task {idx}:")
                print(f"  ID: {task.id}")
                print(f"  Position: (x={task.position.x}, y={task.position.y}, theta={task.position.theta})")
                print(f"  Type: {task.type}")
                print(f"  Direction: {task.direction}")
                print(f"  Score: {task.score}")
                print(f"  Linear Velocity Choice: {task.nav_linear_velocity_choice}")
                print(f"  Angular Velocity Choice: {task.nav_angular_velocity_choice}")
                print(f"  Linear Acceleration Choice: {task.nav_linear_acceleration_choice}")
                print(f"  Angular Acceleration Choice: {task.nav_angular_acceleration_choice}")
                print("-" * 40)  # Separator for clarity"""
        else:
            print("No available tasks.")

        if obstacles:
            for idx, obstacle in enumerate(obstacles):
                """print(f"Obstacle {idx}:")
                print(f"  Position: (x={obstacle.x}, y={obstacle.y})")
                print("-" * 40)"""
        else:    
            print("No obstacles.")


class GetNextTaskPointService(Node):
    """
    here the flow of the commands is :
        1-the statemachine node sends a request to get the ext point to go to
        2-the strategy node gets the avaliable tasks from the static_task_generator node
        3-the strategy node decides which is the next task to go to based on the tasks_list[] 
        and the task_id that the state machine node sent(sends the next id and it's caracterisations) 
    """
    def __init__(self):    
        super().__init__('get_next_task_point_service')
        self.srv = self.create_service(GetNextTaskPoint, 'get_next_task_point_service', self.handle_get_next_task_point_callback)
        self.get_logger().info('Get next task point service is available now')

    def handle_get_next_task_point_callback(self, request, response):
        global available_tasks
        global strategy_instance
        global clients_instance
        global obstacle_positions
        global current_position
        global enemy_position
        global publishers_instance
        self.get_logger().info('Got a request about next task... Now sending a response')
        get_map_state()
        #next_task = get_next_task(request.task_id) #heer i changed to returning the next
        #available task in teh available_tasks list
        response = GetNextTaskPoint.Response()
        if request.task_id == -1000:
            print("HEEEEEERE")
            next_task = find_finished_task()
        else:    
            next_task = available_tasks[0]
        #print("ii m here noooooooooooooooow")
        next_task_position = Pose2D(x=next_task.position.x, y=next_task.position.y, theta=next_task.position.theta)
        enemy_position = Pose2D(x=enemy_position.x, y=enemy_position.y, theta=enemy_position.theta)
        #here the static task generator node should return the right obstacle positions based on the robot color  
        rrt_obstacles = convert_obstacles_to_rrt_argument(obstacle_positions)
        path = get_path_from_rrt(current_position, next_task_position, rrt_obstacles, enemy_position)
        if len(path) >=2:
            intermediate_point = path[1]
            print("intermediate_point", intermediate_point)
        rrt_failure_flag = path[0]
        if (rrt_failure_flag.x == -1 and rrt_failure_flag.y == -1 and rrt_failure_flag.theta == 0):
            self.get_logger().warn('Didnt find valid path for the first time')
            path = get_path_from_rrt(current_position, next_task_position, rrt_obstacles, enemy_position)
            if len(path) >=2:
                intermediate_point = path[1]
            rrt_failure_flag = path[0]
            if (rrt_failure_flag.x == -1 and rrt_failure_flag.y == -1 and rrt_failure_flag.theta == 0):
                print("IMPOSSIBLE TO REACH THE TARGET")
                response = convert_intermediate_point_to_response_msg(rrt_failure_flag, response, next_task)
                path_msg=convert_path_to_path_msg(path)
                publishers_instance.strategy_rrt_path_publisher.strategy_rrt_path_publisher.publish(path_msg)
                return response
                #here i should return the flag not the intermediate point i got an error
                ########HEEEEEEEERE I NEED TO EDGE CASE THIS ONE....I STILL DIDN'T FIGURE OUT HOW TO DO IT
                ####### i think i will be adding a request to change the task to go to if the next task is bui
                ####### building dropdown (to the static task generator)
            else:
                response = convert_intermediate_point_to_response_msg(intermediate_point, response, next_task)
                # Convert the task to the response message format
                """print(next_task.id)
                print(next_task.type)
                print("Response details:")
                print(f"task_id: {response.task_id}")
                print(f"nav_target_x: {response.nav_target_x}")
                print(f"nav_target_y: {response.nav_target_y}")
                print(f"nav_target_angle: {response.nav_target_angle}")
                print(f"task_type: {response.task_type}")
                print(f"nav_is_forward: {response.nav_is_forward}")
                print(f"score: {response.score}")
                print(f"nav_linear_velocity_choice: {response.nav_linear_velocity_choice}")
                print(f"nav_angular_velocity_choice: {response.nav_angular_velocity_choice}")
                print(f"nav_linear_acceleration_choice: {response.nav_linear_acceleration_choice}")
                print(f"nav_angular_acceleration_choice: {response.nav_angular_acceleration_choice}")"""
                # Return the response (no need to reassign the response variable)
                path_msg=convert_path_to_path_msg(path)
                publishers_instance.strategy_rrt_path_publisher.strategy_rrt_path_publisher.publish(path_msg)
                return response  
        else:
            #print("intermediate_point heeeeeeere", intermediate_point)
            response = convert_intermediate_point_to_response_msg(intermediate_point, response, next_task)
            path_msg=convert_path_to_path_msg(path)
            publishers_instance.strategy_rrt_path_publisher.strategy_rrt_path_publisher.publish(path_msg)
            # Convert the task to the response message format
            """print(next_task.id)
            print(next_task.type)
            print("Response details:")
            print(f"task_id: {response.task_id}")
            print(f"nav_target_x: {response.nav_target_x}")
            print(f"nav_target_y: {response.nav_target_y}")
            print(f"nav_target_angle: {response.nav_target_angle}")
            print(f"task_type: {response.task_type}")
            print(f"nav_is_forward: {response.nav_is_forward}")
            print(f"score: {response.score}")
            print(f"nav_linear_velocity_choice: {response.nav_linear_velocity_choice}")
            print(f"nav_angular_velocity_choice: {response.nav_angular_velocity_choice}")
            print(f"nav_linear_acceleration_choice: {response.nav_linear_acceleration_choice}")
            print(f"nav_angular_acceleration_choice: {response.nav_angular_acceleration_choice}")"""
            # Return the response (no need to reassign the response variable)
            return response
    
################### COMMUNICATION INIT FUNCTION #############

def communication_instances_init():
    global clients_instance
    global servers_instance
    global subscribers_instance
    global publishers_instance
    clients_instance = Clients()
    servers_instance = Services()
    subscribers_instance = Subscribers()
    publishers_instance = Publishers()
    
################## UTILITY FUNCTIONS ###################

def convert_path_to_path_msg(path: List[Pose2D]):
    path_msg = PathMsg()
    for point in path:
        pose = Pose2D(x=point.x, y=point.y, theta=point.theta)  # Directly using Pose2D attributes
        path_msg.path.append(pose)
    return path_msg

def get_path_from_rrt(start_pos, goal_pos, obstacles_position, enemy_robot_position):    
    global path_planner_instance
    path = path_planner_instance.get_path_rrt(start_pos, goal_pos, obstacles_position, enemy_robot_position)
    return path

def find_finished_task():
    global available_tasks
    for task in available_tasks:
        if task.type == TaskType.RETURN_TO_FINISH_SPOT.value:
            return task
        
    print("No finished task found.")    
    return None

def get_map_state():
        global clients_instance
        clients_instance.get_map_state_client.send_request()

def get_task_from_id(task_id):
    global available_tasks
    for task in available_tasks:
        if task.id == task_id:
            return task  # Return the task when found
    
def get_next_task(task_id): #I no longer use this function
    """
    Note:if (task_id == -100):the statemachine node will send -100
    when current_stask = None:we start the match now
    """
    global available_tasks
    global tasks_list
    if (task_id == -100):
        next_task_index = 0
    else:
        next_task_index = tasks_list.index(task_id) + 1
    next_task = get_task_from_id(tasks_list[next_task_index])
    return next_task

def convert_intermediate_point_to_response_msg(intermediate_point: Pose2D, response:GetNextTaskPoint.Response, next_task: TaskMsg):
    #if task is None:
    #   raise ValueError("The task provided is None.")
    response = GetNextTaskPoint.Response()
    if intemediate_point_is_task(intermediate_point, next_task) :
        response.task_id = next_task.id
        response.nav_target_x = next_task.position.x
        response.nav_target_y = next_task.position.y
        response.nav_target_angle = next_task.position.theta
        response.task_type = next_task.type
        response.nav_is_forward = bool(next_task.direction)
        response.score = next_task.score
        response.nav_linear_velocity_choice = next_task.nav_linear_velocity_choice
        response.nav_angular_velocity_choice = next_task.nav_angular_velocity_choice
        response.nav_linear_acceleration_choice = next_task.nav_linear_acceleration_choice
        response.nav_angular_acceleration_choice = next_task.nav_angular_acceleration_choice
        response.nav_is_with_platforms = next_task.nav_is_with_platforms
        return response
    else:
        response.task_id = next_task.id
        response.nav_target_x = intermediate_point.x
        response.nav_target_y = intermediate_point.y
        response.nav_target_angle = intermediate_point.theta
        response.task_type = TaskType.NORMAL_NAV.value
        response.nav_is_forward = True
        response.score = next_task.score
        response.nav_linear_velocity_choice = next_task.nav_linear_velocity_choice
        response.nav_angular_velocity_choice = next_task.nav_angular_velocity_choice
        response.nav_linear_acceleration_choice = next_task.nav_linear_acceleration_choice
        response.nav_angular_acceleration_choice = next_task.nav_angular_acceleration_choice
        response.nav_is_with_platforms = next_task.nav_is_with_platforms
        return response

def convert_obstacles_to_rrt_argument(obstacles):
    obstacles_position = []
    for obstacle in obstacles:
        obstacles_position.append(Pose2D(x=obstacle.x, y=obstacle.y, theta=0.0))
    return obstacles_position        

def intemediate_point_is_task(intermediate_point: Pose2D, next_task: TaskMsg):
    if intermediate_point.x == next_task.position.x and intermediate_point.y == next_task.position.y:
        return True
    return False

################## MAIN FUNCTION ######################

def main(args=None):
    global strategy_instance
    global clients_instance
    global subscribers_instance
    rclpy.init(args=args)
    communication_instances_init()
    # Initialize the global strategy instance
    strategy_instance = Strategy()
    # Create a MultiThreadedExecutor to manage both nodes concurrently
    executor = MultiThreadedExecutor()

    # Add both nodes to the executor
    executor.add_node(subscribers_instance.strategy_my_robot_color)
    executor.add_node(publishers_instance.strategy_rrt_path_publisher)
    executor.add_node(servers_instance.get_next_task_point_service)
    executor.add_node(clients_instance.get_map_state_client)
    executor.add_node(subscribers_instance.strategy_navigator_receive_position_subscriber)
    executor.add_node(subscribers_instance.strategy_enemy_robot_position_subscriber)
    executor.add_node(strategy_instance)

    try:
        # Spin all nodes in the executor (it will handle them in separate threads)
        executor.spin()
    except KeyboardInterrupt:
        pass  # Handle graceful shutdown on keyboard interrupt

    # Clean up
    strategy_instance.destroy_node()
    publishers_instance.strategy_rrt_path_publisher.destroy_node()
    servers_instance.get_next_task_point_service.destroy_node()
    clients_instance.get_map_state_client.destroy_node()
    subscribers_instance.strategy_enemy_robot_position_subscriber.destroy_node()
    subscribers_instance.strategy_navigator_receive_position_subscriber.destroy_node()
    subscribers_instance.strategy_my_robot_color.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()