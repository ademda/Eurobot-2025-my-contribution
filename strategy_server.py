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
from ros_custom_messages.msg import TaskMsg, ObstacleMsg
from ros_custom_messages.srv import GetNextTaskPoint
from strategy.static_task_generator import Task, SpeedChoice
############# GLOBAL VARIABLES ################

publishers_instance = None
subscribers_instance = None
clients_instance = None
servers_instance = None
strategy_instance = None

obstacle_positions = []
available_tasks = []
#tasks_list=[2, 5, 8, 9] #based on task id 2, 5, 8, 9

################# CLASSES ##################

class Strategy(Node):
    global clients_instance

    def __init__(self):
        super().__init__('strategy_server')

    def get_path_from_rrt(self):    
        print('got path from rrt')
        # NEED TO COMPLETE AFTER V0

    def get_enemy_position(self):
        print("Got enemy position")    
        # NEED TO COMPLETE AFTER V0

    def get_map_obstacles(self):
        print("got map obstacles")    

########## COMMUNICATION CLASSES ###########

class Services():
    def __init__(self):
        self.get_next_task_point_service= GetNextTaskPointService()


class Clients():
    def __init__(self):
        self.get_map_state_client = GetMapStateClient()


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
                self.get_logger().warn(f'Received tasks: {response.tasks}')
                self.get_logger().warn(f'Received obstacle positions: {response.obstacle_positions}')

                #self.process_response(available_tasks, obstacle_positions) removed it for easier debugging
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

    def process_response(self, tasks, obstacles):
        if tasks:
            for idx, task in enumerate(tasks):
                print(f"Task {idx}:")
                print(f"  ID: {task.id}")
                print(f"  Position: (x={task.position.x}, y={task.position.y}, theta={task.position.theta})")
                print(f"  Type: {task.type}")
                print(f"  Direction: {task.direction}")
                print(f"  Score: {task.score}")
                print(f"  Linear Velocity Choice: {task.nav_linear_velocity_choice}")
                print(f"  Angular Velocity Choice: {task.nav_angular_velocity_choice}")
                print(f"  Linear Acceleration Choice: {task.nav_linear_acceleration_choice}")
                print(f"  Angular Acceleration Choice: {task.nav_angular_acceleration_choice}")
                print("-" * 40)  # Separator for clarity
        else:
            print("No available tasks.")

        if obstacles:
            for idx, obstacle in enumerate(obstacles):
                print(f"Obstacle {idx}:")
                print(f"  Position: (x={obstacle.x}, y={obstacle.y})")
                print("-" * 40)
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
        self.get_logger().info('Got a request about next task... Now sending a response')
        get_map_state()
        #next_task = get_next_task(request.task_id) #heer i changed to returning the next
        #available task in teh available_tasks list
        response = GetNextTaskPoint.Response()
        # Convert the task to the response message format
        next_task = available_tasks[0]
        response = convert_task_to_response_msg(next_task, response)
        print(next_task.id)
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
        print(f"nav_angular_acceleration_choice: {response.nav_angular_acceleration_choice}")
        # Return the response (no need to reassign the response variable)
        return response
    
################### COMMUNICATION INIT FUNCTION #############

def communication_instances_init():
    global clients_instance
    global servers_instance
    clients_instance = Clients()
    servers_instance = Services()
    
################## UTILITY FUNCTIONS ###################

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

def convert_task_to_response_msg(task: TaskMsg, response:GetNextTaskPoint.Response):
    #if task is None:
    #   raise ValueError("The task provided is None.")
    response = GetNextTaskPoint.Response()
    response.task_id = task.id
    response.nav_target_x = task.position.x
    response.nav_target_y = task.position.y
    response.nav_target_angle = task.position.theta
    response.task_type = task.type
    response.nav_is_forward = bool(task.direction)
    response.score = task.score
    response.nav_linear_velocity_choice = task.nav_linear_velocity_choice
    response.nav_angular_velocity_choice = task.nav_angular_velocity_choice
    response.nav_linear_acceleration_choice = task.nav_linear_acceleration_choice
    response.nav_angular_acceleration_choice = task.nav_angular_acceleration_choice
    return response

################## MAIN FUNCTION ######################

def main(args=None):
    global strategy_instance
    global clients_instance
    print("AAAAAAAAAAAAAAAAAA")
    rclpy.init(args=args)
    communication_instances_init()

    # Initialize the global strategy instance
    strategy = Strategy()
    
    # Create a MultiThreadedExecutor to manage both nodes concurrently
    executor = MultiThreadedExecutor()

    # Add both nodes to the executor
    executor.add_node(servers_instance.get_next_task_point_service)
    executor.add_node(clients_instance.get_map_state_client)
    executor.add_node(strategy)

    try:
        # Spin all nodes in the executor (it will handle them in separate threads)
        executor.spin()
    except KeyboardInterrupt:
        pass  # Handle graceful shutdown on keyboard interrupt

    # Clean up
    strategy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()