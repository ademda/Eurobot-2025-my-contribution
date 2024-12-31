import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_custom_messages.srv import GetAvailableTasks
from enum import Enum
from ros_custom_messages.msg import TaskMsg
from ros_custom_messages.srv import GetNextTaskPoint
from strategy.static_task_generator import Task, SpeedChoice
############# GLOBAL VARIABLES ################

publishers_instance = None
subscribers_instance = None
clients_instance = None
servers_instance = None
strategy_instance = None

available_tasks = []
tasks_list=[2, 5, 8, 9] #based on task id

################# CLASSES ##################
def get_available_tasks():
        global clients_instance
        clients_instance.get_available_tasks_client.send_request()

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
        self.get_available_tasks_client = GetAvailableTasksClient()


class GetAvailableTasksClient(Node):
    def __init__(self):
        super().__init__('get_available_tasks_client')
        self.client = self.create_client(GetAvailableTasks, 'get_available_tasks')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get_available_tasks_client Service not available, waiting again...')
        self.get_logger().info('get_available_tasks_client Service is available, ready to send request!')

    def send_request(self):
        global available_tasks
        request = GetAvailableTasks.Request()  # Create a request object
        self.get_logger().info('Sending request for available tasks...')
        
        # Call the service and block until the response is complete
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)  # Wait until the service call is completed
        
        if self.future.done():
            try:
                response = self.future.result()  # Get the response from the service
                available_tasks = response.tasks
                self.get_logger().warn(f'Received tasks: {response.tasks}')
                #self.process_response(available_tasks)
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

    def process_response(self, tasks):
        if tasks:
            for idx, task in enumerate(tasks):
                print(f"Task {idx + 1}:")
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

class GetNextTaskPointService(Node):
    
    def __init__(self):    
        super().__init__('get_next_task_point_service')
        self.srv = self.create_service(GetNextTaskPoint, 'get_next_task_point_service', self.handle_get_next_task_point_callback)
        self.get_logger().info('Get next task point service is available now')

    def handle_get_next_task_point_callback(self, request, response):
        global available_tasks
        global strategy_instance
        global clients_instance
        self.get_logger().info('Got a request about next task... Now sending a response')
        get_available_tasks()
        next_task = get_next_task(request.task_id)
        response = GetNextTaskPoint.Response()
        # Convert the task to the response message format
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

def get_task_from_id(task_id):
    global available_tasks
    for task in available_tasks:
        if task.id == task_id:
            return task  # Return the task when found
    
def get_next_task(task_id):
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
    rclpy.init(args=args)
    communication_instances_init()

    # Initialize the global strategy instance
    strategy = Strategy()

    # Create a MultiThreadedExecutor to manage both nodes concurrently
    executor = MultiThreadedExecutor()

    # Add both nodes to the executor
    executor.add_node(servers_instance.get_next_task_point_service)
    executor.add_node(clients_instance.get_available_tasks_client)
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