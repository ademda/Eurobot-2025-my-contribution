"""
Author: <Adem Daly>

Description:
    This module is responsible for :
    -returning the next subtask navigation point if the next subtask is a navigation subtask
    -executing mechanism subtasks if the next subtask is a mechanism subtask

Note:
    -Maybe I need to add a publisher to inform the statemachine node that the mechanism subtask is done    
    -If i will add new subscrrbers or publishers i need to create a new node for each one of them
Classes:
    -GetNextSubTaskService: a service that will return the next subtask to the statemachine node
    -GetAvailableTasksClient: a client that will request the available tasks from the task generator node

Nodes:
    -get_next_subtask_service
    -get_available_tasks_client_for_sub_task

Functions:
    -command_mechanism: this function will command the mechanism using the mechanism module
    -communication_instances_init: initializes the clients and servers instances

"""
import rclpy
from mecanisme.mechanism_1 import FirstMechanism
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Pose2D
from typing import List
from typing import Tuple
from ros_custom_messages.srv import GetMapState, GetNextSubTaskPoint
from ros_custom_messages.msg import TaskMsg, SubTaskMsg, ObstacleMsg
from strategy.static_task_generator import Task, SubTaskType, MachanismSubTaskType, MechanismSubTask, NavSubTask, TaskType, Direction, TaskTypeScore, SpeedChoice
################################## GLOBAL VARIABLES #############################
mechanism_instance = None
servers_instance = None
clients_instance = None
available_tasks = []
###################################### SERVERS CLASSES ###################################

class Services:
    def __init__(self):
        self.get_next_subtask_service = GetNextSubTaskService()

class Clients:
    def __init__(self):
        self.get_available_tasks_client = GetMapStateClient()

class GetNextSubTaskService(Node):

    """
    this class is a service that will return the next subtask to the statemachine node
    request:
    int32 current_task_id
    int32 current_subtask_order
    ---
    response:
    int32 current_subtask_order
    float32 nav_target_x
    float32 nav_target_y
    float32 nav_target_angle
    bool nav_is_forward
    int8 nav_linear_velocity_choice
    int8 nav_angular_velocity_choice
    int8 nav_linear_acceleration_choice
    int8 nav_angular_acceleration_choice
    """
    def __init__(self):
        super().__init__('get_next_subtask_service')
        self.srv = self.create_service(GetNextSubTaskPoint, 'get_next_subtask', self.handle_get_next_subtask)
        self.get_logger().info('Created get_next_subtask Service')    

    def handle_get_next_subtask(self, request, response):
        global available_tasks
        global clients_instance
        clients_instance.get_available_tasks_client.send_request()
        self.get_logger().info('Got a request About NextSubtask... Now sending a response')    
        #print_all_subtasks()
        current_task = get_task_from_id(request.current_task_id)
        print(request.current_subtask_order)
        print(request.current_task_id)
        current_index = request.current_subtask_order

        if 0 <= current_index < len(current_task.subtask_list):
            next_subtask = current_task.subtask_list[current_index]
            print(f"Next subtask: {next_subtask}")
            if (next_subtask.type == SubTaskType.NAV_SUBTASK.value):
                response = convert_subtask_to_nav_message(next_subtask, response)
                self.get_logger().info('The next subtask is navigation')    
                self.get_logger().warn('Returning:{response}')  
                return response
            elif (next_subtask.type == SubTaskType.MECHANISM_SUBTASK.value):
                self.get_logger().info('The next subtask is of type mechanism. No movement.')
                command_mechanism(next_subtask)
                response.current_subtask_order = current_index
                self.get_logger().info(f'Returning special response for mechanism subtask: {response}')
                return response #empty navigation response will check it in samch
            else:
                self.get_logger().warn('Error detecting next_subtask type') 
                return response #empty response
        else:
            self.get_logger().warn("Next subtask does not exist in the list.")
            response.current_subtask_order = -1  # Indicating no more tasks
    
            # Optionally, leave the other fields as is (empty or default values).
            # Or, you can explicitly set them to default values, but this is not strictly necessary if the ROS 2 service defaults will be used.
            
            self.get_logger().info(f'Returning special response for end of tasks: {response}')
            return response
            
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
        request = GetMapState.Request()  # Create a request object
        self.get_logger().info('Sending request for available tasks...')
        
        # Call the service and block until the response is complete
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)  # Wait until the service call is completed
        
        if self.future.done():
            try:
                response = self.future.result()  # Get the response from the service
                available_tasks = response.tasks
                self.get_logger().warn(f'Received tasks: {response.tasks}')
                self.process_response(available_tasks)
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

    def process_response(self, tasks):
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
 

################# SUBTASK HANDLING FUNCTIONS ###############

def command_mechanism(next_subtask):
    """
    this function will command the mechanism using the mechanism module
    i think i need to add a service to tell the statemachine node that the mechanism subtask is done
    instead of executing a delay in the smach node
    or i can call the mechanism functions in the samch node itself(i don't recommend it)
    """
    global mechanism_instance
    if (next_subtask.mechanism_subtask_type == MachanismSubTaskType.GRAB_CANS):
        mechanism_instance.hold_cans()
    elif (next_subtask.mechanism_subtask_type == MachanismSubTaskType.DROP_CANS):
        mechanism_instance.place_first_cans()
    elif (next_subtask.mechanism_subtask_type == MachanismSubTaskType.ELEVATE_PLANKS_ABOVE):
        mechanism_instance.lift_planks()
    elif (next_subtask.mechanism_subtask_type == MachanismSubTaskType.ELEVATE_PLANKS_ALOT):
        mechanism_instance.lift_planks_alot()    
    elif (next_subtask.mechanism_subtask_type == MachanismSubTaskType.LOWER_PLANKS):
        mechanism_instance.place_first_plank_and_pick_last_cans()
    elif (next_subtask.mechanism_subtask_type == MachanismSubTaskType.BONDEROLE_SERVO_REALEASE):
        mechanism_instance.release_bonderole_servo()
    elif (next_subtask.mechanism_subtask_type == MachanismSubTaskType.BONDEROLE_SERVO_HOLD):
        mechanism_instance.hold_bonderole_servo()            
    else:
        print("Error: Mechanism subtask type not recognized")


################# COMMUNICATION INIT FUNCTION ###############

def communication_instances_init():
    global clients_instance
    global servers_instance
    clients_instance = Clients()
    servers_instance = Services()   

###################### UTILITY FUNCIONS ######################
def print_all_subtasks():
    global available_tasks
    if not available_tasks:
        print("No tasks available.")
        return
    
    for task_index, task in enumerate(available_tasks):
        print(f"Task {task_index + 1}:")
        print(f"  ID: {task.id}")
        print(f"  Position: (x={task.position.x}, y={task.position.y}, theta={task.position.theta})")
        print(f"  Type: {task.type}")
        print(f"  Direction: {task.direction}")
        print(f"  Score: {task.score}")
        print(f"  Linear Velocity Choice: {task.linear_velocity}")
        print(f"  Angular Velocity Choice: {task.angular_velocity}")
        print(f"  Linear Acceleration Choice: {task.linear_acceleration}")
        print(f"  Angular Acceleration Choice: {task.angular_acceleration}")
        
        if not task.subtask_list:
            print("  No subtasks available for this task.")
        else:
            print("  Subtasks:")
            for subtask_index, subtask in enumerate(task.subtask_list):
                print(f"    Subtask {subtask_index + 1}:")
                print(f"      Order: {subtask.order}")
                print(f"      Type: {subtask.type}")
                print(f"      Position: (x={subtask.position.x}, y={subtask.position.y}, theta={subtask.position.theta})")
                print(f"      Subtask Type: {subtask.subtask_type}")
                print(f"      Is Forward: {'True' if subtask.nav_is_forward else 'False'}")
                print(f"      Linear Velocity Choice: {subtask.nav_linear_velocity_choice}")
                print(f"      Angular Velocity Choice: {subtask.nav_angular_velocity_choice}")
                print(f"      Linear Acceleration Choice: {subtask.nav_linear_acceleration_choice}")
                print(f"      Angular Acceleration Choice: {subtask.nav_angular_acceleration_choice}")
        print("-" * 50)  # Separator for clarity

def get_task_from_id(task_id):
    global available_tasks
    for task in available_tasks:
        if task.id == task_id:
            print("SHHHHHHHHHHHHHHHHIT")
            return task  # Return the task when found
    
def convert_subtask_to_nav_message(subtask: SubTaskMsg, response: GetNextSubTaskPoint.Response):
        response.current_subtask_order = subtask.order   
        response.nav_target_x = subtask.position.x
        response.nav_target_y = subtask.position.y
        response.nav_target_angle = subtask.position.theta
        response.nav_is_forward = subtask.nav_is_forward
        response.nav_linear_velocity_choice = subtask.nav_linear_velocity_choice
        response.nav_angular_velocity_choice = subtask.nav_angular_velocity_choice
        response.nav_linear_acceleration_choice = subtask.nav_linear_acceleration_choice
        response.nav_linear_acceleration_choice = subtask.nav_angular_acceleration_choice
        return response

def main(args=None):
    global clients_instance
    global servers_instance
    global mechanism_instance
    rclpy.init(args=args)
    communication_instances_init()
    mechanism_instance = FirstMechanism()
    rclpy.spin(servers_instance.get_next_subtask_service)
    #rclpy.spin(clients_instance.get_available_tasks_client) here i don't need to spib the client 
    #because it is invoked by the get_next_subtask_service node
    clients_instance.get_available_tasks_client.destroy_node()
    servers_instance.get_next_subtask_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()