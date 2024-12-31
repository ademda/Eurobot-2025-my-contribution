import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Pose2D
from typing import List
from typing import Tuple
from ros_custom_messages.srv import GetAvailableTasks, GetNextSubTaskPoint
from ros_custom_messages.msg import TaskMsg
from strategy.static_task_generator import Task, MachanismSubTaskType, MechanismSubTask, NavSubTask

################################## GLOBAL VARIABLES #############################

servers_instance = None
clients_instance = None
available_tasks = []

###################################### SERVERS CLASSES ###################################

class Services:
    def __init__(self):
        self.get_available_tasks_service = GetNextSubTaskService()

class Clients:
    def __init__(self):
        self.get_available_tasks_client = GetAvailableTasksClient()

class GetNextSubTaskService(Node):
    global available_tasks

    def __init__(self):
        super().__init__('get_next_subtask_service')
        self.srv = self.create_service(GetNextSubTaskPoint, 'get_next_subtask', self.handle_get_next_subtask)
        self.get_logger().info('Created get_next_subtask Service')    
    def handle_get_next_subtask(self, request, response):
        available_tasks
        self.get_logger().info('Got a request About NextSubtask... Now sending a response')    
        current_task = get_task_from_id(request.current_task_id)
        next_index = request.current_subtask_order + 1

        if 0 <= next_index < len(current_task.subtask_list):
            next_subtask = current_task.subtask_list[next_index]
            print(f"Next subtask: {next_subtask}")
            if isinstance(next_subtask, NavSubTask):
                response = convert_subtask_to_nav_message(next_subtask, response)
                self.get_logger().info('The next subtask is navigation')    
                self.get_logger().warn('Returning:{response}')  
                return response
            elif isinstance(next_subtask, MechanismSubTask):
                self.get_logger().info('The next subtask is of type mechanism. No movement.')
                command_mechanism(next_subtask)
                response.current_subtask_order = next_index
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
            
class GetAvailableTasksClient(Node):
    global available_tasks

    def __init__(self):
        super().__init__('get_available_tasks_client')
        self.client = self.create_client(GetAvailableTasks, 'get_available_tasks')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service is available, ready to send request!')
        self.received_tasks = None

    def send_request(self):
        request = GetAvailableTasks.Request()  # Create a request object
        self.get_logger().info('Sending request for available tasks...')
        
        # Call the service asynchronously
        future = self.client.call_async(request)
        self.get_logger().info('Called Async')
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        global available_tasks
        self.get_logger().info('Response callback triggered!')
        try:
            response = future.result()  # Get the response from the service
            self.get_logger().info(f'Received tasks: {response.tasks}')
            available_tasks = response.tasks
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')        

################# SUBTASK HANDLING FUNCTIONS ###############

def command_mechanism(next_subtask):
    return True

################# COMMUNICATION INIT FUNCTION ###############

def communication_instances_init():
    global clients_instance
    global servers_instance
    clients_instance = Clients()
    servers_instance = Services()   

###################### UTILITY FUNCIONS ######################

def get_task_from_id(task_id):
    global available_tasks
    for task in available_tasks:
        if task.id == task_id:
            return task  # Return the task when found
    
def convert_subtask_to_nav_message(subtask: NavSubTask, response: GetNextSubTaskPoint.Response):
        response.current_subtask_order = subtask.order   
        response.nav_target_x = subtask.position.x
        response.nav_target_y = subtask.position.y
        response.nav_target_angle = subtask.position.theta
        response.nav_is_forward = subtask.direction
        response.nav_linear_velocity_choice = subtask.linear_velocity
        response.nav_angular_velocity_choice = subtask.angular_velocity

        return response

def main(args=None):
    global clients_instance
    global servers_instance
    rclpy.init(args=args)
    communication_instances_init()
    rclpy.spin(servers_instance.get_available_tasks_service)
    #rclpy.spin(clients_instance.get_available_tasks_client)
    clients_instance.get_available_tasks_client.destroy_node()
    servers_instance.get_available_tasks_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()