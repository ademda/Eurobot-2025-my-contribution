import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Pose2D, PoseArray, Pose, Quaternion, Point
from typing import List
from typing import Tuple
from ros_custom_messages.srv import GetAvailableTasks
from ros_custom_messages.msg import TaskMsg, SubTaskMsg

from abc import ABC, abstractmethod 


######################################## GLOBAL VARIABLES #########################

servers_instance = None

########################################### CLASSES ###############################


class TaskType(Enum):
    NORMAL_NAV = 0
    BONDEROLE_POSE = 1
    BUILDING_PICKUP = 2
    BUILDING_DROPDOWN = 3
    RETURN_TO_FINISH_SPOT = 4

class SpeedChoice (Enum):
    SLOW = 0
    NORMAL = 1
    FAST = 2
    VERY_FAST = 3

class TaskTypeScore(Enum):
    BONDEROLE_POSE = 10  
    BUILDING_PICKUP = 0  
    BUILDING_DROPDOWN = 12  
    RETURN_TO_FINISH_SPOT = 10  


class SubTaskType(Enum):
    NAV_SUBTASK = 0
    MECHANISM_SUBTASK = 1


class MachanismSubTaskType(Enum):
    GRAB_CANS = 0
    DROP_CANS = 1
    ELEVATE_OUTSIDE_CANS = 2
    ELEVATE_PLANKS_ABIT = 3
    LOWER_PLANKS_ABIT = 4
    LOWER_PLANKS = 5
    ELEVATE_PLANKS_ABOVE = 6
    HOLD_UPPER_PLANK = 7
    RELEASE_UPPER_PLANK = 8


class ObstacleType(Enum):
    BUILDING = 0
    ENEMY = 1
    CONSTRUCTION_ZONE = 2

class Direction(Enum):
    BACKWARD = 0
    FORWARD = 1

class Task:
    def __init__(self, id: int, position: Pose2D, type: TaskType, direction: Direction ,
                 score: TaskTypeScore,linear_velocity: SpeedChoice,subtask_list: List['SubTask'],
                 angular_velocity: SpeedChoice, linear_acceleration: int, angular_acceleration: int):
        self.id = id
        self.position = position
        self.type = type
        self.direction = direction
        self.score = score
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration
        self.angular_acceleration = angular_acceleration
        self.subtask_list = subtask_list
        


class SubTask(ABC): #ABSTRACT CLASS
    def __init__(self, order: int, type: int):
        self.order = order
        self.type = type


class NavSubTask(SubTask):
    def __init__(self, order: int, type: int, position: Pose2D, direction:Direction,
                 linear_velocity: SpeedChoice,angular_velocity: SpeedChoice,):
        super().__init__(order, type)
        self.position = position
        self.direction = direction
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        


class MechanismSubTask(SubTask):
    def __init__(self, order: int, type: int, mechanism_subtask_type: MachanismSubTaskType):
        super().__init__(order, type)
        self.mechanism_subtask_type = mechanism_subtask_type


class DoubleBuildingDropDownArea:
    def __init__(self, start_point: Tuple[float, float], end_point: Tuple[float, float], 
                 first_intermediate_point: Pose2D, second_intermediate_point: Pose2D):
        self.start_point = start_point
        self.end_point = end_point
        self.first_intermediate_point = first_intermediate_point
        self.second_intermediate_point = second_intermediate_point


class SingleBuildingDropDownArea:
    def __init__(self, start_point: Tuple[float, float], end_point: Tuple[float, float], 
                 intermediate_point: Pose2D):
        self.start_point = start_point
        self.end_point = end_point
        self.intermediate_point = intermediate_point


class BuildingPickUpArea:
    def __init__(self, position: Pose2D):
        self.position = position


class Obstacle:
    def __init__(self, position: Tuple[float, float], obstacle_type: ObstacleType):
        self.position = position
        self.type = obstacle_type
        
########################## SUB_TASKS LISTS ##################

subtasks_id5 = [NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1000.0, y=1500.0, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW),
                NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2000.0, y=1500.0, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW), ]

subtasks_id8 = [MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_OUTSIDE_CANS),
                NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=5000.0, y=5000.0, theta=30.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW),
                ]

####################################################### TASKS LISTS #############################################################

tasks = [Task(id=0, position=Pose2D(x=1234.0, y=123.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),  
    Task(id=1, position=Pose2D(x=1069.0, y=707.7, theta=float(90.0)), type=TaskType.NORMAL_NAV,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=2, position=Pose2D(x=788.0, y=442.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=3, position=Pose2D(x=265.0, y=392.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=4, position=Pose2D(x=265.0, y=1307.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=5, position=Pose2D(x=1853.0, y=731.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=subtasks_id5),
    Task(id=6, position=Pose2D(x=2219.0, y=515.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=7, position=Pose2D(x=2669.0, y=396.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=8, position=Pose2D(x=2669.0, y=1323.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=subtasks_id8),
    Task(id=9, position=Pose2D(x=2669.0, y=1323.0, theta=float(0.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.BACKWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),]
common_tasks = [
    Task(id=0, position=Pose2D(x=1234.0, y=123.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),  
    Task(id=1, position=Pose2D(x=1069.0, y=707.7, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=2, position=Pose2D(x=788.0, y=442.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=3, position=Pose2D(x=265.0, y=392.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=4, position=Pose2D(x=265.0, y=1307.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=5, position=Pose2D(x=1853.0, y=731.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=6, position=Pose2D(x=2219.0, y=515.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=7, position=Pose2D(x=2669.0, y=396.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
    Task(id=8, position=Pose2D(x=2669.0, y=1323.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, subtask_list=[]),
]

blue_tasks = [
    Task(id=9, position=Pose2D(x=819.0, y=1492.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0,subtask_list=[]),
    Task(id=11, position=Pose2D(x=2219.0, y=381.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_DROPDOWN,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0,subtask_list=[]),
    Task(id=12, position=Pose2D(x=227.0, y=343.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_DROPDOWN,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0,subtask_list=[]),
    Task(id=13, position=Pose2D(x=654.0, y=873.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_DROPDOWN,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0,subtask_list=[]),
    Task(id=14, position=Pose2D(x=1784.0, y=630.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_DROPDOWN,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0,subtask_list=[]),
]

yellow_tasks = []

building_dropDown_areas = []

building_pickUp_areas = []




########################## SERVERS CLASSES ##################

class Services:
    def __init__(self):
        self.get_available_tasks_service = GetAvailableTasksService()
        

class GetAvailableTasksService(Node):
    global tasks

    def __init__(self):
        super().__init__('get_available_tasks_service')
        self.srv = self.create_service(GetAvailableTasks, 'get_available_tasks', self.handle_get_available_tasks)
        self.get_logger().warn('Created get_available_tasks Service')

    def handle_get_available_tasks(self, request, response):
        self.get_logger().info('Got a request... Now sending a response')

        if not tasks:
            self.get_logger().warn('No tasks available')

        # Convert your Task objects to TaskMsg objects
        response.tasks = []
        for task in tasks:
            task_msg = TaskMsg()  # Create a new Task message
            task_msg.id = task.id
            task_msg.position = task.position  # Ensure this is compatible with the Pose2D message
            task_msg.type = task.type.value  # Use the value of the Enum for type
            task_msg.direction = task.direction.value
            task_msg.score = task.score.value
            task_msg.nav_linear_velocity_choice = task.linear_velocity.value
            task_msg.nav_angular_velocity_choice = task.angular_velocity.value
            task_msg.nav_linear_acceleration_choice = task.linear_acceleration
            task_msg.nav_angular_acceleration_choice = task.angular_acceleration
            task_msg.subtask_list = convert_to_subtask_list_msg(task.subtask_list)
            # Append the Task message to the response
            response.tasks.append(task_msg)
        position = task_msg.position
        task_msg_dict = {
            'id': task_msg.id,
            'position': {
                'x': position.x,
                'y': position.y,
                'theta': position.theta
            },
            'type': task_msg.type,
            'direction': task_msg.direction,
            'score': task_msg.score,
            'nav_linear_velocity_choice': task_msg.nav_linear_velocity_choice,
            'nav_angular_velocity_choice': task_msg.nav_angular_velocity_choice,
            'nav_linear_acceleration_choice': task_msg.nav_linear_acceleration_choice,
            'nav_angular_acceleration_choice': task_msg.nav_angular_acceleration_choice,
            'subtask_list': task_msg.subtask_list
        }
            
        # Log the task message as a dictionary
        self.get_logger().info(f'Sent task message: {task_msg_dict}')
        return response

################### COMMUNICATION INIT FUNCTION #############

def communication_instances_init():

    global servers_instance
    servers_instance = Services()
    

############################# FUNCTIONS #####################
def convert_to_subtask_list_msg (subtask_list: List['SubTask']):
    subtask_list_msg = []

    for subtask in subtask_list:
        subtask_msg = SubTaskMsg()  # Create a new SubTaskMsg object
        if subtask.type == SubTaskType.NAV_SUBTASK:
            subtask_msg.order = subtask.order
            subtask_msg.type = subtask.type.value  # Assuming type matches with SubTaskType enum
            subtask_msg.position = subtask.position  # Directly assigning the Pose2D object
            subtask_msg.subtask_type = subtask.type.value  # You can adjust if needed to match SubTaskType
            subtask_msg.nav_is_forward = bool(subtask.direction) 
            subtask_msg.nav_linear_velocity_choice = subtask.linear_velocity.value
            subtask_msg.nav_angular_velocity_choice = subtask.angular_velocity.value
            subtask_msg.nav_linear_acceleration_choice = 0  # Assuming a default value
            subtask_msg.nav_angular_acceleration_choice = 0  # Assuming a default value
        else:    
            subtask_msg.order = subtask.order
            subtask_msg.type = subtask.type.value  # Assuming type matches with SubTaskType enum
            subtask_msg.position = Pose2D(x=0.0, y=0.0, theta=0.0) # Directly assigning the Pose2D object
            subtask_msg.subtask_type = subtask.type.value  # You can adjust if needed to match SubTaskType
            subtask_msg.nav_is_forward = True 
            subtask_msg.nav_linear_velocity_choice = 0
            subtask_msg.nav_angular_velocity_choice = 0
            subtask_msg.nav_linear_acceleration_choice = 0  # Assuming a default value
            subtask_msg.nav_angular_acceleration_choice = 0  # Assuming a default value
        subtask_list_msg.append(subtask_msg)
    return subtask_list_msg    
        
"""
def update_availalable_tasks():
    global tasks
    minimal_error_distance = 125
    for task in tasks:
          if(( current_position.x - minimal_error_distance <= task.position.position.x <= current_position.x + minimal_error_distance ) and
             ( current_position.y - minimal_error_distance <= task.position.position.y <= current_position.y + minimal_error_distance ) and 
             ( current_state == task.type.value )):
               
               tasks.remove(task)

    # I need to add enemy position area          
    #  
"""

def main(args=None):
    rclpy.init(args=args)
    communication_instances_init()
    rclpy.spin(servers_instance.get_available_tasks_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()    