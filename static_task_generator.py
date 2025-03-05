"""
Author: <Adem Daly>

Description:
    This module is responsible for : 
        -Generating available tasks for the robot to perform
        -Generating Obstacle positions

Note:
    - The available tasks are generated based on the the enemy robot position and a request from the stateamchine to remove a stask 
    my robot completetd
    - The obstacle positions are generated based on the enemy robot position and the current robot position
    - if the enmy crosses a task of type pick up :i replace it wil the next pickup task in teh available tasks list and remvoe the last dropdown task in the list
    
Enums:
    - TaskType(Enum): Enum class for the type of the task
    - SpeedChoice(Enum): Enum class for the speed choice of the task
    - TaskTypeScore(Enum): Enum class for the score of the task
    - SubTaskType(Enum): Enum class for the type of the subtask
    - MachanismSubTaskType(Enum): Enum class for the type of the mechanism subtask
    - ObstacleType(Enum): Enum class for the type of the obstacle
    - Direction(Enum): Enum class for the direction of the robot when performing the task or in normal navigation

Classes:
    - Task: Class for the task
    - SubTask(ABC): Abstract class for the subtask
    - NavSubTask(SubTask): Class for the navigation subtask
    - MechanismSubTask(SubTask): Class for the mechanism subtask
    - DoubleBuildingDropDownArea: Class for the drop down area that can support two buildings /still didn't use it
    - SingleBuildingDropDownArea: Class for the drop down area that can support one building /still didn't use it
    - BuildingPickUpArea: Class for the pick up area of the building /still didn't use it
    - Obstacle: Class for the obstacle /still didn't use it
    - Services: Class for the services:
        - GetAvailableTasksService: Class for the get available tasks service 
        it has connections with the nodes:subtask_generator,strategy_server
        - GetMapStateService: Class for the get map state service    
        -DeleteTaskService: Class for the delete task service
    - Subscribers: Class for the subscribers:
        - Enemy_Robot_Position_Subscriber: Class for the enemy robot position subscriber
        - My_Robot_Position_Subscriber: Class for the my robot position subscriber
Functions:
    - convert_to_subtask_list_msg(subtask_list: List['SubTask']) -> List[SubTaskMsg]: Function to convert the subtask list 
    to a list of subtask messages
    - communication_instances_init(): Function to initialize the communication instances


"""

import rclpy
import logging
import threading
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Pose2D, PoseArray, Pose, Quaternion, Point
from typing import List
from typing import Tuple
from ros_custom_messages.srv import GetMapState, DeleteTask
from ros_custom_messages.msg import TaskMsg, SubTaskMsg, ObstacleMsg, NavigatorNavigationMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy
from abc import ABC, abstractmethod 
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty
######################################## GLOBAL VARIABLES #########################

servers_instance = None
subscribers_instance = None
enemy_position = Pose2D(x=0.0, y=0.0, theta=0.0)
current_position = Pose2D(x=1200.0, y=275.5, theta=90.0) #initialise the current position to not get errors
robot_color = None
robot_current_state = 0

########################################### CLASSES ###############################

class Point2D:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

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
    ELEVATE_PLANKS_ALOT = 7
    HOLD_UPPER_PLANK = 8
    RELEASE_UPPER_PLANK = 9
    BONDEROLE_SERVO_REALEASE = 10
    BONDEROLE_SERVO_HOLD = 11


class ObstacleType(Enum):
    BUILDING = 0
    ENEMY = 1
    CONSTRUCTION_ZONE = 2
    FIXED_OBSTACLE = 3


class Direction(Enum):
    BACKWARD = 0
    FORWARD = 1

class IS_WITH_PLATFORMS(Enum):
    NO = 0
    YES = 1
class Task:
    def __init__(self, id: int, position: Pose2D, type: TaskType, direction: Direction ,
                 score: TaskTypeScore, linear_velocity: SpeedChoice, angular_velocity: SpeedChoice,
                 linear_acceleration: int, angular_acceleration: int, is_with_platforms: IS_WITH_PLATFORMS,
                 subtask_list: List['SubTask']):
        self.id = id
        self.position = position
        self.type = type
        self.direction = direction
        self.score = score
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration
        self.angular_acceleration = angular_acceleration
        self.is_with_platforms = is_with_platforms
        self.subtask_list = subtask_list


class SubTask(ABC): #ABSTRACT CLASS
    def __init__(self, order: int, type: int):
        self.order = order
        self.type = type


class NavSubTask(SubTask):
    def __init__(self, order: int, type: int, position: Pose2D, direction:Direction,
                 linear_velocity: SpeedChoice,angular_velocity: SpeedChoice, 
                 linear_acceleration: int, angular_acceleration: int, is_with_platforms: IS_WITH_PLATFORMS):
        
        super().__init__(order, type)
        self.position = position
        self.direction = direction
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration
        self.angular_acceleration = angular_acceleration
        self.is_with_platforms = is_with_platforms

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
        

class MapStateUpdater(Node):
    def __init__(self):
        super().__init__('map_state_updater')
        self.get_logger().warn('Created map_state_updater node')
        self.create_timer(0.01, self.update_available_tasks)  # Call every 0.01 seconds
        self.create_timer(0.01, self.update_obstacles_positions)  # Call every 0.01 seconds

    def update_available_tasks(self):
        global available_tasks
        global current_position
        global enemy_position
        global true_tasks
        minimal_error_distance = 75
        minimal_error_distance_enemy = 100 
        max_distance_between_obstacles = 600
        # here used the minimal error distance of the last year maybe needs to check it
        #I changed it a bit and i will test it
        index = 0
        print("current postion from update available task:",current_position.x,current_position.y)
        while index < len(true_tasks):
            task = true_tasks[index]
            if ((enemy_position.x - minimal_error_distance_enemy <= task.position.x <= enemy_position.x + minimal_error_distance_enemy) and
                (enemy_position.y - minimal_error_distance_enemy <= task.position.y <= enemy_position.y + minimal_error_distance_enemy) and
                task.type == TaskType.BUILDING_PICKUP):
                
                self.get_logger().warn(f'Replacing task at position {task.position.x} and {task.position.y} based on ENEMY POSITION')
                print("index in the true tasks list",index)
                # Start cascading replacements from the current index
                current_index = find_equivalent_task(index)
                true_tasks.pop(index)
                while current_index < len(available_tasks) - 1:
                    # Find the next BUILDING_PICKUP task for replacement
                    next_task_index = next((i for i in range(current_index + 1, len(available_tasks)) 
                                            if (available_tasks[i].type == TaskType.BUILDING_PICKUP and  available_tasks[i].id != 1)), None)
                    
                    if next_task_index is None:
                        self.get_logger().warn('No more BUILDING_PICKUP tasks to cascade replacements')
                        break

                    # Replace the current task with the next BUILDING_PICKUP task
                    available_tasks[current_index] = available_tasks[next_task_index]
                    self.get_logger().warn(f'Replaced task at index {current_index} with task at position {available_tasks[next_task_index].position}')

                    # Move to the next task in the chain
                    current_index = next_task_index

                # Remove the last replaced task from the list to avoid duplication
                last_task_index = next((i for i in range(current_index, len(available_tasks)) 
                                        if available_tasks[i].type == TaskType.BUILDING_PICKUP), None)
                print("last task index",last_task_index)
                if last_task_index:
                    self.get_logger().warn(f'Removing final replaced task at index {last_task_index} from available_tasks')
                    available_tasks.pop(last_task_index)

                next_dropoff_index = next((i for i in range(last_task_index, len(available_tasks)) 
                               if available_tasks[i].type == TaskType.BUILDING_DROPDOWN), None)
    
                if next_dropoff_index is not None:
                    self.get_logger().warn(f'Removing next BUILDING_DROPDOWN task at index {next_dropoff_index} from available_tasks')
                    true_task_next_dropoff_index=find_equivalent_true_task(next_dropoff_index)
                    available_tasks.pop(next_dropoff_index)    
                    true_tasks.pop(true_task_next_dropoff_index)
            index += 1


    def update_obstacles_positions(self):
        global obstacle_positions
        global current_position
        global enemy_position
        global available_tasks
        minimal_error_distance = 75
        minimal_error_distance_enemy = 100
        max_distance_between_obstacles = 450
        for obstacle in obstacle_positions:
            if((current_position.x - minimal_error_distance <= obstacle.position.x <= current_position.x + minimal_error_distance) and
            (current_position.y - minimal_error_distance <= obstacle.position.y <= current_position.y + minimal_error_distance) and
            obstacle.type.value == ObstacleType.BUILDING.value):
                
                print('Removing obstacle at position based on MY POSITION: ', obstacle.position.x)
                print('Removing obstacle at position based on MY POSITION: ', obstacle.position.y)
                for i in range(len(obstacle_positions) - 1, -1, -1):
                    another_obstacle = obstacle_positions[i]
                    if (calculate_distance(obstacle, another_obstacle) <= max_distance_between_obstacles and 
                        another_obstacle.type.value == ObstacleType.BUILDING.value and 
                        another_obstacle != obstacle):
                        print("another obstacle", another_obstacle.position.x)
                        print("obstacle", obstacle.position.y)
                        obstacle_positions.pop(i)
                        i = i-1
                obstacle_positions.remove(obstacle)

        for obstacle in obstacle_positions:
            if((enemy_position.x - minimal_error_distance_enemy <= obstacle.position.x <= enemy_position.x + minimal_error_distance_enemy) and
            (enemy_position.y - minimal_error_distance_enemy <= obstacle.position.y <= enemy_position.y+ minimal_error_distance_enemy) and 
            obstacle.type.value == ObstacleType.BUILDING.value):
                
                print('Removing obstacle at position based on ENEMY POSITION: ', obstacle.position.x)
                print('Removing obstacle at position based on ENEMY POSITION: ', obstacle.position.y)
                for i in range(len(obstacle_positions) - 1, -1, -1):
                    another_obstacle = obstacle_positions[i]
                    if (calculate_distance(obstacle, another_obstacle) <= max_distance_between_obstacles and 
                        another_obstacle.type.value == ObstacleType.BUILDING.value and 
                        another_obstacle != obstacle):
                        print("another obstacle", another_obstacle.position.x)
                        print("obstacle", obstacle.position.y)
                        obstacle_positions.pop(i)
                        i = i-1
                obstacle_positions.remove(obstacle)
        
        #here maybe i need to add that if ly robot dopped a building i should add that building as an obstacle
        #to do that i need to get from the statemachine that the robot is now during the building drop down task 
        #or from the strategy get the next_task and check if it's a building drop down task (and if my position 
        #is near the obstacle i add the obstacle
        #(MAYBE I DON'T) to do taht because construction sites are near the wall so i will not cross them

    def get_next_task(self, task):
        global available_tasks
        try:
            # Find the index of the current task in available_tasks
            index = available_tasks.index(task)
            # Return the next task if it exists
            if index + 1 < len(available_tasks):
                return available_tasks[index + 1]
            else:
                self._logger.error('No next task exists')
                return None  # No next task exists
        except ValueError:
            return None  # Task not found in available_tasks


########################## SUB_TASKS LISTS ##################

###########################################################
#subtasks for the yellow robot
subtasks_id0 = [
    #MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.BONDEROLE_SERVO_HOLD),
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.BONDEROLE_SERVO_REALEASE),
    NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1196.0, y=500.0, theta=655.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),#y was 200
]   

#blacet subtasks id1 mrigla
subtasks_id1 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ALOT),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    #NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=744.0, y=310.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.SLOW, is_with_platforms=IS_WITH_PLATFORMS.NO),#gadamt 4 cm fl y:na7it le subtask 
    #5atr sartli el robot mgadem chwya wbech y5dem task id2 trasilou ywa5r bl forward
]#this configuration is good for yellow robot not for blue robot....when buildibgn blue robot i need to create special subtask for him

subtasks_id2 = [
    #NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=744.0, y=258.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),#na9ast 4 cm 
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=744.0,y=500.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),#y was 310
]
#blacet subtasks id3 mrigla
subtasks_id3 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1100.0, y=823.0, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),

]

subtasks_id4 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=315.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),#y was 350
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=550.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),#y was 550
]

subtasks_id5 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=180.5, y=400.0, theta=180.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=360.0, y=400.0, theta=180.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
]

subtasks_id6 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=470.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),#y was 458
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=635.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]    

#subtask id7 mrigl

subtasks_id7 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=180.5, y=1325.0, theta=180.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),#no and yes are reversed now
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=360.0, y=1325.0, theta=180.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
]

subtasks_id8 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2720.0, y=884.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO), 
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2585.0, y=884.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id9 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),#maybe these lower planks repeated i don't need them
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2819.5, y=400.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2640.0, y=400.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
]

subtasks_id10 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2769.0, y=260.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2769.0, y=330.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id11 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2819.5, y=1325.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2640.0, y=1325.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
]

subtasks_id12 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2542.0, y=884.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2365.0, y=884.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id14 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ALOT),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
]#this configuration is good for yellow robot not for blue robot....when buildibgn blue robot i need to create special subtask for him    

subtasks_id15 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1900.0, y=823.0, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
]

subtasks_id16 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=831.0, y=1555.5, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=831.0, y=1385.0, theta=90.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

###########################################################

subtask_for_test = [NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1000.0, y=1500.0, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.SLOW, is_with_platforms=IS_WITH_PLATFORMS.NO)]
####################################################### TASKS LISTS #############################################################
#yellow
"""available_tasks = [
    Task(id=0, position=Pose2D(x=1196.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  
    Task(id=1, position=Pose2D(x=770.0, y=571.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),#blacet  id1 mrigla
    Task(id=2, position=Pose2D(x=770.0, y=258.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id2), #stwit el x m3 id1y=628.0was
    Task(id=3, position=Pose2D(x=1100.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),#blacet subtasks id3 mrigla
    Task(id=4, position=Pose2D(x=1226.0, y=635.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id4),#y was 415 changed ebcause no rrt
    Task(id=5, position=Pose2D(x=360.0, y=400.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),#mrigla
    Task(id=6, position=Pose2D(x=1226.0, y=635.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id6),
    Task(id=7, position=Pose2D(x=360.0, y=1325.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id7),#mrigla
    Task(id=8, position=Pose2D(x=2585.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id8),
    Task(id=9, position=Pose2D(x=2640.0, y=400.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=10, position=Pose2D(x=2769.0, y=330.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id10),
    Task(id=11, position=Pose2D(x=2640.0, y=1325.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=12, position=Pose2D(x=2365.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id12),
    Task(id=13, position=Pose2D(x=365.0, y=1780.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),
]"""

available_tasks = [
    Task(id=0, position=Pose2D(x=1196.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  
    Task(id=3, position=Pose2D(x=1100.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),#blacet subtasks id3 mrigla
    Task(id=4, position=Pose2D(x=1226.0, y=635.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id4),#y was 415 changed ebcause no rrt
    Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),
    Task(id=6, position=Pose2D(x=1226.0, y=635.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id6),
    Task(id=1, position=Pose2D(x=770.0, y=571.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),#blacet  id1 mrigla
    Task(id=2, position=Pose2D(x=770.0, y=258.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id2), #stwit el x m3 id1y=628.0was
    Task(id=13, position=Pose2D(x=365.0, y=1780.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),
    Task(id=5, position=Pose2D(x=360.0, y=400.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),#mrigla
    Task(id=7, position=Pose2D(x=360.0, y=1325.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id7),#mrigla
    Task(id=8, position=Pose2D(x=2585.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id8),
    Task(id=9, position=Pose2D(x=2640.0, y=400.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=10, position=Pose2D(x=2769.0, y=330.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id10),
    Task(id=11, position=Pose2D(x=2640.0, y=1325.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=12, position=Pose2D(x=2365.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id12),
    Task(id=14, position=Pose2D(x=2230.0, y=521.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14)
]

yellow_private_task = Task(id=16, position=Pose2D(x=831.0, y=1385.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id16)

bonus_task1 = Task(id=14, position=Pose2D(x=2230.0, y=521.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14)
bonus_task2 = Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15)
true_yellow_tasks = [
    Task(id=0, position=Pose2D(x=1196.0, y=0.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  
    Task(id=1, position=Pose2D(x=780.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),#blacet  id1 mrigla
    Task(id=2, position=Pose2D(x=780.0, y=69.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id2), #stwit el x m3 id1y=628.0was
    Task(id=3, position=Pose2D(x=1101.0, y=942.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),#blacet subtasks id3 mrigla
    Task(id=4, position=Pose2D(x=1238.0, y=180.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id4),
    Task(id=5, position=Pose2D(x=80.0, y=384.5, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),#mrigla
    Task(id=6, position=Pose2D(x=1226.0, y=350.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id6),
    Task(id=7, position=Pose2D(x=80.0, y=1315.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id7),#mrigla
    Task(id=8, position=Pose2D(x=2830.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id8),
    Task(id=9, position=Pose2D(x=2930.0, y=403.5, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=10, position=Pose2D(x=2769.0, y=84.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id10),
    Task(id=11, position=Pose2D(x=2934.0, y=1315.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=12, position=Pose2D(x=2703.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id12),
    Task(id=13, position=Pose2D(x=365.0, y=1780.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),
    Task(id=14, position=Pose2D(x=2216.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),
    Task(id=15, position=Pose2D(x=1920.0, y=950.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),
    Task(id=16, position=Pose2D(x=831.0, y=1730.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id16)
]

true_tasks = true_yellow_tasks
#here based on the trobot color i should change the true tasks

# i need to handle if the task is empty
#blue
blue_tasks = [Task(id=0, position=Pose2D(x=1777.0, y=150.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),  
    Task(id=1, position=Pose2D(x=2226.0, y=456.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=2, position=Pose2D(x=2226.0, y=231.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=3, position=Pose2D(x=1904.0, y=743.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=4, position=Pose2D(x=1774.0, y=280.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=5, position=Pose2D(x=2735.0, y=407.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO,subtask_list=subtask_for_test),
    Task(id=6, position=Pose2D(x=1774.0, y=635.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=7, position=Pose2D(x=2735.0, y=1319.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=8, position=Pose2D(x=297.0, y=884.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=9, position=Pose2D(x=265.0, y=407.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=10, position=Pose2D(x=231.0, y=296.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=11, position=Pose2D(x=265.0, y=1319.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=12, position=Pose2D(x=620.0, y=873.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),
    Task(id=13, position=Pose2D(x=2635.0, y=1780.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 0,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtask_for_test),]

yellow_tasks = []

building_dropDown_areas = []

building_pickUp_areas = []

###################################### OBSTACLES LISTS ####################################

#these obstacle positions are just for testing...they are random
#here i need to put blue obstacles and yellow obstacles and the common obstacles 
#because the obstacles may vary (construction site)
obstacle_positions = [
    Obstacle(position=Point2D(80.0, 542.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(80.0, 442.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(80.0, 342.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(80.0, 242.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(630.0, 242.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(730.0, 242.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(830.0, 242.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(930.0, 242.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(80.0, 1165.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(80.0, 1265.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(80.0, 1365.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(80.0, 1465.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2080.0, 250.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2180.0, 250.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2280.0, 250.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2380.0, 250.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 542.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 442.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 342.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 242.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 1165.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 1265.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 1365.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2930.0, 1465.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(960.0, 945.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(1060.0, 945.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(1160.0, 945.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(1260.0, 945.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(1760.0, 945.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(1860.0, 945.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(1960.0, 945.0), obstacle_type=ObstacleType.BUILDING),
    Obstacle(position=Point2D(2060.0, 945.0), obstacle_type=ObstacleType.BUILDING),
]

"""obstacles = [
    [80, 542],[80, 442],[80, 342],[80, 242],[630, 242],[730, 242],[830, 242],[930, 242],
    [80, 1165],[80,1265],[80,1365],[80, 1465],[2080, 250],[2180, 250],[2280, 250],[2380, 250],
    [2930, 542],[2930, 442],[2930, 342],[2930, 242], [2930, 1165],[2930, 1265],[2930, 1365],
    [2930, 1465], [960, 945], [1060, 945],[1160, 945],[1260, 945],[1760,945],[1860,945],[1960,945],[2060,945]
    
]"""

########################## SERVERS CLASSES ##################

class Services:
    def __init__(self):
        self.get_map_state_service = GetMapStateService()
        self.delete_task_service = DeleteTaskService()
        self.get_true_tasks_service = TrueTasksPLacesService()

class DeleteTaskService(Node):
    def __init__(self):
        super().__init__('delete_task_service')
        self.srv = self.create_service(DeleteTask, 'delete_task_service', self.handle_delete_task)
        self.get_logger().warn('Created delete_task_service')

    def handle_delete_task(self, request, response):
        global available_tasks
        self.get_logger().info('Got a request about delete task... Now sending a response')
        for task in available_tasks:
            if task.id == request.id:
                available_tasks.remove(task)
                self.get_logger().error(f'Removed task with id {request.id}')
                break
        return response
                

class GetMapStateService(Node):
    def __init__(self):
        super().__init__('get_map_state_service')
        self.srv = self.create_service(GetMapState, 'get_map_state_service', self.handle_get_map_state)
        self.get_logger().warn('Created get_map_state Service')

    def handle_get_map_state(self, request, response):
        global robot_color
        global available_tasks
        global obstacle_positions
        self.get_logger().info('Got a request... Now sending a response')

        if not available_tasks:
            self.get_logger().warn('No tasks available')

        # Convert your Task objects to TaskMsg objects
        response.tasks = []
        for task in available_tasks:
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
            task_msg.nav_is_with_platforms = bool(task.is_with_platforms.value)
            task_msg.subtask_list = convert_to_subtask_list_msg(task.subtask_list)
            # Append the Task message to the response
            response.tasks.append(task_msg)
        response.obstacle_positions = []
        for obstacle in obstacle_positions:
            obstacle_msg = ObstacleMsg()    
            obstacle_msg.x = obstacle.position.x
            obstacle_msg.y = obstacle.position.y
            obstacle_msg.type = obstacle.type.value
            response.obstacle_positions.append(obstacle_msg)
        #self.get_logger().info(f'Obstacle at position ({obstacle_msg.x}, {obstacle_msg.y}) with type {obstacle_msg.type}')

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
            'nav_is_with_platforms': task_msg.nav_is_with_platforms,
            'subtask_list': task_msg.subtask_list
        }
            
        # Log the task message as a dictionary
        self.get_logger().info(f'Sent task message: {task_msg_dict}')
        return response



class TrueTasksPLacesService(Node):
    def __init__(self):
        super().__init__('true_tasks_places_service')
        self.srv = self.create_service(GetMapState, 'true_tasks_places_service', self.handle_true_tasks_places)
        self.get_logger().warn('Created true_tasks_places_service')

    def handle_true_tasks_places(self, request, response):
        global true_tasks
        self.get_logger().info('Got a request... Now sending a response')

        if not true_tasks:
            self.get_logger().warn('No tasks available')

        # Convert your Task objects to TaskMsg objects
        response.tasks = []
        for task in true_tasks:
            task_msg = TaskMsg()
            task_msg.id = task.id
            task_msg.position = task.position  
            task_msg.type = task.type.value
            task_msg.direction = task.direction.value
            task_msg.score = task.score.value
            task_msg.nav_linear_velocity_choice = task.linear_velocity.value
            task_msg.nav_angular_velocity_choice = task.angular_velocity.value         
            task_msg.nav_linear_acceleration_choice = task.linear_acceleration
            task_msg.nav_angular_acceleration_choice = task.angular_acceleration
            task_msg.nav_is_with_platforms = bool(task.is_with_platforms.value)
            task_msg.subtask_list = convert_to_subtask_list_msg(task.subtask_list)
            response.tasks.append(task_msg)   

        response.obstacle_positions = []
        return response
    
class Subscribers:
    def __init__(self):
        self.enemy_robot_position_subscriber = EnemyRobotPositionSubscriber()
        self.my_robot_position_subscriber = MyRobotPositionSubscriber()
        self.my_robot_color = RobotColorSubscriber()

class EnemyRobotPositionSubscriber(Node):
    def __init__(self):
        super().__init__('enemy_robot_position_subscriber')
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
        #self.get_logger().info(f'Enemy position: {enemy_position}')


class RobotColorSubscriber(Node):
    def __init__(self):
        super().__init__('static_task_robot_color_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.sub = self.create_subscription(String, 'robot_color', self.robot_color_callback, qos_profile) 
        self.get_logger().warn('Created enemy_robot_position_subscriber')

    def robot_color_callback(self, msg):
        global robot_color
        robot_color = msg.data
        self.get_logger().info(f'Robot color: {robot_color}')
        
class MyRobotPositionSubscriber(Node):
    def __init__(self):
        super().__init__('static_task_my_robot_position_subscriber')
        self.sub = self.create_subscription(NavigatorNavigationMessage, 'ros_can_interface/navigator_nav',
                                            self.Navigator_Receive_Position_Subscriber_callback, 10)
        self.get_logger().warn('Created my_robot_position_subscriber:static_task_generator node')

    def Navigator_Receive_Position_Subscriber_callback(self, msg):
        global current_position
        global robot_current_state
        robot_current_state = msg.nav_current_state
        if (robot_current_state != 0):
            current_position.x = msg.nav_current_x
            current_position.y = msg.nav_current_y
            current_position.theta = msg.nav_current_angle
        else :
            current_position.x = 1200.0
            current_position.y = 275.5
            current_position.theta = 90.0
        #self.get_logger().info(f'Current position: {current_position}')
        print("robot_current_state", robot_current_state)
        print("current_position.x",current_position.x)
        print("current_position.y",current_position.y)
################### COMMUNICATION INIT FUNCTION #############

def communication_instances_init():

    global servers_instance
    global subscribers_instance
    subscribers_instance = Subscribers()
    servers_instance = Services()
    
############################# UTILITY FUNCTIONS #####################

def display_available_tasks():
        global available_tasks
        for task in available_tasks:
            print(f"Task ID: {task.id}")
            print(f"Position: ({task.position.x}, {task.position.y}, {task.position.theta})")
            print(f"Type: {task.type}")
            print(f"Direction: {task.direction}")
            print(f"Score: {task.score}")
            print(f"Linear Velocity: {task.linear_velocity}")
            print(f"Angular Velocity: {task.angular_velocity}")
            print(f"Linear Acceleration: {task.linear_acceleration}")
            print(f"Angular Acceleration: {task.angular_acceleration}")
            print(f"Is with Platforms: {task.is_with_platforms}")
            print("Subtasks:")
            for subtask in task.subtask_list:
                print(f"  Subtask Order: {subtask.order}")
                print(f"  Subtask Type: {subtask.type}")
                if isinstance(subtask, NavSubTask):
                    print(f"  Position: ({subtask.position.x}, {subtask.position.y}, {subtask.position.theta})")
                    print(f"  Direction: {subtask.direction}")
                    print(f"  Linear Velocity: {subtask.linear_velocity}")
                    print(f"  Angular Velocity: {subtask.angular_velocity}")
                    print(f"  Linear Acceleration: {subtask.linear_acceleration}")
                    print(f"  Angular Acceleration: {subtask.angular_acceleration}")
                    print(f"  Is with Platforms: {subtask.is_with_platforms}")
                elif isinstance(subtask, MechanismSubTask):
                    print(f"  Mechanism Subtask Type: {subtask.mechanism_subtask_type}")
            print("----------")

def find_equivalent_task(index):
    global available_tasks
    global true_tasks
    display_available_tasks()
    i = 0
    for task in available_tasks:
        print("task.id",task.id)
    print("id in true tasks",true_tasks[index].id)
    for task in available_tasks:
        if task.id == true_tasks[index].id:
            return i
        i += 1
    print("!!!!!!didn't find the equivalent task in availabele tasks list now returning the same index")    
    return index

def find_equivalent_true_task(index):
    global available_tasks
    global true_tasks
    i = 0
    for task in true_tasks:
        print("task.id",task.id)
    print("id in available tasks",available_tasks[index].id)
    for task in true_tasks:
        if task.id == available_tasks[index].id:
            return i
        i += 1
    print("!!!!!!didn't find the equivalent task in true tasks list now returning the same index")    
    return index

def calculate_distance(obstacle1:Obstacle, obstacle2:Obstacle):
    print("obstacle1.position.x",obstacle1.position.x)
    print("obstacle1.position.y",obstacle1.position.y)
    print("obstacle2.position.x",obstacle2.position.x)
    print("obstacle2.position.y",obstacle2.position.y)
    return math.sqrt((obstacle1.position.x - obstacle2.position.x) ** 2 + (obstacle1.position.y - obstacle2.position.y) ** 2)

def convert_to_subtask_list_msg (subtask_list: List['SubTask']):
    subtask_list_msg = []

    for subtask in subtask_list:
        subtask_msg = SubTaskMsg()  # Create a new SubTaskMsg object
        if subtask.type == SubTaskType.NAV_SUBTASK:
            subtask_msg.order = subtask.order
            subtask_msg.type = subtask.type.value  # Assuming type matches with SubTaskType enum
            subtask_msg.position = subtask.position  # Directly assigning the Pose2D object
            subtask_msg.mechanism_subtask_type = subtask.type.value  # You can adjust if needed to match SubTaskType
            subtask_msg.nav_is_forward = bool(int(subtask.direction.value)) 
            subtask_msg.nav_linear_velocity_choice = subtask.linear_velocity.value
            subtask_msg.nav_angular_velocity_choice = subtask.angular_velocity.value
            subtask_msg.nav_linear_acceleration_choice = subtask.linear_acceleration.value  # Assuming a default value
            subtask_msg.nav_angular_acceleration_choice = subtask.angular_acceleration.value  # Assuming a default value
            subtask_msg.nav_is_with_platforms = bool(subtask.is_with_platforms.value)
        else:    
            subtask_msg.order = subtask.order
            subtask_msg.type = subtask.type.value  # Assuming type matches with SubTaskType enum
            subtask_msg.position = Pose2D(x=0.0, y=0.0, theta=0.0) # Directly assigning the Pose2D object
            subtask_msg.mechanism_subtask_type = subtask.mechanism_subtask_type.value  # You can adjust if needed to match SubTaskType
            subtask_msg.nav_is_forward = True 
            subtask_msg.nav_linear_velocity_choice = 0
            subtask_msg.nav_angular_velocity_choice = 0
            subtask_msg.nav_linear_acceleration_choice = 0  # Assuming a default value
            subtask_msg.nav_angular_acceleration_choice = 0  # Assuming a default value
            subtask_msg.nav_is_with_platforms = False
        subtask_list_msg.append(subtask_msg)
    return subtask_list_msg    

################################################# MAIN FUNCTION ######################################

def main(args=None):
    rclpy.init(args=args)
    global servers_instance
    global subscribers_instance
    map_state_updater = MapStateUpdater()
    communication_instances_init()
    #here i need to add based on my color i will be returning a specific obstacle_positions = yellow one or blue one    
    # now i will be using the blue_obstacle_positions (the positions of the blue ones)
    executor = MultiThreadedExecutor()
    executor.add_node(servers_instance.get_map_state_service)
    executor.add_node(subscribers_instance.enemy_robot_position_subscriber)
    executor.add_node(subscribers_instance.my_robot_position_subscriber)
    executor.add_node(map_state_updater)
    executor.add_node(subscribers_instance.my_robot_position_subscriber)
    executor.add_node(servers_instance.delete_task_service)
    executor.add_node(servers_instance.get_true_tasks_service)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        servers_instance.get_map_state_service.destroy_node()
        subscribers_instance.enemy_robot_position_subscriber.destroy_node()
        subscribers_instance.my_robot_position_subscriber.destroy_node()
        map_state_updater.destroy_node()
        subscribers_instance.my_robot_position_subscriber.destroy_node()
        servers_instance.delete_task_service.destroy_node()
        servers_instance.get_true_tasks_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    