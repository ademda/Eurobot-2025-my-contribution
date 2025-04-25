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
from ros_custom_messages.msg import TaskMsg, SubTaskMsg, ObstacleMsg, NavigatorNavigationMessage, CurrentTask
from rclpy.qos import QoSProfile, ReliabilityPolicy
from abc import ABC, abstractmethod 
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty
######################################## GLOBAL VARIABLES #########################

servers_instance = None
subscribers_instance = None
enemy_position = Pose2D(x=0.0, y=0.0, theta=0.0)
current_position = Pose2D(x=1200.0, y=195.5, theta=90.0) #initialise the current position to not get errors
robot_color = None
robot_current_state = 0
current_task_id = None
currently_doing_subtask = None

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
        global current_task_id
        global currently_doing_subtask
        minimal_error_distance = 75
        minimal_error_distance_enemy = 150 # was 175
        max_distance_between_obstacles = 600
        # here used the minimal error distance of the last year maybe needs to check it
        #I changed it a bit and i will test it
        index = 0
        #print("current postion from update available task:",current_position.x,current_position.y)
        while index < len(true_tasks):
            task = true_tasks[index]
            if ((enemy_position.x - minimal_error_distance_enemy <= task.position.x <= enemy_position.x + minimal_error_distance_enemy) and
                (enemy_position.y - minimal_error_distance_enemy <= task.position.y <= enemy_position.y + minimal_error_distance_enemy) and
                task.type == TaskType.BUILDING_PICKUP and (current_task_id != task.id or (current_task_id == task.id and currently_doing_subtask == TaskType.NORMAL_NAV.value) )):
                
                if task.id ==1 or task.id == 14: #here the blue i need to add that case
                    print("i m in id 1111111111111111111111111111")
                    self.get_logger().warn(f'Replacing task at position {task.position.x} and {task.position.y} based on ENEMY POSITION')
                    print("index in the true tasks list",index)
                    # Start cascading replacements from the current index
                    current_index = find_equivalent_task(index)
                    true_tasks.pop(index)
                    next_dropoff_task_index = next((i for i in range(current_index + 1, len(available_tasks)) 
                                                if (available_tasks[i].type == TaskType.BUILDING_DROPDOWN )), None)
                    if next_dropoff_task_index is None:
                            self.get_logger().warn('No more BUILDING_DROPDOWN after the building pickup to be deteld')
                    else:
                        true_task_next_dropoff_index=find_equivalent_true_task(next_dropoff_task_index)
                        if true_task_next_dropoff_index is not None:
                            true_tasks.pop(true_task_next_dropoff_index)
                        else:
                            self.get_logger().warn('Didnt find the equivalent dropoff task of:',next_dropoff_index)

                    while current_index < len(available_tasks) - 1:
                        # Find the next BUILDING_PICKUP task for replacement
                        next_task_index = next((i for i in range(current_index + 1, len(available_tasks)) 
                                                if (available_tasks[i].type == TaskType.BUILDING_PICKUP and  available_tasks[i].id != 1)), None) # be aware here no id 14
 
                        next_dropoff_task_index = next((i for i in range(current_index + 1, len(available_tasks)) 
                                                if (available_tasks[i].type == TaskType.BUILDING_DROPDOWN )), None)

                        if next_dropoff_task_index is not None :
                            next_next_dropoff_task_index = next((i for i in range(next_dropoff_task_index + 1, len(available_tasks)) 
                                                    if (available_tasks[i].type == TaskType.BUILDING_DROPDOWN )), None)
                        else :
                            next_next_dropoff_task_index = None
                            
                        if next_task_index is None:
                            self.get_logger().warn('No more BUILDING_PICKUP tasks to cascade replacements')
                            break

                        if next_next_dropoff_task_index is None:
                            self.get_logger().warn('No more BUILDING_DROPDOWN tasks to cascade replacements')
                            break
                            
                        print("next next drop off index",next_next_dropoff_task_index)
                        print("next drop off index",next_dropoff_task_index)
                        # Replace the current task with the next BUILDING_PICKUP task
                        available_tasks[current_index] = available_tasks[next_task_index]
                        self.get_logger().warn(f'Replaced task at index {current_index} with task at position {available_tasks[next_task_index].position}')
                        available_tasks[next_dropoff_task_index] = available_tasks[next_next_dropoff_task_index]

                        # Move to the next task in the chain
                        current_index = next_task_index

                    last_task_index = next((i for i in range(current_index , len(available_tasks)) 
                                            if available_tasks[i].type == TaskType.BUILDING_PICKUP), None)
                    print("last task index",last_task_index)
                    if last_task_index:
                        self.get_logger().warn(f'Removing final replaced task at index {last_task_index} from available_tasks')
                        if last_task_index is not None:
                            print(f"Task ID related to last_task_index: {available_tasks[last_task_index].id}")
                        available_tasks.pop(last_task_index)
                    if last_task_index is not None: 
                        next_dropoff_index = next((i for i in range(last_task_index, len(available_tasks)) 
                                    if available_tasks[i].type == TaskType.BUILDING_DROPDOWN), None)
            
                    if next_dropoff_index is not None:
                        self.get_logger().warn(f'Removing next BUILDING_DROPDOWN task at index {next_dropoff_index} from available_tasks')
                        true_task_next_dropoff_index=find_equivalent_true_task(next_dropoff_index)
                        available_tasks.pop(next_dropoff_index)    
                        if true_task_next_dropoff_index is not None :
                            true_tasks.pop(true_task_next_dropoff_index)
                        else:
                            print("when trying to remove last dropoff from true tasks.didn't find it(no equivalent task)")
                else:
                    self.get_logger().warn(f'Replacing task at position {task.position.x} and {task.position.y} based on ENEMY POSITION')
                    print("index in the true tasks list",index)
                    # Start cascading replacements from the current index
                    current_index = find_equivalent_task(index)
                    true_tasks.pop(index)
                    while current_index < len(available_tasks) - 1:
                        # Find the next BUILDING_PICKUP task for replacement
                        next_task_index = next((i for i in range(current_index + 1, len(available_tasks)) 
                                                if (available_tasks[i].type == TaskType.BUILDING_PICKUP and  available_tasks[i].id != 1 and available_tasks[i].id != 14)), None)
                        
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
                    print("last building pickup task index",last_task_index)
                    if last_task_index:
                        self.get_logger().warn(f'Removing final replaced task at index {last_task_index} from available_tasks')
                        available_tasks.pop(last_task_index)

                    next_dropoff_index = next((i for i in range(last_task_index, len(available_tasks)) 
                                if available_tasks[i].type == TaskType.BUILDING_DROPDOWN), None)
        
                    if next_dropoff_index is not None:
                        self.get_logger().warn(f'Removing next BUILDING_DROPDOWN task at index {next_dropoff_index} from available_tasks')
                        true_task_next_dropoff_index=find_equivalent_true_task(next_dropoff_index)
                        available_tasks.pop(next_dropoff_index)    
                        if true_task_next_dropoff_index is not None:
                            true_tasks.pop(true_task_next_dropoff_index)
            index += 1


    def update_obstacles_positions(self):
        global obstacle_positions
        global current_position
        global enemy_position
        global available_tasks
        minimal_error_distance = 90
        minimal_error_distance_enemy = 175
        max_distance_between_obstacles = 450
        for obstacle in obstacle_positions:
            if((current_position.x - minimal_error_distance <= obstacle.position.x <= current_position.x + minimal_error_distance) and
            (current_position.y - minimal_error_distance <= obstacle.position.y <= current_position.y + minimal_error_distance) and
            obstacle.type.value == ObstacleType.BUILDING.value):
                
                #print('Removing obstacle at position based on MY POSITION: ', obstacle.position.x)
                #print('Removing obstacle at position based on MY POSITION: ', obstacle.position.y)
                for i in range(len(obstacle_positions) - 1, -1, -1):
                    another_obstacle = obstacle_positions[i]
                    if (calculate_distance(obstacle, another_obstacle) <= max_distance_between_obstacles and 
                        another_obstacle.type.value == ObstacleType.BUILDING.value and 
                        another_obstacle != obstacle):
                        #print("another obstacle", another_obstacle.position.x)
                        #print("obstacle", obstacle.position.y)
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
                        #print("another obstacle", another_obstacle.position.x)
                        #print("obstacle", obstacle.position.y)
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
    NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1200.0, y=500.0, theta=655.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
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
    NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=770.0,y=500.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]
#blacet subtasks id3 mrigla
subtasks_id3 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1100.0, y=923.0, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),#theta was 90
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),

]

subtasks_id4 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=315.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=645.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id5 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=140.5, y=400.0, theta=180.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),# x was 180.5
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=675.0, y=400.0, theta=180.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),# x was 400
]

subtasks_id6 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=470.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1226.0, y=645.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]    

#subtask id7 mrigl

subtasks_id7 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=140.5, y=1325.0, theta=180.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=675.0, y=1325.0, theta=180.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
]

subtasks_id8 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2720.0, y=884.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES), 
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2585.0, y=884.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id9 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),#maybe these lower planks repeated i don't need them
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2859.5, y=400.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2325.0, y=400.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),# x was 2600
]

subtasks_id10 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2769.0, y=260.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2769.0, y=330.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id11 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2859.5, y=1325.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2325.0, y=1325.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
]

subtasks_id12 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2542.0, y=884.0, theta=0.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2365.0, y=884.0, theta=0.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id14 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ALOT),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
]#this configuration is good for yellow robot not for blue robot....when buildibgn blue robot i need to create special subtask for him    

subtasks_id15 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1900.0, y=923.0, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),#theta was 90.0 # y was 823
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
]

subtasks_id16 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=831.0, y=1555.5, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=831.0, y=1385.0, theta=90.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
]

######subtasks for the blue robot######
subtasks_id20 = [
    #MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.BONDEROLE_SERVO_HOLD),
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.BONDEROLE_SERVO_REALEASE),
    NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1800.0, y=500.0, theta=655.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]   

subtasks_id24 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1774.0, y=315.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1774.0, y=645.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id22 = [
    #NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=744.0, y=258.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.SLOW, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),#na9ast 4 cm 
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    NavSubTask(order=1, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2230.0,y=500.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id26 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1774.0, y=470.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=1774.0, y=645.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]  

subtasks_id32 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=458.0, y=884.0, theta=180.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=635.0, y=884.0, theta=180.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id28 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=280.0, y=884.0, theta=180.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES), 
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=415.0, y=884.0, theta=180.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id30 = [
    NavSubTask(order=0, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=231.0, y=260.0, theta=270.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.DROP_CANS),
    MechanismSubTask(order=2,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    NavSubTask(order=3, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=231.0, y=330.0, theta=270.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
]

subtasks_id36 = [
    MechanismSubTask(order=0,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.LOWER_PLANKS),
    MechanismSubTask(order=1,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.GRAB_CANS),
    NavSubTask(order=2, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2169.0, y=1555.5, theta=90.0),direction=Direction.FORWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.NO),
    MechanismSubTask(order=3,type=SubTaskType.MECHANISM_SUBTASK, mechanism_subtask_type=MachanismSubTaskType.ELEVATE_PLANKS_ABOVE),
    NavSubTask(order=4, type=SubTaskType.NAV_SUBTASK, position=Pose2D(x=2169.0, y=1385.0, theta=90.0),direction=Direction.BACKWARD, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=SpeedChoice.NORMAL, angular_acceleration=SpeedChoice.NORMAL, is_with_platforms=IS_WITH_PLATFORMS.YES),
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
available_tasks = []
true_tasks = []

yellow_tasks = [
    Task(id=0, position=Pose2D(x=1200.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  
    Task(id=1, position=Pose2D(x=770.0, y=560.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),
    Task(id=2, position=Pose2D(x=770.0, y=258.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id2),
    Task(id=5, position=Pose2D(x=400.0, y=400.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),  # x was x=360.0                              
    Task(id=4, position=Pose2D(x=1226.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id4), # was y=675.0  
    Task(id=3, position=Pose2D(x=1100.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),     
    Task(id=6, position=Pose2D(x=1226.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id6),  # was y=675.0  
    Task(id=13, position=Pose2D(x=400.0, y=1660.0, theta=float(655.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),#y was 1780 x was 365            
    Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),            
    Task(id=7, position=Pose2D(x=675.0, y=1325.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id7),            
]#return this after the simulation finishes

"""yellow_tasks = [
    Task(id=0, position=Pose2D(x=1200.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  #return this after the simulation finishes  
    Task(id=1, position=Pose2D(x=770.0, y=560.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),
    Task(id=2, position=Pose2D(x=770.0, y=258.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id2),
    Task(id=5, position=Pose2D(x=400.0, y=400.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),  # x was x=360.0                              
    Task(id=4, position=Pose2D(x=1226.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id4), # was y=675.0  
    Task(id=3, position=Pose2D(x=1100.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),     
    Task(id=6, position=Pose2D(x=1226.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id6),  # was y=675.0  
    Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),            
    Task(id=12, position=Pose2D(x=2365.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id12),            
    Task(id=7, position=Pose2D(x=400.0, y=1325.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id7),            
    Task(id=8, position=Pose2D(x=2585.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id8),
    Task(id=9, position=Pose2D(x=2600.0, y=400.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=10, position=Pose2D(x=2769.0, y=330.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id10),
    Task(id=13, position=Pose2D(x=400.0, y=1730.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),#y was 1780 x was 365            #return this after the simulation finishes    
    Task(id=11, position=Pose2D(x=2600.0, y=1325.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=14, position=Pose2D(x=2230.0, y=560.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),
    Task(id=16, position=Pose2D(x=831.0, y=1730.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id16)
]"""

yellow_private_task = Task(id=16, position=Pose2D(x=831.0, y=1385.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id16)

bonus_task1 = Task(id=14, position=Pose2D(x=2230.0, y=521.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14)
bonus_task2 = Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15)

true_yellow_tasks = [
    #Task(id=20, position=Pose2D(x=1196.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  #will remove it later
    Task(id=0, position=Pose2D(x=1196.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  
    Task(id=1, position=Pose2D(x=770.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),
    Task(id=2, position=Pose2D(x=770.0, y=69.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id2), 
    Task(id=3, position=Pose2D(x=1101.0, y=942.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),
    Task(id=4, position=Pose2D(x=1238.0, y=180.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id4),
    Task(id=5, position=Pose2D(x=80.0, y=384.5, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),
    Task(id=6, position=Pose2D(x=1226.0, y=350.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id6),
    Task(id=13, position=Pose2D(x=400.0, y=1730.0, theta=float(655.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),#y was 1780 x was 365            
    Task(id=15, position=Pose2D(x=1920.0, y=950.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),
    Task(id=7, position=Pose2D(x=80.0, y=1315.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id7),
]#return this after the simulation finishes


"""true_yellow_tasks = [
    #Task(id=20, position=Pose2D(x=1196.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  #will remove it later
    Task(id=0, position=Pose2D(x=1196.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id0),  
    Task(id=1, position=Pose2D(x=770.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),
    Task(id=2, position=Pose2D(x=770.0, y=69.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id2), 
    Task(id=3, position=Pose2D(x=1101.0, y=942.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),
    Task(id=4, position=Pose2D(x=1238.0, y=180.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id4),
    Task(id=5, position=Pose2D(x=80.0, y=384.5, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),
    Task(id=6, position=Pose2D(x=1226.0, y=350.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id6),
    Task(id=7, position=Pose2D(x=80.0, y=1315.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id7),
    Task(id=8, position=Pose2D(x=2830.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id8),
    Task(id=9, position=Pose2D(x=2930.0, y=403.5, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=10, position=Pose2D(x=2769.0, y=84.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id10),
    Task(id=11, position=Pose2D(x=2934.0, y=1315.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=12, position=Pose2D(x=2703.0, y=884.0, theta=float(0.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id12),
    Task(id=13, position=Pose2D(x=400.0, y=1730.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),#y was 1780 x was 365            
    Task(id=14, position=Pose2D(x=2216.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),
    Task(id=15, position=Pose2D(x=1920.0, y=950.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),
    Task(id=16, position=Pose2D(x=831.0, y=1385.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id16)
]"""

#here based on the trobot color i should change the true tasks

# i need to handle if the task is empty
#blue
blue_tasks = [
    Task(id=20, position=Pose2D(x=1800.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id20),  
    Task(id=14, position=Pose2D(x=2230.0, y=560.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),# y was 521.7
    Task(id=22, position=Pose2D(x=2230.0, y=258.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id22),     
    Task(id=9, position=Pose2D(x=2600.0, y=400.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),# x was 2640.0    
    Task(id=24, position=Pose2D(x=1774.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id24),
    Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),                    
    Task(id=26, position=Pose2D(x=1774.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id26),        
    Task(id=33, position=Pose2D(x=2680.0, y=1660.0, theta=float(655.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),               
    Task(id=3, position=Pose2D(x=1100.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),    
    Task(id=11, position=Pose2D(x=2325.0, y=1325.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),

]

"""blue_tasks = [
    Task(id=20, position=Pose2D(x=1800.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id20),  
    Task(id=14, position=Pose2D(x=2230.0, y=560.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),# y was 521.7
    Task(id=22, position=Pose2D(x=2230.0, y=258.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id22),     
    Task(id=9, position=Pose2D(x=2600.0, y=400.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),# x was 2640.0    
    Task(id=24, position=Pose2D(x=1774.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id24),
    Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),                    
    Task(id=26, position=Pose2D(x=1774.0, y=645.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id26),        
    Task(id=33, position=Pose2D(x=2680.0, y=1730.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),               
    Task(id=3, position=Pose2D(x=1100.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),    
    Task(id=28, position=Pose2D(x=415.0, y=884.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id28),    
    Task(id=11, position=Pose2D(x=2600.0, y=1325.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=32, position=Pose2D(x=635.0, y=884.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id32),            
    Task(id=5, position=Pose2D(x=400.0, y=400.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),                    
    Task(id=30, position=Pose2D(x=231.0, y=330.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id30),    
    Task(id=7, position=Pose2D(x=400.0, y=1325.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id7),    
    Task(id=1, position=Pose2D(x=770.0, y=560.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),
    Task(id=36, position=Pose2D(x=2169.0, y=1385.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id36)
]"""

"""blue_tasks = [
    Task(id=20, position=Pose2D(x=1800.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 0,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id20),  
    Task(id=15, position=Pose2D(x=1900.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),            
    
    Task(id=3, position=Pose2D(x=1100.0, y=608.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),
    Task(id=24, position=Pose2D(x=1774.0, y=635.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id24),
    Task(id=1, position=Pose2D(x=770.0, y=571.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),
    Task(id=22, position=Pose2D(x=2230.0, y=258.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id22), 
    Task(id=5, position=Pose2D(x=360.0, y=400.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),                    
    Task(id=26, position=Pose2D(x=1774.0, y=635.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id26),    
    Task(id=33, position=Pose2D(x=2600.0, y=1730.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),           
    Task(id=7, position=Pose2D(x=360.0, y=1325.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id7),    
    Task(id=32, position=Pose2D(x=635.0, y=884.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id32),            
    Task(id=28, position=Pose2D(x=415.0, y=884.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id28),
    Task(id=9, position=Pose2D(x=2640.0, y=400.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=30, position=Pose2D(x=231.0, y=330.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id30),
    Task(id=11, position=Pose2D(x=2640.0, y=1325.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=14, position=Pose2D(x=2230.0, y=521.7, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),
    Task(id=36, position=Pose2D(x=2169.0, y=1385.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration=1, angular_acceleration=1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id36)
]"""

true_blue_tasks = [
    Task(id=20, position=Pose2D(x=1800.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id20),  
    Task(id=22, position=Pose2D(x=2230.0, y=69.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id20), 
    Task(id=3, position=Pose2D(x=1101.0, y=942.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),
    Task(id=24, position=Pose2D(x=1774.0, y=180.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id24),
    Task(id=26, position=Pose2D(x=1774.0, y=350.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id26),
    Task(id=9, position=Pose2D(x=2930.0, y=403.5, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=33, position=Pose2D(x=2680.0, y=1730.0, theta=float(655.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),           
    Task(id=14, position=Pose2D(x=2216.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),
    Task(id=15, position=Pose2D(x=1920.0, y=950.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),
    Task(id=11, position=Pose2D(x=2934.0, y=1315.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
]


"""true_blue_tasks = [
    Task(id=20, position=Pose2D(x=1800.0, y=50.0, theta=float(655.0)), type=TaskType.BONDEROLE_POSE,direction=Direction.BACKWARD, score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id20),  
    Task(id=1, position=Pose2D(x=770.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id1),
    Task(id=22, position=Pose2D(x=2230.0, y=69.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id20), 
    Task(id=3, position=Pose2D(x=1101.0, y=942.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id3),
    Task(id=24, position=Pose2D(x=1774.0, y=180.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id24),
    Task(id=5, position=Pose2D(x=80.0, y=384.5, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id5),
    Task(id=26, position=Pose2D(x=1774.0, y=350.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id26),
    Task(id=7, position=Pose2D(x=80.0, y=1315.0, theta=float(180.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id7),
    Task(id=28, position=Pose2D(x=170.0, y=884.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.YES, subtask_list=subtasks_id28),
    Task(id=9, position=Pose2D(x=2930.0, y=403.5, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id9),
    Task(id=30, position=Pose2D(x=231.0, y=84.0, theta=float(270.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id30),
    Task(id=11, position=Pose2D(x=2934.0, y=1315.0, theta=float(0.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id11),
    Task(id=32, position=Pose2D(x=297.0, y=884.0, theta=float(180.0)), type=TaskType.BUILDING_DROPDOWN,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.SLOW, linear_acceleration = 1,angular_acceleration = 0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id32),
    Task(id=33, position=Pose2D(x=2680.0, y=1730.0, theta=float(90.0)), type=TaskType.RETURN_TO_FINISH_SPOT,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP,linear_velocity=SpeedChoice.NORMAL, angular_velocity=SpeedChoice.NORMAL, linear_acceleration = 1,angular_acceleration = 1, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=[]),           
    Task(id=14, position=Pose2D(x=2216.0, y=250.0, theta=float(270.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id14),
    Task(id=15, position=Pose2D(x=1920.0, y=950.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP, direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id15),
    Task(id=36, position=Pose2D(x=2169.0, y=1730.0, theta=float(90.0)), type=TaskType.BUILDING_PICKUP,direction=Direction.FORWARD, score=TaskTypeScore.BUILDING_PICKUP, linear_velocity=SpeedChoice.SLOW, angular_velocity=SpeedChoice.SLOW, linear_acceleration=0, angular_acceleration=0, is_with_platforms=IS_WITH_PLATFORMS.NO, subtask_list=subtasks_id36)
]
"""

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

movable_obstacles_id4 = [
    Obstacle(position=Point2D(1076.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1176.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1276.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1376.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id6 = [
    Obstacle(position=Point2D(1076.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1176.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1276.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1376.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id2 = [
    Obstacle(position=Point2D(620.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(720.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(820.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(920.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id12 = [
    Obstacle(position=Point2D(2722.0, 734.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2722.0, 834.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2722.0, 934.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2722.0, 1034.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id8 = [
    Obstacle(position=Point2D(2900.0, 734.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2900.0, 834.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2900.0, 934.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2900.0, 1034.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id10 = [
    Obstacle(position=Point2D(2619.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2719.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2819.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2919.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]    

movable_obstacles_id24 = [
    Obstacle(position=Point2D(1624.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1724.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1824.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1924.0, 135.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id26 = [
    Obstacle(position=Point2D(1624.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1724.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1824.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1924.0, 290.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id22 = [
    Obstacle(position=Point2D(2080.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2180.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2280.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2380.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id30 = [
    Obstacle(position=Point2D(80.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(181.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(281.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(381.0, 110.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id28 = [
    Obstacle(position=Point2D(100.0, 734.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(100.0, 834.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(100.0, 934.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(100.0, 1034.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

movable_obstacles_id32 = [
    Obstacle(position=Point2D(278.0, 734.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(278.0, 834.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(278.0, 934.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(278.0, 1034.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

fixed_inside_prohibted_zone_area_of_blue = [
    Obstacle(position=Point2D(361.0, 1023.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(150.0, 1023.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(361.0, 746.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(150.0, 746.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1650.0, 330.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1900.0, 330.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1650.0, 150.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1900.0, 150.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1200.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1200.0, 1600.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1800.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1800.0, 1600.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2150.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(850.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
]

fixed_inside_prohibted_zone_area_of_yellow = [
    Obstacle(position=Point2D(2640.0, 1023.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2850.0, 1023.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2640.0, 746.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2850.0, 746.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1350.0, 330.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1100.0, 330.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1350.0, 150.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1100.0, 150.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1200.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1200.0, 1600.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1800.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(1800.0, 1600.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(2150.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
    Obstacle(position=Point2D(850.0, 1900.0), obstacle_type=ObstacleType.FIXED_OBSTACLE),
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
        self.switch_task_service = SwitchTaskService()

class DeleteTaskService(Node):
    def __init__(self):
        super().__init__('delete_task_service')
        self.srv = self.create_service(DeleteTask, 'delete_task_service', self.handle_delete_task)
        self.get_logger().warn('Created delete_task_service')

    def handle_delete_task(self, request, response):
        global available_tasks
        global true_tasks
        self.get_logger().info('Got a request about delete task... Now sending a response')
        for task in available_tasks:
            if task.id == request.id:
                equivalent_true_task = find_equivalent_true_task(available_tasks.index(task))
                if equivalent_true_task:
                    true_tasks.pop(equivalent_true_task)
                available_tasks.remove(task)
                self.get_logger().error(f'Removed task with id {request.id}')
                selected_task = task
                #break # need to test not breaking
            add_obstacles_to_obstacles_list(selected_task)    
        return response
                
class SwitchTaskService(Node):
    def __init__(self):
        super().__init__('switch_task_service')
        self.srv = self.create_service(DeleteTask, 'switch_task_service', self.handle_switch_task)
        self.get_logger().warn('Created switch_task_service')

    def handle_switch_task(self, request: DeleteTask.Request, response):
        response = DeleteTask.Response()
        to_switch_task_id = request.id  
        to_switch_task = None
        for task in available_tasks:
            if task.id == to_switch_task_id:
                to_switch_task = task
                break               
        if to_switch_task is None:
            self.get_logger().error(f'Task with id {to_switch_task_id} not found')
            return response
        
        # Find the RETURN_TO_FINISH_SPOT task
        return_to_finish_spot_task = None
        for task in available_tasks:
            if task.type == TaskType.RETURN_TO_FINISH_SPOT:
                return_to_finish_spot_task = task
                break

        if return_to_finish_spot_task is None:
            self.get_logger().error('RETURN_TO_FINISH_SPOT task not found')
            return response
            
        if to_switch_task.id == 1 or to_switch_task.id == 14 :
            self.get_logger().warn('Switching task with id 1 or 14')
            # Find the next BUILDING_DROPDOWN task after the to_switch_task
            to_switch_task_index = available_tasks.index(to_switch_task)
            next_building_dropdown_task = None
            for i in range(to_switch_task_index + 1, len(available_tasks)):
                if available_tasks[i].type == TaskType.BUILDING_DROPDOWN:
                    next_building_dropdown_task = available_tasks[i]
                    break

            if next_building_dropdown_task is None:
                self.get_logger().error('No BUILDING_DROPDOWN task found after the to_switch_task')
                return response

            # Find the next BUILDING_PICKUP task after the to_switch_task
            to_switch_task_index = available_tasks.index(to_switch_task)
            next_building_pickup_task = None
            for i in range(to_switch_task_index + 1, len(available_tasks)):
                if available_tasks[i].type == TaskType.BUILDING_PICKUP:
                    next_building_pickup_task = available_tasks[i]
                    break

            if next_building_pickup_task is None:
                self.get_logger().error('No BUILDING_PICKUP task found after the to_switch_task')
                return response

            # Find the next BUILDING_DROPDOWN task after the next_building_pickup_task
            next_next_building_dropdown_task = None
            for i in range(available_tasks.index(next_building_pickup_task) + 1, len(available_tasks)):
                if available_tasks[i].type == TaskType.BUILDING_DROPDOWN:
                    next_next_building_dropdown_task = available_tasks[i]
                    break

            if next_next_building_dropdown_task is None:
                self.get_logger().error('No BUILDING_DROPDOWN task found after the next BUILDING_PICKUP task')
                return response
            print("next_building_oickup id:", next_building_pickup_task.id)
            print("to_switch_task_id ", to_switch_task_id)
            next_building_pickup_task_index = available_tasks.index(next_building_pickup_task)
            # Switch the to_switch_task with the next BUILDING_PICKUP task
            available_tasks[to_switch_task_index], available_tasks[next_building_pickup_task_index] = (
                available_tasks[next_building_pickup_task_index],
                available_tasks[to_switch_task_index],
            )

            next_building_dropdown_task_index = available_tasks.index(next_building_dropdown_task)
            next_next_building_dropdown_task_index = available_tasks.index(next_next_building_dropdown_task)

            available_tasks[next_next_building_dropdown_task_index], available_tasks[next_building_dropdown_task_index] = (
                available_tasks[next_building_dropdown_task_index],
                available_tasks[next_next_building_dropdown_task_index],
            )

            # Switch the next BUILDING_DROPDOWN task with the one after it
            

            self.get_logger().info(f'Task with id {to_switch_task_id} switched with the next BUILDING_PICKUP task, and the next BUILDING_DROPDOWN task switched with the one after it')
            return response

        else:
            self.get_logger().warn('Switching tasks that are normal')
            # Find the task just before the RETURN_TO_FINISH_SPOT task of type BUILDING_PICKUP
            return_to_finish_spot_index = available_tasks.index(return_to_finish_spot_task)
            last_building_pickup_task = None
            for i in range(return_to_finish_spot_index - 1, -1, -1):
                if available_tasks[i].type == TaskType.BUILDING_PICKUP and available_tasks[i].id != 14 and available_tasks[i].id != 1 and available_tasks[i].id != to_switch_task_id:
                    last_building_pickup_task = available_tasks[i]
                    self.get_logger().info(f'Found BUILDING_PICKUP task with id {last_building_pickup_task.id} before RETURN_TO_FINISH_SPOT task') 
                    break

            if last_building_pickup_task is None:
                self.get_logger().error('No BUILDING_PICKUP task exists before RETURN_TO_FINISH_SPOT task')
                return response

            # Replace the to_switch_task with the last BUILDING_PICKUP task before RETURN_TO_FINISH_SPOT
            to_switch_task_index = available_tasks.index(to_switch_task)
            last_building_pickup_task_index = available_tasks.index(last_building_pickup_task)
            available_tasks[to_switch_task_index] = last_building_pickup_task

            # Add the replaced task back to the list at the position of the last BUILDING_PICKUP task
            available_tasks[last_building_pickup_task_index] = to_switch_task

            self.get_logger().info(f'Task with id {to_switch_task_id} replaced the last BUILDING_PICKUP task before RETURN_TO_FINISH_SPOT task')
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
        display_available_tasks()
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
        self.current_task_subscriber = CurrentTaskSubscriber()


class CurrentTaskSubscriber(Node):
    def __init__(self):
        super().__init__('static_task_generator_current_task_subscriber')
        self.current_task_sub = self.create_subscription(CurrentTask, 'current_task', self.current_task_callback, 10) 
        self.get_logger().warn('Created current_task_subscriber')
    
    def current_task_callback(self, msg):
        global currently_doing_subtask
        global current_task_id
        current_task_id=msg.current_task_id
        currently_doing_subtask=msg.currently_doing_subtask 
        #print("current_task_id:",currently_doing_subtask)
        #print("currently_doing_subtask",currently_doing_subtask)


class EnemyRobotPositionSubscriber(Node):
    def __init__(self):
        super().__init__('static_task_generator_enemy_robot_position_subscriber')
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
        self.sub = self.create_subscription(String, 'robot_color', self.robot_color_callback, 10) 
        self.get_logger().warn('Created robot_color_subscriber:static_task_generator node')
        self.counter = 0
    def robot_color_callback(self, msg):
        global robot_color
        robot_color = msg.data
        if self.counter == 0:
            initialize_tasks()
            self.counter += 1
        #self.get_logger().info(f'Robot color: {robot_color}')
        
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
        current_position.x = msg.nav_current_x
        current_position.y = msg.nav_current_y
        current_position.theta = msg.nav_current_angle
        #self.get_logger().info(f'Current position: {current_position}')
        #print("robot_current_state", robot_current_state)
        #print("current_position.x",current_position.x)
        #print("current_position.y",current_position.y)
################### COMMUNICATION INIT FUNCTION #############

def communication_instances_init():

    global servers_instance
    global subscribers_instance
    subscribers_instance = Subscribers()
    servers_instance = Services()
    
############################# UTILITY FUNCTIONS #####################

def add_obstacles_to_obstacles_list(task: Task):
    global obstacle_positions
    global movable_obstacles_id4
    global movable_obstacles_id6
    global movable_obstacles_id2
    global movable_obstacles_id12
    global movable_obstacles_id8
    global movable_obstacles_id10
    global movable_obstacles_id24
    global movable_obstacles_id26
    global movable_obstacles_id22
    global movable_obstacles_id30
    global movable_obstacles_id28
    global movable_obstacles_id32    
    
    
    if task.id == 4:
        obstacle_positions.extend(movable_obstacles_id4)
    elif task.id == 6:
        obstacle_positions.extend(movable_obstacles_id6)
    elif task.id == 2:
        obstacle_positions.extend(movable_obstacles_id2)
    elif task.id == 12:
        obstacle_positions.extend(movable_obstacles_id12)
    elif task.id == 8:
        obstacle_positions.extend(movable_obstacles_id8)
    elif task.id == 10:
        obstacle_positions.extend(movable_obstacles_id10)
    elif task.id == 24:
        obstacle_positions.extend(movable_obstacles_id24)
    elif task.id == 26:
        obstacle_positions.extend(movable_obstacles_id26)
    elif task.id == 22:
        obstacle_positions.extend(movable_obstacles_id22)
    elif task.id == 30:
        obstacle_positions.extend(movable_obstacles_id30)
    elif task.id == 28:
        obstacle_positions.extend(movable_obstacles_id28)
    elif task.id == 32: 
        obstacle_positions.extend(movable_obstacles_id32)                    

def display_available_tasks():
        global available_tasks
        for task in available_tasks:
            print(f"Task ID: {task.id}")
            #print(f"Position: ({task.position.x}, {task.position.y}, {task.position.theta})")
            #print(f"Type: {task.type}")
            #print(f"Direction: {task.direction}")
            #print(f"Score: {task.score}")
            #print(f"Linear Velocity: {task.linear_velocity}")
            #print(f"Angular Velocity: {task.angular_velocity}")
            #print(f"Linear Acceleration: {task.linear_acceleration}")
            #print(f"Angular Acceleration: {task.angular_acceleration}")
            #print(f"Is with Platforms: {task.is_with_platforms}")
            #print("Subtasks:")
            """for subtask in task.subtask_list:
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
                    print(f"  Mechanism Subtask Type: {subtask.mechanism_subtask_type}")"""
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
    print("!!!!!!didn't find the equivalent task in availabele tasks list now returning the same None")    
    return None

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
    return None

def calculate_distance(obstacle1:Obstacle, obstacle2:Obstacle):
    #print("obstacle1.position.x",obstacle1.position.x)
    #print("obstacle1.position.y",obstacle1.position.y)
    #print("obstacle2.position.x",obstacle2.position.x)
    #print("obstacle2.position.y",obstacle2.position.y)
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

def initialize_tasks():
    global robot_color
    global available_tasks
    global true_tasks
    global obstacle_positions
    global blue_tasks
    global yellow_tasks
    global true_blue_tasks
    global true_yellow_tasks
    global fixed_inside_prohibted_zone_area_of_blue
    global fixed_inside_prohibted_zone_area_of_yellow
    if robot_color == "blue":
        obstacle_positions.extend(fixed_inside_prohibted_zone_area_of_yellow)
        available_tasks = blue_tasks
        true_tasks = true_blue_tasks
        print("initializing blue tasks")
    elif robot_color == "yellow":
        obstacle_positions.extend(fixed_inside_prohibted_zone_area_of_blue) 
        available_tasks = yellow_tasks
        true_tasks = true_yellow_tasks    
        print("initializing yellow tasks")
    else:
        print("Robot color is not defined")


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
    executor.add_node(subscribers_instance.my_robot_color)
    executor.add_node(servers_instance.get_map_state_service)
    executor.add_node(subscribers_instance.enemy_robot_position_subscriber)
    executor.add_node(subscribers_instance.my_robot_position_subscriber)
    executor.add_node(map_state_updater)
    executor.add_node(subscribers_instance.current_task_subscriber)
    executor.add_node(subscribers_instance.my_robot_position_subscriber)
    executor.add_node(servers_instance.delete_task_service)
    executor.add_node(servers_instance.get_true_tasks_service)
    executor.add_node(servers_instance.switch_task_service)
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
        subscribers_instance.my_robot_color.destroy_node()
        subscribers_instance.current_task_subscriber.destroy_node()
        servers_instance.switch_task_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()    
