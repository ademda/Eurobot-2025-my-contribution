"""
Author: <Adem Daly>

Description:
    This code conations the state machine logic for the Master node.
    The state machine is implemented using the smach library.

Classes:
    - MainStateMachine: The main state machine class that contains all the states and transitions.
    - Idle: The Idle state class.
    - Init: The Init state class.
    - WaitForSignal: The WaitForSignal state class.
    - MoveOn: The MoveOn state class.
    - WaitToReach: The WaitToReach state class.
    - DestinationReached: The DestinationReached state class.
    - Building_PickUp_Init: The Building_PickUp_Init state class.
    - Building_PickUp_MoveOn: The Building_PickUp_MoveOn state class.
    - Building_PickUp_WaitToReach: The Building_PickUp_WaitToReach state class.
    - Building_PickUp_DestinationReached: The Building_PickUp_DestinationReached state class.
    - Building_DropDown_Init: The Building_DropDown_Init state class.
    - Building_DropDown_MoveOn: The Building_DropDown_MoveOn state class.
    - Building_DropDown_WaitToReach: The Building_DropDown_WaitToReach state class.
    - Building_DropDown_DestinationReached: The Building_DropDown_DestinationReached state class.
    - BlockageDetected: The BlockageDetected state class.
    - SuddenStop: The SuddenStop state class.
    - WaitSuddenStop: The WaitSuddenStop state class.
    - ConfirmedStop: The ConfirmedStop state class.
    - Finished: The Finished state class.

    Explanation of classes structure decision:
        -from Idle class to DestinationReached class: these classes are for navigation tasks.
        -from Building_PickUp_Init class to Building_DropDown_DestinationReached class: these 
        classes are for building tasks: either going to intermediate tasks points or commanding mechanisms
        -BlockageDetected class: this class is when the robot gets blocked.
        -From SuddenStop to ConfirmedStop:these classes are when the enemy robot is near and we need to stop our robot.
        -Finished class: this class is when the robot finishes all tasks or the allowed time ends.
"""

import rclpy
import smach
import threading
import time
import signal
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from enum import Enum
from geometry_msgs.msg import Pose2D
from ros_custom_messages.msg import MasterAcknowledgeSignal, MasterNavigationMessage, MasterResetSignal, MasterStopSignal, NavigatorNavigationMessage
from ros_custom_messages.srv import GetNextTaskPoint, GetNextSubTaskPoint
from strategy.static_task_generator import Task, TaskType ,Direction, TaskTypeScore, SpeedChoice
from std_msgs.msg import Int32

##################################### TYPEDEF ENUMS HERE ####################################

"""
Enum for the Navigator states:
    these states represent the states of the STM32 Navigator StateMachine used 
    that the Master keeps track of the STM32 current state(there are more states in the STM32
    chip but we reduced to only what we need to know)
"""

class MASTER_NavigatorStates_ten(Enum):

    MASTER_NavigatorIdle_en = 0
    MASTER_NavigatorReady_en = 1
    MASTER_NavigatorBusy_en = 2
    MASTER_NavigatorWaitToReceiveAcknowledge_en = 3
    MASTER_NavigatorReachedDest_en = 4
    MASTER_NavigatorConfirmedStop_en = 5
    MASTER_NavigatorInit_en = 6

"""
Enum for the Master states:
   these states represent the states of the Raspberry (Master) StateMachine used
   to keep track of the current state of the Master
"""

class MASTER_SMachineStates_ten(Enum):

    MASTER_SMachine_Idle_en = 0
    MASTER_SMachine_WaitForSignal_en = 1
    MASTER_SMachine_MoveOn_en = 2
    MASTER_SMachine_WaitToReach_en = 3
    MASTER_SMachine_DestinationReached_en = 4
    MASTER_SMachine_SuddenStop_en = 5
    MASTER_SMachine_ConfirmedStop_en = 6
    MASTER_SMachine_Finish_en = 7
    MASTER_SMachine_Building_PickUp_Init_en = 8
    MASTER_SMachine_Building_PickUp_MoveOn_en = 9
    MASTER_SMachine_Building_PickUp_WaitToReach_en = 10
    MASTER_SMachine_Building_PickUp_DestinationReached_en = 11
    MASTER_SMachine_Building_PickUp_Finish_en = 12
    MASTER_SMachine_Building_DropDown_Init_en = 13
    MASTER_SMachine_Building_DropDown_MoveOn_en = 14
    MASTER_SMachine_Building_DropDown_WaitToReach_en = 15
    MASTER_SMachine_Building_DropDown_DestinationReached_en = 16
    MASTER_SMachine_Building_DropDown_Finish_en = 17

"""
STM32 State: These states are the states that the can returns to the state machine
we use them to transition between states
"""

"""STM32 State:

    0:MASTER_NavigatorIdle_en,                        
    1:MASTER_NavigatorReady_en,                       
    2:MASTER_NavigatorBusy_en,                        
	3:MASTER_NavigatorWaitToReceiveAcknowledge_en,        
    4:MASTER_NavigatorReachedDest_en,                
	5:MASTER_NavigatorOrderSuddenStop_en,             
	6:MASTER_NavigatorSuddenStop_en,                  
	7:MASTER_NavigatorConfirmedStop_en,              
	8:MASTER_NavigatorInit_en,                        
	9:MASTER_NavigatorBlockageDetected_en,           
	10:MASTER_NavigatorWallBlockageDetected_en         
    """
##################################### GLOBAL VARIABLES ####################################
####GLOBAL VARIABLES#####

publishers_instance = None
subscribers_instance = None
clients_instance = None
servers_instance = None
current_position = None #Pose2D format
current_task = None
next_task = None
current_subtask_order = 0
next_subtask_order = 0
"""current_task = Task(id=-100,position=Pose2D(x=100.0,y=100.0,theta=90),
                    type=TaskType.NORMAL_NAV,direction=Direction.BACKWARD,
                    score=TaskTypeScore.BONDEROLE_POSE, linear_velocity=SpeedChoice.SLOW,
                    angular_velocity=SpeedChoice.SLOW,linear_acceleration=0,angular_acceleration=0,
                    subtask_list=[])"""
current_subtask = None
current_navigator_state = None
MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorIdle_en
MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Idle_en

init_position = MasterNavigationMessage(nav_target_x=100.0,
                                        nav_target_y=100.0,
                                        nav_target_angle=90.0,
                                        nav_is_forward=True,
                                        nav_linear_velocity_choice=0,
                                        nav_angular_velocity_choice=0,
                                        nav_linear_acceleration_choice=0,
                                        nav_angular_acceleration_choice=0,
                                        )
#should subscribe to game launcher insted in the future to get from it the initial position
########################

communication_wait_time = 0.2  #used to wait until the message is transmited to the Navigator

###################################### CLASSES #########################################






####################################### GENERAL STATE MACHINE CLASSES ################################################

class MainStateMachine(smach.StateMachine):
    def __init__(self):
        super().__init__(outcomes=['End'])
        with self:
            smach.StateMachine.add('Idle', Idle(),
                               transitions={'Go to: Init': 'Init',
                                            'Go to: Idle': 'Idle'})
        
            smach.StateMachine.add('Init', Init(),
                                transitions={'Go to: WaitForSignal': 'WaitForSignal',
                                            'Go to: Init': 'Init',})
                                                
            smach.StateMachine.add('WaitForSignal', WaitForSignal(),
                                transitions={'Go to: MoveOn': 'MoveOn',
                                             'Go to: WaitForSignal': 'WaitForSignal',})
                                                
            smach.StateMachine.add('MoveOn', MoveOn(),
                                transitions={'Go to: WaitToReach': 'WaitToReach',
                                             'Go to: MoveOn': 'MoveOn',})
                                                
            smach.StateMachine.add('WaitToReach', WaitToReach(),
                                transitions={'Go to: DestinationReached': 'DestinationReached',
                                            'Go to: BlockageDetected': 'BlockageDetected',
                                            'Go to: SuddenStop': 'SuddenStop',
                                            'Go to: WaitToReach': 'WaitToReach', })
                                                
            smach.StateMachine.add('BlockageDetected', BlockageDetected(),
                                transitions={ 'Go to: MoveOn': 'MoveOn',
                                            'Go to: BlockageDetected': 'BlockageDetected',})

            smach.StateMachine.add('SuddenStop', SuddenStop(),
                               transitions={'Go to: WaitSuddenStop': 'WaitSuddenStop',
                                            'Go to: SuddenStop': 'SuddenStop'
                                            })
        
            smach.StateMachine.add('WaitSuddenStop', WaitSuddenStop(),
                               transitions={'Go to: ConfirmedStop': 'ConfirmedStop',
                                            'Go to: WaitSuddenStop': 'WaitSuddenStop'
                                            })

            smach.StateMachine.add('ConfirmedStop', ConfirmedStop(),
                               transitions={'Go to: MoveOn': 'MoveOn',
                                            'Go to: ConfirmedStop': 'ConfirmedStop',})                                         
                                               
            smach.StateMachine.add('DestinationReached', DestinationReached(),
                                transitions={'Go to: MoveOn': 'MoveOn',
                                            'Go to: Finished': 'Finished',
                                            'Go to: Building_PickUp_Init': 'Building_PickUp_Init',
                                            'Go to: Building_DropDown_Init': 'Building_DropDown_Init',
                                            'Go to: DestinationReached': 'DestinationReached',})
                                            
            smach.StateMachine.add('Building_PickUp_Init',Building_PickUp_Init(),
                                transitions={'Go to: Building_PickUp_MoveOn': 'Building_PickUp_MoveOn',
                                            'Go to: Building_PickUp_Init': 'Building_PickUp_Init',})
            
            smach.StateMachine.add('Building_PickUp_MoveOn',Building_PickUp_MoveOn(),
                                transitions={'Go to: Building_PickUp_WaitToReach': 'Building_PickUp_WaitToReach',
                                            'Go to: Building_PickUp_MoveOn': 'Building_PickUp_MoveOn',
                                            'Go to: Building_PickUp_DestinationReached': 'Building_PickUp_DestinationReached',
                                            'Go to: MoveOn': 'MoveOn',})
            
            smach.StateMachine.add('Building_PickUp_WaitToReach',Building_PickUp_WaitToReach(),
                                transitions={'Go to: Building_PickUp_DestinationReached': 'Building_PickUp_DestinationReached',
                                             'Go to: SuddenStop': 'SuddenStop',
                                            'Go to: Building_PickUp_WaitToReach': 'Building_PickUp_WaitToReach',})

            smach.StateMachine.add('Building_PickUp_DestinationReached',Building_PickUp_DestinationReached(),
                                transitions={'Go to: Building_PickUp_MoveOn': 'Building_PickUp_MoveOn',
                                            'Go to: Building_PickUp_DestinationReached': 'Building_PickUp_DestinationReached',})

            smach.StateMachine.add('Building_DropDown_Init',Building_DropDown_Init(),
                                transitions={'Go to: Building_DropDown_MoveOn': 'Building_DropDown_MoveOn',
                                            'Go to: Building_DropDown_Init': 'Building_DropDown_Init',})
            
            smach.StateMachine.add('Building_DropDown_MoveOn',Building_DropDown_MoveOn(),
                                transitions={'Go to: Building_DropDown_WaitToReach': 'Building_DropDown_WaitToReach',
                                            'Go to: Building_DropDown_MoveOn': 'Building_DropDown_MoveOn',
                                            'Go to: Building_DropDown_DestinationReached': 'Building_DropDown_DestinationReached',
                                            'Go to: MoveOn': 'MoveOn',})
            
            smach.StateMachine.add('Building_DropDown_WaitToReach',Building_DropDown_WaitToReach(),
                                transitions={'Go to: Building_DropDown_DestinationReached': 'Building_DropDown_DestinationReached',
                                            'Go to: Building_DropDown_WaitToReach': 'Building_DropDown_WaitToReach',
                                            'Go to: SuddenStop': 'SuddenStop'})
            
            smach.StateMachine.add('Building_DropDown_DestinationReached',Building_DropDown_DestinationReached(),
                                transitions={'Go to: Building_DropDown_MoveOn': 'Building_DropDown_MoveOn',
                                            'Go to: Building_DropDown_DestinationReached': 'Building_DropDown_DestinationReached',})

            smach.StateMachine.add('Finished', Finished(),
                                transitions={'Go to: Finished': 'Finished'})                                                    

############################################ EACH STATE MACHINE CLASS ###############################################

class Idle(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Init',
                                             'Go to: Idle'])
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        signal.signal(signal.SIGINT, exit_handler)
        send_acknowledge_signal()
        time.sleep(communication_wait_time)
        if(current_navigator_state == 8):
            MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
            MASTER_SMachineCurrentState_en == MASTER_SMachineStates_ten.MASTER_SMachine_WaitForSignal_en
            return 'Go to: Init'
        else:
            return 'Go to: Idle'
        

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: WaitForSignal',
                                             'Go to: Init'])
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global init_position
        global communication_wait_time
        signal.signal(signal.SIGINT, exit_handler)
        send_target_order(init_position)
        time.sleep(communication_wait_time)

        if(current_navigator_state == 1):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_WaitForSignal_en
            return 'Go to: WaitForSignal'
        else:
            return 'Go to: Init'        


class WaitForSignal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: MoveOn',
                                             'Go to: WaitForSignal'])
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(0.1)
        if(MASTER_CurrentNavigatorState_en == MASTER_NavigatorStates_ten.MASTER_NavigatorReady_en):
            #time.sleep(1) #Maybe i need to add the sleep function 
            return 'Go to: MoveOn'
        else:
            return 'Go to: WaitForSignal'   
        
class MoveOn(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: WaitToReach',
                                             'Go to: MoveOn'])
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        global current_task
        global next_task
        global current_subtask_order
        global next_subtask_order
        current_subtask_order = 0
        next_subtask_order = 0
        signal.signal(signal.SIGINT, exit_handler)
        get_next_task_point()
        print("Nav Roder Details:")
        #print(f"nav_target_x: {current_task.position.x}")
        #print(f"nav_target_y: {current_task.position.y}")
        #print(f"nav_target_angle: {current_task.position.theta}")
        print(f"nav_is_forward: {next_task.direction}")
        print(f"nav_linear_velocity_choice: {next_task.linear_velocity}")
        print(f"nav_angular_velocity_choice: {next_task.angular_velocity}")
        print(f"nav_linear_acceleration_choice: {next_task.linear_acceleration}")
        print(f"nav_angular_acceleration_choice: {next_task.angular_acceleration}")
        nav_order = convert_next_task_to_nav_order(next_task)
        send_target_order(nav_order)
        time.sleep(communication_wait_time)
        if(current_navigator_state == 9 or current_navigator_state == 10 or current_navigator_state == 7 ):
            send_acknowledge_signal()
            time.sleep(communication_wait_time)
        elif (current_navigator_state == 2):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorBusy_en
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_WaitToReach_en
            return 'Go to: WaitToReach'
        else:
            return 'Go to: MoveOn'
        """
        Maybe i need to add
        elif (current_navigator_state == 3):
            send_acknowledge_signal()
            time.sleep(communication_wait_time)  # Use time.sleep in ROS 2
            return 'Go to: MoveOn'
        """
        
class WaitToReach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: DestinationReached',
                                             'Go to: BlockageDetected',
                                             'Go to: SuddenStop',
                                             'Go to: WaitToReach'])
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global current_task
        global next_task
        current_task = next_task
        signal.signal(signal.SIGINT, exit_handler)
        # Maybe i need to add time.sleep(0.05)

        if(current_navigator_state == 3):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorWaitToReceiveAcknowledge_en
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_DestinationReached_en
            return 'Go to: DestinationReached'
        elif ( current_navigator_state == 9 or current_navigator_state == 10 ):

            return 'Go to: BlockageDetected'
        else:
            return 'Go to: WaitToReach'         
        #should handle sudden stop state transition
        
    
class DestinationReached(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: MoveOn',
                                             'Go to: Finished',
                                             'Go to: Building_PickUp_Init',
                                             'Go to: Building_DropDown_Init',
                                             'Go to: DestinationReached']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        global current_task
        signal.signal(signal.SIGINT, exit_handler)
        send_acknowledge_signal()
        time.sleep(communication_wait_time)

        if(current_navigator_state == 1):
            if (current_task.type == TaskType.RETURN_TO_FINISH_SPOT.value):
                MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Finish_en
                return 'Go to: Finished' 
            elif (current_task.type == TaskType.BUILDING_PICKUP.value):
                MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_PickUp_Init_en
                return 'Go to: Building_PickUp_Init'
            elif (current_task.type == TaskType.BUILDING_DROPDOWN.value):
                MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_DropDown_Init_en
                return 'Go to: Building_DropDown_Init'    
            else:
                MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_MoveOn_en
                return 'Go to: MoveOn'
                # i think i need to add a bonderole Pose state alone    
                # to isolate it's blockage detected edge case
                # here the bonderole pose has the same comportement as 
                # Normal Nav
        else:
            return 'Go to: DestinationReached' 
        
class Building_PickUp_Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_PickUp_MoveOn',
                                             'Go to: Building_PickUp_Init']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        #here i should initialize the mechanism if it is not ready
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(0.05)

        if(True):
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_PickUp_Init_en
            return 'Go to: Building_PickUp_MoveOn'
        else:
            return 'Go to: Building_PickUp_Init'

class Building_PickUp_MoveOn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_PickUp_WaitToReach',
                                             'Go to: Building_PickUp_MoveOn',
                                             'Go to: Building_PickUp_DestinationReached',
                                             'Go to: MoveOn']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        global current_subtask_order
        global next_subtask_order
        signal.signal(signal.SIGINT, exit_handler)
        response = get_next_subtask()
        if (response.current_subtask_order == -1):
            """
            the subtask generator node returns -1 if all subtasks are done
            so we need to return to normal navigation
            """
            return 'Go to: MoveOn'
        elif(response.nav_target_x == 0.0 and response.nav_target_y == 0.0):
            """
            here if the subtask is a mechanism command.the subtask generator node will return
            an absurd positiion(0.0) so we know we will comamnd a mechanism
            """
            wait_mechanism_mouvement()
            #the wait for mechanism to end his function should block until the mechanism is done
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_PickUp_DestinationReached_en
            next_subtask_order = current_subtask_order + 1 
            return 'Go to: Building_PickUp_DestinationReached'
        
        elif(response.nav_target_x != 0.0 or response.nav_target_y != 0.0): 
            nav_order = convert_next_subtask_to_nav_order(response)
            send_target_order(nav_order)
            time.sleep(communication_wait_time)  
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_PickUp_WaitToReach_en
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorBusy_en
            next_subtask_order = current_subtask_order + 1 
            return 'Go to: Building_PickUp_WaitToReach'
                    
        else:
            return 'Go to: Building_PickUp_MoveOn'            
    
class Building_PickUp_WaitToReach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_PickUp_DestinationReached',
                                             'Go to: Building_PickUp_WaitToReach',
                                             'Go to: SuddenStop']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        time.sleep(3)
        signal.signal(signal.SIGINT, exit_handler)
        if(current_navigator_state == 3):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorWaitToReceiveAcknowledge_en
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_PickUp_DestinationReached_en
            return 'Go to: Building_PickUp_DestinationReached'
        else:
            return 'Go to: Building_PickUp_WaitToReach'
    #should handle sudden stop state transition

    
class Building_PickUp_DestinationReached(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_PickUp_MoveOn',
                                             'Go to: Building_PickUp_DestinationReached']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        global current_subtask_order
        global next_subtask_order
        current_subtask_order = next_subtask_order
        time.sleep(3)#CAAAAAAAAAAAAAAAAAAAUUUUUTION
        signal.signal(signal.SIGINT, exit_handler)
        send_acknowledge_signal()
        time.sleep(communication_wait_time)
        if(current_navigator_state == 1):
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_PickUp_MoveOn_en

            return 'Go to: Building_PickUp_MoveOn'
        else:
            return 'Go to: Building_PickUp_DestinationReached'    


class Building_DropDown_Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_DropDown_MoveOn',
                                             'Go to: Building_DropDown_Init']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        #here i should initialize the mechanism if it is not ready
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(0.05)

        if(True):
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_DropDown_Init_en
            return 'Go to: Building_DropDown_MoveOn'
        else:
            return 'Go to: Building_DropDown_Init'

class Building_DropDown_MoveOn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_DropDown_WaitToReach',
                                             'Go to: Building_DropDown_MoveOn',
                                             'Go to: Building_DropDown_DestinationReached',
                                             'Go to: MoveOn']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        global current_subtask_order
        global next_subtask_order
        signal.signal(signal.SIGINT, exit_handler)
        response = get_next_subtask()
        if (response.current_subtask_order == -1):
            return 'Go to: MoveOn'
        
        elif(response.nav_target_x == 0.0 and response.nav_target_y == 0.0):
            wait_mechanism_mouvement()
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_DropDown_DestinationReached_en
            next_subtask_order = current_subtask_order + 1 
            return 'Go to: Building_DropDown_DestinationReached'
        
        elif(response.nav_target_x != 0.0 or response.nav_target_y != 0.0): 
            nav_order = convert_next_subtask_to_nav_order(response)
            send_target_order(nav_order)
            time.sleep(communication_wait_time)  
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_DropDown_WaitToReach_en
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorBusy_en
            next_subtask_order = current_subtask_order + 1 
            return 'Go to: Building_DropDown_WaitToReach'
                    
        else:
            return 'Go to: Building_DropDown_MoveOn'            
    
class Building_DropDown_WaitToReach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_DropDown_DestinationReached',
                                             'Go to: Building_DropDown_WaitToReach',
                                             'Go to: SuddenStop']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        time.sleep(3)
        signal.signal(signal.SIGINT, exit_handler)
        if(current_navigator_state == 3):
            MASTER_CurrentNavigatorState_en = MASTER_NavigatorStates_ten.MASTER_NavigatorWaitToReceiveAcknowledge_en
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_PickUp_DestinationReached_en
            return 'Go to: Building_DropDown_DestinationReached'
        else:
            return 'Go to: Building_DropDown_WaitToReach'
    #should handle sudden stop state transition

    
class Building_DropDown_DestinationReached(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Building_DropDown_MoveOn',
                                             'Go to: Building_DropDown_DestinationReached']) 
    
    def execute(self, userdata):
        global current_navigator_state #returned as a msg
        global MASTER_CurrentNavigatorState_en
        global MASTER_SMachineCurrentState_en
        global communication_wait_time
        global current_subtask_order
        global next_subtask_order
        current_subtask_order = next_subtask_order
        time.sleep(3)#CAAAAAAAAAAAAAAAAAAAUUUUUTION
        signal.signal(signal.SIGINT, exit_handler)
        send_acknowledge_signal()
        time.sleep(communication_wait_time)
        if(current_navigator_state == 1):
            MASTER_SMachineCurrentState_en = MASTER_SMachineStates_ten.MASTER_SMachine_Building_DropDown_MoveOn_en

            return 'Go to: Building_DropDown_MoveOn'
        else:
            return 'Go to: Building_DropDown_DestinationReached'    

        
class BlockageDetected(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: MoveOn',
                                             'Go to: BlockageDetected']) 
    
    def execute(self, userdata):
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(1)  #()
        if(True):
            return 'Go to: MoveOn'
        else:
            return 'Go to: BlockageDetected'  


class SuddenStop(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: WaitSuddenStop',
                                             'Go to: SuddenStop']) 
    
    def execute(self, userdata):
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(1)  #()
        if(True):
            return 'Go to: WaitSuddenStop'
        else:
            return 'Go to: SuddenStop'


class WaitSuddenStop(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: ConfirmedStop',
                                             'Go to: WaitSuddenStop']) 
    
    def execute(self, userdata):
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(1)  #()
        if(True):
            return 'Go to: ConfirmedStop'
        else:
            return 'Go to: WaitSuddenStop'


class ConfirmedStop(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: MoveOn',
                                             'Go to: ConfirmedStop']) 
    
    def execute(self, userdata):
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(1)  #()
        if(True):
            return 'Go to: MoveOn'
        else:
            return 'Go to: ConfirmedStop'


class Finished(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['Go to: Finished']) 
    
    def execute(self, userdata):
        signal.signal(signal.SIGINT, exit_handler)
        time.sleep(1)  #()
        if(True):
            return 'Go to: Finished'
        

########################################## PUBLISHERS CLASS ###############################################

class Publishers(Node):

    def __init__(self):
        super().__init__('publishers')

        # Create publishers as attributes
        self.Navigator_Acknowledge_Publisher = self.create_publisher(MasterAcknowledgeSignal, 'ros_can_interface/master_acknowledge', 10)
        self.Navigator_Target_Publisher = self.create_publisher(MasterNavigationMessage, 'ros_can_interface/master_nav', 10)
        self.Navigator_Stop_Publisher = self.create_publisher(MasterStopSignal, 'ros_can_interface/stop_signal', 10)
        self.Navigator_Reset_Publisher = self.create_publisher(MasterResetSignal, 'ros_can_interface/master_reset', 10)

        self.get_logger().info(f"Initialised all publishers ")    


########################################## SUBSCRIBERS CLASS ###############################################

class Subscribers(Node):
    
    def __init__(self):
        super().__init__('subscribers')
        self.Navigator_Receive_Position_Subscriber=self.create_subscription(NavigatorNavigationMessage, 'ros_can_interface/navigator_nav',
                                                                            self.Navigator_Receive_Position_Subscriber_callback, 10)
        self.get_logger().info(f"Initialised all subscribers ")   
        

    def Navigator_Receive_Position_Subscriber_callback(self ,nav_data: NavigatorNavigationMessage):
        global current_position
        global current_navigator_state
        self.get_logger().info(f"Received: navigation data")    
        current_position = Pose2D(x=nav_data.nav_current_x, y=nav_data.nav_current_y, theta=nav_data.nav_current_angle)
        current_navigator_state = nav_data.nav_current_state
    
    

#for testing without the stm32 returning the real robot state
class NavigatorStateSubscriber(Node):

    def __init__(self):
        super().__init__('navigator_state_subscriber')
        self.nav_current_state_simulation = self.create_subscription(Int32,'simulation_navigator_state',
                                                     self.navigator_state_callback,10)#for testing 

        self.get_logger().info('NavigatorStateSubscriber node has been started.')

    def navigator_state_callback(self, msg):
        
        global current_navigator_state
        self.get_logger().info('NavigatorStateSubscriber node has been started.')
        # Assign the received value to the global variable
        current_navigator_state = msg.data       

       
########################################## CLIENTS CLASSES ##############################################

class Clients():

    global clients_instance

    def __init__(self):
        self.get_next_task_point_client = GetNextTaskPointClient()
        self.get_next_subtask_client = GetNextSubTaskPointClient()
        

class GetNextTaskPointClient(Node):

    def __init__(self):
        super().__init__('get_next_task_point_client')
        self.client = self.create_client(GetNextTaskPoint, 'get_next_task_point_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get_Next_Task_Point_Service not available, waiting again...')
        self.get_logger().warn('GetNextTaskPoint Service is available, ready to send request!')

    def send_request(self):
        global current_task
        global next_task
        request = GetNextTaskPoint.Request()  # Create a request object
        
        self.get_logger().info('Sending request for available tasks...')
        if (current_task is None):
            #here send -100 so the strategy node knows that the current_stask = Nonde: we start the match now
            request.task_id = -100
        # Send the request and block until a response is received
        else:
            request.nav_target_x = current_task.position.x
            request.nav_target_y = current_task.position.y
            request.nav_target_angle = current_task.position.theta
            request.task_id = current_task.id
            request.task_type = current_task.type
            print("Request Details:")
            print(f"nav_target_x: {request.nav_target_x}")
            print(f"nav_target_y: {request.nav_target_y}")
            print(f"nav_target_angle: {request.nav_target_angle}")
            print(f"task_id: {request.task_id}")
            print(f"task_type: {request.task_type}")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)  # Wait until the service call is completed

        if future.done():
            try:
                response = future.result()  # Get the response
                print("Response Details:")
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
                # Update current_task with the received data
                next_task = Task(
                    id=response.task_id,
                    position=Pose2D(x=response.nav_target_x, y=response.nav_target_y, theta=response.nav_target_angle),
                    type=response.task_type,
                    direction=response.nav_is_forward,
                    score=response.score,
                    linear_velocity=response.nav_linear_velocity_choice,
                    angular_velocity=response.nav_angular_velocity_choice,
                    linear_acceleration=response.nav_linear_acceleration_choice,
                    angular_acceleration=response.nav_angular_acceleration_choice,
                    subtask_list=[]
                )
                return next_task
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return None


class GetNextSubTaskPointClient(Node):

    def __init__(self):
        super().__init__('get_next_subtask_client')
        self.client = self.create_client(GetNextSubTaskPoint, 'get_next_subtask')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().warn('GetNextSubTaskPoint Service is available, ready to send request!')

    def send_request(self):
        global current_subtask_order
        global current_task
        request = GetNextSubTaskPoint.Request()  # Create a request object
        request.current_subtask_order = current_subtask_order
        request.current_task_id = current_task.id
        self.get_logger().info('Sending request for available tasks...')
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)  # Wait until the service call is completed

        if self.future.done():
            try:
                response = self.future.result()
                return response
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return None      



########################################## UTILITY FUNCTIONS ############################################

def convert_next_task_to_nav_order(current_task: Task):
    nav_order = MasterNavigationMessage()
    nav_order.nav_target_x = current_task.position.x
    nav_order.nav_target_y = current_task.position.y
    nav_order.nav_target_angle = current_task.position.theta  
    nav_order.nav_is_forward = current_task.direction
    nav_order.nav_linear_velocity_choice = current_task.linear_velocity
    nav_order.nav_angular_velocity_choice = current_task.angular_velocity
    nav_order.nav_linear_acceleration_choice = current_task.linear_acceleration
    nav_order.nav_angular_acceleration_choice = current_task.angular_acceleration

    return nav_order

def convert_next_subtask_to_nav_order(current_subtask_msg: GetNextSubTaskPoint.Response):
    nav_order = MasterNavigationMessage()
    nav_order.nav_target_x = current_subtask_msg.nav_target_x
    nav_order.nav_target_y = current_subtask_msg.nav_target_y
    nav_order.nav_target_angle = current_subtask_msg.nav_target_angle
    nav_order.nav_is_forward = current_subtask_msg.nav_is_forward
    nav_order.nav_linear_velocity_choice = current_subtask_msg.nav_linear_velocity_choice
    nav_order.nav_angular_velocity_choice = current_subtask_msg.nav_angular_velocity_choice
    nav_order.nav_linear_acceleration_choice = current_subtask_msg.nav_linear_acceleration_choice
    nav_order.nav_angular_acceleration_choice = current_subtask_msg.nav_angular_acceleration_choice

    return nav_order

def wait_mechanism_mouvement():
    time.sleep(5)
    #here i can block until the subtask generator node returns publishes that the mechanism ended his action
    #than i go to destination reached
    #should implement wait time for each mechanism mouvement type

def communication_instances_init():
    global publishers_instance
    global subscribers_instance
    global clients_instance
    global servers_instance

    publishers_instance = Publishers()
    subscribers_instance= Subscribers()
    clients_instance = Clients()

########################################## SUBTASK FUNCTION  ############################################

def get_next_subtask():
    global clients_instance
    response = clients_instance.get_next_subtask_client.send_request()
    """
    response type:
    int32 current_subtask_order
    float32 nav_target_x
    float32 nav_target_y
    float32 nav_target_angle
    bool nav_is_forward
    int8 nav_linear_velocity_choice
    int8 nav_angular_velocity_choice
    """
    """
    the form i want to use :
    float32 nav_target_x
    float32 nav_target_y
    float32 nav_target_angle
    bool nav_is_forward
    int8 nav_linear_velocity_choice
    int8 nav_angular_velocity_choice
    int8 nav_linear_acceleration_choice
    int8 nav_angular_acceleration_choice

    """
    return response 

########################################## COMMUNICATION FUNCTIONS ############################################

def get_next_task_point():
    global clients_instance
    clients_instance.get_next_task_point_client.send_request()

def send_target_order(nav_order):
    global publishers_instance
    publishers_instance.Navigator_Target_Publisher.publish(nav_order)

def send_reset_signal():
    global publishers_instance
    ResetSignal = MasterResetSignal()
    ResetSignal.master_reset_signal = True
    publishers_instance.Navigator_Reset_Publisher.publish(ResetSignal)

def send_stop_signal():
    global publishers_instance
    StopSignal = MasterStopSignal()
    StopSignal.stop_signal_state = True
    publishers_instance.Navigator_Stop_Publisher.publish(StopSignal)

def send_acknowledge_signal():
    global publishers_instance
    AcknowledgeSignal = MasterAcknowledgeSignal()
    AcknowledgeSignal.master_acknowledge_signal = True
    publishers_instance.Navigator_Acknowledge_Publisher.publish(AcknowledgeSignal)

########################################## SPECIAL EXIT HANDLING ###########################################

def exit_handler(sig, frame) -> None:
    print('Ctrl+C pressed. Exiting...')
    rclpy.shutdown()

########################################## MAIN FUNCTION ####################################################

def main(args=None):
    global clients_instance
    global subscribers_instance

    # Initialize ROS 2
    rclpy.init(args=args)
    communication_instances_init()

    # Create all nodes
    current_nav_state_sub = NavigatorStateSubscriber()

    # Use MultiThreadedExecutor to spin all nodes concurrently
    executor = MultiThreadedExecutor()

    # Add nodes to the executor
    executor.add_node(clients_instance.get_next_task_point_client)
    executor.add_node(clients_instance.get_next_subtask_client)
    executor.add_node(subscribers_instance)
    executor.add_node(current_nav_state_sub)

    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, exit_handler)

    # Spin the executor in a separate thread to handle callbacks
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    try:
        # Run the state machine logic here in the main thread
        sm = MainStateMachine()
        sm.execute()  # Execute the state machine in the main thread

    except KeyboardInterrupt:
        print("Shutting down...")

    finally:
        # Clean up and destroy all nodes when done
        clients_instance.get_next_task_point_client.destroy_node()
        clients_instance.get_next_subtask_client.destroy_node()
        subscribers_instance.destroy_node()
        current_nav_state_sub.destroy_node()

        # Shutdown ROS 2
        rclpy.shutdown()

        # Wait for the executor's spin thread to finish
        spin_thread.join()

if __name__ == '__main__':
    main()

