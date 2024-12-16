import rclpy
from rclpy.node import Node
import smach

# Define the sub-state machine (SubStateMachine)
class SubStateMachine(smach.StateMachine):
    def __init__(self):
        super().__init__(outcomes=['sub_finished', 'sub_aborted'])
        with self:
            # Add sub-states
            smach.StateMachine.add('SUB_INIT', SubInitState(), transitions={'done': 'SUB_MOVE_ON'})
            smach.StateMachine.add('SUB_MOVE_ON', SubMoveOnState(), transitions={'reached': 'SUB_WAIT_TO_REACH'})
            smach.StateMachine.add('SUB_WAIT_TO_REACH', SubWaitToReachState(), transitions={'reached': 'SUB_DESTINATION_REACHED'})
            smach.StateMachine.add('SUB_DESTINATION_REACHED', SubDestinationReachedState(), transitions={'done': 'sub_finished'})

# Define the main state machine (MainStateMachine)
class MainStateMachine(smach.StateMachine):
    def __init__(self):
        super().__init__(outcomes=['main_end'])
        with self:
            # Add main states
            smach.StateMachine.add('MAIN_INIT', MainInitState(), transitions={'done': 'WAIT_FOR_SIGNAL'})
            smach.StateMachine.add('WAIT_FOR_SIGNAL', WaitForSignalState(), transitions={'signal_received': 'MAIN_MOVE_ON'})
            smach.StateMachine.add('MAIN_MOVE_ON', MainMoveOnState(), transitions={'reached': 'MAIN_WAIT_TO_REACH'})
            smach.StateMachine.add('MAIN_WAIT_TO_REACH', MainWaitToReachState(), transitions={'reached': 'MAIN_DESTINATION_REACHED'})
            smach.StateMachine.add('MAIN_DESTINATION_REACHED', MainDestinationReachedState(), transitions={'done': 'PICKING_COLUMNS'})

            # Nest SubStateMachine inside the main state machine
            smach.StateMachine.add('PICKING_COLUMNS', SubStateMachine(), transitions={'sub_finished': 'main_end', 'sub_aborted': 'main_end'})

# Define individual states for the sub-state machine
class SubInitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rclpy.logging.get_logger('SubInitState').info('SubStateMachine: Initializing...')
        return 'done'

class SubMoveOnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rclpy.logging.get_logger('SubMoveOnState').info('SubStateMachine: Moving on...')
        return 'reached'

class SubWaitToReachState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rclpy.logging.get_logger('SubWaitToReachState').info('SubStateMachine: Waiting to reach destination...')
        return 'reached'

class SubDestinationReachedState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rclpy.logging.get_logger('SubDestinationReachedState').info('SubStateMachine: Destination reached.')
        return 'done'

# Define individual states for the main state machine
class MainInitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rclpy.logging.get_logger('MainInitState').info('MainStateMachine: Initializing...')
        return 'done'

class WaitForSignalState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['signal_received'])

    def execute(self, userdata):
        rclpy.logging.get_logger('WaitForSignalState').info('MainStateMachine: Waiting for signal...')
        return 'signal_received'

class MainMoveOnState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rclpy.logging.get_logger('MainMoveOnState').info('MainStateMachine: Moving on...')
        return 'reached'

class MainWaitToReachState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached'])

    def execute(self, userdata):
        rclpy.logging.get_logger('MainWaitToReachState').info('MainStateMachine: Waiting to reach destination...')
        return 'reached'

class MainDestinationReachedState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rclpy.logging.get_logger('MainDestinationReachedState').info('MainStateMachine: Destination reached.')
        return 'done'

# Main ROS 2 Node
class SmachHierarchicalExampleNode(Node):
    def __init__(self):
        super().__init__('smach_hierarchical_example_node')
        self.get_logger().info('SMACH Hierarchical Example Node started.')

        # Create and execute the main state machine
        sm = MainStateMachine()
        outcome = sm.execute()
        self.get_logger().info(f'State machine finished with outcome: {outcome}')

# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = SmachHierarchicalExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
