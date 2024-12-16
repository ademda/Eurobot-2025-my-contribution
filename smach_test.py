import rclpy
from rclpy.node import Node
import smach

# Define StateOne
class StateOne(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_state_two', 'end'])
        self.counter = 0

    def execute(self, userdata):
        rclpy.logging.get_logger('StateOne').info('Executing StateOne')
        self.counter += 1

        if self.counter < 3:
            return 'to_state_two'
        else:
            return 'end'

# Define StateTwo
class StateTwo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_state_one', 'end'])

    def execute(self, userdata):
        rclpy.logging.get_logger('StateTwo').info('Executing StateTwo')
        return 'to_state_one'

# Main Node
class SmachExampleNode(Node):
    def __init__(self):
        super().__init__('smach_example_node')
        self.get_logger().info('SMACH Example Node started.')

        # Create a state machine
        sm = smach.StateMachine(outcomes=['end'])

        # Add states to the state machine
        with sm:
            smach.StateMachine.add('STATE_ONE', StateOne(), transitions={
                'to_state_two': 'STATE_TWO',
                'end': 'end'
            })
            smach.StateMachine.add('STATE_TWO', StateTwo(), transitions={
                'to_state_one': 'STATE_ONE',
                'end': 'end'
            })

        # Execute the state machine
        outcome = sm.execute()
        self.get_logger().info(f'State machine finished with outcome: {outcome}')

# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = SmachExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
