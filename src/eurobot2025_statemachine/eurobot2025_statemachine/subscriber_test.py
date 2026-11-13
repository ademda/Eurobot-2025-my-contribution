import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Creating a publisher for an integer message type
        self.publisher = self.create_publisher(Int32, 'my_topic', 10)
        
        # Creating a publisher for a Pose2D message type
        self.pose_publisher = self.create_publisher(Pose2D, 'pose_topic', 10)

        # Creating a subscriber for an integer message type
        self.int_subscriber = self.create_subscription(
            Int32, 'another_topic', self.int_callback, 10
        )
        
        # Creating a subscriber for a Pose2D message type
        self.pose_subscriber = self.create_subscription(
            Pose2D, 'pose_input_topic', self.pose_callback, 10
        )

        self.get_logger().info("Node has been started")

    # Callback for receiving integer messages
    def int_callback(self, msg):
        self.get_logger().info(f"Received integer: {msg.data}")
        # Publishing a message in response
        self.publisher.publish(Int32(data=msg.data + 1))

    # Callback for receiving Pose2D messages
    def pose_callback(self, msg):
        self.get_logger().info(f"Received Pose: x={msg.x}, y={msg.y}, theta={msg.theta}")
        # Publishing a pose message in response
        self.pose_publisher.publish(Pose2D(x=msg.x + 1, y=msg.y + 1, theta=msg.theta))

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = MyNode()

    # Spin the node to keep it running
    rclpy.spin(node)

    # Shut down the node when finished
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
