import rclpy
from rclpy.node import Node
from ros_custom_messages.srv import GetAvailableTasks
from enum import Enum
from ros_custom_messages.msg import TaskMsg
from ros_custom_messages.srv import GetNextTaskPoint
from strategy.static_task_generator import Task

class GetNextTaskPointClient(Node):
    def __init__(self):
        super().__init__('get_next_task_point_client')
        self.client = self.create_client(GetNextTaskPoint, 'get_next_task_point_service')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service is available, ready to send request!')

    def send_request(self, task_id):
        request = GetNextTaskPoint.Request()
        request.task_id = task_id
        self.get_logger().info(f'Sending request with task_id: {task_id}')
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    get_next_task_point_client = GetNextTaskPointClient()
    get_next_task_point_client.send_request(task_id=1)
    rclpy.spin(get_next_task_point_client)
    rclpy.shutdown()
