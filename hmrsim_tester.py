from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

import time


class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('Ros2ActionClient')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.goal_handle = goal_handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        msg = 'Received feedback'
        msg += '\nDistance remaining: {0}'.format(feedback.feedback.distance_remaining)
        msg += '\nPosition x: {0}, y: {1}'.format(feedback.feedback.current_pose.pose.position.x, feedback.feedback.current_pose.pose.position.y)
        self.get_logger().info(msg)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))


    def send_goal(self, name, x, y):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = name
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def cancelTask(self):
        self.get_logger().info('Canceling current task.')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.on_cancel)
        rclpy.spin_until_future_complete(self, future)

    def on_cancel(self, future):
        if not future.done():
            self.get_logger().info('Cancel request was rejected!')
        else:
            self.get_logger().info('Cancel request was accepted!')


def test_cancelation_and_new_goal(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()
    action_client.send_goal('wall-e', 250.0, 65.0)

    start_time = time.time()

    time_to_cancel = 20
    cancel_sent = False
    time_to_send_new_goal = 23
    new_goal_sent = False
    while True:
        if time.time()-start_time > time_to_cancel and not cancel_sent:
            print("Sending cancelation...")
            action_client.cancelTask()
            cancel_sent = True
        if time.time()-start_time > time_to_send_new_goal and not new_goal_sent:
            print("Sending new goal...")
            action_client.send_goal('wall-e', 70.0, 180.0)
            new_goal_sent = True
        rclpy.spin_once(action_client, timeout_sec=1)

def test_new_goal_with_another_running():
    rclpy.init()

    action_client = MinimalActionClient()
    action_client.send_goal('wall-e', 250.0, 65.0)

    start_time = time.time()

    time_to_send_new_goal = 20
    new_goal_sent = False
    while True:
        if time.time()-start_time > time_to_send_new_goal and not new_goal_sent:
            print("Sending new goal...")
            action_client.send_goal('wall-e', 70.0, 180.0)
            new_goal_sent = True
        rclpy.spin_once(action_client, timeout_sec=1)


if __name__ == '__main__':
    test_cancelation_and_new_goal()