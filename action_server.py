from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalEvent
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup


import rclpy
import time
import math

from logging import getLogger
import logging

class Robot(object):

    def __init__(self) -> None:
        self.pos = [0.0, 0.0] # (x, y)
        self.velocity = 3.0
        self.acceptable_distance = 3.2
        self.destiny = None
    
    def move_towards_destiny(self):
        if not self.destiny:
            return
        
        if self.arrived():
            return
        
        if self.distance_remaining() > self.acceptable_distance:
            for i in range(len(self.pos)):
                forward = self.pos[i]-self.destiny[i] < 0
                if forward:
                    self.pos[i] += self.velocity
                else:
                    self.pos[i] -= self.velocity
    
    def arrived(self):
        return self.distance_remaining() <= self.acceptable_distance

    def distance_remaining(self):
        if self.destiny:
            x = self.pos[0]-self.destiny[0]
            y = self.pos[1]-self.destiny[1]
            return math.sqrt(x**2 + y**2)

class Ros2ActionServer(Node):

    def __init__(self, robot: Robot) -> None:
        super(Ros2ActionServer, self).__init__("Ros2ActionServer")

        self.action_server = ActionServer(
            self,
            action_name="nav_to_pose_test",
            action_type=NavigateToPose,
            execute_callback=self.execute_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )
        self.current_goal = None
        self.robot = robot
        self.get_logger().info("Ros2 action server created.")
        self.goal_qeue = []
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        self.goal_qeue.append(goal_handle)
        goal_pose = goal_handle.request.pose.pose
        self.get_logger().info("Received a goal from a client: " + str(goal_pose))
        self.get_logger().info("Setting goal...")
        self.current_goal = goal_handle
        self.robot.destiny = (goal_pose.position.x, goal_pose.position.y)
        self.get_logger().info('Deferring execution...')

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Robot final position: %f, %f" % (self.robot.pos[0], self.robot.pos[1]))
        result = NavigateToPose.Result()
        return result

class RobotController(object):

    def __init__(self, robot: Robot, goal_handle: ServerGoalHandle):
        self.robot = robot
        self.goal_handle = goal_handle

    def move_towards_destiny(self):
        if self.robot.destiny:
            if self.robot.arrived() and self.goal_handle.is_active:
                getLogger(self.__class__.__name__).info("Yeah, the robot arrived.")
                self.goal_handle.execute(self.execute_callback)
                return
            getLogger(self.__class__.__name__).info("moving robot if necessary...")
            self.robot.move_towards_destiny()
            getLogger(self.__class__.__name__).info("Robot position: " + str(self.robot.pos[0]) + ", " + str(self.robot.pos[1]))
            if self.goal_handle:
                feedback = NavigateToPose.Feedback()
                distance = self.robot.distance_remaining()
                if distance:
                    feedback.distance_remaining = float(distance)
                self.goal_handle.publish_feedback(feedback)
                getLogger(self.__class__.__name__).info("Published feedback...")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        getLogger(self.__class__.__name__).info("Robot final position: %f, %f" % (self.robot.pos[0], self.robot.pos[1]))
        if self.robot.arrived():
            self.robot.destiny = None
            goal_handle.succeed()
        result = NavigateToPose.Result()
        return result

def main():
    logging.basicConfig()
    logging.root.setLevel(logging.DEBUG)
    rclpy.init()
    robot = Robot()
    node = Ros2ActionServer(robot)
    goal = node.current_goal
    controller = RobotController(robot, goal)

    i = 0
    while i <= 50:
        getLogger(__name__).info("Iteraction " + str(i))
        rclpy.spin_once(node, timeout_sec=2)
        goal = node.current_goal
        controller.goal_handle = goal
        controller.move_towards_destiny()
        i += 1
        time.sleep(1)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()