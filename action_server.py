from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from nav2_msgs.action import NavigateToPose


import rclpy
import time
import math

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
            execute_callback=self.execute_callback
        )
        self.current_goal = None
        self.robot = robot
        self.get_logger().info("Ros2 action server created.")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a goal from a client")

        done = False
        result = NavigateToPose.Result()
        goal_pose = goal_handle.request.pose.pose
        self.get_logger().info("Goal received:")
        self.get_logger().info(str(goal_pose))
        self.current_goal = goal_handle
        self.robot.destiny = (goal_pose.position.x, goal_pose.position.y)

        rate = self.create_rate(2.0) # this should be used only in sigle threaded

        while not done and rclpy.ok():
            self.get_logger().info("Processing...")
            self.get_logger().info("Robot position: %f, %f" % (self.robot.pos[0], self.robot.pos[1]))
            if self.robot.arrived():
                self.get_logger().info("Yeah, the robot arrived.")
                self.current_goal.succeed()
                done = True
            else:
                self.get_logger().info("Moving robot towards destiny...")
                feedback = NavigateToPose.Feedback()
                distance = self.robot.distance_remaining()
                if distance:
                    feedback.distance_remaining = float(distance)
                self.current_goal.publish_feedback(feedback)
                self.robot.move_towards_destiny()
            time.sleep(1.5)

            goal_handle.execute()

        return result

def main():
    rclpy.init()
    robot = Robot()
    node = Ros2ActionServer(robot)

    goal = node.current_goal
    i = 0
    while i <= 150:
        rclpy.spin_once(node, timeout_sec=2)
        goal = node.current_goal
        if goal and not goal.is_active:
            feedback = NavigateToPose.Feedback()
            distance_remaining = robot.distance_remaining()
            if distance_remaining:
                feedback.distance_remaining = float(robot.distance_remaining())
            goal.publish_feedback(feedback)
            print('published feedback')
        i += 1
        time.sleep(1)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()