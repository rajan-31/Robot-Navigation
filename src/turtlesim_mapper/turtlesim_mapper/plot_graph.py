#!/usr/bin/env python3
import rclpy
from turtlesim.msg import Pose 
from rclpy.node import Node
import datetime

class ListNode(Node):

    def __init__(self):
        super().__init__("news_receiver")
        self.get_logger().info("Reciever is up and running listening from /turtle1/pose")
        self.subscriber_ = self.create_subscription(Pose,"turtle1/pose",self.callback_robot_pose,10)

        self.timer = self.create_timer(0.25, self.timer_callback)
        self.file_path = "pose_data.txt"

    
    def callback_robot_pose(self,msg):
        current_time = datetime.datetime.now().timestamp()
        self.latest_pose = f"{current_time},{msg.x},{msg.y},{msg.theta},{msg.angular_velocity}\n"
        self.get_logger().info(self.latest_pose)
    
    def timer_callback(self):

        with open(self.file_path, "a") as file:
            file.write(str(self.latest_pose) + "\n")


def main(args = None):
    rclpy.init(args = args)
    node = ListNode()
    rclpy.spin(node)
    rclpy.shutdown()   

if __name__=="__main__":
    main()