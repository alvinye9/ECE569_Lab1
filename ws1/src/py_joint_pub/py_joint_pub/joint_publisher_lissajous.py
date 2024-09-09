import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pandas as pd
import pkg_resources
import numpy as np

class JointPublisherLissajous(Node):

    def __init__(self):
        super().__init__('joint_publisher_lissajous')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # read the csv file
        csv_file = pkg_resources.resource_filename('py_joint_pub', '../resource/lissajous.csv')
        self.df = pd.read_csv(csv_file)
        self.df_length = len(self.df)
        print(self.df_length)

    def timer_callback(self):
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        print(self.df.iloc[self.i, 1:].tolist())
        msg.position = self.df.iloc[self.i, 1:].tolist()
        
        msg.velocity = []
        msg.effort = []
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing position: "{msg.position}"')
        
        self.i += 1
        self.i %= self.df_length # loop back to the beginning of the csv file


def main(args=None):
    rclpy.init(args=args)

    joint_publisher_lissajous_node = JointPublisherLissajous()

    rclpy.spin(joint_publisher_lissajous_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_publisher_lissajous_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()