import rclpy
from rclpy.node import Node
import numpy as np
import os

from duckietown_msgs.msg import WheelsCmdStamped

class RandomActionNode(Node):

    def __init__(self):
        super().__init__("random_action_node")

        self.vehicle = os.getenv("VEHICLE_NAME")
        topic = "/{}/wheels_driver_node/wheels_cmd".format(self.vehicle)

        self.vel_pub = self.create_publisher(WheelsCmdStamped, topic, 1)

        self.timer = self.create_timer(0.1, self._publish_random_action)
    
    
    def _publish_random_action(self):
        msg = WheelsCmdStamped()
        msg.vel_left = np.random.random()
        msg.vel_right = np.random.random()

        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomActionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()