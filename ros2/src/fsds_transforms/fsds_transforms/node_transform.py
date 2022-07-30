
# import ROS2 libraries
import rclpy
from rclpy.node import Node
# import ROS2 message libraries
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
# import tf2
from tf2_ros import TransformBroadcaster


class TF2Publisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # callback function on each message
        self.create_subscription(Odometry, "/testing_only/odom", self.callback, 1)


    def callback(self, msg: Odometry):
        pose = msg.pose.pose

        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'fsds/FSCar'

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        # Send the transformation
        self.br.sendTransform(t)


def main():
    # begin ros node
    rclpy.init()

    node = TF2Publisher()
    rclpy.spin(node)
    
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()