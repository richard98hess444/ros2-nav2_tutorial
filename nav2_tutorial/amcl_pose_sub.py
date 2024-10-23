import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class AmclPoseSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_pose_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.pose_callback,
            1)
        self.subscription  # prevent unused variable warning

    def pose_callback(self, data: TFMessage) -> None:
        a = data.transforms
        print(a)


def main(args=None):
    rclpy.init(args=args)
    amcl_subscriber = AmclPoseSubscriber()
    rclpy.spin(amcl_subscriber)
    amcl_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()