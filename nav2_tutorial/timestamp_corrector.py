import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class TimestampCorrector(Node):
    def __init__(self):
        super().__init__('timestamp_corrector')
        
        # Subscribe to the original PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/robot0/point_cloud2',  # Replace with the actual radar topic
            self.listener_callback,
            5)
        
        # Publisher for the corrected PointCloud2 topic
        self.publisher = self.create_publisher(
            PointCloud2,
            '/robot0/timestamp/point_cloud2',  # Replace with the new topic name
            5)

    def listener_callback(self, msg):
        # Correct the timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        # Republish the message with the correct timestamp
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimestampCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
