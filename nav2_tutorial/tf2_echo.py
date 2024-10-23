import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np

class Tf2Echo(Node):
    def __init__(self, source_frame, target_frame):
        super().__init__('tf2_echo')
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_rec = np.array([])

        # Timer to periodically call the lookup function
        self.timer = self.create_timer(0.5, self.lookup_transform) # 2Hz

    def lookup_transform(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now
            )
            self.print_transform(transform)
        except Exception as e:
            self.get_logger().error(f'Could not transform {self.source_frame} to {self.target_frame}: {e}')

    def print_transform(self, transform: TransformStamped):
        self.get_logger().info(f'Transform from {self.source_frame} to {self.target_frame}:')
        self.get_logger().info(f'Translation: x={round(transform.transform.translation.x, 2)}, '
                                            f'y={round(transform.transform.translation.y, 2)}, '
                                            f'z={round(transform.transform.translation.z, 2)}')
        self.get_logger().info(f'Rotation: x={round(transform.transform.rotation.x, 2)}, '
                                         f'y={round(transform.transform.rotation.y, 2)}, '
                                         f'z={round(transform.transform.rotation.z, 2)}, '
                                         f'w={round(transform.transform.rotation.w, 2)}')
        self.pose_rec = np.append(self.pose_rec, np.array([transform.transform.translation.x, 
                                                           transform.transform.translation.y,
                                                           transform.transform.rotation.x,
                                                           transform.transform.rotation.y,
                                                           transform.transform.rotation.z,
                                                           transform.transform.rotation.w,]))
        np.save('/home/hsuhanjaya/Richard/notebook/nav2/pose_rec_0918.npy', self.pose_rec)

def main(args=None):
    rclpy.init(args=args)
    source_frame = 'robot0/base_link'  # Replace with your source frame
    target_frame = 'map'  # Replace with your target frame

    node = Tf2Echo(source_frame, target_frame)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
