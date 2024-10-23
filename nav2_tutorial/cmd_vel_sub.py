import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.velocityCallback,
            1)
        self.subscription  # prevent unused variable warning
        self.vel_rec = np.array([])
        self.ang_rec = np.array([])

    def velocityCallback(self, data: Twist) -> None:
        linear = data.linear
        angular = data.angular
        self.vel_rec = np.append(self.vel_rec, linear)
        self.ang_rec = np.append(self.ang_rec, angular)
        np.save('/home/hsuhanjaya/Richard/notebook/nav2/vel.npy', self.vel_rec)
        np.save('/home/hsuhanjaya/Richard/notebook/nav2/ang.npy', self.ang_rec)
        
        print('received!')
        

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()