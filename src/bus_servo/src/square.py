import rclpy
import numpy as np
from datetime import datetime
from rclpy.node import Node

from std_msgs.msg import String
from bus_servo_interfaces.msg import ServoCommand


class SquarePublisher(Node):

    def __init__(self):
        super().__init__('square_publisher')
        self.publisher1 = self.create_publisher(ServoCommand, '/servo1_cmd', 1)
        self.publisher2 = self.create_publisher(ServoCommand, '/servo2_cmd', 1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.path_t = [0.0, 2.0, 4.0, 6.0]
        self.path_yaw = [200.0, 800.0, 800.0, 200.0]
        self.path_pitch = [200.0, 200.0, 500.0, 500.0]
        self.start_time = datetime.now()
        self.path_period = 8.0;

    def timer_callback(self):
        #msg = String()
        elapsed = datetime.now()-self.start_time
        yaw = np.interp(elapsed.total_seconds(), self.path_t, self.path_yaw, period=self.path_period)
        msg = ServoCommand()
        msg.angle = yaw
        msg.max_vel = 5000.0
        msg.max_accel = 5000.0
        self.publisher1.publish(msg)

        pitch = np.interp(elapsed.total_seconds(), self.path_t, self.path_pitch, period=self.path_period)
        msg2 = ServoCommand()
        msg2.angle = pitch
        msg2.max_vel = 5000.0
        msg2.max_accel = 5000.0
        self.publisher2.publish(msg2)
        #self.get_logger().info(f'Publishing: {msg.angle}, {msg2.angle}')
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    square_publisher = SquarePublisher()

    rclpy.spin(square_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    square_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()