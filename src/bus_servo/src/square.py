from os import name
from numpy.core.numeric import NaN
import rclpy
import numpy as np
from datetime import datetime
from rclpy.node import Node
import math

from std_msgs.msg import String
from std_msgs.msg import Float64
from bus_servo_interfaces.msg import ServoCommand


class SquarePublisher(Node):


        

    def yaw_callback(self, msg):
        if math.isnan(msg.data):
            self.get_logger().info("got a null yaw")
            self.get_logger().info(str(msg))
            
        self.yaw = msg.data

    def pitch_callback(self, msg):
        self.pitch = msg.data

    def __init__(self):
        super().__init__('square_publisher')
        self.yaw_publisher = self.create_publisher(ServoCommand, '/cmd_servo1', 1)
        self.pitch_publisher = self.create_publisher(ServoCommand, '/cmd_servo2', 1)
        self.timer_period = 0.01 # seconds
        self.look_ahead_time = 0.05 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.path_t = [0.0, 10.0, 20.0, 30.0]
        self.path_yaw = [200.0, 800.0, 800.0, 200.0]
        self.path_pitch = [200.0, 200.0, 500.0, 500.0]
        #self.path_pitch = [200.0, 500.0, 200.0, 500.0]
        self.start_time = datetime.now()
        self.path_period = 40.0;
        self.pitch = NaN
        self.yaw = NaN

        self.yaw_subscription = self.create_subscription(
            Float64,
            '/servo1',
            self.yaw_callback,
            10)
        self.pitch_subscription = self.create_subscription(
            Float64,
            '/servo2',
            self.pitch_callback,
            10)


    def timer_callback(self):

        # since we need current positionto calculate a velocity,
        # return if we don't have it yet
        #if math.isnan(self.pitch) or math.isnan(self.yaw):
        if math.isnan(self.pitch):
            self.get_logger().info("waiting for current position")
            return
        #return

        #msg = String()
        elapsed = datetime.now()-self.start_time
        #yaw = np.interp(elapsed.total_seconds(), self.path_t, self.path_yaw, period=self.path_period)
        t_ahead = elapsed.total_seconds() + self.look_ahead_time
        next_yaw = np.interp(t_ahead, self.path_t, self.path_yaw, period=self.path_period)
        msg = ServoCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle = next_yaw
        msg.max_vel = abs((next_yaw-self.yaw)/(self.look_ahead_time)) if self.look_ahead_time > 0 else 0.0
        msg.max_accel = 5000.0
        self.yaw_publisher.publish(msg)

        next_pitch = np.interp(t_ahead, self.path_t, self.path_pitch, period=self.path_period)
        msg2 = ServoCommand()
        msg2.header.stamp = msg.header.stamp
        msg2.angle = next_pitch
        msg2.max_vel = abs( (next_pitch-self.pitch)/(self.look_ahead_time))  if self.look_ahead_time > 0 else 0.0
        msg2.max_accel = 5000.0
        self.pitch_publisher.publish(msg2)

        #self.get_logger().info(f'Current: ({self.yaw},{self.pitch})  Yaw:{msg.angle:.0f} Yaw Velocity: {msg.max_vel:.1f}  Pitch: {msg2.angle:.1f} Pitch Velocity {msg2.max_vel:.1f}')
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)

    square_publisher = SquarePublisher()

    while rclpy.ok:
        rclpy.spin_once(square_publisher)
    
    # rclpy.spin(square_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    square_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()