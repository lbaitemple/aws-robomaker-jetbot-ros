import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from qwiic_scmd import QwiicScmd

SUBSCRIBE_TOPIC = '/cmd_vel'

MAX_PWM = 115.0
MOTOR_LEFT_ID = 1
MOTOR_RIGHT_ID = 2

class Twist_to_Move(Node):

    def __init__(self):
        super().__init__('twist_to_move')
        
        self.motor_driver = QwiicScmd()
        
        self.subscription = self.create_subscription(
            Twist,
            SUBSCRIBE_TOPIC,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        x = msg.linear.x
        y = msg.angular.z/10

        if x > 0 and y < 0:  # backward right
            self.set_speed(MOTOR_LEFT_ID, (abs(y)+0.1))
            self.set_speed(MOTOR_LEFT_ID, (0.2+y+0.1))
        
        elif x > 0 and y > 0:  # backward left
            self.set_speed(MOTOR_LEFT_ID, (0.2-y+0.1))
            self.set_speed(MOTOR_LEFT_ID, (y+0.1))
        
        elif x < 0 and y > 0:  # forward left
            self.set_speed(MOTOR_LEFT_ID, (-(0.2-y)-0.1))
            self.set_speed(MOTOR_LEFT_ID, -(y+0.1))
        
        elif x < 0 and y < 0:  # forward right
            self.set_speed(MOTOR_LEFT_ID, y-0.1)
            self.set_speed(MOTOR_LEFT_ID, (-(0.2+y)-0.1))
        
        else:
            self.all_stop()
    
    
    def set_speed(self, motor_ID, value):
        max_pwm = float(MAX_PWM)
        speed = int(min(max(abs(value * max_pwm), 0), max_pwm))
        
        if (speed > 0):
            dir = 1
        else:
            dir = -1
        
        self.motor_driver.set_drive(motor_ID, dir, abs(speed))
        self.motor_driver.set_drive(motor_ID, dir, abs(speed))
        self.motor_driver.enable()
    
    def all_stop(self):
        self.motor_driver.disable()


def main(args=None):
    rclpy.init(args=args)

    twist_to_move = Twist_to_Move()

    rclpy.spin(twist_to_move)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_to_move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()