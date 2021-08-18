import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT

SUBSCRIBE_TOPIC = '/cmd_vel'

I2C_BUS = 1
MAX_PWM = 115.0
SPEED = 0.10
MOTOR_LEFT_ID = 1
MOTOR_RIGHT_ID = 2

class Twist_to_Move(Node):

    def __init__(self):
        super().__init__('twist_to_move')
        
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=int(I2C_BUS))
        self.motor_left = self.motor_driver.getMotor(MOTOR_LEFT_ID)
        self.motor_right = self.motor_driver.getMotor(MOTOR_LEFT_ID)
        self.all_stop()
        
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

        if motor_ID == 1:
            motor = self.motor_left
            _ina = 1
            _inb = 0
        elif motor_ID == 2:
            motor = self.motor_right
            _ina = 2
            _inb = 3
        
        self.motor_driver._pwm.setPWM(_ina, 0, 0)
        self.motor_driver._pwm.setPWM(_inb, 0, speed*16)
        
        if speed > 0:
            motor.run(Adafruit_MotorHAT.FORWARD)
        else:
            motor.run(Adafruit_MotorHAT.BACKWARD)
    
    def all_stop(self):
        self.motor_left.setSpeed(0)
        self.motor_right.setSpeed(0)
        self.motor_left.run(Adafruit_MotorHAT.RELEASE)
        self.motor_right.run(Adafruit_MotorHAT.RELEASE)


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