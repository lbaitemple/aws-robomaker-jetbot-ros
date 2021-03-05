#!/usr/bin/env python2

# python imports
import logging
import json
import os
import time
# ros imports
import rospy
import rospkg

#Set the verbosity of logs and the logger name
LOGLEVEL = logging.DEBUG 
LOGNAME = 'roslaunch'
MOTOR_CONTROLLER = os.environ['MOTOR_CONTROLLER'].lower()

def setup_logging(loglevel = LOGLEVEL, logname = LOGNAME):
    """
    Return a logger for this ROS node

    Logs can be viewed in AWS CloudWatch Logs
    /aws/robomaker/SimulationJobs/...RobotApplicationLogs
    """
    logger = logging.getLogger(LOGNAME)
    logger.setLevel(LOGLEVEL)
    ch = logging.StreamHandler()
    ch.setLevel(loglevel)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    return logger
    

if MOTOR_CONTROLLER== 'adafruit':
    from Adafruit_MotorHAT import Adafruit_MotorHAT
elif MOTOR_CONTROLLER == 'qwiic':
    from qwiic_scmd import *
else:
    raise ImportError
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Move():
	def __init__(self):
		#logger.info("init move")
		# setup motor controller
		self.motor_left_ID = 1
		self.motor_right_ID = 2
		if MOTOR_CONTROLLER == 'adafruit':
			self.motor_driver = Adafruit_MotorHAT(i2c_bus=int(rospy.get_param("i2c_bus")))
#			self.motor_left_ID = 1
#			self.motor_right_ID = 2
			self.motor_left = self.motor_driver.getMotor(self.motor_left_ID)
			self.motor_right = self.motor_driver.getMotor(self.motor_right_ID)
			self.all_stop()
		elif MOTOR_CONTROLLER == 'qwiic':
			logger.info("init move - qwiic")
			self.motor_driver = QwiicScmd()
			logger.info("init move - qwiic send commd")
			if self.motor_driver.connected == False:
				logger.info("init move -  not conneectedd")
			else:
				logger.info("init move - conneectedd")

#			self.motor_driver.disable()
#			logger.info("init move - qwiic send disable")

	def set_speed(self, motor_ID, value):
		max_pwm = float(rospy.get_param("max_pwm"))
		speed = int(min(max(abs(value * max_pwm), 0), max_pwm))
		
		
		if MOTOR_CONTROLLER == 'adafruit':
			if motor_ID == 1:
				motor = self.motor_left
				_ina = 1
        			_inb = 0
			elif motor_ID == 2:
				motor = self.motor_right
				_ina = 2
        			_inb = 3				
			else:
				rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
				return
			self.motor_driver._pwm.setPWM(_ina,0,0)
    			self.motor_driver._pwm.setPWM(_inb,0,speed*16)
			# motor.setSpeed(speed)
			if speed > 0:
				motor.run(Adafruit_MotorHAT.FORWARD)
			else:
				motor.run(Adafruit_MotorHAT.BACKWARD)

		elif MOTOR_CONTROLLER== 'qwiic':
			if (speed>0):
				dir = 1
			else:
				dir = -1
			logger.info("Subsribing to spped value %f", abs(speed))
			self.motor_driver.set_drive(motor_ID, dir, abs(speed))
			self.motor_driver.set_drive(motor_ID, dir, abs(speed))
			self.motor_driver.enable()
	
	
	def start(self):
		logger.info("Subsribing to topic %s", "move")
		# rospy.loginfo("Subsribing to topic %s", "move")
#		rospy.Subscriber('~cmd_dir', String, self.on_cmd_dir)
	
		rospy.Subscriber('/move/cmd_vel', Twist, self.on_cmd_vel)
		logger.info("Donee Subsribing to topic %s", "move")
#		rospy.Subscriber('~cmd_raw', String, self.on_cmd_raw)
#		rospy.Subscriber('~cmd_str', String, self.on_cmd_str)
		rospy.spin()
#		while True:
#			time.sleep(5)
#		return
	
	# stops all motors
	def all_stop(self):
		if MOTOR_CONTROLLER == 'adafruit':
			self.motor_left.setSpeed(0)
			self.motor_right.setSpeed(0)
			self.motor_left.run(Adafruit_MotorHAT.RELEASE)
			self.motor_right.run(Adafruit_MotorHAT.RELEASE)
		elif MOTOR_CONTROLLER == 'qwiic':
			self.motor_driver.disable()
	
	def move_dir(self, val):
		if val == "left":
			self.set_speed(self.motor_left_ID,  float(rospy.get_param("speed")))
			self.set_speed(self.motor_right_ID, (-1 * float(rospy.get_param("speed")))) 
		elif val == "right":
			self.set_speed(self.motor_left_ID,   float(rospy.get_param("speed")))
			self.set_speed(self.motor_right_ID, float(rospy.get_param("speed"))) 
		elif val == "backward":
			self.set_speed(self.motor_left_ID, float(rospy.get_param("speed")))
			self.set_speed(self.motor_right_ID, float(rospy.get_param("speed")))
		elif val == "forward":
			self.set_speed(self.motor_left_ID,  (-1 * float(rospy.get_param("speed"))))
			self.set_speed(self.motor_right_ID, (-1 * float(rospy.get_param("speed"))))  
		elif val == "stop":
			self.all_stop()
		else:
			rospy.logerror('Direction not supported.')
	
	# directional commands (degree, speed)
	def on_cmd_dir(self, msg):
		rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)
	
	# velocity, twist commands (Twist)
	def on_cmd_vel(self, msg):
	    x = msg.linear.x
	    y = msg.angular.z/10
	    
	    logger.info('Received from message: %s', msg.linear) 
	    
	    if x>0 and y<0: #backward right
#	    	rospy.loginfo(rospy.get_caller_id() + ', backward right (left, right)=(%s,%s)' % ((abs(y)+0.1), (0.2+y+0.1)))
	    	self.set_speed(self.motor_left_ID, (abs(y)+0.1))
	    	self.set_speed(self.motor_right_ID, (0.2+y+0.1))
	    elif x>0 and y>0: #backward left
#	    	rospy.loginfo(rospy.get_caller_id() + ', backward left (left, right)=(%s,%s)' % ((0.2-y+0.1), (y+0.1)))
	    	self.set_speed(self.motor_left_ID, (0.2-y+0.1))
	    	self.set_speed(self.motor_right_ID, (y+0.1))
	    elif x<0 and y>0: #forward left
#	    	rospy.loginfo(rospy.get_caller_id() + ', forward left (left, right)=(%s,%s)' % ((-(0.2-y)-0.1), -(y+0.1)))
	    	self.set_speed(self.motor_left_ID, (-(0.2-y)-0.1))
	    	self.set_speed(self.motor_right_ID, -(y+0.1))
	    elif x<0 and y<0: #forward right
#	    	rospy.loginfo(rospy.get_caller_id() + ', forward right (left, right)=(%s,%s)' % (y-0.1, (-(0.2+y)-0.1)))
	    	self.set_speed(self.motor_left_ID, y-0.1)
	    	self.set_speed(self.motor_right_ID, (-(0.2+y)-0.1))
	    else:
	    	self.all_stop()
	    
	# raw L/R motor commands (speed, speed)
	def on_cmd_raw(self, msg):
		rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)
		move_data_recv = json.loads(msg.data)
		self.set_speed(self.motor_left_ID, float(move_data_recv['left']))
		self.set_speed(self.motor_right_ID, float(move_data_recv['right']))
	
	# simple string commands (left/right/forward/backward/stop)
	def on_cmd_str(self, msg):
		rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)
		self.move_dir(msg.data.lower())

def main():
	logger.info('jetbot_sim_app Move Node starting...')
	rospy.init_node('move')
	move = Move()
	
	try:
		move.start()
	except rospy.ROSInterruptException:
		pass
	finally:
		# stop motors before exiting
		move.all_stop()
		
# initialization
if __name__ == '__main__':
	logger = setup_logging()
	logger.info("before movee main ")
	main()
