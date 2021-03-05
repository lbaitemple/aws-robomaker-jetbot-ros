
from Adafruit_MotorHAT import Adafruit_MotorHAT,Adafruit_DCMotor

import time
import atexit

mh=Adafruit_MotorHAT(i2c_bus=1, addr=0x60)
MAX_SPEED=150

def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

def runMotor(channel, speed):
    motor=mh.getMotor(channel)
    if(channel == 1):
        _ina = 1
        _inb = 0
    else:
        _ina = 2
        _inb = 3

    if (speed>0):    
        motor.run(Adafruit_MotorHAT.BACKWARD)
    else:
        motor.run(Adafruit_MotorHAT.FORWARD)
    # The two lines below are required for the Waveshare JetBot Board only
    mh._pwm.setPWM(_ina,0,0)
    mh._pwm.setPWM(_inb,0,speed*16)
    time.sleep(0.1)

def stopMotor(channel):
    motor=mh.getMotor(channel)
    if(channel == 1):
        _ina = 1
        _inb = 0
    else:
        _ina = 2
        _inb = 3

    motor.run(Adafruit_MotorHAT.BACKWARD)
    # The two lines below are required for the Waveshare JetBot Board only
    mh._pwm.setPWM(_ina,0,0)
    mh._pwm.setPWM(_inb,0,0)
    time.sleep(0.1)

    
while True:
    runMotor(1, 70)
    runMotor(2, 70)
    time.sleep(2)
    stopMotor(1)
    stopMotor(2)
    time.sleep(1)


