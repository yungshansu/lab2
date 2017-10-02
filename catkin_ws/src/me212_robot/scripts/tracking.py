#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT
import rospy
from std_msgs.msg import Float64MultiArray
from math import pi, radians
class Tracking:
	def __init__(self):
		self.node_name = rospy.get_name()	
		self.state = 1
		self.trig = None
		self.motorhat = Adafruit_MotorHAT(addr= 0x60)
		self.leftMotor 	= self.motorhat.getMotor(1)
		self.rightMotor = self.motorhat.getMotor(2)
		self.right_pwm = 60
		self.left_pwm = 60
		self.leftMotor.setSpeed(self.left_pwm)
		self.rightMotor.setSpeed(self.right_pwm)
		self.subPosition=rospy.Subscriber("/serial_node/odometry",Float64MultiArray,self.cbPosition)

                self.task_flag = True
                self.record_x = 0.
                self.record_y = 0.
                self.record_theta = 0.

		rospy.on_shutdown(self.custom_shutdown)
		rospy.loginfo("[%s] Initialized!" %self.node_name)
	def cbPosition(self,msg):
		x     = msg.data[0]
		y     = msg.data[1]
		theta = msg.data[2]
		theta = theta % (2* pi)
		print x,y,theta

		# stages: 1) straight line,
		#         2) semi-circle
		#         3) straight line again.
                if self.state==1:
                    # walk straight
                    self.leftMotor.run(1)
                    self.rightMotor.run(1)
                    # start walking straight
                    if self.task_flag:
                        self.record_x = x
                        self.record_y = y
                        self.task_flag = False
                    # check 1m straight line done  
                    dist = sqrt((x-self.record_x)*(x-self.record_x)+(y-self.record_y)*(y-self.record_y))
                    if abs(dist-1)<0.1:
                        self.task_flag = True
                        state = 2
                elif self.state==2:
                    # rotate in radius 0.25
                        
                    # start rotating
                    if self.task_flag:
                        self.record_theta = theta
                        self.task_flag = False
                        self.leftMotor.setSpeed(60)
                        self.leftMotor.setSpeed(120)
                    # check rotate semi-circle
                    del_theta = abs(theta-self.record_theta)
                    if abs(del_theta-pi)<0.05:
                        self.leftMotor.setSpeed(60)
                        self.leftMotor.setSpeed(60)
                        self.task_flag = True
                        state = 3
                elif self.state==3:
                    # walk straight
                    self.leftMotor.run(1)
                    self.rightMotor.run(1)
                    # start walking straight
                    if self.task_flag:
                        self.record_x = x
                        self.record_y = y
                        self.task_flag = False
                    # check 1m straight line done  
                    dist = sqrt(x*x+y*y)
                    if abs(dist-1)<0.1:
                        self.task_flag = True
                        state = 0
                else:
                    # brake
                    self.leftMotor.run(3)
                    self.rightMotor.run(3)
                    rospy.loginfo("The task is done")

	def custom_shutdown(self):
		self.leftMotor.run(4)
		self.rightMotor.run(4)
		del self.motorhat

if __name__ == '__main__':
	rospy.init_node('tracking', anonymous = False)
	Track = Tracking()
	rospy.spin()
