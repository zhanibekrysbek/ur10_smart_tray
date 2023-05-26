#!/usr/bin/env python3

import rospy
import subprocess
import time
from datetime import datetime
from std_msgs.msg import Float64, String
from rft_sensor_serial.srv import rft_operation
from smart_tray.srv import trigger_srv


class smart_tray(object):
	'''
	Class handle of entire tray designed at Robotics Lab, UIC.
	Mainly, it triggers resonsible services for each device including:
		- 2 RFT60 sensors
		- 3 Cameras
		- 1 IMU
		- Camera Pose Estimation Module
	'''

	def __init__(self):

		# Set COM port params
		rospy.set_param('RFT_COM_PORT', '/dev/ttyUSB0')
		rospy.set_param('RFT_COM_PORT_2', '/dev/ttyUSB1')

		# wait until services will be available in the network
		rospy.wait_for_service('rft_serial_op_service')
		rospy.wait_for_service('rft_serial_op_service_2')
		rospy.wait_for_service('imu_srv')
		rospy.wait_for_service('camera_1')
		rospy.wait_for_service('camera_2')
		# rospy.wait_for_service('camera_3')
		rospy.wait_for_service('camera_4')

		rospy.loginfo('Both RFT sensors are available!')
		rospy.loginfo('Cameras are ready!')
		rospy.loginfo('IMU is ready!')


		rospy.sleep(5)
		# User intent to start the system
		input('Press ENTER to start the system... ')

		rospy.loginfo('Initiating ServiceProxy... ')
		self.rft_srv_1 = rospy.ServiceProxy('rft_serial_op_service', rft_operation)
		self.rft_srv_2 = rospy.ServiceProxy('rft_serial_op_service_2', rft_operation)
		self.imu_srv = rospy.ServiceProxy('imu_srv', trigger_srv)
		self.camera_1_srv = rospy.ServiceProxy('camera_1', trigger_srv)
		self.camera_2_srv = rospy.ServiceProxy('camera_2', trigger_srv)
		# self.camera_3_srv = rospy.ServiceProxy('camera_3', trigger_srv)
		self.camera_4_srv = rospy.ServiceProxy('camera_4', trigger_srv)



	def start(self):
		# Call Serial Number so it gets written in message frame.
		rospy.loginfo("Requesting RFT serial number...")
		res1 = self.rft_srv_1(2,0,0,0)
		res2 = self.rft_srv_2(2,0,0,0)

		rospy.loginfo("Starting imu... ")
		res3 = self.imu_srv('start')

		rospy.loginfo("Starting cameras... ")
		res4 = self.camera_1_srv('start')
		res5 = self.camera_2_srv('start')
		# res6 = self.camera_3_srv('start')
		res6 = self.camera_4_srv('start')

		if res1.result !=0: #or res2.result !=0:
			rospy.logwarn('Something wrong with Sensor Communication!')

		rospy.sleep(0.5)

		rospy.loginfo("Requesting RFT to broadcast the data... ")
		# trigger two nodes to broadcast data
		self.rft_srv_1(11,0,0,0)
		self.rft_srv_2(11,0,0,0)

		# rospy.logwarn('!!!! Keep the tray at rest. Applying bias... !!!')

		# rospy.sleep(2.)
		# apply bias
		# res1 = self.rft_srv_1(17,1,0,0)
		# res2 = self.rft_srv_2(17,1,0,0)

		if res1.result ==0 and res2.result ==0:
			rospy.loginfo('Both sensors are publishing!')


	# to be called on rospy shutdown
	def stop(self):

		rospy.loginfo('Ending the program!')
		self.rft_srv_1(12,0,0,0)
		self.rft_srv_2(12,0,0,0)
		rospy.sleep(1)
		self.imu_srv('stop')
		self.camera_1_srv('stop')
		self.camera_2_srv('stop')
		# self.camera_3_srv('stop')
		self.camera_4_srv('stop')

		rospy.sleep(3)

'''
	This node is a central program that initiates sensor readings.

	1. Service call for RFT_1
	2. Service call for RFT_2
	3. Service call for IMU
	4. Service call for camera_1
	5. Service call for camera_2
	6. 
'''


def main():

	rospy.init_node('ur10e_smart_tray')

	rospy.loginfo('Starting smart_tray node!')

	tray = smart_tray()

	tray.start()

	rospy.on_shutdown(tray.stop)

	rospy.spin()


if __name__=='__main__':

	main()
