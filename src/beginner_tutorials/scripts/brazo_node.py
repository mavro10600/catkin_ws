#!/usr/bin/env python
'''
launchpad node , recibimos valores de sensores de la tarjeta stellaris, y 
los publicamos como topicos
el codigo es de chefbot
'''
#Python client library for ROS

import rospy
import sys
import time
import math

from SerialDataGateway import SerialDataGateway
from std_msgs.msg import Int16,Int32,Int64,Float32,String,Header,UInt64
#from sensor_msgs.msg import Imu

#Clase para manejar datos seriales y convertirla en topicos de ROS

class Launchpad_Class(object):
	
	def __init__(self):
		print "Iniciando clase launchpad"
		
######################################
#Variables de los sensores		
		self._Counter=0
		self._reset_base1_value=0;
		self._reset_shoulder1_value=0;
		self._reset_elbow1_value=0;

		self._base_encoder_value=0
		self._base_wheel_speed=0

		self._shoulder_encoder_value=0
		self._shoulder_wheel_speed=0
		
		self._elbow_encoder_value=0
		self._elbow_wheel_speed=0
		
		self._roll_encoder_value=0
		self._roll_wheel_speed=0

		self._pitch_encoder_value=0
		self._pitch_wheel_speed=0
		
		self._yaw_encoder_value=0
		self._yaw_wheel_speed=0
		
		self._LastUpdate_Microsec=0
		self._Second_Since_Last_Update=0
#########################################
#Asignamos valores del puerto y baudios de la stellaris
#		port=rospy.get_param("~port","/dev/ttyACM0")
		port=rospy.get_param("~port","/dev/ttyACM0")
		baudRate=int(rospy.get_param("~baudRate",115200))

#########################################
		rospy.loginfo("starting with serialport:"+port+"baudrate:"+str(baudRate))
		self._SerialDataGateway=SerialDataGateway(port,baudRate,self._HandleReceivedLine)
		rospy.loginfo("started serial communication")
		
###########################################
#publisher y suscribers
		self._SerialPublisher=rospy.Publisher('serial_base',String,queue_size=10)
		self.deltat=0
		self.lastUpdate=0
		self._Base_Encoder=rospy.Publisher('base_lec',Int16,queue_size=10)
		self._Shoulder_Encoder=rospy.Publisher('shoulder_lec',Int16,queue_size=10)
		self._Elbow_Encoder=rospy.Publisher('elbow_lec',Int16,queue_size=10)		
		self._Roll_Encoder=rospy.Publisher('roll_lec',Int16,queue_size=10)
		self._Pitch_Encoder=rospy.Publisher('pitch_lec',Int16,queue_size=10)
		self._Yaw_Encoder=rospy.Publisher('yaw_lec',Int16,queue_size=10)

		self._Base1_Reset=rospy.Publisher('base1_reset',Int16,queue_size=10)
		self._Shoulder1_Reset=rospy.Publisher('shoulder1_reset',Int16,queue_size=10)		
		self._Elbow1_Reset=rospy.Publisher('elbow1_reset',Int16,queue_size=10)		
		

		self._base_motor_speed=rospy.Subscriber('base_out',Int16,self._Update_Base_Speed)
		self._shoulder_motor_speed=rospy.Subscriber('shoulder_out',Int16,self._Update_Shoulder_Speed)
		self._elbow__motor_speed=rospy.Subscriber('elbow_out',Int16,self._Update_Elbow_Speed)
		self._roll_motor_speed=rospy.Subscriber('roll_out',Int16,self._Update_Roll_Speed)
		self._pitch_motor_speed=rospy.Subscriber('pitch_out',Int16,self._Update_Pitch_Speed)
		self._yaw_motor_speed=rospy.Subscriber('yaw_out',Int16,self._Update_Yaw_Speed)
		
	def _Update_Base_Speed(self,base_speed):
		self._base_wheel_speed=base_speed.data
		rospy.loginfo(base_speed.data)
		speed_message='s  %d %d %d %d %d %d\r' %(int(self._base_wheel_speed),int(self._shoulder_wheel_speed),int(self._elbow_wheel_speed),int(self._roll_wheel_speed),int(self._pitch_wheel_speed),int(self._yaw_wheel_speed))
		self._WriteSerial(speed_message)
		

	def _Update_Shoulder_Speed(self,shoulder_speed):
		self._shoulder_wheel_speed=shoulder_speed.data
		rospy.loginfo(shoulder_speed.data)
		speed_message='s  %d %d %d %d %d %d\r' %(int(self._base_wheel_speed),int(self._shoulder_wheel_speed),int(self._elbow_wheel_speed),int(self._roll_wheel_speed),int(self._pitch_wheel_speed),int(self._yaw_wheel_speed))
		self._WriteSerial(speed_message)
		
	def _Update_Elbow_Speed(self,elbow_speed):
		self._elbow_wheel_speed=elbow_speed.data
		rospy.loginfo(elbow_speed.data)
		speed_message='s  %d %d %d %d %d %d\r' %(int(self._base_wheel_speed),int(self._shoulder_wheel_speed),int(self._elbow_wheel_speed),int(self._roll_wheel_speed),int(self._pitch_wheel_speed),int(self._yaw_wheel_speed))
		self._WriteSerial(speed_message)
		
	def _Update_Roll_Speed(self,roll_speed):
		self._roll_wheel_speed=roll_speed.data
		rospy.loginfo(roll_speed.data)
		speed_message='s  %d %d %d %d %d %d\r' %(int(self._base_wheel_speed),int(self._shoulder_wheel_speed),int(self._elbow_wheel_speed),int(self._roll_wheel_speed),int(self._pitch_wheel_speed),int(self._yaw_wheel_speed))
		self._WriteSerial(speed_message)
		
	def _Update_Pitch_Speed(self,pitch_speed):
		self._pitch_wheel_speed=pitch_speed.data
		rospy.loginfo(pitch_speed.data)
		speed_message='s  %d %d %d %d %d %d\r' %(int(self._base_wheel_speed),int(self._shoulder_wheel_speed),int(self._elbow_wheel_speed),int(self._roll_wheel_speed),int(self._pitch_wheel_speed),int(self._yaw_wheel_speed))
		self._WriteSerial(speed_message)
		
	def _Update_Yaw_Speed(self,yaw_speed):
		self._yaw_wheel_speed=yaw_speed.data
		rospy.loginfo(yaw_speed.data)
		speed_message='s  %d %d %d %d %d %d\r' %(int(self._base_wheel_speed),int(self._shoulder_wheel_speed),int(self._elbow_wheel_speed),int(self._roll_wheel_speed),int(self._pitch_wheel_speed),int(self._yaw_wheel_speed))
		self._WriteSerial(speed_message)


	def _HandleReceivedLine(self,line):	
		self._Counter=self._Counter+1
		self._SerialPublisher.publish(String(str(self._Counter)+", in:"+line))
		
		if(len(line)>0):
			lineParts=line.split('\t')
			try:
				if(lineParts[0]=='e'):
					self._base_encoder_value=long(lineParts[1])
					self._shoulder_encoder_value=long(lineParts[2])
					self._elbow_encoder_value=long(lineParts[3])					
					self._roll_encoder_value=long(lineParts[4])
					self._pitch_encoder_value=long(lineParts[5])
					self._yaw_encoder_value=long(lineParts[6])

					self._Base_Encoder.publish(self._base_encoder_value)
					self._Shoulder_Encoder.publish(self._shoulder_encoder_value)
					self._Elbow_Encoder.publish(self._elbow_encoder_value)
					self._Roll_Encoder.publish(self._roll_encoder_value)
					self._Pitch_Encoder.publish(self._pitch_encoder_value)
					self._Yaw_Encoder.publish(self._yaw_encoder_value)
					
				if(lineParts[0]=='n'):
					self.reset_base1_value=int(lineParts[1])
					self.reset_shoulder1_value=int(lineParts[2])
					self.reset_elbow1_value=int(lineParts[3])
				
					self._Base1_Reset.publish(self.reset_base1_value)
					self._Shoulder1_Reset.publish(self.reset_shoulder1_value)
					self._Elbow1_Reset.publish(self.reset_elbow1_value)
					
			except:
				rospy.logwarn("Error in sensor values")
				rospy.logwarn(lineParts)
				pass

	def _WriteSerial(self,message):
		self._SerialPublisher.publish(String(str(self._Counter)+", out:"+message))
		self._SerialDataGateway.Write(message)
		
	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stoping")
		self._SerialDataGateway.Stop()
		
	def Reset_Launchpad(self):
		print "reset"
		reset='r\r'
		self._WriteSerial(reset)
		time.sleep(1)
		self._WriteSerial(reset)
		time.sleep(2)
		
	def SubscribeSpeed(self):
		a=1
	def SendSpeed(self):
		a=3
	
if __name__=='__main__':
	rospy.init_node('launchpad_ros',anonymous=True)
	launchpad=Launchpad_Class()
	try:
		launchpad.Start()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("error in main function")
	
	launchpad.Reset_Launchpad()
	launchpad.Stop()
		
		
