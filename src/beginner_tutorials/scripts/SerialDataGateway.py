#!/usr/bin/env python

import threading
import serial
from io import StringIO
import time
import rospy

def _OnLineReceived(self):
	print(line)
	
class SerialDataGateway(object):

	def __init__(self,port="/dev/ttyUSB0",baudrate=115200,lineHandler=_OnLineReceived):
		self._Port=port
		self._Baudrate=baudrate
		self.ReceivedLineHandler=lineHandler
		self._KeepRunning=False

	def Start(self):
		self._Serial=serial.Serial(port=self._Port,baudrate=self._Baudrate,timeout=1)
		self._KeepRunning=True
		self._ReceiverThread=threading.Thread(target=self._Listen)
		self._ReceiverThread.start()
	
	def Stop(self):
		rospy.loginfo("stopping serial gateway")
		self._KeepRunning=False
		time.sleep(.1)
		self._Serial.close()
		
	def _Listen(self):
		stringIO=StringIO()
		while self._KeepRunning:

			data=str(self._Serial.read(), 'utf-8')
			#rospy.loginfo(data)
			if data=='\r':
				pass
			if data=='\n':
				strin=stringIO.getvalue()
				#print("strin: ", strin)
				self.ReceivedLineHandler(strin)
#				rospy.loginfo(stringIO)
				stringIO.close()
				stringIO=StringIO()
				
			else:
				stringIO.write(data)
	def Write(self,data):
		info="Writing to serial port: %s" %data 
#		rospy.loginfo(info)
		self._Serial.write(data)
		
	if __name__=='__main__':
		dataReceiver=SerialDataGateway("/dev/ttyUSB0",115200)
		dataReceiver.Start()
		
		raw_input("Hit <Enter> to end")
		dataReceiver.Stop()
