#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from math import *

class Left_node:

	def __init__(self, node_name_override = 'enc_lec_node'):

		rospy.init_node(node_name_override)
		self.nodename = rospy.get_name()
		rospy.loginfo("enc_lec_node %s", self.nodename) 
		self.rate = rospy.get_param("param_global_rate", 10)

		self.leftAngPub = rospy.Publisher("left_ang", Float32)
		self.leftVelPub = rospy.Publisher("left_vel", Float32)

        
		self.left_enc_ana = False
		self.left_enc_max = 1023
		self.left_offset = 0
        
		self.left_lec = 0
		self.left_ang = 0
		self.left_vel = 0

        # helper variables
		self.left_ang_tmp = 0
		self.left_ang_lst = 0
		self.left_ang_abs = 0
		self.left_ang_lap = 0

		self.left_offset_internal = 0.
		self.left_ang_tmp_internal = 0      
		self.left_ang_lst_internal = 0
		self.left_ang_abs_internal = 0

		self.times = 0
        
        #self.leftResetSub = rospy.Subscriber("left_reset", Int16, self.leftResetCb)
        
		self.leftLecSub = rospy.Subscriber("left_lec", Int16, self.leftLecCb)
		self.offsetSub = rospy.Subscriber("offset", Int16, self.offsetCb)
       
       

	def offsetCb(self, data):

		if (data.data == 1):

			self.left_offset  = self.left_lec
            
			self.left_ang_tmp = 0
			self.left_ang_lst = 0
			self.left_ang_abs = 0

			self.left_ang = 0
			self.left_ang_lap =  0
			self.left_ang_lap_lst = 0

#funcion que resetea lA VARIABLE DE CONTROL
	#def leftResetCb(self, data):
        
	#	if (data.data == 1):
	#		self.left_ang_des = 0   
   
            
            
#Funcion asociada al suscriptor al valor del encoder dado por el micro    
	def leftLecCb(self, data):

		self.left_lec = data.data
		self.angCalc()


# Funcion auxiliar a la funcion principal que lee el encoder

	def map(self, x, in_min, in_max, out_min, out_max):

		return (x - in_min) * (out_max - out_min + 0.) / (in_max - in_min + 0.) + out_min

            
            
#Esta es la funcion que hace la conversion de la lectura del encoder a radianes

	def angCalc(self):

		self.times += 1

		self.left_offset_internal = self.map(self.left_offset, 0., self.left_enc_max, 2 * pi, 0)
		self.left_ang_tmp_internal = self.map(self.left_lec, 0., self.left_enc_max, 2 * pi, 0)
		self.left_ang_lst_internal = self.left_ang_abs_internal
		self.left_ang_abs_internal = self.left_ang_tmp_internal

		if (self.times > 4):
            # encuentra si el cambio fue de 0 a 2pi
			if (self.left_ang_abs_internal > 1.7 * pi and self.left_ang_lst_internal < 0.3 * pi):
				self.left_ang_lap -= 1
            # encuentra si el cambio fue de 2pi a 0
			if (self.left_ang_abs_internal < 0.3 * pi and self.left_ang_lst_internal > 1.7 * pi):
				self.left_ang_lap += 1

		self.left_ang_lap_lst = self.left_ang
		self.left_ang = 2 * pi * self.left_ang_lap + self.left_ang_abs_internal - self.left_offset_internal 
		self.left_vel = 10 * (self.left_ang - self.left_ang_lap_lst)


#Funcion donde publicamos las variables obtenidas en angCalc()

	def update(self):

		self.leftAngPub.publish(self.left_ang)
		self.leftVelPub.publish(self.left_vel)


	def spin(self):

		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()


if __name__ == '__main__':

	""" main """
	left_node = Left_node()
	left_node.spin()
