#! /usr/bin/env python
import rospy

#Nos da la capacidad de correr el dynamic reconfigure server
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

#importar el message a la mdida y la svariables del dynamic reconfigure
from dynamic_tutorials.msg import NodeExampleData
from dynamic_tutorials.cfg import nodeExampleConfig as ConfigType

class NodeExample(object):
	def __init__(self):
		"""Read in parameters"""
		#obtenes los paranmetros del espacio de nombres privado desde el parameter serveer:
		rate=rospy.get_param('rate',1.0)
		self.enable=True
		self.server=DynamicReconfigureServer(ConfigType, self.reconfigure_cb)
		self.pub=rospy.Publisher('example',NodeExampleData,queue_size=10)
		self.enable=rospy.get_param('enable',True)
		self.int_a=rospy.get_param('a',1)
		self.int_b=rospy.get_param('b',2)
		self.message=rospy.get_param('message','hello')
		if self.enable:
			self.start()
		else:
			self.stop()
		rospy.Timer(rospy.Duration(1/rate),self.timer_cb)
	def start(self):
		"""Turn on publisher"""
		self.pub=rospy.Publisher('example',NodeExampleData,queue_size=10)
	
	def stop(self):
		"""Turn off publisher"""
		self.pub.unregister()
		
	def timer_cb(self,_event):
		"""call a specified interval to publish message"""
		if not self.enable:
			return
		#establce el tipo de mensaje a publicar como el tipo a la medida que hicimos
		msg=NodeExampleData()
		#asigna los valores del msg usando lo que hayq en el parameter server
		msg.message=rospy.get_param('message',self.message)
		msg.a=rospy.get_param('a',self.int_a)
		msg.b=rospy.get_param('b',self.int_b)
		
		self.message=msg.message
		self.int_a=msg.a
		self.int_b=msg.b
		
		self.pub.publish(msg)
	
	def reconfigure_cb(self,config,dummy):
		"""create a callback function to the dynamic reconfigure server"""
		self.message=config["Message"]
		self.int_a=config["a"]
		self.int_b=config["b"]
		
		if self.enable!=config["enable"]:
			if config["enable"]:
				self.start()
			else:
				self.stop()
		self.enable=config["enable"]
		return config
if __name__ == '__main__':
	rospy.init_node('pytalker')
	try:
		NodeExample()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()
