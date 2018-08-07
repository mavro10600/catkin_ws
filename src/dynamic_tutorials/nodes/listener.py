#! /usr/bin/env python

import rospy
from dynamic_tutorials.msg import NodeExampleData

def callback(data):
	rospy.loginfo(rospy.get_name()+"I heard %s",data.message+",a+b=%d"%(data.a + data.b))
	
def listener():
	rospy.Subscriber('example',NodeExampleData,callback)
	
if __name__=="__main__":
	rospy.init_node('pylistener')
	listener()
	rospy.spin()
