#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <math.h>

class Conversor
{
	public:
		Conversor();
			
		void spin();
	private:

		ros::NodeHandle n;
		ros::Subscriber base_sub;

		int base_lec;
		float base_val;
		ros::Publisher base_pub;
		ros::Publisher base_degrees;		
		void baseEncoderCb(const std_msgs::Int16::ConstPtr& base);
		
		ros::Subscriber shoulder_sub;
		int shoulder_lec;
		float shoulder_val;
		ros::Publisher shoulder_pub;
		ros::Publisher shoulder_degrees;		
		void shoulderEncoderCb(const std_msgs::Int16::ConstPtr& shoulder);
		
		ros::Subscriber elbow_sub;
		int elbow_lec;
		float elbow_val;
		ros::Publisher elbow_pub;
		ros::Publisher elbow_degrees;		
		void elbowEncoderCb(const std_msgs::Int16::ConstPtr& elbow);
		
		
		void init_variables();
		
		void update();		

};

Conversor::Conversor()
{
	init_variables();
	ROS_INFO("Started computing encoder value");
	base_sub=n.subscribe("/base_lec",10,&Conversor::baseEncoderCb,this);
	base_pub=n.advertise<std_msgs::Float32>("base_radians",20);
	base_degrees=n.advertise<std_msgs::Float32>("base_degrees",20);
	
	shoulder_sub=n.subscribe("/shoulder_lec",10,&Conversor::shoulderEncoderCb,this);
	shoulder_pub=n.advertise<std_msgs::Float32>("shoulder_radians",20);
	shoulder_degrees=n.advertise<std_msgs::Float32>("shoulder_degrees",20);
	
	elbow_sub=n.subscribe("/elbow_lec",10,&Conversor::elbowEncoderCb,this);
	elbow_pub=n.advertise<std_msgs::Float32>("elbow_radians",20);
	elbow_degrees=n.advertise<std_msgs::Float32>("elbow_degrees",20);
}

void Conversor::init_variables()
{
base_lec=0;
base_val=0;

shoulder_lec=0;
shoulder_val=0;

elbow_lec=0;
elbow_val=0;		
}


void Conversor::spin()
{
		ros::spin();
}


void Conversor::baseEncoderCb(const std_msgs::Int16::ConstPtr& base)
{
//	ROS_INFO("calllback");
	base_lec=base->data;
	base_val=(base_lec*3.1416/200) - 5*3.1416/4;	
	std_msgs::Float32 base_conv;
	std_msgs::Float32 base_deg;
	base_deg.data=base_val*180/3.1416;
	base_conv.data=base_val;		
	base_pub.publish(base_conv);
	base_degrees.publish(base_deg);
}

void Conversor::shoulderEncoderCb(const std_msgs::Int16::ConstPtr& shoulder)
{
//	ROS_INFO("calllback");
	shoulder_lec=shoulder->data;
	shoulder_val=(shoulder_lec*3.1416/200)-3.1416/2;	
	std_msgs::Float32 shoulder_conv;
	std_msgs::Float32 shoulder_deg;
	shoulder_deg.data=shoulder_val*180/3.1416;
	shoulder_conv.data=shoulder_val;		
	shoulder_pub.publish(shoulder_conv);
	shoulder_degrees.publish(shoulder_deg);
}

void Conversor::elbowEncoderCb(const std_msgs::Int16::ConstPtr& elbow)
{
//	ROS_INFO("calllback");
	elbow_lec=elbow->data;
	elbow_val=(elbow_lec*3.1416/200) - 7*3.1416/5;	
	std_msgs::Float32 elbow_conv;
	std_msgs::Float32 elbow_deg;
	elbow_deg.data=elbow_val*180/3.1416;
	elbow_conv.data=elbow_val;		
	elbow_pub.publish(elbow_conv);
	elbow_degrees.publish(elbow_deg);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"conversor_base");
	Conversor obj;
	obj.spin();

//	return 0;
}
