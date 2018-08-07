#include <dynamic_tutorials/listener.h>
namespace node_example
{
	ExampleListener::ExampleListener(ros::NodeHandle nh)
	{
		sub_=nh.subscribe("example",10,&ExampleListener::messageCallback,this);
	}
	void ExampleListener::messageCallback(const node_example::NodeExampleData::ConstPtr &msg)
	{
		ROS_INFO("message is %s, a+b=%d", msg->message.c_str(), msg->a + msg->b);
	}
}
