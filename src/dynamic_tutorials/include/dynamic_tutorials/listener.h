#ifndef NODE_EXAMPLE_LISTENER_H
#define NODE_EXAMPLE_LISTENER_H

#include <ros/ros.h>
#include<ros/time.h>

#include <node_example/NodeexampleData.h>
namespace node_example
{
	class ExampleListener
	{
		public:
			explicit ExampleListener(ros::Nodehandle nh);
			void messageCallaback(const node_example::NodeexampleData::ConstPtr &msg);
		private:
			ros::Subscriber sub_;
	};
}
