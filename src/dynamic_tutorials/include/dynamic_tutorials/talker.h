#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

#include <ros/ros.h>
#include <ros/time.h>

#include <dynamic_tutorials/NodeExampleData.h>

#include<dynamic_reconfigure/server.h>

#include<dynamic_tutorials/nodeExampleConfig.h>
/*
namespace node_example
{

class ExampleTalker
{
public:
  //! Constructor.
  ExampleTalker(ros::NodeHandle nh);

  //! Callback function for dynamic reconfigure server.
  void configCallback(node_example::nodeExampleConfig& config, uint32_t level);

  //! Timer callback for publishing message.
  void timerCallback(const ros::TimerEvent& event);

private:
  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;

  //! Message publisher.
  ros::Publisher pub_;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<node_example::nodeExampleConfig> dr_srv_;

  //! The actual message.
  std::string message_;

  //! The first integer to use in addition.
  int a_;

  //! The second integer to use in addition.
  int b_;
};

}
*/

namespace node_example
{
	class ExampleTalker
	{
		public:
		explicit ExampleTalker(ros::NodeHandle nh);
		
		private:
		void configCallback(node_example::nodeExampleConfig &config,uint32_t level);
		void timerCallback(const ros::TimerEvent &event);
		void start();
		void stop();
		
		ros::NodeHandle nh_;
		ros::Timer timer_;
		ros::Publisher pub_;
		dynamic_reconfigure::Server<node_example::nodeExampleConfig> dr_srv_;
		std::string message_;
		int a_;
		int b_;
		bool enable_;		
	};
}
#endif 
