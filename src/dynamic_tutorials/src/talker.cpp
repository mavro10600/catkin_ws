#include <dynamic_tutorials/talker.h>
/*
namespace node_example
{

ExampleTalker::ExampleTalker(ros::NodeHandle nh) :
  a_(1), b_(2), message_("hello")
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
  cb = boost::bind(&ExampleTalker::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Declare variables that can be modified by launch file or command line.
  int rate;

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");
  pnh.param("a", a_, a_);
  pnh.param("b", b_, b_);
  pnh.param("message", message_, message_);
  pnh.param("rate", rate, 1);

  // Create a publisher and name the topic.
  pub_ = nh.advertise<dynamic_tutorials::NodeExampleData>("example", 10);

  // Create timer.
  timer_ = nh.createTimer(ros::Duration(1 / rate), &ExampleTalker::timerCallback, this);
}

void ExampleTalker::timerCallback(const ros::TimerEvent& event)
{
  dynamic_tutorials::NodeExampleData msg;
  msg.message = message_;
  msg.a = a_;
  msg.b = b_;

  pub_.publish(msg);
}

void ExampleTalker::configCallback(node_example::nodeExampleConfig& config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  message_ = config.Message.c_str();
  a_ = config.a;
  b_ = config.b;
}

}
*/

namespace node_example
{
	ExampleTalker::ExampleTalker ( ros::NodeHandle nh) : nh_(nh), message_("hello"),a_(1),b_(2), enable_(true)
	{
		dynamic_reconfigure::Server<node_example::nodeExampleConfig>::CallbackType cb;
		cb=boost::bind(&ExampleTalker::configCallback, this, _1,_2);
		dr_srv_.setCallback(cb);
		
		
		int rate =1;
		ros::NodeHandle pnh("~");
		pnh.param("a",a_,a_);
		pnh.param("b",b_,b_);
		pnh.param("message",message_,message_);
		pnh.param("rate",rate,rate);
		pnh.param("enable",enable_,enable_);
		
		if(enable_)
		{
			start();
		}
		timer_=nh_.createTimer(ros::Duration(1/rate),&ExampleTalker::timerCallback,this);
		
	}
	
	
	void ExampleTalker::start()
	{
		pub_=nh_.advertise<dynamic_tutorials::NodeExampleData>(message_,10);
	}
	
	void ExampleTalker::stop()
	{
		pub_.shutdown();	
	}
	
	
	void ExampleTalker::timerCallback(const ros::TimerEvent &event)
	{
		if(!enable_)
		return;
		
		dynamic_tutorials::NodeExampleData msg;
		msg.message=message_;
		msg.a=a_;
		msg.b=b_;
		pub_.publish(msg);		
	}
	
	void ExampleTalker::configCallback(node_example::nodeExampleConfig &config, uint32_t level)
	{
		message_=config.Message;
		a_=config.a;
		b_=config.b;
		if (enable_!= config.enable)
		{
			if(config.enable)
			{start();}
			else
			{stop();}
		}
		enable_=config.enable;
	}
}

