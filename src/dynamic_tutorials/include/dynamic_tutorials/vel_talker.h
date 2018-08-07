#ifndef NODE_VEL_TALKER_H
#define NODE_VEL_TALKER_H

#include <ros/ros.h>
#include <ros/time.h>

#include <dynamic_tutorials/NodeExampleData.h>

#include<dynamic_reconfigure/server.h>

#include<dynamic_tutorials/vel_PIDConfig.h>

#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <ros/console.h>
#include <string>
#include <iostream>



namespace node_vel
{
	class vel_PID
	{
		public:
		explicit vel_PID(ros::NodeHandle nh);
		std::string node_name;
		void get_params();
		void spin();		
				
		private:
		
		//functions for dynamic reconfigure setup
		

		
		void configCallback(dynamic_tutorials::vel_PIDConfig &config,uint32_t level);
		void timer1Callback(const ros::TimerEvent &event);
		void start(); //alike init variables, publishers, subscribers
		void stop();
		/////////////////////////////////////////////////
		//variables used for dynamic reconfigure
		ros::NodeHandle nh_;
		ros::Timer timer1_;
		ros::Publisher pub_;
		dynamic_reconfigure::Server<dynamic_tutorials::vel_PIDConfig> dr_srv_;

		double kp_vel_;
		double ki_vel_;
		double kd_vel_;
		double km_vel_;
		bool enable_;		
		bool enable;
		////////////////////////////////////////////////
		//variables used for PID control
		
		std::string name_nodo;
		ros::Subscriber offsetSub;
		ros::Subscriber wheel_sub;
//topics associated with vel input and output
		ros::Subscriber vel_desSub;
		ros::Publisher out_pub;
//publishers with info from encoder
		ros::Publisher ang_pub;
		ros::Publisher vel_pub;
		ros::Publisher name_pub;

		
		int rate;
		int rate_act;
		int ticks_since_target;
		int timeout_ticks;		

		double des;
		double vel_des;
		int out;
		double kp_vel;
		double ki_vel;
		double kd_vel;
		double km_vel;
		double umbral_vel;
		double range_vel;//max pwm allowed
		double kierr_vel;		
		double kimax_vel;
		double kisum_vel;
		double error_vel;


	
		double prev_error;


///encoder lecture and treatment associated variables
		int times;		
		double enc_;
		int lec;
		double ang;
		double vel;
		double vel_prev;
		
		int enc_max;
		int offset;
		
		double ang_tmp;
		double ang_lst;
		double ang_abs;
		double ang_lap;
		
		double ang_lap_lst	;
		double offset_internal;
		double ang_tmp_internal;
		double ang_lst_internal;
		double ang_abs_internal;
		
//variables asociadas a la odometria
		
		double ticks_meter;//param which relate wheel width with encoder ticks
		double base_width;
		
		//timing associated variables
		ros::Duration t_delta;
		
		ros::Time t_next;
		
		ros::Time then;
		
		ros::Time current_time,last_time;
//function declaration for PID control

		void encoderCb(const std_msgs::Int16::ConstPtr& enc);
		
		void offsetCb(const std_msgs::Int16::ConstPtr& offst);
		
		void desCb(const std_msgs::Float32::ConstPtr& des);
		
		void init_variables();
		

		
		void update();		
	};
}
#endif 
