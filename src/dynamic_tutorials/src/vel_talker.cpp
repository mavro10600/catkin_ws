#include <dynamic_tutorials/vel_talker.h>
#include "ros/ros.h"
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
	vel_PID::vel_PID(ros::NodeHandle nh) : nh_(nh), kp_vel(30), ki_vel(0.1), kd_vel(0), km_vel(10),enable_(true)
	{
	dynamic_reconfigure::Server <dynamic_tutorials::vel_PIDConfig>::CallbackType cb;
	cb = boost::bind(&vel_PID::configCallback,this,_1,_2);
	dr_srv_.setCallback(cb);
	rate_act=1;
	rate=10;
	ros::NodeHandle pnh("~");
	pnh.param("kp_vel",kp_vel_,kp_vel_);
	pnh.param("ki_vel",ki_vel_,ki_vel_);
	pnh.param("kd_vel",kd_vel_,kd_vel_);
	pnh.param("km_vel",km_vel_,km_vel_);
	pnh.param("enable", enable_, enable_);
	if(enable_)
	{
		start();
	}
	timer1_=nh.createTimer(ros::Duration(1/rate_act),&vel_PID::timer1Callback,this);
	
	}
	
	void vel_PID::start()
	{
		//inicia las demas variables y los publicadores y subscriptores
			init_variables();
	ROS_INFO("started PID_node");


	wheel_sub=nh_.subscribe("lec",1,&vel_PID::encoderCb,this);
	offsetSub=nh_.subscribe("offset",10,&vel_PID::offsetCb,this);
	vel_desSub=nh_.subscribe("vel_des",10,&vel_PID::desCb,this);
	
	ang_pub=nh_.advertise<std_msgs::Float32>("ang",20);
	vel_pub=nh_.advertise<std_msgs::Float32>("vel",20);
	out_pub=nh_.advertise<std_msgs::Int16>("out",1);
	name_pub=nh_.advertise<std_msgs::String>("name",1);
	ROS_INFO("end_start");
	}
	void vel_PID::stop()
	{
		//puedes poner algun topico en off 
	}
	void vel_PID::timer1Callback(const ros::TimerEvent &event)
	{
		if(!enable_)
		return;
		kp_vel=kp_vel_;
		ki_vel=ki_vel_;
		kd_vel=kd_vel_;
		km_vel=km_vel_;
		enable=enable_;
		ROS_INFO_STREAM("kp" << kp_vel);
	ROS_INFO_STREAM("ki" << ki_vel);
	ROS_INFO_STREAM("kd" << kd_vel);
	ROS_INFO_STREAM("km" << km_vel);
	
	}
	
	void vel_PID::configCallback(dynamic_tutorials::vel_PIDConfig &config, uint32_t level)
	{
		kp_vel_=config.kp;
		ki_vel_=config.ki;
		kd_vel_=config.kd;
		km_vel_=config.km;
		enable_=config.enable;
	}


void vel_PID::init_variables()
{
	
	rate=30;
	rate_act=1;
	timeout_ticks=4;
	ticks_since_target=0;
	prev_error=0;
	vel_prev=0;
	name_nodo=node_name;
//////////////////////////////////////////////////////////

/////////////////////////////////
	
ROS_INFO("name_nodo");
std::cout<<name_nodo<<std::endl;
		des=0;
		vel_des=0;
		out=0;

		kp_vel=30;
		ki_vel=1;
		kd_vel=0;
		km_vel=20;
		umbral_vel=0.1;
		range_vel=100;//maximo pwm permitido
		kierr_vel=0.1;		
		kimax_vel=100;
		kisum_vel=0;
		error_vel=0;


//vaiable sasociadas al encoder

 		times=0;		
		lec=0;
		ang=0;
		vel=0;
		
		enc_max=511;
		offset=0;
		
		ang_tmp;
		ang_lst=0;
		ang_abs=0;
		ang_lap=0;
		
		offset_internal=0;
		ang_tmp_internal=0;
		ang_lst_internal=0;
		ang_abs_internal=0;
		ang_lap_lst=0;
	
////////////////////////////////////////////////////////////
//variable sasociadas al tiempo	
	
	t_delta=ros::Duration(1.0/rate);
	t_next=ros::Time::now()+t_delta;
	
	then=ros::Time::now();
	
	current_time=ros::Time::now();
	last_time=ros::Time::now();
		
		ROS_INFO("todo bien 2");
}



void vel_PID::get_params()
{
ROS_INFO("parametros:");


		if(nh_.getParam("rate",rate))
	{
		ROS_INFO_STREAM("Rate form param" << rate);
	}	
	
		if(nh_.getParam("/"+ node_name +"/kp_vel",kp_vel))
	{
		ROS_INFO_STREAM("Kp_vel from param" << kp_vel);
	}

		if(nh_.getParam("/"+ node_name +"/ki_vel",ki_vel))
	{
		ROS_INFO_STREAM("Ki_vel from param" << ki_vel);
	}

		if(nh_.getParam("/"+ node_name +"/kd_vel",kd_vel))
	{
		ROS_INFO_STREAM("Kd_vel from param" << kd_vel);
	}	

		if(nh_.getParam("/"+ node_name +"/km_vel",km_vel))
	{
		ROS_INFO_STREAM("Km_vel from param" << km_vel);
	}

		if(nh_.getParam("/"+ node_name +"range_vel",range_vel))
	{
		ROS_INFO_STREAM("range_vel from param" << range_vel);
	}

}


void vel_PID::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}

	//	update();
	
}

void vel_PID::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	double d_left;
	double vel;
	double temp;
	double error_temp;

	if(now > t_next)
	{
	
	//calculate velocity
		elapsed=now.toSec()-then.toSec();
		ROS_INFO("elapsed: %0.4f",elapsed);
//		ROS_INFO("ang: %0.2f",ang);
//		ROS_INFO("ang_lst: %0.2f",ang_lap_lst);
		temp=fabs(ang-ang_lap_lst);
		ROS_INFO("temp: %0.2f",temp);		
		//if(abs(vel)>.1)
//		vel=vel/elapsed;  velocidad angular, pero el target vine en velocidad lineal
		if(temp > 0.1 )
		vel=temp*0.05/elapsed;
		else
		vel=0;
		ROS_INFO("vel: %0.2f",vel);
		//else
		//vel=0;		
//		ROS_DEBUG("vel: %0.2f",vel);

		if (fabs(vel_prev)>0.1)
		vel=(vel_prev + vel)/2;
		
	if(ticks_since_target<timeout_ticks)
	{	

	//do pid	
	
		error_temp=vel_des-vel;
		
		if(error_temp > -umbral_vel && error_temp < umbral_vel)	
		{error_vel=0;}
		else{error_vel=error_temp;}
	
		if(error_temp < -kierr_vel || error_temp > kierr_vel)
		{
		if(error_temp > -umbral_vel && error_temp < umbral_vel )
		{kisum_vel=0;}
			else
			{
				kisum_vel+=error_vel*elapsed;
				if(kisum_vel > kimax_vel){kisum_vel=kimax_vel;}
				if(kisum_vel < -kimax_vel){kisum_vel=-kimax_vel;}
			}
		}
		else
		{kisum_vel=0;}
	


	out=kp_vel*error_vel+kisum_vel*ki_vel-kd_vel*(error_vel-prev_error)/elapsed;
//	

	prev_error=error_vel;	

	
	if(out > range_vel){out=range_vel;}
	if(out < -range_vel){out=-range_vel;}
	//ROS_DEBUG("vel: %0.2f tar: %0.2f err:%0.2f kisum: %0.2f left_out: %0.2f ticks:%d",vel,left_vel_des,error_vel,kisum_vel,left_out,ticks_since_target);		
	
	/*ROS_INFO_STREAM("Vel" << vel);
	ROS_INFO_STREAM("Tar" << vel_des);

	ROS_INFO_STREAM("Err"<< error_vel);
	

	ROS_INFO_STREAM("out" << out);
	ROS_INFO_STREAM("ticks" << ticks_since_target);
*/
		//end of pid
	ticks_since_target+=1;
		
	if(ticks_since_target==timeout_ticks)
	{
		out=0;
	}
		
	}	
		std_msgs::Float32 l_ang; 
		std_msgs::Float32 l_vel;
		std_msgs::Int16 l_out;
		
 		l_ang.data=ang;
		l_vel.data=vel;	
		l_out.data=out;	

		ang_pub.publish(l_ang);
		vel_pub.publish(l_vel);
		out_pub.publish(l_out);		
	
		vel_prev=vel;
		then=now;
		ang_lap_lst=ang;
		ros::spinOnce();
		//returns control to ros to handle callbacks		
	}
	else
	{;}
}


void vel_PID::encoderCb(const std_msgs::Int16::ConstPtr& enc)
{
	lec=enc->data;
	
	times=times+1;
	
	offset_internal= (offset-0)*(0-2*M_PI+0)/(enc_max-0)+enc_max;
	
	ang_tmp_internal=(lec-0)*(0-2*M_PI+0)/(enc_max-0)+enc_max;
	
	ang_lst_internal=ang_abs_internal;
	ang_abs_internal=ang_tmp_internal;
	
	if(times>4)
	{
		if(ang_abs_internal > -1.7*M_PI && ang_lst_internal < -0.3*M_PI)
		{
			ang_lap-=1;
		}
				if(ang_abs_internal < -0.3*M_PI && ang_lst_internal < -1.7*M_PI)
		{
			ang_lap+=1;
		}
	}
	
	
	//Aqui hacer la magia tomando en cuenta que es un encoder absoluto
	
//	prev_lencoder=lec;
	
	//ang_lap_lst=ang;
	ang=2*M_PI*ang_lap+ang_abs_internal-offset_internal;
//	ROS_INFO("ang: %0.2f",ang);
//	ROS_INFO("ang_last: %0.2f",ang_lap_lst);
//	vel=(ang-ang_lap_lst);
//	ang_lap_lst=ang;

}

void vel_PID::offsetCb(const std_msgs::Int16::ConstPtr& offst)
{
	int offset=offst->data;
	
	if(offset==1)
	{
		offset=lec;
		ang_tmp=0;
		ang_lst=0;
		ang_abs=0;
		
		ang=0;
		ang_lap=0;
		ang_lap_lst=0;
	}
	
	
}


void vel_PID::desCb(const std_msgs::Float32::ConstPtr& des)
{
	vel_des=des->data;
	
    ticks_since_target=0;	
}



	
}
