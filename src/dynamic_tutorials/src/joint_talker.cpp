#include <dynamic_tutorials/joint_talker.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <ros/console.h>
#include <string>
#include <iostream>

namespace node_joint
{
	joint_PID::joint_PID(ros::NodeHandle nh) : nh_(nh), kp_pos(30), ki_pos(0.1), kd_pos(0), km_pos(10),enable_(true)
	{
	dynamic_reconfigure::Server <dynamic_tutorials::joint_PIDConfig>::CallbackType cb;
	cb = boost::bind(&joint_PID::configCallback,this,_1,_2);
	dr_srv_.setCallback(cb);
	rate_act=1;
	rate=10;
	ros::NodeHandle pnh("~");
	pnh.param("offset_start",offset_start_,offset_start_);
	pnh.param("offset_end",offset_end_,offset_end_);
	pnh.param("kp_pos",kp_pos_,kp_pos_);
	pnh.param("ki_pos",ki_pos_,ki_pos_);
	pnh.param("kd_pos",kd_pos_,kd_pos_);
	pnh.param("km_pos",km_pos_,km_pos_);
	pnh.param("enable", enable_, enable_);
	if(enable_)
	{
		start();
	}
	timer1_=nh.createTimer(ros::Duration(1/rate_act),&joint_PID::timer1Callback,this);
	
	}
	
	void joint_PID::start()
	{
		//inicia las demas variables y los publicadores y subscriptores
			init_variables();
	ROS_INFO("started PID_node");


	joint_sub=nh_.subscribe("lec",10,&joint_PID::encoderCb,this);
	offsetSub=nh_.subscribe("offset",10,&joint_PID::offsetCb,this);
	pos_desSub=nh_.subscribe("pos_des",10,&joint_PID::desCb,this);
	
	ang_pub=nh_.advertise<std_msgs::Float32>("ang",20);
	pos_pub=nh_.advertise<std_msgs::Float32>("pos",20);
	vel_pub=nh_.advertise<std_msgs::Float32>("vel",20);
	out_pub=nh_.advertise<std_msgs::Int16>("out",1);
	name_pub=nh_.advertise<std_msgs::String>("name",1);
	ROS_INFO("end_start");
	}
	void joint_PID::stop()
	{
		//puedes poner algun topico en off 
	}
	void joint_PID::timer1Callback(const ros::TimerEvent &event)
	{
		if(!enable_)
		return;
		//rate=rate_;
		offset_start=offset_start_;
		offset_end=offset_end_;
		kp_pos=kp_pos_;
		ki_pos=ki_pos_;
		kd_pos=kd_pos_;
		km_pos=km_pos_;
		enable=enable_;
		
		ROS_INFO_STREAM("offset_start" << offset_start);
		ROS_INFO_STREAM("offset_end" << offset_end);
		ROS_INFO_STREAM("kp" << kp_vel);
		ROS_INFO_STREAM("ki" << ki_vel);
		ROS_INFO_STREAM("kd" << kd_vel);
		ROS_INFO_STREAM("km" << km_vel);
	
	}
	
	void joint_PID::configCallback(dynamic_tutorials::joint_PIDConfig &config, uint32_t level)
	{
		rate_=config.rate_dyn;
		offset_start_=config.offset_init;
		offset_end_=config.offset_final;
		kp_pos_=config.kp;
		ki_pos_=config.ki;
		kd_pos_=config.kd;
		km_pos_=config.km;
		enable_=config.enable;
	}


void joint_PID::init_variables()
{
	
	rate=10;
	rate_act=1;
	timeout_ticks=4;
	ticks_since_target=0;
	prev_error=0;
	name_nodo=node_name;
//////////////////////////////////////////////////////////

/////////////////////////////////
	
ROS_INFO("name_nodo");
std::cout<<name_nodo<<std::endl;
		des=0;
		pos_des=0;
		out=0;

		kp_pos=30;
		ki_pos=1;
		kd_pos=0;
		km_pos=20;
		umbral_pos=0.1;
		range_pos=100;//max pwm allowed
		kierr_pos=0.1;		
		kimax_pos=100;
		kisum_pos=0;
		error_pos=0;


//encoder AS5043 associated variables

 		times=0;		
		lec=0;
		ang=0;
//		vel=0;
		
		enc_max=1023;
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
//Timing variables	
	
	t_delta=ros::Duration(1.0/rate);
	t_next=ros::Time::now()+t_delta;
	
	then=ros::Time::now();
	
	current_time=ros::Time::now();
	last_time=ros::Time::now();
		
		ROS_INFO("todo bien 2");
}



void joint_PID::get_params()
{
ROS_INFO("parameters:");


		if(nh_.getParam("rate",rate))
	{
		ROS_INFO_STREAM("Rate form param" << rate);
	}	
	
		if(nh_.getParam("/"+ node_name +"/kp_vel",kp_pos))
	{
		ROS_INFO_STREAM("Kp_vel from param" << kp_pos);
	}

		if(nh_.getParam("/"+ node_name +"/ki_vel",ki_pos))
	{
		ROS_INFO_STREAM("Ki_vel from param" << ki_pos);
	}

		if(nh_.getParam("/"+ node_name +"/kd_vel",kd_pos))
	{
		ROS_INFO_STREAM("Kd_vel from param" << kd_pos);
	}	

		if(nh_.getParam("/"+ node_name +"/km_vel",km_pos))
	{
		ROS_INFO_STREAM("Km_vel from param" << km_pos);
	}

		if(nh_.getParam("/"+ node_name +"range_vel",range_pos))
	{
		ROS_INFO_STREAM("range_vel from param" << range_pos);
	}

}


void joint_PID::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}

	//	update();
	
}

void joint_PID::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	double d_left;
	double vel;
	double error_temp;
	double pos;

	if(now > t_next)
	{
	
	//calculate velocity
		elapsed=now.toSec()-then.toSec();

		//if(abs(vel)>.1)
//		vel=vel/elapsed;  velocidad angular, pero el target vine en velocidad lineal
		vel=(ang-ang_lap_lst)/elapsed; 
		//map from the values from the encoder taking the offsets into account (offset_start) and (offset_end)
		if(offset_start>offset_end)
		pos=ang-offset_start;
		if(offset_start<offset_end)
		pos=offset_start-ang;
		//else
		//vel=0;		
//		ROS_DEBUG("vel: %0.2f",vel);		
	if(ticks_since_target<timeout_ticks)
	{	

	//do pid	
	
		error_temp=pos_des-pos;
		
		if(error_temp > -umbral_pos && error_temp < umbral_pos)	
		{error_pos=0;}
		else{error_pos=error_temp;}
	
		if(error_temp < -kierr_pos || error_temp > kierr_pos)
		{
		if(error_temp > -umbral_pos && error_temp < umbral_pos )
		{kisum_pos=0;}
			else
			{
				kisum_pos+=error_pos*elapsed;
				if(kisum_pos > kimax_pos){kisum_pos=kimax_pos;}
				if(kisum_pos < -kimax_pos){kisum_pos=-kimax_pos;}
			}
		}
		else
		{kisum_pos=0;}
	


	out=kp_pos*error_pos+kisum_pos*ki_pos-kd_pos*(error_pos-prev_error)/elapsed;
//	

	prev_error=error_pos;	

	if(out > range_pos){out=range_pos;}
	if(out < -range_pos){out=-range_pos;}
	
	//here, we limit the output of the controller in one direction taking into account the offset_start and offset_end variables. So if we take the direction of change of the joint named velocity, we could have than the offset_start is greater than offset_end, and in that case we have a positive or negative velocity, so if gives the info we need to define into zero the out variable in the software limit declared.

if(offset_end>offset_start)
{
	if(vel>0) out=0;
	else out=out;
	
}
if(offset_start>offset_end)
{
	if(vel>0) out=out;
	else out=0;
}

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
		std_msgs::Float32 l_pos; 		 
		std_msgs::Float32 l_vel;
		std_msgs::Int16 l_out;
		
		l_ang.data=ang;
		l_pos.data=pos;
		l_vel.data=vel;	
		l_out.data=out;	

		ang_pub.publish(l_ang);
		pos_pub.publish(l_pos);
		vel_pub.publish(l_vel);
		out_pub.publish(l_out);		
				
		then=now;
		ang_lap_lst=ang;
		ros::spinOnce();
		//returns control to ros to handle callbacks		
	}
	else
	{;}
}


void joint_PID::encoderCb(const std_msgs::Int16::ConstPtr& enc)
{
	lec=enc->data;
	
	times=times+1;
	
	offset_internal= (offset-0)*(0-2*M_PI+0)/(enc_max-0)+enc_max;
	
	ang_tmp_internal=(lec-0)*(0-2*M_PI+0)/(enc_max-0)+enc_max;
	
	ang_lst_internal=ang_abs_internal;
	ang_abs_internal=ang_tmp_internal;
	
	if(times>4)
	{
		if(ang_abs_internal>1.7*M_PI && ang_lst_internal<0.3*M_PI)
		{
			ang_lap-=1;
		}
				if(ang_abs_internal<0.3*M_PI && ang_lst_internal<1.7*M_PI)
		{
			ang_lap+=1;
		}
	}
	
	
	
	//prev_lencoder=enc;
	
	//ang_lap_lst=ang;
	ang=2*M_PI*ang_lap+ang_abs_internal-offset_internal;
	//vel=(ang-ang_lap_lst);
	
}

void joint_PID::offsetCb(const std_msgs::Int16::ConstPtr& offst)
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


void joint_PID::desCb(const std_msgs::Float32::ConstPtr& des)
{
	pos_des=des->data;
	
    ticks_since_target=0;	
}



	
}
