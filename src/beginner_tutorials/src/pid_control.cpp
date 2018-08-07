#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <ros/console.h>

class PID_control
{
	public:
		PID_control();
		void spin();
		
	private:

		ros::NodeHandle n;
		ros::Subscriber offsetSub;
		ros::Subscriber left_wheel_sub;
//variables asociads al motor
		ros::Subscriber left_pos_desSub;
		ros::Publisher left_out_pub;
//variables asociadas al encoder
		ros::Publisher left_ang_pub;
		ros::Publisher left_vel_pub;
		
		
		double rate;
		
		int ticks_since_target;
		int timeout_ticks;		
//variables asociadas al control de posicion
		double left_des;
		double left_ang_des;
		int left_out;

		double kp_pos;
		double ki_pos;
		double kd_pos;
		double km_pos;
		double umbral_pos;
		double range_pos;//maximo pwm permitido
		double kierr_pos;		
		double kimax_pos;
		double kisum_pos;
		double error_pos;
	
		double kp_vel;
		double ki_vel;
		double kd_vel;
		double km_vel;
		double umbral_vel;
		double range_vel;//maximo pwm permitido
		double kierr_vel;		
		double kimax_vel;
		double kisum_vel;
		double error_vel;


	
		double prev_error;


///variables asociadas al encoder		
		int times;		
		double enc_left;
		int left_lec;
		double left_ang;
		double left_vel;
		
		int left_enc_max;
		int left_offset;
		
		double left_ang_tmp;
		double left_ang_lst;
		double left_ang_abs;
		double left_ang_lap;
		
		double left_ang_lap_lst	;
		double left_offset_internal;
		double left_ang_tmp_internal;
		double left_ang_lst_internal;
		double left_ang_abs_internal;
		
//variables asociadas a la odometria
		
		double ticks_meter;//o algun valor que relacione el diametro de la llanta con el desplazamiento lineal
		double base_width;
		
		//variables asociadas al tiempo
		ros::Duration t_delta;
		
		ros::Time t_next;
		
		ros::Time then;
		
		ros::Time current_time,last_time;
		
//		void leftencoderCb(const std_msgs::Int64::ConstPtr& left_lec);
//Hay que modificar esta instruccion, el programa base_node toma los valores
//que vienen de la stellaris y los publica en que formato?		

		void leftencoderCb(const std_msgs::Int16::ConstPtr& left);
		
		void offsetCb(const std_msgs::Int16::ConstPtr& offst);
		
		void left_desCb(const std_msgs::Float32::ConstPtr& left_des);
		
		void init_variables();
		
		void get_params();
		
		void update();		
};

PID_control::PID_control()
{
	init_variables();
	ROS_INFO("started PID_node");


	left_wheel_sub=n.subscribe("left_lec",10,&PID_control::leftencoderCb,this);
	offsetSub=n.subscribe("offset",10,&PID_control::offsetCb,this);
	left_pos_desSub=n.subscribe("left_pos_des",10,&PID_control::left_desCb,this);
	
	left_ang_pub=n.advertise<std_msgs::Float32>("left_ang",20);
	left_vel_pub=n.advertise<std_msgs::Float32>("left_vel",20);
	left_out_pub=n.advertise<std_msgs::Int16>("left_out",1);

	get_params();	
}

void PID_control::init_variables()
{
	
	rate=10;
	timeout_ticks=4;
	ticks_since_target=0;
	prev_error=0;
//////////////////////////////////////////////////////////

/////////////////////////////////
	
ROS_INFO("todo bien 1");
		left_des=0;
		left_ang_des=0;
		left_out=0;

		kp_pos=30;
		ki_pos=0;
		kd_pos=0;
		km_pos=20;
		umbral_pos=0.1;
		range_pos=100;//maximo pwm permitido
		kierr_pos=1.2;		
		kimax_pos=100;
		kisum_pos=0;
		error_pos=0;

		kp_vel=50;
		ki_vel=0;
		kd_vel=0;
		km_vel=0;
		umbral_vel=0.2;
		range_vel=100;//maximo pwm permitido
		kierr_vel=0.2;		
		kimax_vel=100;
		kisum_vel=0;
		error_vel=0;


//vaiable sasociadas al encoder

 		times=0;		
		left_lec=0;
		left_ang=0;
		left_vel=0;
		
		left_enc_max=1023;
		left_offset=0;
		
		left_ang_tmp;
		left_ang_lst=0;
		left_ang_abs=0;
		left_ang_lap=0;
		
		left_offset_internal=0;
		left_ang_tmp_internal=0;
		left_ang_lst_internal=0;
		left_ang_abs_internal=0;
		left_ang_lap_lst=0;
	
////////////////////////////////////////////////////////////
//variable sasociadas al tiempo	
	
	t_delta=ros::Duration(1.0/rate);
	t_next=ros::Time::now()+t_delta;
	
	then=ros::Time::now();
	
	current_time=ros::Time::now();
	last_time=ros::Time::now();
		
		ROS_INFO("todo bien 2");
}



void PID_control::get_params()
{
		if(n.getParam("rate",rate))
	{
		ROS_INFO_STREAM("Rate form param" << rate);
	}	
	
		if(n.getParam("Kp_vel",kp_vel))
	{
		ROS_INFO_STREAM("Kp_vel from param" << kp_vel);
	}

		if(n.getParam("Ki_vel",ki_vel))
	{
		ROS_INFO_STREAM("Ki_vel from param" << ki_vel);
	}

		if(n.getParam("Kd_vel",kd_vel))
	{
		ROS_INFO_STREAM("Kd_vel from param" << kd_vel);
	}	

		if(n.getParam("Km_vel",km_vel))
	{
		ROS_INFO_STREAM("Km_vel from param" << km_vel);
	}

		if(n.getParam("range_vel",range_vel))
	{
		ROS_INFO_STREAM("range_vel from param" << range_vel);
	}

}


void PID_control::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}

void PID_control::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	double d_left;
	double vel;
	double error_temp;

	if(now > t_next)
	{
	
	//calculate velocity
		elapsed=now.toSec()-then.toSec();

		left_vel=left_ang-left_ang_lap_lst;
		


//		if(abs(left_vel)>.01)
		vel=left_vel*.05/elapsed;
//		else
//		vel=0;		
//		ROS_DEBUG("vel: %0.2f",vel);		
	if(ticks_since_target<timeout_ticks)
	{	

	//do pid	
	
		error_temp=left_ang_des-vel;
		
		if(error_temp > -umbral_vel && error_temp < umbral_vel)	
		{error_vel=0;}
		else{error_vel=error_temp;}
	
		if(error_temp > -kierr_vel && error_temp < kierr_vel)
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
	


	left_out=kp_vel*error_vel+kisum_vel*ki_vel-kd_vel*(error_vel-prev_error)/elapsed;

	prev_error=error_vel;	

	if(left_out > range_pos){left_out=range_pos;}
	if(left_out < -range_pos){left_out=-range_pos;}
	//ROS_DEBUG("vel: %0.2f tar: %0.2f err:%0.2f kisum: %0.2f left_out: %0.2f ticks:%d",vel,left_ang_des,error_vel,kisum_vel,left_out,ticks_since_target);		
	
	ROS_INFO_STREAM("left_vel" << vel);
	ROS_INFO_STREAM("left_tar" << left_ang_des);

	ROS_INFO_STREAM("left_ang"<< left_ang);
	ROS_INFO_STREAM("left_ang_lst" << left_ang_lap_lst);
	ROS_INFO_STREAM("kierr" << kierr_vel);
	

	ROS_INFO_STREAM("left_out" << left_out);
	ROS_INFO_STREAM("error_vel" << error_vel);

		//end of pid
	ticks_since_target+=1;
		
	if(ticks_since_target==timeout_ticks)
	{
		left_out=0;
	}
		
	}	
		std_msgs::Float32 l_ang; 
		std_msgs::Float32 l_vel;
		std_msgs::Int16 l_out;
		
		l_ang.data=left_ang;
		l_vel.data=vel;	
		l_out.data=left_out;	

		left_ang_pub.publish(l_ang);
		left_vel_pub.publish(l_vel);
		left_out_pub.publish(l_out);		
				
		then=now;
		left_ang_lap_lst=left_ang;		
		ros::spinOnce();
		//returns control to ros to handle callbacks		
	}
	else
	{;}
}


void PID_control::leftencoderCb(const std_msgs::Int16::ConstPtr& left)
{
	left_lec=left->data;
	
	times=times+1;
	
	left_offset_internal= (left_offset-0)*(0-2*M_PI+0)/(left_enc_max-0)+left_enc_max;
	
	left_ang_tmp_internal=(left_lec-0)*(0-2*M_PI+0)/(left_enc_max-0)+left_enc_max;
	
	left_ang_lst_internal=left_ang_abs_internal;
	left_ang_abs_internal=left_ang_tmp_internal;
	
	if(times>4)
	{
		if(left_ang_abs_internal>1.7*M_PI && left_ang_lst_internal<0.3*M_PI)
		{
			left_ang_lap-=1;
		}
				if(left_ang_abs_internal<0.3*M_PI && left_ang_lst_internal<1.7*M_PI)
		{
			left_ang_lap+=1;
		}
	}
	
	
	//Aqui hacer la magia tomando en cuenta que es un encoder absoluto
	
	//prev_lencoder=enc;
	
//	left_ang_lap_lst=left_ang;
	left_ang=2*M_PI*left_ang_lap+left_ang_abs_internal-left_offset_internal;
//	left_vel=(left_ang-left_ang_lap_lst);
	
}

void PID_control::offsetCb(const std_msgs::Int16::ConstPtr& offst)
{
	int offset=offst->data;
	
	if(offset==1)
	{
		left_offset=left_lec;
		left_ang_tmp=0;
		left_ang_lst=0;
		left_ang_abs=0;
		
		left_ang=0;
		left_ang_lap=0;
		left_ang_lap_lst=0;
	}
	
}


void PID_control::left_desCb(const std_msgs::Float32::ConstPtr& left_des)
{
	left_ang_des=left_des->data;
	
    ticks_since_target=0;	
	
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"pid_control");
	PID_control obj;
	obj.spin();
	
	return 0;
}

