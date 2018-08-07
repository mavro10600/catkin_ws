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
		ros::Subscriber wheel_sub;
//variables asociads al motor
		ros::Subscriber pos_desSub;
		ros::Publisher out_pub;
//variables asociadas al encoder
		ros::Publisher ang_pub;
		ros::Publisher vel_pub;
		
		
		double rate;
		
		int ticks_since_target;
		int timeout_ticks;		
//variables asociadas al control de posicion
		double des;
		double ang_des;
		int out;

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
		int times;		

///variables asociadas al encoder		

	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_encoder;
	double enc_first;

	double mult;

	double first;
	
	double ang;
	
	double enc;
		

		
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

		void firstencoderCb(const std_msgs::Int16::ConstPtr& first);
		
		void offsetCb(const std_msgs::Int16::ConstPtr& offst);
		
		void desCb(const std_msgs::Float32::ConstPtr& des);
		
		void init_variables();
		
		void get_params();
		
		void update();		
};

PID_control::PID_control()
{
	init_variables();
	ROS_INFO("started PID_node");


	wheel_sub=n.subscribe("second_wheel",10,&PID_control::firstencoderCb,this);
	offsetSub=n.subscribe("offset",10,&PID_control::offsetCb,this);
	pos_desSub=n.subscribe("second_pos_des",10,&PID_control::desCb,this);
	
	ang_pub=n.advertise<std_msgs::Float32>("ang_second",20);
	vel_pub=n.advertise<std_msgs::Float32>("vel_second",20);
	out_pub=n.advertise<std_msgs::Int16>("first_wheel_speed",1);

	get_params();	
}

void PID_control::init_variables()
{
	
	rate=30;
	timeout_ticks=4;
	ticks_since_target=0;
	prev_error=0;
//////////////////////////////////////////////////////////

/////////////////////////////////
	
ROS_INFO("todo bien 1");
		des=0;
		ang_des=0;
		out=0;

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
		prev_encoder = 0;

		mult = 0;

		first = 0;

		encoder_min =  -2147483648;
		encoder_max =  2147483648;

		rate = 10;

		ticks_meter = 50;

		base_width = 0.3;

		enc_first=0;
	

		encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
		encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;


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
	double d_first;
	double vel;
	double error_temp;

	if(now > t_next)
	{
	
	//calculate velocity
		elapsed=now.toSec()-then.toSec();
	if(enc_first==0)
	{
		d_first=0;
		vel=0;
	}
	else 
	{
		d_first=(first-enc_first)/(ticks_meter);
		//ticks/(ticks/meter ) =meters
		//ticks/(ticks/rad))=rad
		//TODO:aqui revisar si la conversion no es a velocidad angular, porque aqui es a lineal
		vel=d_first/elapsed;
	}
	
	enc_first=first;
	

//		ROS_DEBUG("vel: %0.2f",vel);		
	if(ticks_since_target<timeout_ticks)
	{	

	//do pid	
	
		error_temp=ang_des-vel;
		
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

	prev_error=error_vel;	

	if(out > range_pos){out=range_pos;}
	if(out < -range_pos){out=-range_pos;}
	//ROS_DEBUG("vel: %0.2f tar: %0.2f err:%0.2f kisum: %0.2f left_out: %0.2f ticks:%d",vel,left_ang_des,error_vel,kisum_vel,left_out,ticks_since_target);		
	
	ROS_INFO_STREAM("Vel" << vel);
	ROS_INFO_STREAM("Tar" << ang_des);

	ROS_INFO_STREAM("Err"<< error_vel);
	ROS_INFO_STREAM("kisum" << kisum_vel);
	ROS_INFO_STREAM("kierr" << kierr_vel);
	

	ROS_INFO_STREAM("out" << out);
	ROS_INFO_STREAM("ticks" << ticks_since_target);

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
				
		then=now;
		prev_encoder = enc;
		ros::spinOnce();
		//returns control to ros to handle callbacks		
	}
	else
	{;}
}


void PID_control::firstencoderCb(const std_msgs::Int16::ConstPtr& frst)
{
	enc=frst->data;
	if((enc < encoder_low_wrap) && (prev_encoder > encoder_high_wrap))
	{
		
		mult = mult + 1;
	}
	

	if((enc > encoder_high_wrap) && (prev_encoder < encoder_low_wrap))

	{
		
		mult = mult - 1;
	}

	first = 1.0 * (enc + mult * (encoder_max - encoder_min ));

//	prev_encoder = enc;
	
	
}

void PID_control::offsetCb(const std_msgs::Int16::ConstPtr& offst)
{
	int offset=offst->data;
	
	if(offset==1)
	{
	ang=0;
/*		offset=lec;
		ang_tmp=0;
		ang_lst=0;
		ang_abs=0;
		
		ang=0;
		ang_lap=0;
		ang_lap_lst=0;*/
	}
	
	
}


void PID_control::desCb(const std_msgs::Float32::ConstPtr& des)
{
	ang_des=des->data;
	
    ticks_since_target=0;	
	
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"pid_control");
	PID_control obj;
	obj.spin();
	
	return 0;
}

