#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <math.h>

class Encoder_calc
{
	public:
		Encoder_calc();
			
		void spin();
	private:
		
		ros::NodeHandle n;
		ros::Subscriber offsetSub;
		ros::Subscriber left_wheel_sub;
		ros::Publisher left_ang_pub;
		ros::Publisher left_vel_pub;
		
		
		double rate;
		
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
		
		void init_variables();
		
		void update();		

};

Encoder_calc::Encoder_calc()
{
	init_variables();
	ROS_INFO("Started computing encoder value");
	left_wheel_sub=n.subscribe("/left_lec",10,&Encoder_calc::leftencoderCb,this);
	offsetSub=n.subscribe("offset",10,&Encoder_calc::offsetCb,this);
	
	left_ang_pub=n.advertise<std_msgs::Float32>("left_ang",20);
	left_vel_pub=n.advertise<std_msgs::Float32>("left_vel",20);
	
}

void Encoder_calc::init_variables()
{


	rate=10;
//////////////////////////////////////////////////////////
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
/////////////////////////////////
	

		
}


void Encoder_calc::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}


void Encoder_calc::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	double d_left;
	if(now > t_next)
	{
		elapsed=now.toSec()-then.toSec();
		
		std_msgs::Float32 l_ang; 
		std_msgs::Float32 l_vel;
		
		l_ang.data=left_ang;
		l_vel.data=left_vel;		

		left_ang_pub.publish(l_ang);
		left_vel_pub.publish(l_vel);
		
		then=now;
		
		ros::spinOnce();		
	}
	else
	{;}
}


void Encoder_calc::leftencoderCb(const std_msgs::Int16::ConstPtr& left)
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
	
	left_ang_lap_lst=left_ang;
	left_ang=2*M_PI*left_ang_lap+left_ang_abs_internal-left_offset_internal;
	left_vel=10*(left_ang-left_ang_lap_lst);
	
}

void Encoder_calc::offsetCb(const std_msgs::Int16::ConstPtr& offst)
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


int main(int argc,char **argv)
{
	ros::init(argc,argv,"left_encoder");
	Encoder_calc obj;
	obj.spin();
	
	return 0;
}
