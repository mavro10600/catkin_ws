#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <math.h>

class PID_left
{
	public:
		PID_left();
			
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
		
		void left_desCb(const std_msgs::Int16::ConstPtr& l_des);
		
		void init_variables();
		
		void get_params();
		
		void update();		

};

PID_left::PID_left()
{
	init_variables();
	ROS_INFO("Started computing encoder value");
	left_wheel_sub=n.subscribe("/left_lec",10,&PID_left::leftencoderCb,this);
	offsetSub=n.subscribe("offset",10,&PID_left::offsetCb,this);
	left_pos_desSub=n.subscribe("left_pos_des",10,&PID_left::left_desCb,this);
	
	left_ang_pub=n.advertise<std_msgs::Float32>("left_ang",20);
	left_vel_pub=n.advertise<std_msgs::Float32>("left_vel",20);
	left_out_pub=n.advertise<std_msgs::Int16>("left_out",1);

	get_params();
}

void PID_left::init_variables()
{


	rate=10;
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

void PID_left::get_params()
{
		if(n.getParam("rate",rate))
	{
		ROS_INFO_STREAM("Rate form param" << rate);
	}	
	

}


void PID_left::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}


void PID_left::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	double d_left;
	if(now > t_next)
	{
		elapsed=now.toSec()-then.toSec();
		
		std_msgs::Float32 l_ang; 
		std_msgs::Float32 l_vel;
		std_msgs::Int16 l_out;
		
		l_ang.data=left_ang;
		l_vel.data=left_vel;	
		l_out.data=left_out;	

		left_ang_pub.publish(l_ang);
		left_vel_pub.publish(l_vel);
		left_out_pub.publish(l_out);		
				
		then=now;
		
		ros::spinOnce();		
	}
	else
	{;}
}


void PID_left::leftencoderCb(const std_msgs::Int16::ConstPtr& left)
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

void PID_left::offsetCb(const std_msgs::Int16::ConstPtr& offst)
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


void PID_left::left_desCb(const std_msgs::Int16::ConstPtr& l_des)
{
	left_ang_des=l_des->data;
	
	if(abs(left_ang_des-left_ang)<umbral_pos)
	{error_pos=0;}
	else{error_pos=left_ang_des-left_ang;}
	
	if(abs(left_ang_des-left_ang)<kierr_pos)
	{
		if(abs(left_ang_des-left_ang)<umbral_pos)
		{kisum_pos=0;}
		else
		{
			kisum_pos+=ki_pos*error_pos;
			if(kisum_pos > kimax_pos){kisum_pos=kimax_pos;}
			if(kisum_pos < -kimax_pos){kisum_pos=-kimax_pos;}
		}
	}
	else
	{kisum_pos=0;}

	left_out=kp_pos*error_pos+kisum_pos-kd_pos*(left_ang-left_ang_lst)+km_pos*left_ang_des;
	
	if(left_out > range_pos){left_out=range_pos;}
	if(left_out < -range_pos){left_out=-range_pos;}

	//calcular el pid, bueno el valor de left_out
	
	
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"pid_left_encoder");
	PID_left obj;
	obj.spin();
	
	return 0;
}
