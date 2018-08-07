#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class Odometry_calc
{
	public:
		Odometry_calc();
			
		void spin();
	private:
		
		ros::NodeHandle n;
		ros::Subscriber sub;
		
		ros::Subscriber offset_left_Sub;
		ros::Subscriber offset_right_Sub;
		
		ros::Subscriber left_wheel_sub;
		ros::Subscriber right_wheel_sub;
		
		ros::Publisher odom_pub;
		
		tf::TransformBroadcaster odom_broadcaster;

		double rate;
		
///variables asociadas al encoder izquierdo		
		int times_left;		
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
		
///variables asociadas al encoder deecho
		int times_right;		
		double enc_right;
		int right_lec;
		double right_ang;
		double right_vel;
		
		int right_enc_max;
		int right_offset;
		
		double right_ang_tmp;
		double right_ang_lst;
		double right_ang_abs;
		double right_ang_lap;
		
		double right_ang_lap_lst	;
		double right_offset_internal;
		double right_ang_tmp_internal;
		double right_ang_lst_internal;
		double right_ang_abs_internal;		

//variables asociadas a la odometria
		
		double ticks_meter;//o algun valor que relacione el diametro de la llanta con el desplazamiento lineal
		double base_width;
		
		double dx;
		
		double dy;
		
		double dr;
		
		double x_final,y_final,theta_final;
		
		//variables asociadas al tiempo
		ros::Duration t_delta;
		
		ros::Time t_next;
		
		ros::Time then;
		
		ros::Time current_time,last_time;
		
//		void leftencoderCb(const std_msgs::Int64::ConstPtr& left_lec);
//Hay que modificar esta instruccion, el programa base_node toma los valores
//que vienen de la stellaris y los publica en que formato?		

		void leftencoderCb(const std_msgs::Int16::ConstPtr& left);
		
		void rightencoderCb(const std_msgs::Int16::ConstPtr& right);
		
		void offset_left_Cb(const std_msgs::Int16::ConstPtr& offst);
		
		void offset_right_Cb(const std_msgs::Int16::ConstPtr& offst);
		
		void init_variables();
		
		void get_node_params();
		
		
		void update();		

};

Odometry_calc::Odometry_calc()
{
	init_variables();
	ROS_INFO("Started computing encoder value");
	left_wheel_sub=n.subscribe("/left_lec",10,&Odometry_calc::leftencoderCb,this);
	offset_left_Sub=n.subscribe("offset",10,&Odometry_calc::offset_left_Cb,this);
	right_wheel_sub=n.subscribe("/right_lec",10,&Odometry_calc::rightencoderCb,this);
	offset_right_Sub=n.subscribe("offset",10,&Odometry_calc::offset_right_Cb,this);
	
	
	odom_pub=n.advertise<nav_msgs::Odometry>("odom",50);	
	
	get_node_params();
}



void Odometry_calc::init_variables()
{


	rate=10;
//////////////////////////////////////////////////////////
//vaiable sasociadas al encoder

 		times_left=0;		
 		times_right=0;		 		
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

//////////////////////////////////////////////////////////
//vaiable sasociadas al encoder derecho

		right_lec=0;
		right_ang=0;
		right_vel=0;
		
		right_enc_max=1023;
		right_offset=0;
		
		right_ang_tmp;
		right_ang_lst=0;
		right_ang_abs=0;
		right_ang_lap=0;
		
		right_offset_internal=0;
		right_ang_tmp_internal=0;
		right_ang_lst_internal=0;
		right_ang_abs_internal=0;
		right_ang_lap_lst=0;

	
////////////////////////////////////////////////////////////
//variable sasociadas al tiempo	
	
	t_delta=ros::Duration(1.0/rate);
	t_next=ros::Time::now()+t_delta;
	
	then=ros::Time::now();
	
	current_time=ros::Time::now();
	last_time=ros::Time::now();
/////////////////////////////////
	ticks_meter=.06;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      ;
	base_width=.5;
	dx=0;
	dy=0;
	x_final=0;y_final=0;theta_final=0;

		
}

void Odometry_calc::get_node_params()
{
	if(n.getParam("rate",rate))
	{
		ROS_INFO_STREAM("Rate form param" << rate);
	}	
	
		if(n.getParam("ticks_meter",ticks_meter))
	{
		ROS_INFO_STREAM("ticks_meter" << ticks_meter);
	}	
	
		if(n.getParam("base_width",base_width))
	{
		ROS_INFO_STREAM("Base Width" << base_width);
	}	
}


void Odometry_calc::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}


void Odometry_calc::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	double d_left,d_right,d,th,x,y;
	
	if(now > t_next)
	{
		elapsed=now.toSec()-then.toSec();
		
		if(left_ang==0)
		{
			d_left=0;
			d_right=0;
		}
		else
		{
			d_left=(left_ang-enc_left)*ticks_meter;
			d_right=(right_ang-enc_right)*ticks_meter;
		}
		
		enc_left=left_ang;
		enc_right=right_ang;		
		
		d=(d_right-d_left)/2.0;
		
		th=(d_right-d_left)/base_width;
		
		dx=d/elapsed;
		
		dr=th/elapsed;
		
		if(d!=0)
		{
			x=cos(th)*d;
			y=-sin(th)*d;
			
			x_final=x_final+(cos(theta_final)*x-sin(theta_final)*y);
			y_final=y_final+(sin(theta_final)*x+cos(theta_final)*y);
		}
		
		if(th!=0)
			theta_final=theta_final+th;
		
		geometry_msgs::Quaternion odom_quat;
		
		odom_quat.x=0.0;
		odom_quat.y=0.0;
		odom_quat.z=0.0;
		
		odom_quat.z=sin(theta_final/2);
		odom_quat.w=cos(theta_final/2);
		
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp=now;
		odom_trans.header.frame_id="odom";
		odom_trans.child_frame_id="base_footprint";
		
		odom_trans.transform.translation.x=x_final;
		odom_trans.transform.translation.y=y_final;
		odom_trans.transform.translation.z=0.0;
		odom_trans.transform.rotation=odom_quat;
		
		odom_broadcaster.sendTransform(odom_trans);
		
		nav_msgs::Odometry odom;
		odom.header.stamp=now;
		odom.header.frame_id="odom";
		
		odom.pose.pose.position.x=x_final;
		odom.pose.pose.position.y=y_final;
		odom.pose.pose.position.z=0.0;
		odom.pose.pose.orientation=odom_quat;
			
		odom.child_frame_id="base_footprint";
		odom.twist.twist.linear.x=dx;
		odom.twist.twist.linear.y=0;
		odom.twist.twist.angular.z=dr;
		
		odom_pub.publish(odom);
	
		then=now;
		
		ros::spinOnce();		
	}
	else
	{;}
}


void Odometry_calc::leftencoderCb(const std_msgs::Int16::ConstPtr& left)
{
	left_lec=left->data;
	
	times_left=times_left+1;
	
	left_offset_internal= (left_offset-0)*(0-2*M_PI+0)/(left_enc_max-0)+left_enc_max;
	
	left_ang_tmp_internal=(left_lec-0)*(0-2*M_PI+0)/(left_enc_max-0)+left_enc_max;
	
	left_ang_lst_internal=left_ang_abs_internal;
	left_ang_abs_internal=left_ang_tmp_internal;
	
	if(times_left>4)
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

void Odometry_calc::rightencoderCb(const std_msgs::Int16::ConstPtr& right)
{
	right_lec=right->data;
	
	times_right=times_right+1;
	
	right_offset_internal= (right_offset-0)*(0-2*M_PI+0)/(right_enc_max-0)+right_enc_max;
	
	right_ang_tmp_internal=(right_lec-0)*(0-2*M_PI+0)/(right_enc_max-0)+right_enc_max;
	
	right_ang_lst_internal=right_ang_abs_internal;
	right_ang_abs_internal=right_ang_tmp_internal;
	
	if(times_right>4)
	{
		if(right_ang_abs_internal>1.7*M_PI && right_ang_lst_internal<0.3*M_PI)
		{
			right_ang_lap-=1;
		}
				if(right_ang_abs_internal<0.3*M_PI && right_ang_lst_internal<1.7*M_PI)
		{
			right_ang_lap+=1;
		}
	}
	
	
	//Aqui hacer la magia tomando en cuenta que es un encoder absoluto
	
	//prev_lencoder=enc;
	
	right_ang_lap_lst=right_ang;
	right_ang=2*M_PI*right_ang_lap+right_ang_abs_internal-right_offset_internal;
	right_vel=10*(right_ang-right_ang_lap_lst);
	
}

void Odometry_calc::offset_left_Cb(const std_msgs::Int16::ConstPtr& offst)
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

void Odometry_calc::offset_right_Cb(const std_msgs::Int16::ConstPtr& offst)
{
	int offset=offst->data;
	
	if(offset==1)
	{
		right_offset=left_lec;
		right_ang_tmp=0;
		right_ang_lst=0;
		right_ang_abs=0;
		
		right_ang=0;
		right_ang_lap=0;
		right_ang_lap_lst=0;
	}
	
	
}


int main(int argc,char **argv)
{
	ros::init(argc,argv,"left_encoder");
	Odometry_calc obj;
	obj.spin();
	
	return 0;
}
