#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class TwistToMotors
{

public:
	TwistToMotors();
	void spin();

private:
	ros::NodeHandle n;
	
	ros::Publisher pub_lmotor;
	ros::Publisher pub_rmotor;

	ros::Subscriber cmd_vel_sub;

	float left;
	float right;

	float ticks_since_target;
	double timeout_ticks;

	double w;
	double rate;

	float dx,dy,dr;

	double map_vel;	
	
	void init_variables();
	void get_parameters();

	void spinOnce();
	void twistCallback(const geometry_msgs::Twist &twist_aux);


};

TwistToMotors::TwistToMotors()
{
	init_variables();

	
	ROS_INFO("Started Twist to Motor node");
	
	cmd_vel_sub = n.subscribe("base_controller/command",10, &TwistToMotors::twistCallback, this);
	
	pub_lmotor = n.advertise<std_msgs::Float32>("left_vel_des", 50);

	pub_rmotor = n.advertise<std_msgs::Float32>("right_vel_des", 50);
	


}

void TwistToMotors::init_variables()
{
	left = 0;
	right = 0;

	dx = dy = dr =0;

	w = 0.5 ;
	rate = 50;
	timeout_ticks = 5;

	map_vel=1; //0.3 m/s

}


void TwistToMotors::get_parameters()
{


	
        if(n.getParam("rate", rate)){
	 
		ROS_INFO_STREAM("Rate from param" << rate);	       
	}


	
        if(n.getParam("timeout_ticks", timeout_ticks)){
	 
		ROS_INFO_STREAM("timeout_ticks from param" << timeout_ticks);	       
	}

	
        if(n.getParam("base_width", w)){
	 
		ROS_INFO_STREAM("Base_width from param" << w);	       
	}


        if(n.getParam("100_vel", map_vel)){
	 
		ROS_INFO_STREAM("map_vel from param" << map_vel);	       
	}

}


void TwistToMotors::spin()
{
	ros::Rate r(rate);
	ros::Rate idle(10);

	ros::Time then = ros::Time::now();
	
	ticks_since_target = timeout_ticks;
	
	

	while (ros::ok())
	{
	while (ros::ok() && (ticks_since_target <= timeout_ticks))	
		{		

		spinOnce();
		r.sleep();

		}
	ros::spinOnce();
        idle.sleep();	

	}

}

void TwistToMotors::spinOnce()
{

double temp_right;
double temp_left;
/*
hay que redefinir el mapeo, dx viene en valores de cero a uno
dr viene en valores de 0 a la derecha, 180 a la izquierda 90 hacia adelante
-90 hacia  atras. lo primero seria que el signo de dx dependa del signo de dr. ahora, las ecuaciones significan algo. que el desplazamiento en x es el promedio de los desplazamientos de la rueda izquierda y derecha, y que el desplazamiento angular dr es igual a la distancia recorrida por la rueda derecha menos la izquierda, entre la distancia entre ruedas. Asi que creo que si va a funcionar
*/

        // dx = (l + r) / 2
        // dr = (r - l) / w
if( dr < -1.4 || dr > 1.4   )
{
//if dr>1.4 turn left, the
	right = ((dr * w /2));
	left = - (dr * w /2);
	
}
else
{	
	right =map_vel*( ( 1.0 * dx ) + (dr * w /2));
	left = map_vel*(( 1.0 * dx ) - (dr * w /2));
}

//	ROS_INFO_STREAM("right = " << right << "\t" << "left = " << left << "dr"<< dr);


	std_msgs::Float32 left_;
	std_msgs::Float32 right_;

	left_.data = left;
	right_.data = right;


	pub_lmotor.publish(left_);
	pub_rmotor.publish(right_);

	ticks_since_target += 1;

	ros::spinOnce();


}

void TwistToMotors::twistCallback(const geometry_msgs::Twist &msg)
{

	ticks_since_target = 0;
	
	dx = msg.linear.x;
	dy = msg.linear.y;
	dr = msg.angular.z;

}


int main(int argc, char **argv)
{

	ros::init(argc, argv,"twist_to_motor");
	TwistToMotors obj;

	obj.spin();


}
