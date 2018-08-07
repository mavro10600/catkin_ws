#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

/*
Este programa toma como entradas el yaw pitch roll de la imu, y la salida son los valores de velocidad deseados en los tres motores. Enmedio, lo que hace es control pid en funcion del error.
*/

class Ballbot
{

public:
	Ballbot();
	void spin();
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber yaw_sub;
	ros::Subscriber pitch_sub;
	ros::Subscriber roll_sub;
	ros::Publisher first_motor;
	ros::Publisher second_motor;
	ros::Publisher third_motor;	
	
	double rate;
	
	double yaw, pitch, roll;

	double inc,angulo1,angulo2,valor;
	
	double cosi,seni, coseno, seno, m1, m2,m3;
		
	double	Kp,Ki,Kd;
	
	double v1[],v2[],v3[];

	double vx[],vy[],vz[];	
		
	//TODO:agregar las variables del programa de gabriel que convierten 
	
	
	double kisum,kimax,kierr,umbral,out,range;	
	
	
	
	//variables  asociadas al tiempo
	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;
	
	ros::Time current_time, last_time;

	void yawCb(const std_msgs::Float32::ConstPtr& yw);
	void pitchCb(const std_msgs::Float32::ConstPtr& ptch);
	void rollCb(const std_msgs::Float32::ConstPtr& rll);
	
	void init_variables();
	void get_node_params();
	void update();
};

Ballbot::Ballbot()
{
	init_variables();
	ROS_INFO("Started Ballbot computing node");
	
	yaw_sub=n.subscribe("",10,&Ballbot::yawCb, this);
	pitch_sub=n.subscribe("",10,&Ballbot::pitchCb, this);
	roll_sub=n.subscribe("",10,&Ballbot::rollCb, this);
	
//	first_motor=n.advertise<std_msgs::Float32>("first_pos_des",1);
//	second_motor=n.advertise<std_msgs::Float32>("second_pos_des",1);
//	third_motor=n.advertise<std_msgs::Float32>("third_pos_des",1);
	
	first_motor=n.advertise<std_msgs::Float32>("first_wheel_speed",1);
	second_motor=n.advertise<std_msgs::Float32>("second_wheel_speed",1);
	third_motor=n.advertise<std_msgs::Float32>("third_wheel_speed",1);


	get_node_params();
}

void Ballbot::init_variables()
{
rate=10;
yaw=0;
pitch=0;
roll=0;


//////////////////////////////////////////////////////////
//variables de la conversion

	inc=56;
	angulo1=0;
	angulo2=0;
	valor=0;
	
	cosi=cos(inc*M_PI)/180;
	seni=sin(inc*M_PI)/180;
	coseno=0;
	seno=0;
	m1=0;
	m2=0;
	m3=0;
	
//////////////////////////////////////////////////////////
		
	Kp=34.7;
	Ki=0.000158;
	Kd=145000;
	kisum=0;
	kimax=50;
	kierr=0.1;
	umbral=0.1;
	out=0;	
	range=100;
////////////////////////////////////////////////////////////
//variable sasociadas al tiempo	
	
	t_delta=ros::Duration(1.0/rate);
	t_next=ros::Time::now()+t_delta;
	
	then=ros::Time::now();
	
	current_time=ros::Time::now();
	last_time=ros::Time::now();
}

void Ballbot::get_node_params()
{
}

void Ballbot::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}

void Ballbot::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	//TODO:variables temporales
	double error_temp;
	double prev_error;
	double error;
	double angulo_anterior;
	if(now>t_next)
	{
		//TODO:calcular las direcciones de desplazamiento
		//TODO:Calcular el error
		
	vx[0]=1;
	vy[1]=1;
	vz[2]=1;	
		
		v1[0]=cos(pitch*M_PI/180);		
		v1[1]=0;
		v1[2]=sin(pitch*M_PI/180);
		
		v2[0]=0;
		v2[1]=cos(roll*M_PI/180);
		v2[2]=sin(roll*M_PI/180);
		
		v3[0]=((v1[1]*v2[2])-(v2[1]*v1[2]));
		v3[1]=-((v1[0]*v2[2])-(v2[0]*v1[2]));
		v3[2]=((v1[0]*v2[1])-(v2[0]*v1[1]));		
		
		/******Declaracion de los ángulos importantes*****///////////////////

  		angulo1=acos(((v3[0]*vx[0])+(v3[1]*vx[1]))/((sqrt(pow(v3[0],2)+pow(v3[1],2)))*(sqrt(pow(vx[0],2)))));

  		angulo2=acos(((v3[0]*vz[0])+(v3[1]*vz[1])+(v3[2]*vz[2]))/((sqrt(pow(v3[0],2)+pow(v3[1],2)+pow(v3[2],2)))*(sqrt(pow(vz[2],2)))));

  		if(v3[1]>0)
  		{
    		angulo1=(M_PI*2)-angulo1;
  		}

  		angulo1=(angulo1*180/M_PI);
  		angulo2=(angulo2*180/M_PI);
		
		/****Relación para el movimiento de las ruedas****///////////////////

  		coseno= cos((angulo1*M_PI)/180);
  		seno= sin((angulo1*M_PI)/180);
  		m1=coseno*sqrt(pow(seni,2)+pow(cosi*seno,2));
  		coseno= cos(((angulo1+120)*M_PI)/180);
  		seno= sin(((angulo1+120)*M_PI)/180);
  		m2=coseno*sqrt(pow(seni,2)+pow(cosi*seno,2));
  		coseno= cos(((angulo1+240)*M_PI)/180);
  		seno= sin(((angulo1+240)*M_PI/180));
  		m3=coseno*sqrt(pow(seni,2)+pow(cosi*seno,2));
  		
  		
  		error_temp=0-(angulo2);
		
		if(error_temp > -umbral && error_temp < umbral)	
		{error=0;}
		else{error=error_temp;}
	
		if(error_temp < -kierr || error_temp > kierr)
		{
		if(error_temp > -umbral && error_temp < umbral )
		{kisum=0;}
			else
			{
				if(abs(angulo1-angulo_anterior)>=90)
    		{
				kisum-=error*elapsed;
    		}
    			else
    		{
				kisum+=error*elapsed;
    		}
			//	kisum_vel+=error*elapsed;
				if(kisum > kimax){kisum=kimax;}
				if(kisum < -kimax){kisum=-kimax;}
			}
		}
		else
		{kisum=0;}
	


	out=Kp*error+kisum*Ki-Kd*(error-prev_error)/elapsed;
	
	prev_error=error;	
	angulo_anterior=angulo1;
	if(out > range){out=range;}
	if(out < -range){out=-range;}
	//ROS_DEBUG("vel: %0.2f tar: %0.2f err:%0.2f kisum: %0.2f left_out: %0.2f ticks:%d",vel,left_ang_des,error_vel,kisum_vel,left_out,ticks_since_target);		
	
	ROS_INFO_STREAM("error" << error);
	ROS_INFO_STREAM("kisum" << kisum);
	ROS_INFO_STREAM("out" << out);

	ROS_INFO_STREAM("Yaw"<< yaw);
	ROS_INFO_STREAM("Pitch" << pitch);
	ROS_INFO_STREAM("Roll" << roll);
	
	ROS_INFO_STREAM("m1"<< yaw);
	ROS_INFO_STREAM("m2" << pitch);
	ROS_INFO_STREAM("m3" << roll);
//////////////////////////////////////////////////////////////////////////////////
//Publicar los vlores a los motores
		std_msgs::Float32 motor1;
		std_msgs::Float32 motor2;
		std_msgs::Float32 motor3;
		motor1.data=m1*out;
		motor2.data=m2*out;
		motor3.data=m3*out;		
 		
 		first_motor.publish(motor1);
 		second_motor.publish(motor2);
 		third_motor.publish(motor3);
 		
  		then=now;
  		ros::spinOnce();
  		
	}
	else {;}
}

	void Ballbot::yawCb(const std_msgs::Float32::ConstPtr& yw)
	{yaw=yw->data;}
	void Ballbot::pitchCb(const std_msgs::Float32::ConstPtr& ptch)
	{pitch=ptch->data;}
	void Ballbot::rollCb(const std_msgs::Float32::ConstPtr& rll)
	{roll=rll->data;}
	
int main(int argc, char **argv)
{
ros::init(argc,argv,"ballbot_control");
Ballbot obj;
obj.spin();
return 0;
}
