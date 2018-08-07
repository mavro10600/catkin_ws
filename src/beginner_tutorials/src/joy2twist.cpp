#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int16.h"

#define Abutton joy->buttons[0]
#define Bbutton joy->buttons[1]
#define Xbutton joy->buttons[2]
#define Ybutton joy->buttons[3]
#define RAVstick joy->axes[4]
#define RAHstick joy->axes[3]
#define LAVstick joy->axes[1]
#define LAHstick joy->axes[0]
#define LBbutton joy->buttons[4]
#define RBbutton joy->buttons[5] 
#define LTbutton joy->axes[2]  //va de 1 a -1
#define RTbutton joy->axes[5]  //va de 1 a -1
#define DpadV joy->axes[7]
#define DpadH joy->axes[6]
#define Lstickbutton joy->axes[9]
#define Rstickbutton joy->axes[10]


#define PI 3.14159265

class RobotDriver
{
	private:
	ros::NodeHandle n;
	ros::Subscriber joy_sub;//suscriptor al control xbox

	ros::Publisher cmd_vel_pub;//publicar un mensaje tipo twist para manejar la base
	ros::Publisher cmd_arm_pub;//publicar un mensaje tipo twist para el brazo
	
//TODO agregar publicadores de los flippers
	ros::Publisher cmd_vel_fl1;
	ros::Publisher cmd_vel_fl2;
	ros::Publisher cmd_vel_fl3;
	ros::Publisher cmd_vel_fl4;			
	
///////////////////////////////////////////////////////	
	double rate;
///////////////////////////////////////////////////////
//variables asociadas al stick izquierdo, y al Dpad ara mover la base
	double joy_h;
	double joy_v;
	
///////////////////////////////////////////////////////
//variables asociadas a los flippers
	double joy_flipper1;
	double joy_flipper2;
	double joy_flipper3;
	double joy_flipper4;
	

///////////////////////////////////////////////////////
//variables asociadas al stick derecho, que va a controlar el brazo
	
	double joy_x_arm;
	double joy_y_arm;
	double joy_z_arm;
	double joy_roll;
	double joy_pitch;
	double joy_yaw;
//variables asociadas al tiempo
		ros::Duration t_delta;
		
		ros::Time t_next;
		
		ros::Time then;
		
		ros::Time current_time,last_time;
	
//funciones
	void update();
	
	void init_variables();
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	
	public:
	
	RobotDriver();
	
	void spin();
	
	
};

RobotDriver::RobotDriver()
{
	init_variables();

	joy_sub=n.subscribe("/joy",10,&RobotDriver::joyCallback,this);
	cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/base_controller/command",1);
	cmd_arm_pub=n.advertise<geometry_msgs::Twist>("/arm_controller/command",1);
	/*cmd_vel_fl1=n.advertise<geometry_msgs::Twist>("/base_controller/flipper1",1);
	cmd_vel_fl2=n.advertise<geometry_msgs::Twist>("/base_controller/flipper2",1);
	cmd_vel_fl3=n.advertise<geometry_msgs::Twist>("/base_controller/flipper3",1);
	cmd_vel_fl4=n.advertise<geometry_msgs::Twist>("/base_controller/flipper4",1);
	*/
	cmd_vel_fl1=n.advertise<std_msgs::Int16>("flipper1_out",1);
	cmd_vel_fl2=n.advertise<std_msgs::Int16>("flipper2_out",1);
	cmd_vel_fl3=n.advertise<std_msgs::Int16>("flipper3_out",1);
	cmd_vel_fl4=n.advertise<std_msgs::Int16>("flipper4_out",1);
}

void RobotDriver::init_variables()
{
	rate=10;
	
///////////////////////////////////////////
//variables asociadas a la navegacion
	joy_h=0;
	joy_v=0;
	
///////////////////////////////////////////
//variables asociadas a los flippers
	joy_flipper1=0;
	joy_flipper2=0;
	joy_flipper3=0;
	joy_flipper4=0;
////////////////////////////////////////////
//variables asocidas al brazo
	joy_x_arm=0;
	joy_y_arm=0;
	joy_z_arm=0;
	
	joy_pitch=0;
	joy_roll=0;
	joy_yaw=0;
	
	////////////////////////////////////////////////////////////
//variable sasociadas al tiempo	
	
	t_delta=ros::Duration(1.0/rate);
	t_next=ros::Time::now()+t_delta;
	
	then=ros::Time::now();
	
	current_time=ros::Time::now();
	last_time=ros::Time::now();
/////////////////////////////////
}

void RobotDriver::spin()
{
	ros::Rate loop_rate(rate);
	while(ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}

void RobotDriver::update()
{
	ros::Time now=ros::Time::now();
	double elapsed;
	double temp_linear;
	double temp_angular;
	if(now>t_next)
	{
		elapsed=now.toSec()-then.toSec();
		
		geometry_msgs::Twist base_cmd;
		
		geometry_msgs::Twist arm_cmd;
		
		std_msgs::Int16 flip1, flip2, flip3,flip4;
		
///////////////////////////////////////////////////////////////////////////////
//flippers		
		flip1.data=int(joy_flipper1);
		flip2.data=int(joy_flipper2);
		flip3.data=int(joy_flipper3);
		flip4.data=int(joy_flipper4);
		
		
		//aqui convertir los valores de las variables del joystick a velocidad angular y velocidad en x, el mapeo es de xy a r, theta, del plano a coordenadas polares
		temp_linear=sqrt(joy_h*joy_h+joy_v*joy_v);
		temp_angular=atan2(joy_v,joy_h)*180/PI;
		
		if(temp_linear>0.2 && temp_angular>=0)
		{base_cmd.angular.z=atan2(joy_v,joy_h)-PI/2;}
		if(temp_linear>0.2 && temp_angular < 0)
		{base_cmd.angular.z=atan2(joy_v,joy_h)+PI/2;}
		if(temp_linear <=0.2)
		{base_cmd.angular.z=atan2(joy_v,joy_h);}
		if(temp_angular<0 )
		{base_cmd.linear.x=-sqrt(joy_h*joy_h+joy_v*joy_v);}
		if(temp_angular>=0)
		{base_cmd.linear.x=sqrt(joy_h*joy_h+joy_v*joy_v);}

/////////////////////////////////////////////////////////////////////////////
//Aqui se convierte de xy cartesiano al espacio en 3d, hay que definir los botones que cambian el plano de uso

		arm_cmd.linear.x=joy_x_arm;
		arm_cmd.linear.y=joy_y_arm;
		arm_cmd.linear.z=joy_z_arm;
		
		arm_cmd.angular.x=joy_roll;
		arm_cmd.angular.y=joy_pitch;
		arm_cmd.angular.z=joy_yaw;
//////////////////////////////////////////////////////////////////////////////
//publicar flippers		
		
		cmd_vel_fl1.publish(flip1);
		cmd_vel_fl2.publish(flip2);
		cmd_vel_fl3.publish(flip3);
		cmd_vel_fl4.publish(flip4);
 		
/////////////////////////////////////////////////////////////////////////////
//aqui termina la conversion y se publican los mansajes tipo twist
		cmd_arm_pub.publish(arm_cmd);
		cmd_vel_pub.publish(base_cmd);
		then=now;
		ros::spinOnce();
	}
	else
	{;}
}

void RobotDriver::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
float sens=0.2;

/////////////////////////////////////////////////////////////////////////
//movimiento de la base

if((LAVstick > sens || LAVstick < -sens ) && !LBbutton && !RBbutton)
{
joy_v=LAVstick;
}

if((LAVstick < sens && LAVstick> -sens) && !LBbutton && !RBbutton)
{
	joy_v=0;
}

if((LAHstick > sens || LAHstick < -sens) && !LBbutton && !RBbutton)
{
	joy_h=-LAHstick;
}

if((LAHstick < sens && LAHstick > -sens) && !LBbutton && !RBbutton)
{
	joy_h=0;
}


if(DpadV > sens || DpadV < -sens)
{joy_v=DpadV*0.5;}

if(DpadV < sens && DpadV> -sens && !(LAVstick > sens || LAVstick < -sens))
{joy_v=0;}

if(DpadH > sens || DpadH < -sens)
{joy_h=-DpadH*0.5;}

if(DpadH < sens && DpadH > -sens && !(LAHstick > sens || LAHstick < -sens))
{joy_h=0;}

///////////////////////////////////////////////////////////////////
//movimiento de los flippers

if((LAVstick > sens || LAVstick < -sens ) && LBbutton && RBbutton)
{
joy_flipper1=LAVstick*64;
}

if((LAVstick < sens && LAVstick> -sens) && LBbutton && RBbutton)
{
	joy_flipper1=0;
}

if((LAHstick > sens || LAHstick < -sens) && LBbutton && RBbutton)
{
	joy_flipper2=-LAHstick*64;
}

if((LAHstick < sens && LAHstick > -sens) && LBbutton && RBbutton)
{
	joy_flipper2=0;
}


if((RAVstick > sens || RAVstick < -sens ) && LBbutton && RBbutton)
{
joy_flipper3=RAVstick*64;
}

if((RAVstick < sens && RAVstick> -sens) && LBbutton && RBbutton)
{
	joy_flipper3=0;
}

if((RAHstick > sens || RAHstick < -sens) && LBbutton && RBbutton)
{
	joy_flipper4=-RAHstick*64;
}

if((RAHstick < sens && RAHstick > -sens) && LBbutton && RBbutton)
{
	joy_flipper4=0;
}



//////////////////////////////////////////////////////////////////
//movimiento del brazo

//movimiento en xy, uso de la palanca derecha
if((RAVstick > sens || RAVstick < -sens) && !LBbutton && !RBbutton)
{joy_y_arm=RAVstick;}

if((RAVstick < sens && RAVstick> -sens) && !LBbutton && !RBbutton)
{joy_y_arm=0;}

if((RAHstick > sens || RAHstick < -sens) && !LBbutton && !RBbutton)
{joy_x_arm=-RAHstick;}

if((RAHstick < sens && RAHstick > -sens) && !LBbutton && !RBbutton)
{joy_x_arm=0;}

//movimiento en yz

if((RAVstick > sens || RAVstick < -sens) && LBbutton && !RBbutton)
{joy_z_arm=RAVstick;}

if((RAVstick < sens && RAVstick> -sens) && LBbutton && !RBbutton)
{joy_z_arm=0;}

if((RAHstick > sens || RAHstick < -sens) && LBbutton && !RBbutton)
{joy_y_arm=-RAHstick;}

if((RAHstick < sens && RAHstick > -sens) && LBbutton && !RBbutton)
{joy_y_arm=0;}

//movimiento en xz

if((RAVstick > sens || RAVstick < -sens) && !LBbutton && RBbutton)
{joy_z_arm=RAVstick;}

if((RAVstick < sens && RAVstick> -sens) && !LBbutton && RBbutton)
{joy_z_arm=0;}

if((RAHstick > sens || RAHstick < -sens) && !LBbutton && RBbutton)
{joy_x_arm=-RAHstick;}

if((RAHstick < sens && RAHstick > -sens) && !LBbutton && RBbutton)
{joy_x_arm=0;}

//movimiento roll y pitch

if((LAVstick > sens || LAVstick < -sens) && LBbutton && !RBbutton)
{joy_pitch=LAVstick;}

if((LAVstick < sens && LAVstick> -sens) && LBbutton && !RBbutton)
{joy_pitch=0;}

if((LAHstick > sens || LAHstick < -sens) && LBbutton && !RBbutton)
{joy_roll=-LAHstick;}

if((LAHstick < sens && LAHstick > -sens) && LBbutton && !RBbutton)
{joy_roll=0;}

//movimiento yaw y roll

if((LAVstick > sens || LAVstick < -sens) && !LBbutton && RBbutton)
{joy_yaw=LAVstick;}

if((LAVstick < sens && LAVstick> -sens) && !LBbutton && RBbutton)
{joy_yaw=0;}

if((LAHstick > sens || LAHstick < -sens) && !LBbutton && RBbutton)
{joy_roll=-LAHstick;}

if((LAHstick < sens && LAHstick > -sens) && !LBbutton && RBbutton)
{joy_roll=0;}


}



int main(int argc, char **argv)
{

	ros::init(argc,argv,"joy2twist");
	RobotDriver obj;
	obj.spin();
	
	return 0;
}
