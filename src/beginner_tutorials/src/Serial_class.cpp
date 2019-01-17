#include <iostream>
#include "Serial_soft.h"
#include "Messenger.h"
#include <cstring>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include </usr/include/boost/thread.hpp>
#include <math.h>
#include "std_msgs/Int16.h"

static volatile int is_signaled=0;
static jmp_buf  jmp_exit;

void sigint_handler(int signo)
{
	is_signaled=1;
	printf("Caught signal %d\n", signo); 
	abort();
}


class SerialClass : public Messenger, public SerialSoft
{
	public:
		boost::mutex mutex;
		SerialSoft Serial;
		Messenger Messenger_Handler=Messenger();
		SerialClass();					
		void spin();
		void thread_1();
	private:
		ros::NodeHandle n;
		ros::Publisher alive_pub;		
		ros::Publisher number_pub;
		bool alive_flag=true;		
	//variables asociadas al tiempo
		ros::Duration t_delta;	
		ros::Time t_next;		
		ros::Time then;		
		ros::Time current_time,last_time;				
		int counter=0;
		int left_out, right_out;
		long  alive_int;
		char cinco = '5';
		char nueve = '9';
		char space = ' ';
		char nl='\n';
		std::string cadena1="0 1 2 3 4 \n";
		std::string cadena2="5 6 7 8 9 \n";
	
			
		bool msg_comp=false;
		void init_variables();		
		void update();		
		void OnMssageCompleted();
		void alive_callback(const std_msgs::Bool::ConstPtr& msg);
};

SerialClass::SerialClass() : SerialSoft(),	Messenger()		
{

	init_variables();
//iniciar publicadores y suscriptores	
	number_pub = n.advertise<std_msgs::Int32>("Numbers", 1000);
	alive_pub= n.advertise <std_msgs::Int16>("Alive", 1);
//	Messenger_Handler.attach(&SerialClass::OnMssageCompleted);
//	offset_right_Sub=n.subscribe("offset",10,&Odometry_calc::offset_right_Cb,this);	
//	odom_pub=n.advertise<nav_msgs::Odometry>("odom",50);	
}


void SerialClass::OnMssageCompleted()
{  
   
  char reset[] = "r";
  char set_speed[] = "s";
  char set_flippers[]="f";
  char alive[]="a";
  
  if(Messenger_Handler.checkString(reset))
  {
	msg_comp=true;
	left_out=Messenger_Handler.readInt();
	right_out=Messenger_Handler.readInt();
	std_msgs::Int32 num;
//	mutex.lock();
	num.data=counter;
//	mutex.unlock();
	number_pub.publish(num);	
  }
  if(Messenger_Handler.checkString(set_speed))
  {
	std::cout <<"set speed done" << std::endl;    
     //This will set the speed
//     Set_Speed();
     return; 
  }

  if(Messenger_Handler.checkString(set_flippers))
  {
   	std::cout <<"Set flippers done" << std::endl;    
     //This will set the speed
//     Set_Flippers();
     return; 
	}
  if(Messenger_Handler.checkString(alive))
  {
  	msg_comp=true;
	alive_int=Messenger_Handler.readLong();
   	std::cout <<"Alive: "<<alive_int<<"\n" ;    
	std_msgs::Int16 alive_data;
	alive_data.data=alive_int;
	alive_pub.publish(alive_data);
     return; 
	}
} //end OnMsgCompleted


void SerialClass::init_variables()
{
return;	
}

void SerialClass::spin()
{
	ros::spin();
}

void SerialClass::thread_1()
{

	Serial.begin("/dev/ttyACM0",9600);
	Serial.is_signaled= is_signaled;

	while (ros::ok())
	{
	if (counter==5){	
	char cstr[cadena1.size() +1];
	strcpy(cstr, cadena1.c_str());
	Serial.writebuf(cstr, strlen(cstr));
				}
	int temp;

	do
	{
	temp = Serial.read();
		if (is_signaled)
			{
			Serial.flush();
			Serial.end();
			std::cout<<"serial end"<< std::endl;
			//longjmp(jmp_exit, 1);
			abort();
			}	
	}while(temp== -1 && errno == EINTR);
//	cout<<"data: "<<temp <<"\n";
	Messenger_Handler.process(temp); 
    if ( Messenger_Handler.messageState == 1 )
    {
    mutex.lock();
    this->OnMssageCompleted();
    mutex.unlock();
    }
	if (msg_comp)
		{
	std::cout << "is_signaled: "<<ros::ok()<<"\n";
	std::cout <<"counter: "<< counter <<"\n";
	std::cout <<"left_out: "<< left_out<<"\n";
	std::cout <<"right_out: "<< right_out <<std::endl;
	msg_comp=false;
	
		}
	counter++;
	if (counter==9){	
	char cstr[cadena2.size() +1];
	strcpy(cstr, cadena2.c_str());
	Serial.writebuf(cstr, strlen(cstr));
	counter=0;
		}	
	}
		Serial.end();
}


int main(int argc,char **argv)
{
	signal(SIGINT, sigint_handler);
	ros::init(argc,argv,"serialclass");
	SerialClass obj;
	boost::thread spin_thread(&SerialClass::thread_1, &obj);
	obj.spin();
	spin_thread.join();
	return 0;
}
