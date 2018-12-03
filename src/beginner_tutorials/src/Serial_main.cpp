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
//#include <boost/thread/mutex.hpp>
//#include <thread>
//boost::mutex io_mutex; // The iostreams are not guaranteed to be
//boost::mutex mutex;

using namespace std;
volatile int is_signaled=0;
static jmp_buf  jmp_exit;

void sigint_handler(int signo)
{
	is_signaled=1;
	printf("Caught signal %d\n", signo); 
	abort();
}
int counter=0;
int left_out, right_out;
bool msg_comp=false;
Messenger Messenger_Handler=Messenger();
void OnMssageCompleted()
{  
   
  char reset[] = "r";
  char set_speed[] = "s";
  char set_flippers[]="f";
  
  if(Messenger_Handler.checkString(reset))
  {
//	boost::mutex::scoped_lock scoped_lock(mutex);
	//mutex.lock();
	msg_comp=true;
//	cout <<"reset done \n";    
//     Serial.println("Reset Done"); 
//     Reset();
	left_out=Messenger_Handler.readInt();
	right_out=Messenger_Handler.readInt();
	//boost::mutex::scoped_lock scoped_lock(io_mutex);
//	cout <<"left:" << left_out<< "right: "<< right_out<<"\n";   
	//mutex.unlock();
  }
  if(Messenger_Handler.checkString(set_speed))
  {
	cout <<"set speed done" << endl;    
     //This will set the speed
//     Set_Speed();
     return; 
  }

  if(Messenger_Handler.checkString(set_flippers))
  {
   	cout <<"Set flippers done" << endl;    
     //This will set the speed
//     Set_Flippers();
     return; 
  }
  
}
void alive_callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Alive_callback");
  
}
void thread_1()
{
	SerialSoft Serial;
	Serial.begin("/dev/ttyACM0",9600);
	Serial.is_signaled= is_signaled;

	char cinco = '5';
	char nueve = '9';
	char space = ' ';
	char nl='\n';
	std::string cadena1="0 1 2 3 4 \n";
	std::string cadena2="5 6 7 8 9 \n";
	Messenger_Handler.attach(OnMssageCompleted);
	while (1)
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
			cout<<"serial end"<< endl;
			//longjmp(jmp_exit, 1);
			abort();
			}	
	}while(temp== -1 && errno == EINTR);
//	cout<<"data: "<<temp <<"\n";
	Messenger_Handler.process(temp); 
	
	if (msg_comp)
		{
	cout << std::ostream::goodbit<<"\n";
	cout << std::ostream::eofbit<<"\n";
	cout << std::ostream::failbit<<"\n";
	cout << std::ostream::badbit<<"\n";
	cout <<"counter: "<< counter <<"\n";
	cout <<"left_out: "<< left_out<<"\n";
	cout <<"right_out: "<< right_out <<endl;
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

int main(int argc, char **argv)
{
ros::init(argc, argv, "serial");
ros::NodeHandle n;
ros::Publisher number_pub = n.advertise<std_msgs::Int32>("Numbers", 1);
ros::Subscriber alive_sub= n.subscribe <std_msgs::Bool>("Alive", 1, alive_callback);
ros::Rate loop_rate(10);
 
signal(SIGINT, sigint_handler); 
	char	buffer[64];
	int len;
boost::thread spin_thread(&thread_1);
while (ros::ok())	{
	std_msgs::Int32 num;
	num.data=counter;
	number_pub.publish(num);	
	ros::spinOnce();
    loop_rate.sleep();	
//	ros::spin();
	
	}
	spin_thread.join();

	return 0; 
}

