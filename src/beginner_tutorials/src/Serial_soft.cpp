#include <iostream>
#include "Serial_soft.h"
#include <string.h>



SerialSoft::SerialSoft()
{ //signal(SIGINT, SerialBetter::sigint_handler);
}
 



void SerialSoft::begin(std::string device, int speed)
{
 /*
	if((uart0_fd = open(device.c_str(), O_RDWR | O_NOCTTY )) == -1)
	{
		std::string error_text("cant open device: ");
		error_text += strerror(errno);
		throw std::runtime_error(error_text);
	}
	sleep(2);
	struct termios options;
	tcgetattr(uart0_fd, &options);
	options.c_cflag = speed | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_fd, TCIOFLUSH);
	tcsetattr(uart0_fd, TCSADRAIN, &options);
*/	

 // /*
set:		struct termios options;
		memset(&options,0,sizeof(options));
		rc = tcgetattr(uart0_fd,&options); 
		// Get settings 
//		cfmakeraw(&options);	
//		options.c_cflag = speed | CS8 | CLOCAL | CREAD;
//		options.c_iflag |= IGNPAR | ;
//		options.c_oflag = OPOST | ONLCR;
//		options.c_lflag = 0;
		options.c_cflag = speed | CS8 | CLOCAL | CREAD;
		options.c_iflag = IGNPAR;
		options.c_oflag = 0;
		options.c_lflag =0;
//		options.c_cflag 
		
		
        // Alter settings for raw mode 
		
//		 Apply the settings 
open:   uart0_fd=open(device.c_str(), O_RDWR | O_NOCTTY);
//        tty_fd=open(argv[1], O_RDWR | O_NONBLOCK);		
        if (uart0_fd<0)      
        {perror("Opening port");}
        else{
        sleep(2);}
		rc = tcdrain(uart0_fd);
		if ( rc < 0 ) {
		perror("tcdrain(3)");}
				cfmakeraw(&options);	
		rc = tcflush(uart0_fd,TCIOFLUSH);
		rc = tcsetattr(uart0_fd,TCSANOW,&options); 
		if (rc >= 0)
		{	
			printf("error_setattr \n");
			perror("tcsetattr(3) ");
//			abort();
		}
		if (rc < 0)
		{	
			perror("tcsetattr(3)");
//			goto open;
		}

  // */      
}
void SerialSoft::flush()
{
	rc = tcflush(uart0_fd,TCIOFLUSH);
}
int SerialSoft::available()
{
	int read_avail;
	fd_set set;
	struct timeval timeout;
	FD_ZERO(&set); /* clear the set */
  	FD_SET(uart0_fd, &set); /* add our file descriptor to the set */

  	timeout.tv_sec = 0;
  	timeout.tv_usec = 100000;

	read_avail=select(uart0_fd +1, &set, NULL, NULL, &timeout);
	if (read_avail==-1){
//		std::cout<<"Reading Error"<<std::endl;
		return false;
	}
	else if (read_avail == 0){
//		std::cout<<"Timeout"<<std::endl;
		return false;
	}
	else{
//		std::cout<<"Ready to read"<<std::endl;
		return true;
	}
}


int SerialSoft::polled()
{
struct pollfd fds [1];
	int timeout_poll;
	int pret;
	fds[0].fd= uart0_fd;
    fds[0].events=0;
    fds[0].events |= POLLIN;
    timeout_poll=1000;
    
    pret= poll(fds, 1, timeout_poll);
  	if (pret==0){printf("timeout\n"); }
    
	if (pret==-1){
//		std::cout<<"Reading Error"<<std::endl;
		return false;
	}
	else if (pret == 0){
//		std::cout<<"Timeout"<<std::endl;
		return false;
	}
	else{
//		std::cout<<"Ready to read"<<std::endl;
		return true;
	}
}

int SerialSoft::pollout()
{
struct pollfd fds [1];
	int timeout_poll;
	int pret;
	fds[0].fd= uart0_fd;
    fds[0].events=0;
    fds[0].events |= POLLOUT;
    timeout_poll=1000;
    
    pret= poll(fds, 1, timeout_poll);
  	if (pret==0){printf("timeout\n"); }
    
	if (pret==-1){
//		std::cout<<"Reading Error"<<std::endl;
		return false;
	}
	else if (pret == 0){
//		std::cout<<"Timeout"<<std::endl;
		return false;
	}
	else{
//		std::cout<<"Ready to read"<<std::endl;
		return true;
	}
}

uint8_t SerialSoft::read()
{
	uint8_t R;
	ssize_t code;
	do {
		if (this->polled())
		{code=::read(uart0_fd, &R,1 );}
//      	printf("after_read, c=%d", int (code));
//       	printf(" is signaled %d \n", is_signaled);
       	if (is_signaled){
			{
//			longjmp(jmp_exit, 1);
			abort();
			}
					} 
		}while (code == -1 && errno==EINTR );
	if(code == -1 )
	{
		std::string error_text("HardwareSerial cant read: ");
		error_text += strerror(errno);
		throw std::runtime_error(error_text);
	}
	if (code==0)
	{
		printf("end of file");
		this->end();
		abort();
	}
	return R;
}


void SerialSoft::end()
{
	close(uart0_fd);
}

int SerialSoft::write(uint8_t byte)
{
	int R = ::write(uart0_fd,&byte,1);
	if(R<=0)
	{
		std::string error_text("Serial cant write: ");
		error_text += strerror(errno);
		throw std::runtime_error(error_text);
	}
	return R;
}


int SerialSoft::writebuf(char buf[], int size)
{
	if (this->polled())
	rc = ::write(uart0_fd,buf,size);
if ( rc < 0 ) {
		std::string error_text("Serial cant write: ");
		error_text += strerror(errno);
		throw std::runtime_error(error_text);
}
//assert(rc == size);
return rc;
}
