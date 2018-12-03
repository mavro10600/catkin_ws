#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <stdexcept>
#include <stdlib.h>

#include <string.h>

#include <assert.h>
#include <signal.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>
#include <ctype.h>
#include <setjmp.h>
#include <poll.h>
#include <stdbool.h>
#include <cstdint>
#include <iostream>




//http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart

#ifndef SERIALSOFT_H
#define SERIALSOFT_H

/**
 * @class SerialsSoft
 * @author mauro
 * @file SerialSoft.h
 * @brief the class that emulates a Arduino's HardwareSerial, 
 * every function may be used exact the same as Arduino's HardwareSerial's same-name function
 */
class SerialSoft
{
private:
    //static SerialSoft instance;
	static jmp_buf  jmp_exit;

	int uart0_fd;
	bool have_peeked_data;
	uint8_t peeked_data;
	int rc;
public:
	void func() {signal(SIGINT, SerialSoft::myHandler);}
	static void myHandler(int signum)
	{
	abort();
//	is_signaled=1;
	std::cout <<"fin" << std::endl;
	}
	volatile int is_signaled=0;
	static void sigint_handler(int signo);
	SerialSoft();
/*	int peek();*/
	int write(uint8_t byte);
	uint8_t read();
	int available();
	int polled();
	int pollout();
	void flush();
	void end();
	int writebuf(char buf[], int size);
	void begin(std::string device, int speed);
};

#endif //SERIALSOFT

