#include "Arduino.h"
#include "SimpleServo.h"
#include "Servo.h"
#define MIN_PULSE_WIDTH 544
void SimpleServoClass::attach(int servoPin) {
	servo->attach(servoPin, min, max);
}

SimpleServoClass::SimpleServoClass(Servo *servoobj, int min, int max, int range) {
	servo = servoobj;
	this->min = min;
	this->max = max;
	this->range = range;
}

SimpleServoClass::SimpleServoClass(Servo *servoobj, int range) {
	servo = servoobj;
	min = MIN_SERVO_DEFAULT;
	max = MAX_SERVO_DEFAULT;
	this->range = range;
}

void SimpleServoClass::write(int value) {
	if (value < MIN_PULSE_WIDTH) {
		value = constrain(value, -abs(range), abs(range));
		if (range < 0) value = -value;
		value = map(value, -100, 100, min, max);
	}
	this->writeMicroseconds(value);
}

void SimpleServoClass::writeMicroseconds(int value) {
	servo->writeMicroseconds(value);
}

int SimpleServoClass::read() {
	int value = this->readMicroseconds() + 1;
	value = map(value, min, max, -100, 100);
	if (range < 0) value = -value;
	return value;
}

int SimpleServoClass::readMicroseconds() {
	return servo->readMicroseconds();
}

void SimpleServoClass::stop() {
	this->write(0);
}
