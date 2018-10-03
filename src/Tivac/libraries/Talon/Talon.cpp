#include "Servo.h"
#include "Talon.h"
#define MIN_PULSE_WIDTH 544
void TalonClass::attach(int servoPin) {
	servo.attach(servoPin, min, max);
}

TalonClass::TalonClass(int min, int max, int center, int umbral, int range) : servo() {
	this->min = min;
	this->max = max;
	this->center = center;
	this->umbral = umbral;
	this->range = range;
}

TalonClass::TalonClass(int min, int max, int umbral, int range) : servo() {
	this->min = min;
	this->max = max;
	this->center = (min + max) / 2;
	this->umbral = umbral;
	this->range = range;
}

TalonClass::TalonClass(int umbral, int range) : servo() {
	min = MIN_TALON_DEFAULT;
	max = MAX_TALON_DEFAULT;
	center = CENTER_TALON_DEFAULT;
	this->umbral = umbral;
	this->range = range;
}

void TalonClass::write(int value) {
	if (value < MIN_PULSE_WIDTH) {
		value = constrain(value, -abs(range), abs(range));
		if (abs(value) <= umbral) value = 0;
		if (range < 0) value = -value;
		// Added mapping for center stuff
		if (value <= 0)
			value = map(value, -100, 0, min, center);
		else
			value = map(value, 0, 100, center, max);
	}
	this->writeMicroseconds(value);
}

void TalonClass::writeMicroseconds(int value) {
	servo.writeMicroseconds(value);
}

int TalonClass::read() {
	int value = this->readMicroseconds() + 1;

	if (value <= center)
		value = map(value, min, center, -100, 0);
	else
		value = map(value, center, max, 0, 100);

	if (range < 0) value = -value;
	return value;
}

int TalonClass::readMicroseconds() {
	return servo.readMicroseconds();
}

void TalonClass::calibrate(int value) {
	center += value;
}

void TalonClass::stop() {
	servo.writeMicroseconds(center);
}
