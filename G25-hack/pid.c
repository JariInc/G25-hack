/*
 * pid.c
 *
 * Created: 30.11.2013 16:08:53
 *  Author: Jari
 */ 

#include "pid.h"

int16_t pid(int16_t input, int16_t feedback) {
	
	// stored variables
	static int16_t previous_error = 0;
	static int16_t integral = 0;
	
	int16_t error = input - feedback;
	// update integral
	integral += error;

	// calculate derivative
	int16_t derivative = error - previous_error;

	// store error
	previous_error = error;

	// calculate new output
	int32_t tmp;
	int16_t retval = 0;
	tmp = Kp*error;
	retval += tmp >> 16;
	tmp = Ki*integral;
	retval += tmp >> 16;
	tmp = Kd*derivative;
	retval += tmp >> 16;
	return retval;
	
	
	// really simple
	//int16_t error = input - feedback;
	//return input - (error >> 1);
	
}