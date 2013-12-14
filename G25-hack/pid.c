/*
 * pid.c
 *
 * Created: 30.11.2013 16:08:53
 *  Author: Jari
 */ 

#include "pid.h"

int16_t pid(int16_t input, int16_t feedback) {
	
	static int32_t integral = 0;
	static int16_t output = 0;

	if(input == 0)
		output = 0;
	else {
		// Error
		int16_t error = input - feedback;
		//error >>= 2;

		// Proportional
		output += error >> 2;

		// Integral
		integral += error;
		integral >>= 2;
		output += integral;

		// Clamp output to valid value
		if(output <= 16)
			output = 0;
		else if (output >= PID_MAX)
			output = PID_MAX;
	}

	return output;
}