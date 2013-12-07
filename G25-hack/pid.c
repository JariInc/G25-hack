/*
 * pid.c
 *
 * Created: 30.11.2013 16:08:53
 *  Author: Jari
 */ 

#include "pid.h"
#define N 8
#define NMIN (-(1 << (15+N)))
#define NMAX ((1 << (15+N)) - 1)
#define X_MIN32 (((int32_t)INT16_MIN) << 16)
#define X_MAX32 (((int32_t)INT16_MAX) << 16)
#define KP 1
#define KI2 0

/* 
* satlimit(x, min, max) does the following:
* if x is between min and max, return (x,0)
* if x < min, return (min, -1)
* if x > max, return (max, +1)
*/
void satlimit(int32_t *x, int16_t *sat) {
	if(*x >= X_MIN32 && *x <= X_MAX32) {
		sat = 0;
	}
	else if(*x < X_MIN32) {
		*x = X_MIN32;
		*sat = -1;	
	}
	else if(*x > X_MAX32) {
		*x = X_MAX32;
		*sat = 1;
	}
}

/*
* limit(x, min, max) does the following:
* if x is between min and max, return x
* if x < min, return min
* if x > max, return max
*/

int32_t limit(int32_t x) {
	if(x >= INT16_MIN && x <= INT16_MAX)
		return x;
	else if(x < INT16_MIN)
		return INT16_MIN;
	else if(x > INT16_MAX)
		return INT16_MAX;
	else
		return x; // shouldn't reach this ever
}

int16_t pid(int16_t input, int16_t feedback) {
	
	static int32_t integral = 0;
	static int16_t output = 0;

	if(input == 0)
		output = 0;
	else {
		// Error
		int16_t error = input - feedback;
		error >>= 3;

		// Present
		output += error;

		// Integral
		integral += error;
		integral >>= 4;
		output += integral;

		// Clamp output to valid value
		if(output <= 0)
			output = 0;
		else if (output >= PID_MAX)
			output = PID_MAX;
	}

	return output;

	/*
	// stored variables
	static int16_t previous_error = 0;
	static int32_t integral = 0;
	
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
	retval += tmp >> 8;
	tmp = Ki*integral;
	retval += tmp >> 8;
	tmp = Kd*derivative;
	retval += tmp >> 8;

	if(retval > PID_MAX)
		retval = PID_MAX;
	else if (retval < 0)
		retval = 0;

	return retval;
	*/

	/*
	
	static int16_t sat;
	static int32_t x_integral;
	
	int16_t x;
	int32_t p_term;

	int16_t e = input - feedback;
	if ((sat < 0 && e < 0) || (sat > 0 && e > 0)); 
	// do nothing if there is saturation, and error is in the same direction;
	// if you're careful you can implement as "if (sat*e > 0)"
	else
		x_integral = x_integral + (int32_t)KI2*e;

	satlimit(&x_integral, &sat);
	p_term = limit((int32_t)KP*e);
	x = limit((p_term >> N) + (x_integral >> 16));

	return x;	
	*/
}