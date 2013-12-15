/*
 * pid.c
 *
 * Created: 30.11.2013 16:08:53
 *  Author: Jari
 */ 

#include <LUFA/Drivers/Peripheral/Serial.h>
#include "pid.h"
#include "motor.h"
#include "adc.h"

// current offset
static uint16_t currentoffset = 0;

void initPID() {
	// Current offset
	// do conversion and wait a bit, otherwise first measurement is way off
	ADCGetValue(3);
	_delay_ms(10);

	// find max
	currentoffset = ADCGetValue(3);
	uint8_t i = 0;
	int16_t current = 0;
	while(i < (1 << 3)) {
		current = ADCGetValue(3) - currentoffset;
		if(current > 0) {
			currentoffset++;
			i = 0;
		}
		else
			i++;
		_delay_ms(10);
	}

	TCCR2A |= (0 << COM2A1) | (0 << COM2A0); // Set OC2A on Compare Match
	TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // set prescaler to 1/256
	OCR2A = /*156*/ 62; // 16MHz / (62*256) = 1008 Hz
	//TIMSK2 |= (1 << OCIE2A); // Enable timer ISR
}
void startPID() {
	TIMSK2 |= (1 << OCIE2A);
}
void stopPID() {
	TIMSK2 &= ~(1 << OCIE2A);
}

int16_t pid(int16_t input, int16_t feedback) {
	
	static int32_t integral = 0;
	static int16_t output = 0;

	if(input == 0) {
		output = 0;
		integral = 0;
	}
	else {
		// Error
		int16_t error = input - feedback;

		// Proportional
		output += error >> 3;

		// Integral
		integral += error;
		integral >>= 2;
		output += integral;

		// Clamp output to valid value
		if(output < 0)
			output = 0;
		else if (output >= PID_MAX)
			output = PID_MAX;
	}
	
	/*
	Serial_SendByte(0xff);
	Serial_SendByte(0xff);

	Serial_SendByte(input >> 8);
	Serial_SendByte(input & 0xff);

	Serial_SendByte(feedback >> 8);
	Serial_SendByte(feedback & 0xff);
	
	int16_t err = input - feedback;

	Serial_SendByte(err >> 8);
	Serial_SendByte(err & 0xff);
	*/
	return output;
}

// PID timer
ISR(TIMER2_COMPA_vect)
{
	int16_t pid_value;
	int16_t feedback = ADCGetValue(3) - currentoffset;
	if(feedback < 0)
	feedback = 0;
	//dbg_fb = feedback;

	Serial_SendByte(0xff);
	Serial_SendByte(0xff);

	Serial_SendByte(feedback >> 8);
	Serial_SendByte(feedback & 0xff);

	if(force == 0) {
		FORCE_STOP();
	}
	if(force > 0) {
		pid_value = pid(force, feedback);
		FORCE_LEFT(pid_value >> 1);
	}
	else {
		pid_value = pid(-force, feedback);
		FORCE_RIGHT(pid_value >> 1);
	}
	
	TCNT2 = 0;
}