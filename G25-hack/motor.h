/*
 * motor.h
 *
 * Created: 15.12.2013 16:29:26
 *  Author: Jari
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#define FORCE_LEFT(value)	OCR1B = (value) & 0x3ff;\
							OCR1C = 0;

#define FORCE_RIGHT(value)	OCR1B = 0;\
							OCR1C = (value) & 0x3ff;

#define FORCE_STOP()		OCR1B = 0;\
							OCR1C = 0;

#endif /* MOTOR_H_ */