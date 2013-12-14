/*
 * pid.h
 *
 * Created: 30.11.2013 16:09:12
 *  Author: Jari
 */ 


#ifndef PID_H_
#define PID_H_

#include <stdint.h>

#define PID_MAX 1023

// (requested) force value
extern int16_t force;

int16_t pid(int16_t input, int16_t feedback);

#endif /* PID_H_ */