/*
 * pid.h
 *
 * Created: 30.11.2013 16:09:12
 *  Author: Jari
 */ 


#ifndef PID_H_
#define PID_H_

#include <stdint.h>

// tuning parameters Q0.15 signed fixed point
#define Kp (1 << 8) /* 0.747 */
#define Ki 0 /* 0.10 */
#define Kd 0 /* 0.0001 */

#define PID_MAX 1023

int16_t pid(int16_t input, int16_t feedback);

#endif /* PID_H_ */