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
#define Kp 0b000010000000000 /* 0.747 */
#define Ki 0b000000000000000 /* 0.10 */
#define Kd 0b000000000000000 /* 0.0001 */

int16_t pid(int16_t input, int16_t feedback);

#endif /* PID_H_ */