/*
 * adc.h
 *
 * Created: 15.12.2013 16:30:43
 *  Author: Jari
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

void initADC();
uint16_t ADCGetValue(uint8_t ch);


#endif /* ADC_H_ */