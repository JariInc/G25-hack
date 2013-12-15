/*
 * adc.c
 *
 * Created: 15.12.2013 16:30:57
 *  Author: Jari
 */ 
#include <util/atomic.h>
#include <LUFA/Drivers/Peripheral/SPI.h>
#include "adc.h"

void initADC() {
	// CS (PB0) as output
	DDRB |= (1 << DDB0);
	PORTB |= (1 << DDB0);
}

// ADC (MCP3204)
uint16_t ADCGetValue(uint8_t ch) {
	 uint16_t output = 0;
	 ATOMIC_BLOCK(ATOMIC_FORCEON)
	 {
		 PORTB &= ~(1 << DDB0); // CS low to activate
		 SPI_SendByte(0b00000110); // start conversion command
		 output = SPI_TransferByte(ch << 6) << 8; // read 4 MSB
		 output |= SPI_TransferByte(0xff); // read rest
	 }
	 PORTB |= (1 << DDB0); // CS high to deactivate
	 return output & 0xfff;
}