/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for Joystick.c.
 */

#ifndef _G25HACK_H_
#define _G25HACK_H_

	/* Includes: */
		#include <avr/io.h>
		#include <avr/wdt.h>
		#include <avr/power.h>
		#include <avr/interrupt.h>
		#include <util/atomic.h>
		#include <util/delay.h>
		#include <string.h>

		#include "Descriptors.h"

		#include <LUFA/Drivers/USB/USB.h>
		#include <LUFA/Platform/Platform.h>

		#include <LUFA/Drivers/Peripheral/SPI.h>
		
	/* Type Defines: */
		/** Type define for the joystick HID report structure, for creating and sending HID reports to the host PC.
		 *  This mirrors the layout described to the host in the HID report descriptor, in Descriptors.c.
		 */
		typedef struct
		{
			int16_t Throttle; /**< Current absolute joystick Y position, as a signed 8-bit integer */
			int16_t Brake; /**< Current absolute joystick Z position, as a signed 8-bit integer */
			int16_t Clutch; /**< Current absolute joystick Z position, as a signed 8-bit integer */
			int16_t Wheel; /**< Current absolute joystick X position, as a signed 8-bit integer */
			uint8_t Button; /**< Bit mask of the currently pressed joystick buttons */
		} USB_JoystickReport_Data_t;

	/* Macros: */
		#define FORCE_LEFT(value)	OCR1B = (value) & 0x3ff;\
									OCR1C = 0;
									
		#define FORCE_RIGHT(value)	OCR1B = 0;\
									OCR1C = (value) & 0x3ff;
									
		#define FORCE_STOP()		OCR1B = 0;\
									OCR1C = 0;
	/*
		#define FORCE_LEFT(value)	PORTB |= (1 << DDB4); \
									PORTB &= ~(1 << DDB5);\
									OCR1C = (value) & 1023;
		#define FORCE_RIGHT(value)	PORTB &= ~(1 << DDB4);\
									PORTB |= (1 << DDB5);\
									OCR1C = (value) & 1023;
		#define FORCE_STOP()		PORTB &= ~(1 << DDB4);\
									PORTB &= ~(1 << DDB5);\
									OCR1C = 0;
	*/								
		// how long (ms) to wait before checking wheel position for movement, 3 too low, 4 works, 5 safe
		#define CALIBDELAY	5
		
	/* Function Prototypes: */
		void SetupHardware(void);
		uint16_t ADCGetValue(uint8_t ch);
		void WheelCalibration();

		void EVENT_USB_Device_Connect(void);
		void EVENT_USB_Device_Disconnect(void);
		void EVENT_USB_Device_ConfigurationChanged(void);
		void EVENT_USB_Device_ControlRequest(void);
		void EVENT_USB_Device_StartOfFrame(void);

		bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
		                                         uint8_t* const ReportID,
		                                         const uint8_t ReportType,
		                                         void* ReportData,
		                                         uint16_t* const ReportSize);
		void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
		                                          const uint8_t ReportID,
		                                          const uint8_t ReportType,
		                                          const void* ReportData,
		                                          const uint16_t ReportSize);

#endif

