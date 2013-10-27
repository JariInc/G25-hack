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
 *  Main source file for the Joystick demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "G25-hack.h"

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevJoystickHIDReportBuffer[sizeof(USB_JoystickReport_Data_t)];

// wheel position
static int16_t wheelpos = 0;

// force offset
//static uint8_t forceoffset = 0;

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Joystick_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = 0,
				.ReportINEndpoint             =
					{
						.Address              = JOYSTICK_EPADDR,
						.Size                 = JOYSTICK_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevJoystickHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevJoystickHIDReportBuffer),
			},
	};


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();
	//GlobalInterruptEnable();

	for (;;)
	{
		HID_Device_USBTask(&Joystick_HID_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
	
	/* 
		Encoder interrupts 
	*/
	// set pins as inputs
	DDRD &= ~(1 << DDD0);
	DDRD &= ~(1 << DDD1);
	PORTD |= (1 << DDD0) | (1 << DDD1);
	
	// trigger to any edge
	EICRA &= ~(1 << ISC01);
	EICRA |= (1 << ISC00);
	EICRA &= ~(1 << ISC11);
	EICRA |= (1 << ISC10);
	
	// enable interrupts
	EIMSK |= (1 << INT0) | (1 << INT1);
	
	/* 
		Buttons 
	*/
	// set as inputs
	DDRD &= ~(1 << DDD2);
	DDRD &= ~(1 << DDD3);
	PORTD |= (1 << DDD2) | (1 << DDD3); 
	
	/*
		SPI (using LUFA helper)
	*/
	SPI_Init(SPI_MODE_MASTER | SPI_SPEED_FCPU_DIV_8);
	// CS (PB0) as output
	DDRB |= (1 << DDB0);
	PORTB |= (1 << DDB0);
	
	/* 
		H-bridge control 
	*/
	DDRB |= (1 << DDB4)|(1 << DDB5); // direction pins as outputs
	PORTB &= ~(1 << DDB4); // set low
	PORTB &= ~(1 << DDB5); // set low
	
	/* 
		PWM
	*/
	DDRC |= 1 << PC6; // set pin as output
	TCCR1A |= (1 << COM1A1)|(0 << COM1A0); // set Compare Output Mode
	TCCR1A |= (1 << WGM11)|(1 << WGM10); // set Waveform Generation Mode
	TCCR1B |= (1 << WGM12)|(0 << WGM13);
	TCCR1B |= (0 << CS12)|(0 << CS11)|(1 << CS10); // Set prescaler
	OCR1A = 0; // set zero 
	
	/*
		Calibration
	*/
	GlobalInterruptEnable();
	WheelCalibration();
	
	/* 
		USB Initialization
	*/
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Joystick_HID_Interface);

	USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	//HID_Device_ProcessControlRequest(&Joystick_HID_Interface);
	if (USB_ControlRequest.bmRequestType == 64) {
		// zero torque, disable motors
		if(USB_ControlRequest.wValue == 0) {
			/*
			PORTB &= ~(1 << DDB4);
			PORTB &= ~(1 << DDB5);
			OCR1A = 0;
			*/
			FORCE_STOP();
		}
		// sign bit 0 (positive force)
		if(USB_ControlRequest.wValue >> 15 == 0) {
			/*
			 PORTB |= (1 << DDB4);
			 PORTB &= ~(1 << DDB5);
			 OCR1A = USB_ControlRequest.wValue & 1023;
			 */
			FORCE_LEFT(USB_ControlRequest.wValue);
		}
		// sign bit 1 (negative force)
		else if(USB_ControlRequest.wValue >> 15 == 1) {
			/*
			PORTB &= ~(1 << DDB4);
			PORTB |= (1 << DDB5);
			OCR1A = ((USB_ControlRequest.wValue ^ 0xffff) + 1) & 1023; // abs() = XOR + 1
			*/
			FORCE_RIGHT((USB_ControlRequest.wValue ^ 0xffff) + 1); // abs() = XOR + 1
		}
		
		Endpoint_ClearStatusStage();
	}
	else
		HID_Device_ProcessControlRequest(&Joystick_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Joystick_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	USB_JoystickReport_Data_t* JoystickReport = (USB_JoystickReport_Data_t*)ReportData;
	
	JoystickReport->Wheel = wheelpos;
	JoystickReport->Throttle = (int16_t)ADCGetValue(0);
	JoystickReport->Brake = (int16_t)ADCGetValue(1);
	JoystickReport->Clutch = (int16_t)ADCGetValue(2);
	JoystickReport->Button = ((PIND >> 2) & 0b11) ^ 0b11; // read button states and invert them
	
	*ReportSize = sizeof(USB_JoystickReport_Data_t);
	
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

// encoder interrupt handlers
ISR(INT0_vect) {
	uint8_t pins = PIND & ((1 << PD0) | (1 << PD1));
	
	if(pins == 0b11 || pins == 0b00)
		wheelpos++;
	else
		wheelpos--;
	
	// clear flag
	EIFR |= (1 << INT0);
}

ISR(INT1_vect)
{
	uint8_t pins = PIND & ((1 << PD0) | (1 << PD1));
	
	if(pins == 0b11 || pins == 0b00)
		wheelpos--;
	else
		wheelpos++;

	// clear flag
	EIFR |= (1 << INT1);
}

// ADC (MCP3204)
uint16_t ADCGetValue(uint8_t ch) {
	uint16_t output = 0;
	PORTB &= ~(1 << DDB0); // CS low to activate
	SPI_SendByte(0b00000110); // start conversion command
	output = SPI_TransferByte(ch << 6) << 8; // read 4 MSB
	output |= SPI_TransferByte(0xff); // read rest
	PORTB |= (1 << DDB0); // CS high to deactivate
	return output & 0xfff;
}

void WheelCalibration() {
	/*
		Calibration procedure:
			1. Rotate left until limit is reached
			2. Mark position 0
			3. Move right until right limit is reached
			4. Get position on right edge, offset position by half of the value
			   - Presume center is between left and right edge
			5. Rotate back to center
	*/
	
	//forceoffset = 0;
	wheelpos = 0;
	int16_t prev_wheelpos = wheelpos;
	uint16_t velocity = 0;
	//int16_t force = 0;
	
	/* Find minimum force, needed at all?
	// step 0
	do {
		force += 2;
		FORCE_LEFT(force);
		_delay_ms(CALIBDELAY);
		velocity = wheelpos - prev_wheelpos;
		prev_wheelpos = wheelpos;
	} while (velocity < 1);
	forceoffset = force;
	*/
	
	// step 1
	do {
		FORCE_LEFT(768);
		_delay_ms(CALIBDELAY);
		velocity = wheelpos - prev_wheelpos;
		prev_wheelpos = wheelpos;
	} while (velocity > 0);
	FORCE_STOP();
	
	// step 2
	wheelpos = 0;
	prev_wheelpos = 0;
	
	// step 3
	do {
		FORCE_RIGHT(768);
		_delay_ms(CALIBDELAY);
		velocity = prev_wheelpos - wheelpos;
		prev_wheelpos = wheelpos;
	} while (velocity > 0);
	
	FORCE_STOP();
	
	// step 4
	wheelpos = (wheelpos >> 1);
	
	// step 5
	do {
		FORCE_LEFT(1023);
		_delay_ms(CALIBDELAY);
	} while (wheelpos < 0);
	
	// brake and slowly rotate to center
	do {
		FORCE_RIGHT(256);
		_delay_ms(CALIBDELAY);
	} while (wheelpos > 0);
	
	FORCE_STOP();
}