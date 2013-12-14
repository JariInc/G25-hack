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
#include "pid.h"
#include "ffb.h"

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevJoystickHIDReportBuffer[sizeof(USB_JoystickReport_Data_t)];

// wheel position
static uint16_t wheelpos = 0;

// current offset
static uint16_t currentoffset = 0;

// force offset
//static uint8_t forceoffset = 0;

// debug
//static int16_t dbg_fb = 0;

// (requested) force value
int16_t force;

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

USB_ClassInfo_HID_Device_t FFB_HID_Interface =
{
	.Config =
	{
		.InterfaceNumber              = 0,
		.ReportINEndpoint             =
		{
			.Address              = FFB_EPADDR,
			.Size                 = FFB_EPSIZE,
			.Banks                = 1,
		}
	},
};

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	
	SetupHardware();
	GlobalInterruptEnable();
	
	for (;;)
	{
		HID_Task();
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
		PWM
	*/
	TCCR1A |= (1 << WGM11)|(1 << WGM10); // set Waveform Generation Mode
	TCCR1B |= (1 << WGM12)|(0 << WGM13);
	TCCR1B |= (0 << CS12)|(0 << CS11)|(1 << CS10); // Set prescaler
	
	DDRB |= 1 << PB7; // set pin as output
	TCCR1A |= (1 << COM1C1)|(0 << COM1C0); // set Compare Output Mode
	OCR1C = 0; // set zero 
	
	DDRB |= 1 << PB6; // set pin as output
	TCCR1A |= (1 << COM1B1)|(0 << COM1B0); // set Compare Output Mode
	OCR1B = 0; // set zero 
	
	GlobalInterruptEnable();
	WheelCalibration();
	GlobalInterruptDisable();
	
	/*
	// debug
	DDRC |= (1 << DDC0);
	PORTC |= (1 << DDC0);
	PORTC &= ~(1 << DDC0);
	*/

	/*
		Current offset
	*/
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
	
	/*
		PID timer
	*/
	TCCR2A |= (0 << COM2A1) | (0 << COM2A0); // Set OC2A on Compare Match
	TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // set prescaler to 1/256
	OCR2A = /*156*/ 62; // 16MHz / (62*256) = 1008 Hz
	TIMSK2 |= (1 << OCIE2A); // Enable timer ISR

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

	ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPADDR, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(FFB_EPADDR, EP_TYPE_INTERRUPT, FFB_EPSIZE, 1);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */

	/*
	USB_ControlRequest :=
		uint8_t 	bmRequestType
		uint8_t 	bRequest
		uint16_t 	wValue
		uint16_t 	wIndex
		uint16_t 	wLength
	*/
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				if (USB_ControlRequest.wValue == 0x0307) {
					// Feature 3: PID Pool Feature Report
					USB_FFBReport_PIDPool_Feature_Data_t featureData;
					FfbOnPIDPool(&featureData);

					Endpoint_ClearSETUP();

					// Write the report data to the control endpoint
					Endpoint_Write_Control_Stream_LE(&featureData, sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
					Endpoint_ClearOUT();
				}
				else {
					// Joystick report
					USB_JoystickReport_Data_t JoystickReportData;

					/* Create the next HID report to send to the host */
					GetNextReport(&JoystickReportData);

					Endpoint_ClearSETUP();

					/* Write the report data to the control endpoint */
					Endpoint_Write_Control_Stream_LE(&JoystickReportData, sizeof(JoystickReportData));
					Endpoint_ClearOUT();
				}
			}
			/*
			else if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR)) {
				// use union to represent received unsigned as signed integer
				union {
					int16_t sval;
					uint16_t uval;
				} forceval;
			
				forceval.uval = USB_ControlRequest.wValue;
				force = forceval.sval;
				Endpoint_ClearStatusStage();
			}
			*/
			break;
		case HID_REQ_SetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE)) {
				Endpoint_ClearSETUP();

				uint8_t data[10];	// This is enough room for all reports
				uint16_t len = 0;	// again, enough for all

				len = USB_ControlRequest.wLength;
				
				// Read in the report data from host

				// Read the report data from the control endpoint
				Endpoint_Read_Control_Stream_LE(&data, len);
				Endpoint_ClearStatusStage();

				// Process the incoming report
				if (USB_ControlRequest.wValue == 0x0305) {
					// Feature 1
					_delay_us(500);	// Windows does not like to be answered too quickly

					USB_FFBReport_PIDBlockLoad_Feature_Data_t pidBlockLoadData;
					FfbOnCreateNewEffect((USB_FFBReport_CreateNewEffect_Feature_Data_t*) data, &pidBlockLoadData);

					Endpoint_ClearSETUP();

					// Write the report data to the control endpoint
					Endpoint_Write_Control_Stream_LE(&pidBlockLoadData, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
					Endpoint_ClearOUT();
				}
				else if (USB_ControlRequest.wValue == 0x0306) {
					// Feature 1
					// ???? What should be returned here?
				}
				else if (USB_ControlRequest.wValue == 0x0307) {
					// Feature 1
					// ???? What should be returned here?
				}
			}
			break;
	}
}

/** Fills the given HID report data structure with the next HID report to send to the host.
 *
 *  \param[out] ReportData  Pointer to a HID report data structure to be filled
 *
 *  \return Boolean \c true if the new report differs from the last report, \c false otherwise
 */
bool GetNextReport(USB_JoystickReport_Data_t* const ReportData)
{
	/*
	static uint8_t PrevJoyStatus    = 0;
	static uint8_t PrevButtonStatus = 0;
	uint8_t        JoyStatus_LCL    = Joystick_GetStatus();
	uint8_t        ButtonStatus_LCL = Buttons_GetStatus();
	bool           InputChanged     = true;
	*/

	/* Clear the report contents */
	memset(ReportData, 0, sizeof(USB_JoystickReport_Data_t));

	ReportData->reportId = 1;
	ReportData->Wheel = wheelpos;
	ReportData->Throttle = 0xfff - ADCGetValue(0);
	ReportData->Brake = 0xfff - ADCGetValue(1);
	ReportData->Clutch = 0xfff - ADCGetValue(2);
	ReportData->Button = ((PIND >> 2) & 0b11) ^ 0b11; // read button states and invert them

	/* Check if the new report is different to the previous report */
	//InputChanged = (uint8_t)(PrevJoyStatus ^ JoyStatus_LCL) | (uint8_t)(PrevButtonStatus ^ ButtonStatus_LCL);

	/* Save the current joystick status for later comparison */
	//PrevJoyStatus    = JoyStatus_LCL;
	//PrevButtonStatus = ButtonStatus_LCL;

	/* Return whether the new report is different to the previous report or not */
	return true; //InputChanged;
}
/** Function to manage HID report generation and transmission to the host. */
void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
		return;

	
	/* Select the Joystick Report Endpoint */
	Endpoint_SelectEndpoint(JOYSTICK_EPADDR);

	/* Check to see if the host is ready for another packet */
	if (Endpoint_IsINReady())
	{
		USB_JoystickReport_Data_t JoystickReportData;

		/* Create the next HID report to send to the host */
		GetNextReport(&JoystickReportData);

		/* Write Joystick Report Data */
		Endpoint_Write_Stream_LE(&JoystickReportData, sizeof(JoystickReportData), NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();

		/* Clear the report data afterwards */
		memset(&JoystickReportData, 0, sizeof(JoystickReportData));
	}

	// Receive FFB data
	Endpoint_SelectEndpoint(FFB_EPADDR);

	if (Endpoint_IsOUTReceived()) {
		uint8_t out_ffbdata[64];	// enough for any single OUT-report
		uint8_t total_bytes_read = 0;
		
		while (Endpoint_BytesInEndpoint() && total_bytes_read < 64)
		{
			uint16_t out_wait_report_bytes = 0; //, out_report_data_read = 0;

			// Read the reportID from the package to determine amount of data to expect next
			while (Endpoint_Read_Stream_LE(&out_ffbdata, 1, NULL) == ENDPOINT_RWSTREAM_IncompleteTransfer) {
				// busy loop until the first byte is read out
			}

			total_bytes_read += 1;

			out_wait_report_bytes = OutReportSize[out_ffbdata[0]-1] - 1;
			if (out_wait_report_bytes + total_bytes_read >= 64)
			{
				while (1)  {
					//LEDs_SetAllLEDs(LEDS_NO_LEDS);
					_delay_ms(100);
					//LEDs_SetAllLEDs(LEDS_ALL_LEDS);
					_delay_ms(100); 
				}
			}

			//out_report_data_read = 0;
			while (Endpoint_Read_Stream_LE(&out_ffbdata[1], out_wait_report_bytes, NULL) == ENDPOINT_RWSTREAM_IncompleteTransfer) {
				// busy loop until the rest of the report data is read out
			}

			total_bytes_read += out_wait_report_bytes;

			FfbOnUsbData(out_ffbdata, out_wait_report_bytes + 1);
			_delay_ms(1);
		}
		
		// Clear the endpoint ready for new packet
		Endpoint_ClearOUT();
	}
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

void WheelCalibration() {
	/*
		Calibration procedure:
			1. Rotate left until limit is reached
			2. Mark position 0
			3. Rotate right until limit is reached
			4. Get position on right edge, center is half of the position value at the right
			5. Rotate back to center
	*/
	
	//forceoffset = 0;
	wheelpos = INT16_MAX; // assume worst case, right edge
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
		// fast 3000 positions
		// full speed 6000 positions
		if(wheelpos < INT16_MAX-3000) {
			FORCE_LEFT(0x1ff);
		}
		// then slow down
		else {
			FORCE_LEFT(0x2ff);
		}
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
		// full speed 7500 positions
		if(wheelpos < 7500) {
			FORCE_RIGHT(0x3ff);
		}
		// then slow down
		else {
			FORCE_RIGHT(0x1ff);
		}
		_delay_ms(CALIBDELAY);
		velocity = prev_wheelpos - wheelpos;
		prev_wheelpos = wheelpos;
	} while (velocity > 0);
	
	FORCE_STOP();
	
	// step 4
	//wheelpos = (wheelpos >> 1);
	uint16_t center = wheelpos >> 1;
	
	// step 5
	do {
		if(wheelpos > center+500) {
			FORCE_LEFT(0x3ff);
		}
		// then slow down
		else {
			FORCE_LEFT(0xff);
		}
		_delay_ms(CALIBDELAY);
	} while (wheelpos > center);
	
	// brake and slowly rotate to center
	do {
		FORCE_RIGHT(0xff);
		_delay_ms(CALIBDELAY);
	} while (wheelpos < center);
	
	FORCE_STOP();
}

/* Interrupts */
// Optical encoder
ISR(INT0_vect) {
	uint8_t pins = PIND & ((1 << PD0) | (1 << PD1));
	if(pins == 0b11 || pins == 0b00)
		wheelpos--;
	else
		wheelpos++;
}

ISR(INT1_vect) {
	uint8_t pins = PIND & ((1 << PD0) | (1 << PD1));
	if(pins == 0b11 || pins == 0b00)
		wheelpos++;
	else
		wheelpos--;
}

// PID timer
ISR(TIMER2_COMPA_vect)
{
	uint16_t pid_value;
	int16_t feedback = ADCGetValue(3) - currentoffset;
	if(feedback < 0)
		feedback = 0;
	//dbg_fb = feedback;

	if(force == 0) {
		FORCE_STOP()
	}
	if(force > 0) {
		pid_value = pid(force, feedback);
		FORCE_LEFT(pid_value);
	}
	else {
		pid_value = pid(-force, feedback);
		FORCE_RIGHT(pid_value);
	}
	
	TCNT2 = 0;
}