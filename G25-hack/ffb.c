/*
 * ffb.c
 *
 * Created: 13.12.2013 21:13:39
 *  Author: Jari
 */ 

#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include "ffb.h"
#include "pid.h"

// force value
int16_t force;

volatile uint8_t nextEID = 2; // FFP effect indexes starts from 2 (yes, we waste memory for two effects...)
volatile USB_FFBReport_PIDStatus_Input_Data_t pidState;	// For holding device status flags
static volatile TEffectState gEffectStates[MAX_EFFECTS+1];	// one for each effect (array index 0 is unused to simplify things)

const uint16_t USB_DURATION_INFINITE = 0x7FFF;
const uint16_t MIDI_DURATION_INFINITE = 0;

// Bit-masks for effect states
const uint8_t MEffectState_Free = 0x00;
const uint8_t MEffectState_Allocated = 0x01;
const uint8_t MEffectState_Playing = 0x02;
const uint8_t MEffectState_SentToJoystick = 0x04;

uint8_t GetNextFreeEffect(void);
void StartEffect(uint8_t id);
void StopEffect(uint8_t id);
void StopAllEffects(void);
void FreeEffect(uint8_t id);
void FreeAllEffects(void);

void FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data);
void FfbHandle_SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t *data);
void FfbHandle_SetCondition(USB_FFBReport_SetCondition_Output_Data_t *data);
void FfbHandle_SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t *data);
void FfbHandle_SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t *data);
void FfbHandle_SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t *data);
void FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data);
void FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data);
void FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data);
void FfbHandle_BlockFree(USB_FFBReport_BlockFree_Output_Data_t *data);
void FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data);
void FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data);
void FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data);

// Lengths of each report type
const uint16_t OutReportSize[] = {
	sizeof(USB_FFBReport_SetEffect_Output_Data_t),                 // 1
	sizeof(USB_FFBReport_SetEnvelope_Output_Data_t),               // 2 
	sizeof(USB_FFBReport_SetCondition_Output_Data_t),              // 3 
	sizeof(USB_FFBReport_SetPeriodic_Output_Data_t),               // 4 
	sizeof(USB_FFBReport_SetConstantForce_Output_Data_t),          // 5
	sizeof(USB_FFBReport_SetRampForce_Output_Data_t),              // 6 
	sizeof(USB_FFBReport_SetCustomForceData_Output_Data_t),        // 7 
	sizeof(USB_FFBReport_SetDownloadForceSample_Output_Data_t),    // 8 
	0,                                                             // 9
	sizeof(USB_FFBReport_EffectOperation_Output_Data_t),           // 10 
	sizeof(USB_FFBReport_BlockFree_Output_Data_t),                 // 11 
	sizeof(USB_FFBReport_DeviceControl_Output_Data_t),             // 12 
	sizeof(USB_FFBReport_DeviceGain_Output_Data_t),                // 13 
	sizeof(USB_FFBReport_SetCustomForce_Output_Data_t),            // 14
};

void FfbOnCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData) {
	/*
	USB effect data:
		uint8_t reportId;   // =1
		uint8_t effectType; // Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
		uint16_t byteCount; // 0..511	- only valid with Custom Force
	*/
	outData->reportId = 6;
	outData->effectBlockIndex = GetNextFreeEffect();
	if (outData->effectBlockIndex == 0)
		outData->loadStatus = 2;	// 1=Success, 2=Full, 3=Error
	else {
		outData->loadStatus = 1;	// 1=Success, 2=Full, 3=Error

		// Set defaults to the effect data
		volatile TEffectState *effect_state = &gEffectStates[outData->effectBlockIndex];
		effect_state->usb_duration = USB_DURATION_INFINITE;
		effect_state->usb_fadeTime = USB_DURATION_INFINITE;
		effect_state->usb_gain = 0xFF;
		effect_state->usb_offset = 0;
		effect_state->usb_attackLevel = 0xFF;
		effect_state->usb_fadeLevel = 0xFF;
	}

	outData->ramPoolAvailable = 0xFFFF;	// =0 or 0xFFFF - don't really know what this is used for?

	_delay_ms(5);
}

void FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data) {
	FreeAllEffects();
	data->reportId = 7;
	data->ramPoolSize = 0xFFFF;
	data->maxSimultaneousEffects = 0x0A;	// FFP supports playing up to 10 simultaneous effects
	data->memoryManagement = 3;
}

// Handle incoming data from USB and convert it to MIDI data to joystick
void FfbOnUsbData(uint8_t *data, uint16_t len) {
	// data[0] == reportID
	switch (data[0]) {
		case 1:
			FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t*) data);
			break;
		case 2:
			FfbHandle_SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*) data);
			break;
		case 3:
			FfbHandle_SetCondition((USB_FFBReport_SetCondition_Output_Data_t*) data);
			break;
		case 4:
			FfbHandle_SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*) data);
			break;
		case 5:
			FfbHandle_SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*) data);
			break;
		case 6:
			FfbHandle_SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*) data);
			break;
		case 7:
			FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
			break;
		case 8:
			FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
			break;
		case 9:
			break;
		case 10:
			FfbHandle_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*) data);
			break;
		case 11:
			FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t *) data);
			break;
		case 12:
			FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*) data);
			break;
		case 13:
			FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*) data);
			break;
		case 14:
			FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*) data);
			break;
		default:
			break;
	}
}

uint8_t GetNextFreeEffect(void)
{
	if (nextEID == MAX_EFFECTS)
		return 0;

	uint8_t id = nextEID++;

	// Find the next free effect ID for next time
	while (gEffectStates[nextEID].state != 0)
	{
		if (nextEID >= MAX_EFFECTS)
			break;	// the last spot was taken
		nextEID++;
	}

	gEffectStates[id].state = MEffectState_Allocated;
	memset((void*) &gEffectStates[id].data, 0, sizeof(gEffectStates[id].data));
	
	return id;
}

void StopAllEffects(void)
{
	for (uint8_t id = 1; id <= MAX_EFFECTS; id++)
		StopEffect(id);
	force = 0;
}

void StartEffect(uint8_t id)
{
	if (id > MAX_EFFECTS)
		return;
	gEffectStates[id].state |= MEffectState_Playing;
}

void StopEffect(uint8_t id)
{
	if (id > MAX_EFFECTS)
		return;
	gEffectStates[id].state &= ~MEffectState_Playing;
}

void FreeEffect(uint8_t id)
{
	if (id > MAX_EFFECTS)
		return;

	gEffectStates[id].state = 0;
	if (id < nextEID)
		nextEID = id;
}

void FreeAllEffects(void)
{
	nextEID = 2;
	memset((void*) gEffectStates, 0, sizeof(gEffectStates));
}

void FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data) {
	uint8_t eid = data->effectBlockIndex;
	/*
	USB effect data:
		uint8_t	reportId;	// =1
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t	effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
		uint16_t	duration; // 0..32767 ms
		uint16_t	triggerRepeatInterval; // 0..32767 ms
		uint16_t	samplePeriod;	// 0..32767 ms
		uint8_t	gain;	// 0..255	 (physical 0..10000)
		uint8_t	triggerButton;	// button ID (0..8)
		uint8_t	enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
		uint8_t	directionX;	// angle (0=0 .. 180=0..360deg)
		uint8_t	directionY;	// angle (0=0 .. 180=0..360deg)
	*/

	gEffectStates[eid].usb_duration = data->duration;	// store for later calculation of <fadeTime>

	bool is_periodic = false;

	volatile TEffectState *effect = &gEffectStates[eid];
	uint16_t usbdir;
	uint16_t dir;
	uint8_t magnitude;

	// Fill in the effect type specific data
	switch (data->effectType) {
		case 3:	// square (midi: 5)
		case 4:	// sine (midi: 2) or cosine (midi: 3)
		case 5:	// triangle (midi: 8)
		case 6: // sawtooth up (midi: 0x0a)
		case 7: // sawtooth down (midi: 0x0b)
			is_periodic = true;
		case 1:	// constant force (midi: 0x12)
		case 2:	// ramp up & down (midi: 6 or 7)
			// Convert direction
			usbdir = data->directionX * 2;
			dir = (usbdir & 0x7F) + ( (usbdir & 0x0180) << 1 );
			//midi_data->direction = dir;

			// Recalculate fadeTime for MIDI since change to duration changes the fadeTime too
			if (data->duration == USB_DURATION_INFINITE) {
				//midi_data->fadeTime = MIDI_DURATION_INFINITE;
				asm ("nop");
			}
			else {
				if (effect->usb_fadeTime == USB_DURATION_INFINITE) {
					//midi_data->fadeTime = MIDI_DURATION_INFINITE;
					asm ("nop");
				}
				else {
					if (effect->usb_duration > effect->usb_fadeTime) { // add some safety and special case handling
						//midi_data->fadeTime = UsbUint16ToMidiUint14(effect->usb_duration - effect->usb_fadeTime);
						asm ("nop");
					}
					else {
						//midi_data->fadeTime = midi_data->duration;
						asm ("nop");
					}
				}
			}

			// Gain and its effects (magnitude and envelope levels)
			//bool gain_changed = (gEffectStates[eid].usb_gain != data->gain);
			if (gEffectStates[eid].usb_gain != data->gain) {
				gEffectStates[eid].usb_gain = data->gain;
				//midi_data->attackLevel = CalcGain(effect->usb_attackLevel, data->gain);
				//midi_data->fadeLevel = CalcGain(effect->usb_fadeLevel, data->gain);

				if (is_periodic) {
					// Calculate min-max from magnitude and offset, since magnitude may be affected by gain we must calc them here too for periodic effects
					magnitude = 0; //CalcGain(effect->usb_magnitude, effect->usb_gain);	// already at MIDI-level i.e. 1/2 of USB level!
					//midi_data->param1 = UsbInt8ToMidiInt14(effect->usb_offset + magnitude); // max
					//midi_data->param2 = UsbInt8ToMidiInt14(effect->usb_offset - magnitude); // min
					if (effect->state & MEffectState_SentToJoystick) {
						//FfbSendModify(eid, 0x74, midi_data->param1);
						//FfbSendModify(eid, 0x78, midi_data->param2);
						asm ("nop");
					}
				}
				else {
					//midi_data->magnitude = CalcGain(effect->usb_magnitude, data->gain);
					asm ("nop");
				}
			}

			// Send data to MIDI
			if (gEffectStates[eid].state & MEffectState_SentToJoystick) {	// Send update
				//FfbSendModify(eid, 0x48, midi_data->direction);
				//FfbSendModify(eid, 0x60, midi_data->fadeTime);
				if (gEffectStates[eid].usb_gain != data->gain) {
					//FfbSendModify(eid, 0x6C, midi_data->fadeLevel);	// might have changed due gain
					//FfbSendModify(eid, 0x64, midi_data->attackLevel);	// might have changed due gain
					if (!is_periodic) {
						//FfbSendModify(eid, 0x74, midi_data->magnitude);	// might have changed due gain
						asm ("nop");
					}
				}
			}
			else {
				//FfbSendSysEx((uint8_t*) midi_data, sizeof(FFP_MIDI_Effect_Basic));
				effect->state |= MEffectState_SentToJoystick;
			}
			break;
		case 8:	// spring (midi: 0x0d)
		case 9:	// damper (midi: 0x0e)
		case 10:	// inertia (midi: 0x0f)
			/*
			MIDI effect data:
				uint8_t command;	// always 0x23	-- start counting checksum from here
				uint8_t waveForm;	// 2=sine, 5=Square, 6=RampUp, 7=RampDown, 8=Triange, 0x12=Constant
				uint8_t unknown1;	// ? always 0x7F
				uint16_t duration;	// unit=2ms
				uint16_t unknown2;	// ? always 0x0000
				uint16_t coeffAxis0;
				uint16_t coeffAxis1;
				uint16_t offsetAxis0;
				uint16_t offsetAxis1;
			*/
//			volatile FFP_MIDI_Effect_Spring_Inertia_Damper *midi_data = (FFP_MIDI_Effect_Spring_Inertia_Damper *) &gEffectStates[eid].data;
			//midi_data_len = 0;

			// Send data to MIDI
			if (gEffectStates[eid].state & MEffectState_SentToJoystick) {	// Send update
				asm ("nop");
			}
			break;
		case 11:	// friction (midi: 0x10)
			/*
			MIDI effect data:
				uint8_t command;	// always 0x23	-- start counting checksum from here
				uint8_t waveForm;	// 2=sine, 5=Square, 6=RampUp, 7=RampDown, 8=Triange, 0x12=Constant
				uint8_t unknown1;	// ? always 0x7F
				uint16_t duration;	// unit=2ms
				uint16_t unknown2;	// ? always 0x0000
				uint16_t coeffAxis0;
				uint16_t coeffAxis1;
			*/
//			volatile FFP_MIDI_Effect_Friction *midi_data = (FFP_MIDI_Effect_Friction *) &gEffectStates[eid].data;
			//midi_data_len = 0;

			// Send data to MIDI
			if (gEffectStates[eid].state & MEffectState_SentToJoystick) {	// Send update
				asm ("nop");	
			}
			break;
		case 12:	// custom (midi: ? does FFP support custom forces?)
			break;
		default:
			break;
	}

	// Send full effect data to MIDI if this effect has not been sent yet
	if (!(gEffectStates[eid].state & MEffectState_SentToJoystick)) {
		//FfbSendSysEx((uint8_t*) common_midi_data, midi_data_len);
		gEffectStates[eid].state |= MEffectState_SentToJoystick;
	}
}

void FfbHandle_SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t *data) {
	/*
	USB data:
		uint8_t	reportId;	// =5
		uint8_t	effectBlockIndex;	// 1..40
		int16_t magnitude;	// -255..255

	*/
	force = data->magnitude << 3;
}

void FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data) {
	uint8_t control = data->control;
	// 1=Enable Actuators, 2=Disable Actuators, 3=Stop All Effects, 4=Reset, 5=Pause, 6=Continue

	// PID State Report:
	//	uint8_t	reportId;	// =2
	//	uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
	//	uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)

	pidState.reportId = 2;
	pidState.status |= 1 << 2;
	pidState.status |= 1 << 4;
	pidState.effectBlockIndex = 0;

	// Disable Actuators
	if (control == 0x01) { 
		pidState.status = (pidState.status & 0xFE);
	}
	// Enable Actuators
	else if (control == 0x02) {
		pidState.status |= 1 << 2;
	}
	// Stop all effects (e.g. FFB-application to foreground)
	else if (control == 0x03) {
		pidState.effectBlockIndex = 0;
	}
	// Reset (e.g. FFB-application out of focus)
	else if (control == 0x04) {
		FreeAllEffects();
	}
	// Pause
	else if (control == 0x05) {
		asm ("nop");
	}
	// Continue
	else if (control == 0x06) {
		asm ("nop");
	}
}


void FfbHandle_BlockFree(USB_FFBReport_BlockFree_Output_Data_t *data) {
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF)
	{	// all effects
		FreeAllEffects();
	}
	else
	{
		FreeEffect(eid);
	}
}

void FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data) {
}

void FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data) {
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF)
		eid = 0x7F;	// All effects

	// Start
	if (data->operation == 1) {	
		StartEffect(data->effectBlockIndex);
	}
	// StartSolo
	else if (data->operation == 2) {	
		// Stop all first
		StopAllEffects();

		// Then start the given effect
		StartEffect(data->effectBlockIndex);
	}
	// Stop
	else if (data->operation == 3) {	
		StopEffect(data->effectBlockIndex);
	}
}

void FfbHandle_SetCondition(USB_FFBReport_SetCondition_Output_Data_t *data) {
	//uint8_t eid = data->effectBlockIndex;
	/*
	USB effect data:
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t	parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
		int8_t cpOffset;	// -128..127
		uint8_t	positiveCoefficient;	// 0..255
	*/
	asm ("nop");
}

void FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data) {
	asm ("nop");
}

void FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data) {
	asm ("nop");
}

void FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data) {
	asm ("nop");
}


void FfbHandle_SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t *data) {
	//uint8_t eid = data->effectBlockIndex;
	/*
	USB effect data:
		uint8_t	reportId;	// =2
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t attackLevel;
		uint8_t	fadeLevel;
		uint16_t	attackTime;	// ms
		uint16_t	fadeTime;	// ms
	*/
	asm ("nop");
}


void FfbHandle_SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t *data) {
	//uint8_t eid = data->effectBlockIndex;
	/*
	USB effect data:
		uint8_t	reportId;	// =4
		uint8_t	effectBlockIndex;	// 1..40
		uint8_t magnitude;
		int8_t	offset;
		uint8_t	phase;	// 0..255 (=0..359, exp-2)
		uint16_t	period;	// 0..32767 ms
	*/
	asm ("nop");
}


void FfbHandle_SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t *data) {
	//uint8_t eid = data->effectBlockIndex;
	/*
	USB effect data:
		uint8_t	reportId;	// =6
		uint8_t	effectBlockIndex;	// 1..40
		int8_t start;
		int8_t	end;
	*/
	asm ("nop");
}