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
	//memset((void*) &gEffectStates[id].data, 0, sizeof(gEffectStates[id].data));
	
	return id;
}

void StopAllEffects(void)
{
	for (uint8_t id = 1; id <= MAX_EFFECTS; id++)
		StopEffect(id);
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
}

void FfbHandle_SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t *data) {
	force = data->magnitude;
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

	}
	// Continue
	else if (control == 0x06) {

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
}

void FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data) {
}

void FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data) {
}

void FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data) {
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
}


void FfbHandle_SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t *data)
	{
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
}


void FfbHandle_SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t *data)
	{
	//uint8_t eid = data->effectBlockIndex;
	/*
	USB effect data:
		uint8_t	reportId;	// =6
		uint8_t	effectBlockIndex;	// 1..40
		int8_t start;
		int8_t	end;
	*/
}