/*
 * ffb.h
 *
 * Created: 13.12.2013 21:10:03
 *  Author: Jari
 */ 


#ifndef FFB_H_
#define FFB_H_

#include <stdint.h>

// Maximum number of parallel effects in memory
#define MAX_EFFECTS 20

// Lengths of each report type
extern const uint16_t OutReportSize[];

// FFB: Set Effect Output Report
typedef struct { 
	uint8_t	reportId;	// =1
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t	effectType;	// 1..12 (effect usages: 26,27,30,31,32,33,34,40,41,42,43,28)
	uint16_t	duration; // 0..32767 ms
	uint16_t	triggerRepeatInterval; // 0..32767 ms
	uint16_t	samplePeriod;	// 0..32767 ms
	uint8_t	gain;	// 0..255	 (physical 0..10000)
	uint8_t	triggerButton;	// button ID (0..8)
	uint8_t	enableAxis; // bits: 0=X, 1=Y, 2=DirectionEnable
	uint8_t	directionX;	// angle (0=0 .. 255=360deg)
	uint8_t	directionY;	// angle (0=0 .. 255=360deg)
	//	uint16_t	startDelay;	// 0..32767 ms
} USB_FFBReport_SetEffect_Output_Data_t;

// FFB: Set Envelope Output Report
typedef struct {
	uint8_t	reportId;	// =2
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t attackLevel;
	uint8_t	fadeLevel;
	uint16_t	attackTime;	// ms
	uint16_t	fadeTime;	// ms
} USB_FFBReport_SetEnvelope_Output_Data_t;

// FFB: Set Condition Output Report
typedef struct {
	uint8_t	reportId;	// =3
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t	parameterBlockOffset;	// bits: 0..3=parameterBlockOffset, 4..5=instance1, 6..7=instance2
	uint8_t cpOffset;	// 0..255
	int8_t	positiveCoefficient;	// -128..127
	//	int8_t	negativeCoefficient;	// -128..127
	//	uint8_t	positiveSaturation;	// -128..127
	//	uint8_t	negativeSaturation;	// -128..127
	//	uint8_t	deadBand;	// 0..255
} USB_FFBReport_SetCondition_Output_Data_t;

// FFB: Set Periodic Output Report
typedef struct { 
	uint8_t	reportId;	// =4
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t magnitude;
	int8_t	offset;
	uint8_t	phase;	// 0..255 (=0..359, exp-2)
	uint16_t	period;	// 0..32767 ms
} USB_FFBReport_SetPeriodic_Output_Data_t;

// FFB: Set ConstantForce Output Report
typedef struct {
	uint8_t	reportId;	// =5
	uint8_t	effectBlockIndex;	// 1..40
	int16_t magnitude;	// -255..255
} USB_FFBReport_SetConstantForce_Output_Data_t;

// FFB: Set RampForce Output Report
typedef struct { 
	uint8_t	reportId;	// =6
	uint8_t	effectBlockIndex;	// 1..40
	int8_t start;
	int8_t	end;
} USB_FFBReport_SetRampForce_Output_Data_t;

// FFB: Set CustomForceData Output Report
typedef struct {
	uint8_t	reportId;	// =7
	uint8_t	effectBlockIndex;	// 1..40
	uint8_t dataOffset;
	int8_t	data[12];
} USB_FFBReport_SetCustomForceData_Output_Data_t;

// FFB: Set DownloadForceSample Output Report
typedef struct {
	uint8_t	reportId;	// =8
	int8_t	x;
	int8_t	y;
} USB_FFBReport_SetDownloadForceSample_Output_Data_t;

// FFB: Set EffectOperation Output Report
typedef struct { 
	uint8_t	reportId;	// =10
	uint8_t effectBlockIndex;	// 1..40
	uint8_t operation; // 1=Start, 2=StartSolo, 3=Stop
	uint8_t	loopCount;
} USB_FFBReport_EffectOperation_Output_Data_t;

// FFB: Block Free Output Report
typedef struct {
	uint8_t	reportId;	// =11
	uint8_t effectBlockIndex;	// 1..40
} USB_FFBReport_BlockFree_Output_Data_t;

// FFB: Device Control Output Report
typedef struct { 
	uint8_t	reportId;	// =12
	uint8_t control;	// 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
} USB_FFBReport_DeviceControl_Output_Data_t;

// FFB: DeviceGain Output Report
typedef struct { 
	uint8_t	reportId;	// =13
	uint8_t gain;
} USB_FFBReport_DeviceGain_Output_Data_t;

// FFB: Set Custom Force Output Report
typedef struct { 
	uint8_t		reportId;	// =14
	uint8_t effectBlockIndex;	// 1..40
	uint8_t	sampleCount;
	uint16_t	samplePeriod;	// 0..32767 ms
} USB_FFBReport_SetCustomForce_Output_Data_t;

// ---- Features
// FFB: Create New Effect Feature Report
typedef struct { 
	uint8_t		reportId;	// =1
	uint8_t	effectType;	// Enum (1..12): ET 26,27,30,31,32,33,34,40,41,42,43,28
	uint16_t	byteCount;	// 0..511
} USB_FFBReport_CreateNewEffect_Feature_Data_t;

// FFB: PID Block Load Feature Report
typedef struct {
	uint8_t	reportId;	// =2
	uint8_t effectBlockIndex;	// 1..40
	uint8_t	loadStatus;	// 1=Success,2=Full,3=Error
	uint16_t	ramPoolAvailable;	// =0 or 0xFFFF?
} USB_FFBReport_PIDBlockLoad_Feature_Data_t;

// FFB: PID Pool Feature Report
typedef struct { 
	uint8_t	reportId;	// =3
	uint16_t	ramPoolSize;	// ?
	uint8_t		maxSimultaneousEffects;	// ?? 40?
	uint8_t		memoryManagement;	// Bits: 0=DeviceManagedPool, 1=SharedParameterBlocks
} USB_FFBReport_PIDPool_Feature_Data_t;

typedef struct {
	uint8_t state;                                                 // see constants <MEffectState_*>
	uint16_t usb_duration, usb_fadeTime;                           // used to calculate fadeTime to MIDI, since in USB it is given as time difference from the end while in MIDI it is given as time from start
	uint8_t usb_gain, usb_offset, usb_attackLevel, usb_fadeLevel;  // These are used to calculate effects of USB gain to MIDI data
	uint8_t usb_magnitude;
	//FFP_MIDI_Effect_Basic data;                                  // For FFP, this is enough for all types of effects - cast for other effect types when necessary
} TEffectState;

// ---- Input

typedef struct {
	uint8_t	reportId;	// =2
	uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
	uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)
} USB_FFBReport_PIDStatus_Input_Data_t;

// Handle incoming feature requests
void FfbOnCreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData);
void FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data);
void FfbOnUsbData(uint8_t *data, uint16_t len);

#endif /* FFB_H_ */