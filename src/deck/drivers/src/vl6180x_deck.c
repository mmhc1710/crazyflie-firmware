/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * vl6180x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "VLX"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"

#include "i2cdev.h"

#include "stabilizer_types.h"
#include "../interface/vl6180x_deck.h"

#define VL6180X_DEFAULT_ADDRESS 0b0101001

#define VL6180x_FAILURE_RESET   false

#define VL6180X_IDENTIFICATION_MODEL_ID              0x0000
#define VL6180X_IDENTIFICATION_MODEL_REV_MAJOR       0x0001
#define VL6180X_IDENTIFICATION_MODEL_REV_MINOR       0x0002
#define VL6180X_IDENTIFICATION_MODULE_REV_MAJOR      0x0003
#define VL6180X_IDENTIFICATION_MODULE_REV_MINOR      0x0004
#define VL6180X_IDENTIFICATION_DATE                  0x0006 //16bit value
#define VL6180X_IDENTIFICATION_TIME                  0x0008 //16bit value

#define VL6180X_SYSTEM_MODE_GPIO0                    0x0010
#define VL6180X_SYSTEM_MODE_GPIO1                    0x0011
#define VL6180X_SYSTEM_HISTORY_CTRL                  0x0012
#define VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO         0x0014
#define VL6180X_SYSTEM_INTERRUPT_CLEAR               0x0015
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET            0x0016
#define VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD        0x0017

#define VL6180X_SYSRANGE_START                       0x0018
#define VL6180X_SYSRANGE_THRESH_HIGH                 0x0019
#define VL6180X_SYSRANGE_THRESH_LOW                  0x001A
#define VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD     0x001B
#define VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME        0x001C
#define VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE 0x001E
#define VL6180X_SYSRANGE_CROSSTALK_VALID_HEIGHT      0x0021
#define VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  0x0022
#define VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET   0x0024
#define VL6180X_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   0x0025
#define VL6180X_SYSRANGE_RANGE_IGNORE_THRESHOLD      0x0026
#define VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT      0x002C
#define VL6180X_SYSRANGE_RANGE_CHECK_ENABLES         0x002D
#define VL6180X_SYSRANGE_VHV_RECALIBRATE             0x002E
#define VL6180X_SYSRANGE_VHV_REPEAT_RATE             0x0031

#define VL6180X_SYSALS_START                         0x0038
#define VL6180X_SYSALS_THRESH_HIGH                   0x003A
#define VL6180X_SYSALS_THRESH_LOW                    0x003C
#define VL6180X_SYSALS_INTERMEASUREMENT_PERIOD       0x003E
#define VL6180X_SYSALS_ANALOGUE_GAIN                 0x003F
#define VL6180X_SYSALS_INTEGRATION_PERIOD            0x0040

#define VL6180X_RESULT_RANGE_STATUS                  0x004D
#define VL6180X_RESULT_ALS_STATUS                    0x004E
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO         0x004F
#define VL6180X_RESULT_ALS_VAL                       0x0050
#define VL6180X_RESULT_HISTORY_BUFFER                0x0052
#define VL6180X_RESULT_RANGE_VAL                     0x0062
#define VL6180X_RESULT_RANGE_RAW                     0x0064
#define VL6180X_RESULT_RANGE_RETURN_RATE             0x0066
#define VL6180X_RESULT_RANGE_REFERENCE_RATE          0x0068
#define VL6180X_RESULT_RANGE_RETURN_SIGNAL_COUNT     0x006C
#define VL6180X_RESULT_RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define VL6180X_RESULT_RANGE_RETURN_AMB_COUNT        0x0074
#define VL6180X_RESULT_RANGE_REFERENCE_AMB_COUNT     0x0078
#define VL6180X_RESULT_RANGE_RETURN_CONV_TIME        0x007C
#define VL6180X_RESULT_RANGE_REFERENCE_CONV_TIME     0x0080

#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD      0x010A
#define VL6180X_FIRMWARE_BOOTUP                      0x0119
#define VL6180X_FIRMWARE_RESULT_SCALER               0x0120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS             0x0212
#define VL6180X_INTERLEAVED_MODE_ENABLE              0x02A3

#define VL6180X_ERROR_NONE         0
#define VL6180X_ERROR_SYSERR_1     1
#define VL6180X_ERROR_SYSERR_5     5
#define VL6180X_ERROR_ECEFAIL      6
#define VL6180X_ERROR_NOCONVERGE   7
#define VL6180X_ERROR_RANGEIGNORE  8
#define VL6180X_ERROR_SNR          11
#define VL6180X_ERROR_RAWUFLOW     12
#define VL6180X_ERROR_RAWOFLOW     13
#define VL6180X_ERROR_RANGEUFLOW   14
#define VL6180X_ERROR_RANGEOFLOW   15


typedef struct
{
	uint8_t idModel;
	uint8_t idModelRevMajor;
	uint8_t idModelRevMinor;
	uint8_t idModuleRevMajor;
	uint8_t idModuleRevMinor;
	uint16_t idDate;
	uint16_t idTime;
} VL6180xIdentification;

typedef enum  { //Data sheet shows gain values as binary list

	GAIN_20 = 0, // Actual ALS Gain of 20
	GAIN_10,     // Actual ALS Gain of 10.32
	GAIN_5,      // Actual ALS Gain of 5.21
	GAIN_2_5,    // Actual ALS Gain of 2.60
	GAIN_1_67,   // Actual ALS Gain of 1.72
	GAIN_1_25,   // Actual ALS Gain of 1.28
	GAIN_1 ,     // Actual ALS Gain of 1.01
	GAIN_40,     // Actual ALS Gain of 40

} vl6180x_als_gain;


/** Default constructor, uses external I2C address.
 * @see VL6180X_DEFAULT_ADDRESS
 */
void vl6180xInit(DeckInfo* info);

bool vl6180xTest(void);
void vl6180xTask(void* arg);

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool vl6180xTestConnection();

// Get Model ID.
void VL6180xGetIdentification(VL6180xIdentification *temp);
void VL6180xPrintIdentification(VL6180xIdentification *temp);


// Initialize sensor
bool vl6180xInitSensor();

//void VL6180x_setRegister(uint16_t registerAddr, uint16_t data);
static bool VL6180x_setRegister(uint16_t registerAddr, uint8_t data);
static uint8_t VL6180x_getRegister(uint16_t registerAddr);
//static float VL6180xGetAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN);
static uint8_t VL6180xGetDistance();
static uint8_t readRangeStatus(void);

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

static uint8_t range_last = 0;
uint8_t range_last2[2] = {0, 0};
//static float light_last = 0.0;
uint8_t status[2] = {0, 0};


// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = xTaskGetTickCount())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)xTaskGetTickCount() - timeout_start_ms) > io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

static uint16_t VL6180x_getRegister16bit(uint8_t reg);
static bool VL6180x_setRegister16bit(uint8_t reg, uint16_t val);
//static bool VL6180x_setRegister32bit(uint8_t reg, uint32_t val);

static void delay(uint32_t cycles);

/** Default constructor, uses default I2C address.
 * @see VL6180X_DEFAULT_ADDRESS
 */
void vl6180xInit(DeckInfo* info)
{
	if (isInit)
		return;
	DEBUG_PRINT("Initiating vl6180x...!\n");
	i2cdevInit(I2C1_DEV);
	I2Cx = I2C1_DEV;
	devAddr = VL6180X_DEFAULT_ADDRESS;
	pinMode(DECK_GPIO_IO1, OUTPUT);

	//  xTaskCreate(vl6180xTask, "vl6180x", 2*configMINIMAL_STACK_SIZE, NULL, 3, NULL);

	isInit = true;
}

bool vl6180xTest(void)
{
	bool testStatus;
	DEBUG_PRINT("Testing vl6180x...!\n");
	if (!isInit)
		return false;

	digitalWrite(DECK_GPIO_IO1, LOW);
	testStatus  = vl6180xTestConnection();
	DEBUG_PRINT("vl6180xTestConnection1 done...!\n");
	DEBUG_PRINT("testStatus1 = %d\n", testStatus);
	testStatus &= vl6180xInitSensor();
	DEBUG_PRINT("vl6180xInitSensor1 done...!\n");
	DEBUG_PRINT("testStatus1 = %d\n", testStatus);
	delay(1000);

	digitalWrite(DECK_GPIO_IO1, HIGH);
	testStatus  = vl6180xTestConnection();
	DEBUG_PRINT("vl6180xTestConnection2 done...!\n");
	DEBUG_PRINT("testStatus2 = %d\n", testStatus);
	testStatus &= vl6180xInitSensor();
	DEBUG_PRINT("vl6180xInitSensor2 done...!\n");
	DEBUG_PRINT("testStatus2 = %d\n", testStatus);
	digitalWrite(DECK_GPIO_IO1, LOW);

	//  digitalWrite(DECK_GPIO_IO1, HIGH);
	//  testStatus  = vl6180xTestConnection();
	//  DEBUG_PRINT("vl6180xTestConnection done...!\n");
	//  DEBUG_PRINT("testStatus = %d\n", testStatus);
	//  testStatus &= vl6180xInitSensor();
	//  DEBUG_PRINT("vl6180xInitSensor done...!\n");
	//  DEBUG_PRINT("testStatus = %d\n", testStatus);

	delay(1000);
	return testStatus;
}

void vl6180xTask(void* arg)
{
	systemWaitStart();
	TickType_t xLastWakeTime;

	while (1) {
		xLastWakeTime = xTaskGetTickCount();
		//light_last = VL6180xGetAmbientLight(GAIN_1);
		//DEBUG_PRINT("light_last = %d.%.2d\n", (int)light_last, (int)((light_last-(int)light_last)*100));
		range_last = VL6180xGetDistance();//vl53l0xReadRangeContinuousMillimeters();
		DEBUG_PRINT("range_last = %d\n", range_last);

		//#if defined(ESTIMATOR_TYPE_kalman) && defined(UPDATE_KALMAN_WITH_RANGING)
		//    // check if range is feasible and push into the kalman filter
		//    // the sensor should not be able to measure >3 [m], and outliers typically
		//    // occur as >8 [m] measurements
		//    if (range_last < RANGE_OUTLIER_LIMIT){
		//
		//      // Form measurement
		//      tofMeasurement_t tofData;
		//      tofData.timestamp = xTaskGetTickCount();
		//      tofData.distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
		//      tofData.stdDev = expStdA * (1.0f  + expf( expCoeff * ( tofData.distance - expPointA)));
		//      stateEstimatorEnqueueTOF(&tofData);
		//    }
		//#endif

		vTaskDelayUntil(&xLastWakeTime, F2T(10));
	}
}

//float VL6180xGetAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN)
//{
//	DEBUG_PRINT("VL6180xGetAmbientLight...\n");
//  //First load in Gain we are using, do it everytime incase someone changes it on us.
//  //Note: Upper nibble shoudl be set to 0x4 i.e. for ALS gain of 1.0 write 0x46
//  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | VL6180X_ALS_GAIN)); // Set the ALS gain
//
//  //Start ALS Measurement
//  VL6180x_setRegister(VL6180X_SYSALS_START, 0x01);
//
//    delay(100); //give it time...
//
//  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
//
//  //Retrieve the Raw ALS value from the sensoe
//  unsigned int alsRaw = VL6180x_getRegister16bit(VL6180X_RESULT_ALS_VAL);
//
//  //Get Integration Period for calculation, we do this everytime incase someone changes it on us.
//  unsigned int alsIntegrationPeriodRaw = VL6180x_getRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD);
//
//  float alsIntegrationPeriod = 100.0 / alsIntegrationPeriodRaw ;
//
//  //Calculate actual LUX from Appnotes
//
//  float alsGain = 0.0;
//
//  switch (VL6180X_ALS_GAIN){
//    case GAIN_20: alsGain = 20.0; break;
//    case GAIN_10: alsGain = 10.32; break;
//    case GAIN_5: alsGain = 5.21; break;
//    case GAIN_2_5: alsGain = 2.60; break;
//    case GAIN_1_67: alsGain = 1.72; break;
//    case GAIN_1_25: alsGain = 1.28; break;
//    case GAIN_1: alsGain = 1.01; break;
//    case GAIN_40: alsGain = 40.0; break;
//  }
//
////Calculate LUX from formula in AppNotes
//
//  float alsCalculated = (float)0.32 * ((float)alsRaw / alsGain) * alsIntegrationPeriod;
//
//  return alsCalculated;
//}


uint8_t VL6180xGetDistance()
{
	//  DEBUG_PRINT("VL6180xGetDistance...\n");
	VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
	delay(10);
	uint8_t distance;
	distance = VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);
	VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
	return distance;
}

/**************************************************************************/
/*!
    @brief  Error message (retreive after ranging)
 */
/**************************************************************************/

uint8_t readRangeStatus(void) {
	return (VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS) >> 4);
}


//TickType_t xLastWakeTime;
void proximityVL6180xFreeRunningRanging(const uint32_t tick)
{

	//	if (RATE_DO_EXECUTE(100, tick)) {
	//			digitalWrite(DECK_GPIO_IO1, LOW);
	//			range_last2[0] = VL6180xGetDistance();
	//			digitalWrite(DECK_GPIO_IO1, HIGH);
	//			range_last2[1] = VL6180xGetDistance();
	//		}

	if (RATE_DO_EXECUTE(100, tick)) {
		digitalWrite(DECK_GPIO_IO1, LOW);
		// wait for device to be ready for range measurement
		while (! (VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS) & 0x01));

		// Start a range measurement
		VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01);

		// Poll until bit 2 is set
		while (! (VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO) & 0x04));

		// read range in mm
		range_last2[0] = VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);

		// clear interrupt
		VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

		status[0] = readRangeStatus();

		//		  if (status[0] == VL6180X_ERROR_NONE) {
		//			  DEBUG_PRINT("No error");
		//		  }

		// Some error occurred, print it out!

		//		if  ((status[0] >= VL6180X_ERROR_SYSERR_1) && (status[0] <= VL6180X_ERROR_SYSERR_5)) {
		//			DEBUG_PRINT("System error");
		//		}
		//		else if (status[0] == VL6180X_ERROR_ECEFAIL) {
		//			DEBUG_PRINT("ECE failure");
		//		}
		//		else if (status[0] == VL6180X_ERROR_NOCONVERGE) {
		//			DEBUG_PRINT("No convergence");
		//		}
		//		else if (status[0] == VL6180X_ERROR_RANGEIGNORE) {
		//			DEBUG_PRINT("Ignoring range");
		//		}
		//		else if (status[0]== VL6180X_ERROR_SNR) {
		//			DEBUG_PRINT("Signal/Noise error");
		//		}
		//		else if (status[0] == VL6180X_ERROR_RAWUFLOW) {
		//			DEBUG_PRINT("Raw reading underflow");
		//		}
		//		else if (status[0] == VL6180X_ERROR_RAWOFLOW) {
		//			DEBUG_PRINT("Raw reading overflow");
		//		}
		//		else if (status[0] == VL6180X_ERROR_RANGEUFLOW) {
		//			DEBUG_PRINT("Range reading underflow");
		//		}
		//		else if (status[0] == VL6180X_ERROR_RANGEOFLOW) {
		//			DEBUG_PRINT("Range reading overflow");
		//		}


		digitalWrite(DECK_GPIO_IO1, HIGH);
		// wait for device to be ready for range measurement
		while (! (VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS) & 0x01));

		// Start a range measurement
		VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01);

		// Poll until bit 2 is set
		while (! (VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO) & 0x04));

		// read range in mm
		range_last2[1] = VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);

		// clear interrupt
		VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

		status[1] = readRangeStatus();

		//				  if (status[1] == VL6180X_ERROR_NONE) {
		//					  DEBUG_PRINT("No error");
		//				  }

		// Some error occurred, print it out!

		//		if  ((status[1] >= VL6180X_ERROR_SYSERR_1) && (status[1] <= VL6180X_ERROR_SYSERR_5)) {
		//			DEBUG_PRINT("System error");
		//		}
		//		else if (status[1] == VL6180X_ERROR_ECEFAIL) {
		//			DEBUG_PRINT("ECE failure");
		//		}
		//		else if (status[1] == VL6180X_ERROR_NOCONVERGE) {
		//			DEBUG_PRINT("No convergence");
		//		}
		//		else if (status[1] == VL6180X_ERROR_RANGEIGNORE) {
		//			DEBUG_PRINT("Ignoring range");
		//		}
		//		else if (status[1] == VL6180X_ERROR_SNR) {
		//			DEBUG_PRINT("Signal/Noise error");
		//		}
		//		else if (status[1] == VL6180X_ERROR_RAWUFLOW) {
		//			DEBUG_PRINT("Raw reading underflow");
		//		}
		//		else if (status[1] == VL6180X_ERROR_RAWOFLOW) {
		//			DEBUG_PRINT("Raw reading overflow");
		//		}
		//		else if (status[1] == VL6180X_ERROR_RANGEUFLOW) {
		//			DEBUG_PRINT("Range reading underflow");
		//		}
		//		else if (status[1] == VL6180X_ERROR_RANGEOFLOW) {
		//			DEBUG_PRINT("Range reading overflow");
		//		}
	}


	//	if (RATE_DO_EXECUTE(500, tick)) {
	//		digitalWrite(DECK_GPIO_IO1, LOW);
	//		if ((0b00000100 & VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO))>>2){
	//			range_last2[0] = VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);
	//			VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
	//		}
	//		else if (0b00000001 & VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS)){
	//			VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
	//		}
	//
	//		digitalWrite(DECK_GPIO_IO1, HIGH);
	//		if ((0b00000100 & VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO))>>2){
	//			range_last2[1] = VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);
	//			VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
	//		}
	//		else if (0b00000001 & VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS)){
	//			VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode
	//		}
	//	}



	//	if (RATE_DO_EXECUTE(100, tick)) {
	//		digitalWrite(DECK_GPIO_IO1, LOW);
	//		if ((0b00000001 & VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS)) & ((0b00000100 & VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO))>>2))
	//			range_last2[0] = VL6180xGetDistance();
	//		digitalWrite(DECK_GPIO_IO1, HIGH);
	//		if ((0b00000001 & VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS)) & ((0b00000100 & VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO))>>2))
	//			range_last2[1] = VL6180xGetDistance();
	//	}


	//	if (RATE_DO_EXECUTE(100, tick)) {
	//		digitalWrite(DECK_GPIO_IO1, LOW);
	//		if ((0b00000001 & VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS)) & ((0b00000100 & VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO))>>2))
	//			sensorData->range.front = VL6180xGetDistance();
	//		digitalWrite(DECK_GPIO_IO1, HIGH);
	//		if ((0b00000001 & VL6180x_getRegister(VL6180X_RESULT_RANGE_STATUS)) & ((0b00000100 & VL6180x_getRegister(VL6180X_RESULT_INTERRUPT_STATUS_GPIO))>>2))
	//			sensorData->range.back = VL6180xGetDistance();
	//	}
}


static void delay(uint32_t cycles)
{
	volatile uint32_t i;

	for(i = 0UL; i < cycles ;++i)
	{
		__NOP();
	}
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */

VL6180xIdentification identification;

//VL6180xIdentification identification;
bool vl6180xTestConnection()
{
	bool ret = true;
	DEBUG_PRINT("Testing connection...\n");
	VL6180xGetIdentification(&identification);
	VL6180xPrintIdentification(&identification); // Helper function to print all the Module information

	//  ret &= (identification.idModel) == VL6180X_IDENTIFICATION_MODEL_ID;
	//  ret &= vl53l0xGetRevisionID() == VL53L0X_IDENTIFICATION_REVISION_ID;
	return ret;
}

// Get Model ID.
void VL6180xGetIdentification(VL6180xIdentification *temp){
	DEBUG_PRINT("VL6180xGetIdentification...\n");
	temp->idModel =  VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_ID);
	temp->idModelRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR);
	temp->idModelRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MINOR);
	temp->idModuleRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR);
	temp->idModuleRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MINOR);

	temp->idDate = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_DATE);
	temp->idTime = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_TIME);
}

void VL6180xPrintIdentification(VL6180xIdentification *temp){
	DEBUG_PRINT("Model ID = %d\n", temp->idModel);
	DEBUG_PRINT("Model Rev = %d.%d\n", temp->idModelRevMajor, temp->idModelRevMinor);
	DEBUG_PRINT("Module Rev = %d.%d\n", temp->idModuleRevMajor, temp->idModuleRevMinor);
	DEBUG_PRINT("Manufacture Date = %d/%d/1%d\n", (temp->idDate >> 3) & 0x001F, (temp->idDate >> 8) & 0x000F, (temp->idDate >> 12) & 0x000F);
	DEBUG_PRINT("Phase: = %d\n", temp->idDate & 0x0007);
	DEBUG_PRINT("Manufacture Time (s)= %d\n", temp->idTime * 2);
}

// Initialize sensor
bool vl6180xInitSensor()
{
	uint8_t data; //for temp data storage

	DEBUG_PRINT("vl6180xInitSensor...!\n");

	data = VL6180x_getRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	DEBUG_PRINT("vl6180xInitSensor data = %d\n", data);
	if(data != 1) return VL6180x_FAILURE_RESET;
	////DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);

	//Required by datasheet
	//http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
	VL6180x_setRegister(0x0207, 0x01);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x0208, 0x01);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x0096, 0x00);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x0097, 0xfd);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00e3, 0x00);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00e4, 0x04);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00e5, 0x02);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00e6, 0x01);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00e7, 0x03);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00f5, 0x02);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00d9, 0x05);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00db, 0xce);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00dc, 0x03);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00dd, 0xf8);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x009f, 0x00);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00a3, 0x3c);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00b7, 0x00);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00bb, 0x3c);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00b2, 0x09);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00ca, 0x09);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x0198, 0x01);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x01b0, 0x17);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x01ad, 0x00);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x00ff, 0x05);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x0100, 0x05);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x0199, 0x05);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x01a6, 0x1b);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x01ac, 0x3e);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x01a7, 0x1f);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(0x0030, 0x00);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);

	//Recommended settings from datasheet
	//http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf

	//Enable Interrupts on Conversion Complete (any source)
	VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, (4 << 3)|(4) ); // Set GPIO1 high when sample complete
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);


	VL6180x_setRegister(VL6180X_SYSTEM_MODE_GPIO1, 0x10); // Set GPIO1 high when sample complete
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30); //Set Avg sample period
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, 0x46); // Set the ALS gain
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63); // Set ALS integration time to 100ms
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform a single temperature calibration
	//Optional settings from datasheet
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	//http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
	VL6180x_setRegister(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement period to 100ms
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A); // Set default ALS inter-measurement period to 100ms
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	//Additional settings defaults from community
	VL6180x_setRegister(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister16bit(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);

	VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD,0x30);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN,0x40);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);
	VL6180x_setRegister(VL6180X_FIRMWARE_RESULT_SCALER,0x01);
	//DEBUG_PRINT("vl6180xInitSensor test %d\n", i++);

	VL6180x_setRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	return true;
}

uint16_t VL6180x_getRegister16bit(uint8_t reg)
{
	uint8_t buffer[2] = {};
	i2cdevRead(I2Cx, devAddr, reg, 2, (uint8_t *)&buffer);
	return ((uint16_t)(buffer[0]) << 8) | buffer[1];
}

bool VL6180x_setRegister16bit(uint8_t reg, uint16_t val)
{
	uint8_t buffer[2] = {};
	buffer[0] = ((val >> 8) & 0xFF);
	buffer[1] = ((val     ) & 0xFF);
	return i2cdevWrite(I2Cx, devAddr, reg, 2, (uint8_t *)&buffer);
}

//bool VL6180x_setRegister32bit(uint8_t reg, uint32_t val)
//{
//  uint8_t buffer[4] = {};
//  buffer[0] = ((val >> 24) & 0xFF);
//  buffer[1] = ((val >> 16) & 0xFF);
//  buffer[2] = ((val >>  8) & 0xFF);
//  buffer[3] = ((val      ) & 0xFF);
//  return i2cdevWrite(I2Cx, devAddr, reg, 4, (uint8_t *)&buffer);
//}

bool VL6180x_setRegister(uint16_t registerAddr, uint8_t data)
{
	// Write a data byte to a 16bit internal register of the VL6180
	return i2cdevWrite16(I2Cx, devAddr, registerAddr, 1, &data); //I2Cx and devAddr set during init.
}

uint8_t VL6180x_getRegister(uint16_t registerAddr)
{
	uint8_t data;
	// Read a data byte to a 16bit internal register of the VL6180
	i2cdevRead16(I2Cx, devAddr, registerAddr, 1, &data); //I2Cx and devAddr set during init.
	return data;
}

// TODO: Decide on vid:pid and set the used pins
static const DeckDriver vl6180x_deck = {
		.vid = 0,
		.pid = 0,
		.name = "vl6180x_deck",
		.usedGpio = DECK_GPIO_IO1,

		.init = vl6180xInit,
		.test = vl6180xTest,
};

DECK_DRIVER(vl6180x_deck);

//LOG_GROUP_START(range)
////LOG_ADD(LOG_FLOAT, light, &light_last)
//LOG_ADD(LOG_UINT8, range, &range_last)
//LOG_ADD(LOG_UINT8, range1, &range_last2[0])
//LOG_ADD(LOG_UINT8, range2, &range_last2[1])
//LOG_GROUP_STOP(range)

LOG_GROUP_START(vl6180x_status)
LOG_ADD(LOG_UINT8, status1, &status[0])
LOG_ADD(LOG_UINT8, status2, &status[1])
LOG_GROUP_STOP(vl6180x_status)

