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
 * vl53l0x.h: Time-of-flight distance sensor driver
 */

#ifndef _VL53L0X_H_
#define _VL53L0X_H_

#define VL53L0X_DEFAULT_ADDRESS 0b0101001

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



#define VL53L0X_RA_SYSRANGE_START                              0x018//0x00

#define VL53L0X_RA_SYSTEM_THRESH_HIGH                          0x0019//0x0C
#define VL53L0X_RA_SYSTEM_THRESH_LOW                           0x001A//0x0E

#define VL53L0X_RA_SYSTEM_SEQUENCE_CONFIG                      0x01
#define VL53L0X_RA_SYSTEM_RANGE_CONFIG                         0x09
#define VL53L0X_RA_SYSTEM_INTERMEASUREMENT_PERIOD              0x003E//0x04

#define VL53L0X_RA_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0014//0x0A

#define VL53L0X_RA_GPIO_HV_MUX_ACTIVE_HIGH                     0x84

#define VL53L0X_RA_SYSTEM_INTERRUPT_CLEAR                      0x0015//0x0B

#define VL53L0X_RA_RESULT_INTERRUPT_STATUS                     0x004F//0x13
#define VL53L0X_RA_RESULT_RANGE_STATUS                         0x004D//0x14

#define VL53L0X_RA_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define VL53L0X_RA_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define VL53L0X_RA_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define VL53L0X_RA_RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define VL53L0X_RA_RESULT_PEAK_SIGNAL_RATE_REF                 0xB6

#define VL53L0X_RA_ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28

#define VL53L0X_RA_I2C_SLAVE_DEVICE_ADDRESS                    0x8A

#define VL53L0X_RA_MSRC_CONFIG_CONTROL                         0x60

#define VL53L0X_RA_PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define VL53L0X_RA_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define VL53L0X_RA_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64

#define VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define VL53L0X_RA_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define VL53L0X_RA_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44

#define VL53L0X_RA_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define VL53L0X_RA_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62

#define VL53L0X_RA_PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define VL53L0X_RA_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52

#define VL53L0X_RA_SYSTEM_HISTOGRAM_BIN                        0x81
#define VL53L0X_RA_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define VL53L0X_RA_HISTOGRAM_CONFIG_READOUT_CTRL               0x55

#define VL53L0X_RA_FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define VL53L0X_RA_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define VL53L0X_RA_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20

#define VL53L0X_RA_MSRC_CONFIG_TIMEOUT_MACROP                  0x46

#define VL53L0X_RA_SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define VL53L0X_RA_IDENTIFICATION_MODEL_ID                     0xC0
#define VL53L0X_RA_IDENTIFICATION_REVISION_ID                  0xC2

#define VL53L0X_IDENTIFICATION_MODEL_ID                        0xEEAA
#define VL53L0X_IDENTIFICATION_REVISION_ID                     0x10

#define VL53L0X_RA_OSC_CALIBRATE_VAL                           0xF8

#define VL53L0X_RA_GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define VL53L0X_RA_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5

#define VL53L0X_RA_GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define VL53L0X_RA_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define VL53L0X_RA_DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VL53L0X_RA_POWER_MANAGEMENT_GO1_POWER_FORCE            0x80

#define VL53L0X_RA_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

#define VL53L0X_RA_ALGO_PHASECAL_LIM                           0x30
#define VL53L0X_RA_ALGO_PHASECAL_CONFIG_TIMEOUT                0x30

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection
typedef struct {
  bool tcc;
  bool msrc;
  bool dss;
  bool pre_range;
  bool final_range;
} SequenceStepEnables;

typedef struct
{
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

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

typedef enum vcselPeriodType_t { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

/** Default constructor, uses external I2C address.
 * @see VL53L0X_DEFAULT_ADDRESS
 */
void vl53l0xInit(DeckInfo* info);

bool vl53l0xTest(void);
void vl53l0xTask(void* arg);

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool vl53l0xTestConnection();

/** Get Model ID.
 * This register is used to verify the model number of the device,
 * but only before it has been configured to run
 * @return Model ID
 * @see VL53L0X_RA_IDENTIFICATION_MODEL_ID
 * @see VL53L0X_IDENTIFICATION_MODEL_ID
 */
//uint16_t vl53l0xGetModelID();
void VL6180xGetIdentification(VL6180xIdentification *temp);
void VL6180xPrintIdentification(VL6180xIdentification *temp);
/** Get Revision ID.
 * This register is used to verify the revision number of the device,
 * but only before it has been configured to run
 * @return Revision ID
 * @see VL53L0X_RA_IDENTIFICATION_REVISION_ID
 * @see VL53L0X_IDENTIFICATION_REVISION_ID
 */
uint8_t vl53l0xGetRevisionID();

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool vl53l0xInitSensor(bool io_2v8);

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool vl53l0xSetSignalRateLimit(float limit_Mcps);

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool vl53l0xSetMeasurementTimingBudget(uint32_t budget_us);

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t vl53l0xGetMeasurementTimingBudget(void);

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool vl53l0xSetVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t vl53l0xGetVcselPulsePeriod(vcselPeriodType type);

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void vl53l0xStartContinuous(uint32_t period_ms);

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void vl53l0xStopContinuous(void);

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t vl53l0xReadRangeContinuousMillimeters(void);

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t vl53l0xReadRangeSingleMillimeters(void);

//void VL6180x_setRegister(uint16_t registerAddr, uint16_t data);
void VL6180x_setRegister(uint16_t registerAddr, uint8_t data);
uint8_t VL6180x_getRegister(uint16_t registerAddr);
float VL6180xGetAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN);
uint8_t VL6180xGetDistance();

#endif /* _VL53L0X_H_ */
