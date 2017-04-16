/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#else
#include "estimator.h"
#endif

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
//
//extern uint16_t range_last2[6];
//static uint8_t Linear = 1, nonLinear = 0;
//static float LinearConst = 2.0, nonLinearConst = 0.5;
//static uint8_t centerSensor = 1;
#include "vl53l0x.h"
point_t position;
float kp = 2.0f;

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
	if(isInit)
		return;

	sensorsInit();
	stateEstimatorInit();
	stateControllerInit();
	powerDistributionInit();
#if defined(SITAW_ENABLED)
	sitAwInit();
#endif

	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
			STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= sensorsTest();
	pass &= stateEstimatorTest();
	pass &= stateControllerTest();
	pass &= powerDistributionTest();

	return pass;
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime;
	vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();

	// Wait for sensors to be calibrated
	lastWakeTime = xTaskGetTickCount ();
	while(!sensorsAreCalibrated()) {
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
	}

	while(1) {
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

		getExtPosition(&state);
		vl53l0xReadPosition(&position, tick);
#ifdef ESTIMATOR_TYPE_kalman
		stateEstimatorUpdate(&state, &sensorData, &control);
#else
		sensorsAcquire(&sensorData, tick);
		stateEstimator(&state, &sensorData, tick);
#endif

		commanderGetSetpoint(&setpoint, &state);
		setpoint.attitude.pitch = kp*position.x;
		setpoint.attitude.roll = kp*position.y;
		//		if (nonLinear && !centerSensor) {
		//			if (range_last2[1]<1000.0 && range_last2[1]>0.0) {
		//				setpoint.attitude.roll -= (float)  (nonLinearConst*1000/range_last2[1]);
		//				setpoint.attitude.pitch += (float) (nonLinearConst*1000/range_last2[1]);
		//			}
		//			//
		//			if (range_last2[2]<1000.0 && range_last2[2]>0.0) {
		//				setpoint.attitude.roll -= (float)  (nonLinearConst*1000/range_last2[2]);
		//				setpoint.attitude.pitch -= (float) (nonLinearConst*1000/range_last2[2]);
		//			}
		//
		//			if (range_last2[3]<1000.0 && range_last2[3]>0.0) {
		//				setpoint.attitude.roll += (float)  (nonLinearConst*1000/range_last2[3]);
		//				setpoint.attitude.pitch -= (float) (nonLinearConst*1000/range_last2[3]);
		//			}
		//
		//			if (range_last2[4]<1000.0 && range_last2[4]>0.0) {
		//				setpoint.attitude.roll += (float)  (nonLinearConst*1000/range_last2[4]);
		//				setpoint.attitude.pitch += (float) (nonLinearConst*1000/range_last2[4]);
		//			}
		//		}
		//
		//		if (Linear && !centerSensor) {
		//			if (range_last2[1]<1000.0 && range_last2[1]>0.0) {
		//				setpoint.attitude.roll -= LinearConst*(1 - range_last2[1]/1000);
		//				setpoint.attitude.pitch += LinearConst*(1 - range_last2[1]/1000);
		//			}
		//
		//			if (range_last2[2]<1000.0 && range_last2[2]>0.0) {
		//				setpoint.attitude.roll -= LinearConst*(1 - range_last2[2]/1000);
		//				setpoint.attitude.pitch -= LinearConst*(1 - range_last2[2]/1000);
		//			}
		//
		//			if (range_last2[3]<1000.0 && range_last2[3]>0.0) {
		//				setpoint.attitude.roll += LinearConst*(1 - range_last2[3]/1000);
		//				setpoint.attitude.pitch -= LinearConst*(1 - range_last2[3]/1000);
		//			}
		//
		//			if (range_last2[4]<1000.0 && range_last2[4]>0.0) {
		//				setpoint.attitude.roll += LinearConst*(1 - range_last2[4]/1000);
		//				setpoint.attitude.pitch += LinearConst*(1 - range_last2[4]/1000);
		//			}
		//		}
		//
		//		if (nonLinear && centerSensor) {
		//			if (range_last2[1]<1000.0 && range_last2[1]>0.0) {
		////				setpoint.attitude.roll -= (float)  (nonLinearConst*1000/range_last2[1]);
		//				setpoint.attitude.pitch += (float) (nonLinearConst*1000/range_last2[1]);
		//			}
		//			//
		//			if (range_last2[2]<1000.0 && range_last2[2]>0.0) {
		//				setpoint.attitude.roll -= (float)  (nonLinearConst*1000/range_last2[2]);
		////				setpoint.attitude.pitch -= (float) (nonLinearConst*1000/range_last2[2]);
		//			}
		//
		//			if (range_last2[3]<1000.0 && range_last2[3]>0.0) {
		////				setpoint.attitude.roll += (float)  (nonLinearConst*1000/range_last2[3]);
		//				setpoint.attitude.pitch -= (float) (nonLinearConst*1000/range_last2[3]);
		//			}
		//
		//			if (range_last2[4]<1000.0 && range_last2[4]>0.0) {
		//				setpoint.attitude.roll += (float)  (nonLinearConst*1000/range_last2[4]);
		////				setpoint.attitude.pitch += (float) (nonLinearConst*1000/range_last2[4]);
		//			}
		//		}
		//
		//		if (Linear && centerSensor) {
		//			if (range_last2[1]<1000.0 && range_last2[1]>0.0) {
		////				setpoint.attitude.roll -= LinearConst*(1 - range_last2[1]/1000);
		//				setpoint.attitude.pitch += LinearConst*(1 - range_last2[1]/1000);
		//			}
		//
		//			if (range_last2[2]<1000.0 && range_last2[2]>0.0) {
		//				setpoint.attitude.roll -= LinearConst*(1 - range_last2[2]/1000);
		////				setpoint.attitude.pitch -= LinearConst*(1 - range_last2[2]/1000);
		//			}
		//
		//			if (range_last2[3]<1000.0 && range_last2[3]>0.0) {
		////				setpoint.attitude.roll += LinearConst*(1 - range_last2[3]/1000);
		//				setpoint.attitude.pitch -= LinearConst*(1 - range_last2[3]/1000);
		//			}
		//
		//			if (range_last2[4]<1000.0 && range_last2[4]>0.0) {
		//				setpoint.attitude.roll += LinearConst*(1 - range_last2[4]/1000);
		////				setpoint.attitude.pitch += LinearConst*(1 - range_last2[4]/1000);
		//			}
		//		}


//		if (range_last2[1]<1000.0 && range_last2[1]>0.0) {
//			//				setpoint.attitude.roll -= LinearConst*(1 - range_last2[1]/1000);
//			setpoint.attitude.pitch += LinearConst*(1 - range_last2[1]/1000);
//		}
//
//		if (range_last2[2]<1000.0 && range_last2[2]>0.0) {
//			setpoint.attitude.roll -= LinearConst*(1 - range_last2[2]/1000);
//			//				setpoint.attitude.pitch -= LinearConst*(1 - range_last2[2]/1000);
//		}
//
//		if (range_last2[3]<1000.0 && range_last2[3]>0.0) {
//			//				setpoint.attitude.roll += LinearConst*(1 - range_last2[3]/1000);
//			setpoint.attitude.pitch -= LinearConst*(1 - range_last2[3]/1000);
//		}
//
//		if (range_last2[4]<1000.0 && range_last2[4]>0.0) {
//			setpoint.attitude.roll += LinearConst*(1 - range_last2[4]/1000);
//			//				setpoint.attitude.pitch += LinearConst*(1 - range_last2[4]/1000);
//		}


		sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

		stateController(&control, &setpoint, &sensorData, &state, tick);
		powerDistribution(&control);

		tick++;
	}
}

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(position)
LOG_ADD(LOG_FLOAT, x, &position.x)
LOG_ADD(LOG_FLOAT, y, &position.y)
LOG_ADD(LOG_FLOAT, z, &position.z)
LOG_GROUP_STOP(position)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

//LOG_GROUP_START(range)
////LOG_ADD(LOG_UINT16, range, &range_last)
//LOG_ADD(LOG_UINT16, range1, &range_last2[0])
//LOG_ADD(LOG_UINT16, range2, &range_last2[1])
//LOG_ADD(LOG_UINT16, range3, &range_last2[2])
//LOG_ADD(LOG_UINT16, range4, &range_last2[3])
//LOG_ADD(LOG_UINT16, range5, &range_last2[4])
//LOG_ADD(LOG_UINT16, range6, &range_last2[5])
////LOG_ADD(LOG_UINT8, rangeStatus, &DeviceRangeStatusInternal)
//LOG_GROUP_STOP(range)

PARAM_GROUP_START(posCtlPid)
//PARAM_ADD(PARAM_UINT8, centerSensor, &centerSensor)
//PARAM_ADD(PARAM_UINT8, Linear, &Linear)
//PARAM_ADD(PARAM_FLOAT, LinearConst, &LinearConst)
//PARAM_ADD(PARAM_UINT8, nonLinear, &nonLinear)
//PARAM_ADD(PARAM_FLOAT, nonLinearConst, &nonLinearConst)
PARAM_ADD(PARAM_FLOAT, kp, &kp)
PARAM_GROUP_STOP(posCtlPid)
