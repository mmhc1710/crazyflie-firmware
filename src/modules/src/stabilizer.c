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

#include "math.h"
#include "arm_math.h"
#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)
static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
//
extern bool altHoldMode;
#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]
extern uint16_t range_last2[6];
//static uint8_t Linear = 1, nonLinear = 0;
//static float LinearConst = 2.0, nonLinearConst = 0.5;
//static uint8_t centerSensor = 1;
#include "vl53l0x.h"
static point_t position;
static point_t estimate;
//static point_t estimate_old;
//static velocity_t vel;
static float kp = 0.01f;
static float kd = 0.000f;
static float xC = 0.0f;
static float yC = 0.0f;
static float alpha = 0.93f;
//static uint32_t tickOld = 0;
//static uint8_t makeCenter = 1;
#define N 100

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
//	bool firstRun = true;
//	float prevX[N], sumX = 0.0f;
//	float prevY[N], sumY = 0.0f;
//	uint8_t n = 0;
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
		//vl53l0xReadPosition(&position, tick);
#ifdef ESTIMATOR_TYPE_kalman
		stateEstimatorUpdate(&state, &sensorData, &control);
#else
		sensorsAcquire(&sensorData, tick);
		stateEstimator(&state, &sensorData, tick);
#endif

		commanderGetSetpoint(&setpoint, &state);

		if (vl53l0xReadPosition(&position, tick)) {
//			position.x += xC;
//			position.y += yC;
		}

//		if (firstRun && (position.timestamp==tick)) {
//			prevX[n] = position.x;
//			sumX += position.x;
//			prevY[n] = position.y;
//			sumY += position.y;
//			n++;
//			if (n==N) {
//				estimate.x = (float) sumX/N;
//				estimate.y = (float) sumY/N;
//				estimate_old.x = estimate.x;
//				estimate_old.y = estimate.y;
//				tickOld = tick;
//				firstRun = false;
//				xC = -estimate.x;
//				yC = -estimate.y;
//			}
//		}
//		if (!firstRun && (position.timestamp==tick)) {
//			if ((fabs(position.x-estimate.x)<3000.0) && (fabs(position.y-estimate.y)<3000.0)) {
//				sumX = 0;
//				sumY = 0;
//				for (int i=0;i<N-1;i++) {
//					prevX[i] = prevX[i+1];
//					prevY[i] = prevY[i+1];
//					sumX += prevX[i+1];
//					sumY += prevY[i+1];
//				}
//				prevX[N-1] = position.x;
//				prevY[N-1] = position.y;
//				sumX += position.x;
//				sumY += position.y;
//
//				estimate.x = (float) sumX/N;
//				estimate.y = (float) sumY/N;
//				vel.x = (float) ((estimate.x - estimate_old.x)*1000/(tick-tickOld));
//				vel.y = (float) (estimate.y - estimate_old.y)*1000/(tick-tickOld);
//				estimate_old.x = estimate.x;
//				estimate_old.y = estimate.y;
//				tickOld = tick;
//			}
//		}
//		if (!firstRun) {
//			setpoint.attitude.pitch = kp*(estimate.x) + kd*(vel.x);
//			setpoint.attitude.roll = kp*(-estimate.y) + kd*(-vel.y);;
//		}

		static float phi, theta, psi;
		phi = state.attitude.roll * DEG_TO_RAD;
		theta = state.attitude.pitch * DEG_TO_RAD;
		psi = state.attitude.yaw * DEG_TO_RAD;

//		static float R_ned2b[3][3];
//		static arm_matrix_instance_f32 R_ned2bm = {3, 3, (float *)R_ned2b};
		static float R_b2ned[3][3];
		static arm_matrix_instance_f32 R_b2nedm = {3, 3, (float *)R_b2ned};
		R_b2ned[0][0] = cos(theta)*cos(psi);
		R_b2ned[0][1] = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
		R_b2ned[0][2] = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
		R_b2ned[1][0] = cos(theta)*sin(psi);
		R_b2ned[1][1] = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
		R_b2ned[1][2] = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
		R_b2ned[2][0] = -sin(theta);
		R_b2ned[2][1] = sin(phi)*cos(theta);
		R_b2ned[2][2] = cos(phi)*cos(theta);
//		mat_trans(&R_ned2bm, &R_b2nedm);

		static float P_b[3];
		static arm_matrix_instance_f32 P_bm = {3, 1, (float *)P_b};
		P_b[0] = position.x;
		P_b[1] = position.y;
		P_b[2] = position.z;
		static float P_ned[3];
		static arm_matrix_instance_f32 P_nedm = {3, 1, (float *)P_ned};
		mat_mult(&R_b2nedm, &P_bm, &P_nedm);
		estimate.x = P_ned[0];
		estimate.y = P_ned[1];
		estimate.z = P_ned[2];

		setpoint.attitude.pitch = kp*(estimate.x);// + kd*(vel.x);
		setpoint.attitude.roll = kp*(-estimate.y);// + kd*(-vel.y);;

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
LOG_ADD(LOG_FLOAT, xb, &position.x)
LOG_ADD(LOG_FLOAT, yb, &position.y)
LOG_ADD(LOG_FLOAT, zb, &position.z)
LOG_ADD(LOG_FLOAT, x, &estimate.x)
LOG_ADD(LOG_FLOAT, y, &estimate.y)
LOG_ADD(LOG_FLOAT, z, &estimate.z)
//LOG_ADD(LOG_FLOAT, estimateX, &estimate.x)
//LOG_ADD(LOG_FLOAT, estimateY, &estimate.y)
//LOG_ADD(LOG_FLOAT, velX, &vel.x)
//LOG_ADD(LOG_FLOAT, velY, &vel.y)
LOG_ADD(LOG_UINT32, tick, &position.timestamp)
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
PARAM_ADD(PARAM_FLOAT, kd, &kd)
PARAM_ADD(PARAM_FLOAT, xC, &xC)
PARAM_ADD(PARAM_FLOAT, yC, &yC)
PARAM_ADD(PARAM_FLOAT, alpha, &alpha)
//PARAM_ADD(PARAM_UINT8, makeCenter, &makeCenter)
PARAM_GROUP_STOP(posCtlPid)
