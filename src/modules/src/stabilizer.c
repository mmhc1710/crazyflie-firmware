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

#include "estimator_kalman.h"
#include "estimator.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static void stabilizerTask(void* param);

// My stuff starts
extern uint16_t rangeFront;
extern uint16_t rangeBack;
extern uint16_t rangeRight;
extern uint16_t rangeLeft;
//extern uint16_t rangeUp;
//extern uint16_t range_last;
static float kp = 3.0f;
//static double ki = 0.0f;
static float kd = 0.1f/1000.0f;
static uint16_t threshold = 300;
float errx = 0.0f;
float errx_last = 0.0f;
float erry = 0.0f;
float erry_last = 0.0f;
static float dt = 0.001f;
static float dedt = 0.0f;
//static bool inFlight = false;
bool lonObstPrsnt = false;
bool latObstPrsnt = false;
static float speed_limit = 5.0;
static bool OAEnabled = false;
static float vx = 0.2f; //0.2
static float z = 0.0f;


//#define POS_UPDATE_RATE RATE_1000_HZ
//#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE
//#include "math.h"
//#include "arm_math.h"
//#define DEG_TO_RAD (PI/180.0f)
//#define RAD_TO_DEG (180.0f/PI)
//static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
//{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
//static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
//{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
//static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
//{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
//static point_t position;
//static point_t estimate;
#define isPos(x) (x>0)
#define isNeg(x) (x<0)
// My stuff ends


float clip (float x, float limit)
{
    if (x >= limit)
        return limit;
    else if (x <= -limit)
        return -limit;
    else
        return x;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  stateControllerInit();
  powerDistributionInit();
  if (estimator == kalmanEstimator)
  {
    sitAwInit();
  }

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

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    getExtPosition(&state);
    stateEstimator(&state, &sensorData, &control, tick);

    commanderGetSetpoint(&setpoint, &state);
    // My stuff starts
//    if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
//
//    		static float phi, theta, psi;
//    		phi = state.attitude.roll * DEG_TO_RAD;
//    		theta = state.attitude.pitch * DEG_TO_RAD;
//    		psi = state.attitude.yaw * DEG_TO_RAD;
//
//    //		static float R_ned2b[3][3];
//    //		static arm_matrix_instance_f32 R_ned2bm = {3, 3, (float *)R_ned2b};
//    		static float R_b2ned[3][3];
//    		static arm_matrix_instance_f32 R_b2nedm = {3, 3, (float *)R_b2ned};
//    		R_b2ned[0][0] = cos(theta)*cos(psi);
//    		R_b2ned[0][1] = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
//    		R_b2ned[0][2] = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
//    		R_b2ned[1][0] = cos(theta)*sin(psi);
//    		R_b2ned[1][1] = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
//    		R_b2ned[1][2] = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
//    		R_b2ned[2][0] = -sin(theta);
//    		R_b2ned[2][1] = sin(phi)*cos(theta);
//    		R_b2ned[2][2] = cos(phi)*cos(theta);
//    //		mat_trans(&R_ned2bm, &R_b2nedm);
//
//    		static float P_b[3];
//    		static arm_matrix_instance_f32 P_bm = {3, 1, (float *)P_b};
//    		position.x = (float)rangeBack/2.0f;
//			position.x -= (float)rangeFront/2.0f;
//			position.y = (float)rangeLeft/2.0f;
//			position.y -= (float)rangeRight/2.0f;
//			position.z = range_last;
//    		P_b[0] = position.x;
//    		P_b[1] = position.y;
//    		P_b[2] = position.z;
//    		static float P_ned[3];
//    		static arm_matrix_instance_f32 P_nedm = {3, 1, (float *)P_ned};
//    		mat_mult(&R_b2nedm, &P_bm, &P_nedm);
//    		estimate.x = P_ned[0];
//    		estimate.y = P_ned[1];
//    		estimate.z = P_ned[2];
//
//    		static float acc_b[3];
//    		static arm_matrix_instance_f32 acc_bm = {3, 1, (float *)acc_b};
//    		acc_b[0] = sensorData.acc.x;
//    		acc_b[1] = sensorData.acc.y;
//    		acc_b[2] = sensorData.acc.z;
//    		static float acc_ned[3];
//    		static arm_matrix_instance_f32 acc_nedm = {3, 1, (float *)acc_ned};
//    		mat_mult(&R_b2nedm, &acc_bm, &acc_nedm);
//
//    		}

//    //		setpoint.attitude.pitch = kp*(estimate.x);// + kd*(vel.x);
//    //		setpoint.attitude.roll = kp*(-estimate.y);// + kd*(-vel.y);;
//
//    		if (rangeFront < threshold) setpoint.attitude.pitch = kp*(threshold-rangeFront);
//    		if (rangeBack < threshold) setpoint.attitude.pitch = -kp*(threshold-rangeBack);
//    //		else setpoint.attitude.pitch = 0.0;
//    		if (rangeRight < threshold) setpoint.attitude.roll = -kp*(threshold-rangeRight);
//    		if (rangeLeft < threshold) setpoint.attitude.roll = kp*(threshold-rangeLeft);
//    //		else setpoint.attitude.roll = 0.0;
//
//	if ((rangeFront < threshold) && isNeg(setpoint.attitude.pitch))
//		setpoint.attitude.pitch = 0.0;
//	if ((rangeBack < threshold) && isPos(setpoint.attitude.pitch))
//			setpoint.attitude.pitch = 0.0;
//	if ((rangeRight < threshold) && isPos(setpoint.attitude.roll))
//			setpoint.attitude.roll = 0.0;
//	if ((rangeLeft < threshold) && isNeg(setpoint.attitude.roll))
//			setpoint.attitude.roll = 0.0;
//
//	if ((rangeFront < threshold) && isPos(setpoint.velocity.x))
//		setpoint.velocity.x = 0.0;
//	if ((rangeBack < threshold) && isNeg(setpoint.velocity.x))
//			setpoint.velocity.x = 0.0;
//	if ((rangeRight < threshold) && isNeg(setpoint.velocity.y))
//			setpoint.velocity.y = 0.0;
//	if ((rangeLeft < threshold) && isPos(setpoint.velocity.y))
//			setpoint.velocity.y = 0.0;
	// My stuff ends powerDistribution
//    setpoint.mode.pitch = modeVelocity;
//    setpoint.mode.roll = modeVelocity;
//    setpoint.attitude.pitch = 0.0;
//    setpoint.attitude.roll = 0.0;
//	setpoint.attitudeRate.pitch = 0.0;
//	setpoint.attitudeRate.roll = 0.0;
//    setpoint.velocity.x = 0.0;
//    setpoint.velocity.y = 0.0;

//	if ((rangeFront < threshold) && isPos(setpoint.velocity.x))
//		setpoint.velocity.x = 0.0;
//	if ((rangeBack < threshold) && isNeg(setpoint.velocity.x))
//			setpoint.velocity.x = 0.0;
//	if ((rangeRight < threshold) && isNeg(setpoint.velocity.y))
//			setpoint.velocity.y = 0.0;
//	if ((rangeLeft < threshold) && isPos(setpoint.velocity.y))
//			setpoint.velocity.y = 0.0;

    if (OAEnabled) {
    	//setpoint.velocity.x = vx;
    	//setpoint.position.z = z;
		if (rangeRight < threshold) {
			erry = ((float)threshold - rangeRight)/(float)threshold;
			dedt = (erry - erry_last)/dt;
			setpoint.velocity.y = kp * erry + kd * dedt;
			setpoint.velocity.y = clip(setpoint.velocity.y, speed_limit);
			erry_last = erry;
			latObstPrsnt = true;
			}
		else if (rangeLeft < threshold) {
			erry = -((float)threshold - rangeLeft)/(float)threshold;
			dedt = (erry - erry_last)/dt;
			setpoint.velocity.y = kp * erry + kd * dedt;
			setpoint.velocity.y = clip(setpoint.velocity.y, speed_limit);
			erry_last = erry;
			latObstPrsnt = true;
		}
		else if (latObstPrsnt) {
			setpoint.velocity.y = 0.0f;
			erry_last = 0.0;
			latObstPrsnt = false;
		}

		if (rangeFront < threshold) {
			errx = -((float)threshold - rangeFront)/(float)threshold;
			dedt = (errx - errx_last)/dt;
			setpoint.velocity.x = kp * errx + kd * dedt;
			setpoint.velocity.x = clip(setpoint.velocity.x, speed_limit);
			errx_last = errx;
			lonObstPrsnt = true;
			}
		else if (rangeBack < threshold) {
			errx = ((float)threshold - rangeBack)/(float)threshold;
			dedt = (errx - errx_last)/dt;
			setpoint.velocity.x = kp * errx + kd * dedt;
			setpoint.velocity.x = clip(setpoint.velocity.x, speed_limit);
			errx_last = errx;
			lonObstPrsnt = true;
		}
		else if (lonObstPrsnt) {
			setpoint.velocity.x = 0.0f;
			errx_last = 0.0;
			lonObstPrsnt = false;
		}
		else if (!lonObstPrsnt) {
		        setpoint.velocity.x = vx;
		}

    }

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    stateController(&control, &setpoint, &sensorData, &state, tick);

    checkEmergencyStopTimeout();
//
//    control.pitch = 0.0;
//    control.roll = 0.0;
    if (emergencyStop) {
      powerStop();
    } else {
      powerDistribution(&control);
    }

    tick++;
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
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
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

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

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

//LOG_GROUP_START(stateEstimate)
//LOG_ADD(LOG_FLOAT, x, &state.position.x)
//LOG_ADD(LOG_FLOAT, y, &state.position.y)
//LOG_ADD(LOG_FLOAT, z, &state.position.z)
//LOG_GROUP_STOP(stateEstimate)

// My stuff starts
LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)
LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)
LOG_ADD(LOG_FLOAT, xT, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, yT, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, zT, &setpoint.position.z)
LOG_ADD(LOG_FLOAT, vxT, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, vyT, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, vzT, &setpoint.velocity.z)
//LOG_ADD(LOG_UINT16, rangeRight, &rangeRight)
//LOG_ADD(LOG_FLOAT, err, &err)
//LOG_ADD(LOG_FLOAT, z, &estimate.z)
LOG_GROUP_STOP(stateEstimate)

PARAM_GROUP_START(oa)
PARAM_ADD(PARAM_UINT8, OAEnabled, &OAEnabled)
PARAM_ADD(PARAM_FLOAT, speed_limit, &speed_limit)
PARAM_ADD(PARAM_FLOAT, kp, &kp)
PARAM_ADD(PARAM_FLOAT, kd, &kd)
PARAM_ADD(PARAM_FLOAT, vx, &vx)
PARAM_ADD(PARAM_FLOAT, z, &z)
//PARAM_ADD(PARAM_FLOAT, ki, &ki)
PARAM_ADD(PARAM_UINT16, threshold, &threshold)
PARAM_GROUP_STOP(oa)
// My stuff ends
