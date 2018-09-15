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

uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static StateEstimatorType estimatorType;
static ControllerType controllerType;

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
//static float z = 0.0f;

float clip (float x, float limit)
{
    if (x >= limit)
        return limit;
    else if (x <= -limit)
        return -limit;
    else
        return x;
}
// My stuff ends

static void stabilizerTask(void* param);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  if (estimator == kalmanEstimator)
  {
    sitAwInit();
  }
  estimatorType = getStateEstimator();
  controllerType = getControllerType();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
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
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    // allow to update estimator dynamically
    if (getStateEstimator() != estimatorType) {
      stateEstimatorInit(estimatorType);
      estimatorType = getStateEstimator();
    }
    // allow to update controller dynamically
    if (getControllerType() != controllerType) {
      controllerInit(controllerType);
      controllerType = getControllerType();
    }

    getExtPosition(&state);
    stateEstimator(&state, &sensorData, &control, tick);

    commanderGetSetpoint(&setpoint, &state);

//    if (OAEnabled) {
//        	//setpoint.velocity.x = vx;
//        	//setpoint.position.z = z;
//    		if (rangeRight < threshold) {
//    			erry = ((float)threshold - rangeRight)/(float)threshold;
//    			dedt = (erry - erry_last)/dt;
//    			setpoint.velocity.y = kp * erry + kd * dedt;
//    			setpoint.velocity.y = clip(setpoint.velocity.y, speed_limit);
//    			erry_last = erry;
//    			latObstPrsnt = true;
//    			}
//    		else if (rangeLeft < threshold) {
//    			erry = -((float)threshold - rangeLeft)/(float)threshold;
//    			dedt = (erry - erry_last)/dt;
//    			setpoint.velocity.y = kp * erry + kd * dedt;
//    			setpoint.velocity.y = clip(setpoint.velocity.y, speed_limit);
//    			erry_last = erry;
//    			latObstPrsnt = true;
//    		}
//    		else if (latObstPrsnt) {
//    			setpoint.velocity.y = 0.0f;
//    			erry_last = 0.0;
//    			latObstPrsnt = false;
//    		}
//
//    		if (rangeFront < threshold) {
//    			errx = -((float)threshold - rangeFront)/(float)threshold;
//    			dedt = (errx - errx_last)/dt;
//    			setpoint.velocity.x = kp * errx + kd * dedt;
//    			setpoint.velocity.x = clip(setpoint.velocity.x, speed_limit);
//    			errx_last = errx;
//    			lonObstPrsnt = true;
//    			}
//    		else if (rangeBack < threshold) {
//    			errx = ((float)threshold - rangeBack)/(float)threshold;
//    			dedt = (errx - errx_last)/dt;
//    			setpoint.velocity.x = kp * errx + kd * dedt;
//    			setpoint.velocity.x = clip(setpoint.velocity.x, speed_limit);
//    			errx_last = errx;
//    			lonObstPrsnt = true;
//    		}
//    		else if (lonObstPrsnt) {
//    			setpoint.velocity.x = 0.0f;
//    			errx_last = 0.0;
//    			lonObstPrsnt = false;
//    		}
//    		else if (!lonObstPrsnt) {
//    		        setpoint.velocity.x = vx;
//    		}
//
//        }

    if (OAEnabled) {
            	//setpoint.velocity.x = vx;
            	//setpoint.position.z = z;
    			if ((rangeRight < threshold) && (rangeLeft < threshold)) {
    				erry = (rangeLeft - rangeRight)/2000.0;
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

    controller(&control, &setpoint, &sensorData, &state, tick);

    checkEmergencyStopTimeout();

    if (emergencyStop) {
      powerStop();
    } else {
      powerDistribution(&control);
    }

    calcSensorToOutputLatency(&sensorData);
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

PARAM_GROUP_START(stabilizer)
PARAM_ADD(PARAM_UINT8, estimator, &estimatorType)
PARAM_ADD(PARAM_UINT8, controller, &controllerType)
PARAM_GROUP_STOP(stabilizer)

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

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)
LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)
LOG_ADD(LOG_FLOAT, vxT, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, vyT, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, vzT, &setpoint.velocity.z)
LOG_GROUP_STOP(stateEstimate)

LOG_GROUP_START(latency)
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(latency)

// My stuff starts
//LOG_GROUP_START(stateEstimate)
//LOG_ADD(LOG_FLOAT, x, &state.position.x)
//LOG_ADD(LOG_FLOAT, y, &state.position.y)
//LOG_ADD(LOG_FLOAT, z, &state.position.z)
//LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
//LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
//LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)
//LOG_ADD(LOG_FLOAT, xT, &setpoint.position.x)
//LOG_ADD(LOG_FLOAT, yT, &setpoint.position.y)
//LOG_ADD(LOG_FLOAT, zT, &setpoint.position.z)
//LOG_ADD(LOG_FLOAT, vxT, &setpoint.velocity.x)
//LOG_ADD(LOG_FLOAT, vyT, &setpoint.velocity.y)
//LOG_ADD(LOG_FLOAT, vzT, &setpoint.velocity.z)
////LOG_ADD(LOG_UINT16, rangeRight, &rangeRight)
////LOG_ADD(LOG_FLOAT, err, &err)
////LOG_ADD(LOG_FLOAT, z, &estimate.z)
//LOG_GROUP_STOP(stateEstimate)

PARAM_GROUP_START(oa)
PARAM_ADD(PARAM_UINT8, OAEnabled, &OAEnabled)
PARAM_ADD(PARAM_FLOAT, speed_limit, &speed_limit)
PARAM_ADD(PARAM_FLOAT, kp, &kp)
PARAM_ADD(PARAM_FLOAT, kd, &kd)
PARAM_ADD(PARAM_FLOAT, vx, &vx)
//PARAM_ADD(PARAM_FLOAT, z, &z)
//PARAM_ADD(PARAM_FLOAT, ki, &ki)
PARAM_ADD(PARAM_UINT16, threshold, &threshold)
PARAM_GROUP_STOP(oa)
// My stuff ends
