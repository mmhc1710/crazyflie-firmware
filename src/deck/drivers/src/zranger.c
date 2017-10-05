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
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "VLX"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "i2cdev.h"
#include "zranger.h"
#include "vl53l0x.h"

#include "stabilizer_types.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "arm_math.h"

// Measurement noise model
static float expPointA = 1.0f;
static float expStdA = 0.0025f; // STD at elevation expPointA [m]
static float expPointB = 1.3f;
static float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]

uint16_t range_last = 0;
uint16_t range_last2[6] = {0,0,0,0,0,0};

bool isInit;

static VL53L0xDev dev;

void zRangerInit(DeckInfo* info)
{
  if (isInit)
    return;

	pinMode(DECK_GPIO_IO1, OUTPUT);
	pinMode(DECK_GPIO_IO2, OUTPUT);
	pinMode(DECK_GPIO_IO3, OUTPUT);
	pinMode(DECK_GPIO_IO4, OUTPUT);

	digitalWrite(DECK_GPIO_IO4, LOW);
	for (int i=0; i<=4; i++){
		digitalWrite(DECK_GPIO_IO1, ((i&0b00000001)>>0));
		digitalWrite(DECK_GPIO_IO2, ((i&0b00000010)>>1));
		digitalWrite(DECK_GPIO_IO3, ((i&0b00000100)>>2));
		vl53l0xInit(&dev, I2C1_DEV, true);
		if (1) DEBUG_PRINT("[%d] Initialization test [OK]\n", i);
	}

	digitalWrite(DECK_GPIO_IO4, HIGH);
	digitalWrite(DECK_GPIO_IO1, LOW);
	digitalWrite(DECK_GPIO_IO2, LOW);
	digitalWrite(DECK_GPIO_IO3, LOW);
	vl53l0xInit(&dev, I2C1_DEV, true);
	if (1) DEBUG_PRINT("[%d] Initialization test [OK]\n", 5);

  xTaskCreate(zRangerTask, ZRANGER_TASK_NAME, ZRANGER_TASK_STACKSIZE, NULL, ZRANGER_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRangerTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus  = vl53l0xTestConnection(&dev);

  return testStatus;
}

void zRangerTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodPreRange, 18);
  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodFinalRange, 14);
  vl53l0xStartContinuous(&dev, 0);
  while (1) {
    xLastWakeTime = xTaskGetTickCount();

	digitalWrite(DECK_GPIO_IO4, LOW);
	for (int i=0; i<=4; i++){
		digitalWrite(DECK_GPIO_IO1, ((i&0b00000001)>>0));
		digitalWrite(DECK_GPIO_IO2, ((i&0b00000010)>>1));
		digitalWrite(DECK_GPIO_IO3, ((i&0b00000100)>>2));
		range_last2[i] = vl53l0xReadRangeSingleMillimeters(&dev);
		//  	  range_last = vl53l0xReadRangeContinuousMillimeters();
	}
	digitalWrite(DECK_GPIO_IO4, HIGH);
	digitalWrite(DECK_GPIO_IO1, LOW);
	digitalWrite(DECK_GPIO_IO2, LOW);
	digitalWrite(DECK_GPIO_IO3, LOW);
	range_last2[5] = vl53l0xReadRangeSingleMillimeters(&dev);
	range_last = range_last2[5];

//    range_last = vl53l0xReadRangeContinuousMillimeters(&dev);

    // check if range is feasible and push into the kalman filter
    // the sensor should not be able to measure >3 [m], and outliers typically
    // occur as >8 [m] measurements
    if (getStateEstimator() == kalmanEstimator &&
        range_last < RANGE_OUTLIER_LIMIT) {
      // Form measurement
      tofMeasurement_t tofData;
      tofData.timestamp = xTaskGetTickCount();
      tofData.distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      tofData.stdDev = expStdA * (1.0f  + expf( expCoeff * ( tofData.distance - expPointA)));
      estimatorKalmanEnqueueTOF(&tofData);
    }

    vTaskDelayUntil(&xLastWakeTime, M2T(dev.measurement_timing_budget_ms));
  }
}

bool zRangerReadRange(zDistance_t* zrange, const uint32_t tick)
{
  bool updated = false;

  if (isInit) {
    if (range_last != 0 && range_last < RANGE_OUTLIER_LIMIT) {
      zrange->distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      zrange->timestamp = tick;
      updated = true;
    }
  }
  return updated;
}

static const DeckDriver zranger_deck = {
  .vid = 0xBC,
  .pid = 0x09,
  .name = "bcZRanger",
  .usedGpio = DECK_GPIO_IO1 | DECK_GPIO_IO2 | DECK_GPIO_IO3 | DECK_GPIO_IO4,

  .init = zRangerInit,
  .test = zRangerTest,
};

DECK_DRIVER(zranger_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(range)
LOG_ADD(LOG_UINT16, range1, &range_last2[0])
LOG_ADD(LOG_UINT16, range2, &range_last2[1])
LOG_ADD(LOG_UINT16, range3, &range_last2[2])
LOG_ADD(LOG_UINT16, range4, &range_last2[3])
LOG_ADD(LOG_UINT16, range5, &range_last2[4])
LOG_ADD(LOG_UINT16, range6, &range_last2[5])
LOG_ADD(LOG_UINT16, zrange, &range_last)
LOG_GROUP_STOP(range)
