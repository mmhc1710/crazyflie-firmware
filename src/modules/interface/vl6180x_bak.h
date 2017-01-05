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
 * vl6180x.h: Time-of-flight distance sensor driver
 */

#ifndef _VL6180X_H_
#define _VL6180X_H_

#define VL6180X_ENABLED

/** Default constructor, uses external I2C address.
 * @see VL6180X_DEFAULT_ADDRESS
 */
void vl6180xInit(void);

bool vl6180xTest(void);

//static float VL6180xGetAmbientLight(vl6180x_als_gain VL6180X_ALS_GAIN);
uint8_t VL6180xGetDistance(void);

#endif /* _VL6180X_H_ */
