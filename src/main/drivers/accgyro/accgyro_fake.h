/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/accgyro/accgyro.h"

#define FAKE_ACC_1G         9806        // counts per g (acc_1G)
#define FAKE_GYRO_LSB_DPS   0.0625f     // deg/s per count (gyro scale)

bool fakeAccDetect(accDev_t *acc);
void fakeAccSet(int16_t x, int16_t y, int16_t z);
// machina's native backend: specific force in g, neither clipped nor quantised.
void fakeAccSetG(float x, float y, float z);

bool fakeGyroDetect(gyroDev_t *gyro);
void fakeGyroSet(int16_t x, int16_t y, int16_t z);
// machina's native backend: rates in deg/s, neither clipped nor quantised.
void fakeGyroSetDps(float x, float y, float z);
