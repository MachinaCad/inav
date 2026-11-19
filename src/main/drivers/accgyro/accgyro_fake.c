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

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#include "platform.h"

#include "common/axis.h"
#include "common/utils.h"

#include "fc/runtime_config.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_fake.h"


#ifdef USE_IMU_FAKE

static float fakeGyroADC[XYZ_AXIS_COUNT];

static void fakeGyroInit(gyroDev_t *gyro)
{
    UNUSED(gyro);
}

void fakeGyroSet(int16_t x, int16_t y, int16_t z)
{
    fakeGyroADC[X] = x;
    fakeGyroADC[Y] = y;
    fakeGyroADC[Z] = z;
}

void fakeGyroSetDps(float x, float y, float z)
{
    // gyroADCRaw is float: a power-of-two scale keeps every float32 value exactly.
    fakeGyroADC[X] = x / FAKE_GYRO_LSB_DPS;
    fakeGyroADC[Y] = y / FAKE_GYRO_LSB_DPS;
    fakeGyroADC[Z] = z / FAKE_GYRO_LSB_DPS;
}

static bool fakeGyroRead(gyroDev_t *gyro)
{
    gyro->gyroADCRaw[X] = fakeGyroADC[X];
    gyro->gyroADCRaw[Y] = fakeGyroADC[Y];
    gyro->gyroADCRaw[Z] = fakeGyroADC[Z];
    return true;
}

static bool fakeGyroReadTemperature(gyroDev_t *gyro, int16_t *temperatureData)
{
    UNUSED(gyro);
    UNUSED(temperatureData);
    return true;
}

static bool fakeGyroInitStatus(gyroDev_t *gyro)
{
    UNUSED(gyro);
    return true;
}

bool fakeGyroDetect(gyroDev_t *gyro)
{
    gyro->initFn = fakeGyroInit;
    gyro->intStatusFn = fakeGyroInitStatus;
    gyro->readFn = fakeGyroRead;
    gyro->temperatureFn = fakeGyroReadTemperature;
    gyro->scale = FAKE_GYRO_LSB_DPS;
    gyro->gyroAlign = 0;
    return true;
}

static float fakeAccData[XYZ_AXIS_COUNT];

static void fakeAccInit(accDev_t *acc)
{
    acc->acc_1G = FAKE_ACC_1G;
}

void fakeAccSet(int16_t x, int16_t y, int16_t z)
{ 
    fakeAccData[X] = x;
    fakeAccData[Y] = y;
    fakeAccData[Z] = z;
}

void fakeAccSetG(float x, float y, float z)
{
    // ADCRaw is float, so the specific force reaches INAV unclipped (int16 counts stop at 3.3 g).
    fakeAccData[X] = x * FAKE_ACC_1G;
    fakeAccData[Y] = y * FAKE_ACC_1G;
    fakeAccData[Z] = z * FAKE_ACC_1G;
}

static bool fakeAccRead(accDev_t *acc)
{
    acc->ADCRaw[X] = fakeAccData[X];
    acc->ADCRaw[Y] = fakeAccData[Y];
    acc->ADCRaw[Z] = fakeAccData[Z];
    return true;
}

bool fakeAccDetect(accDev_t *acc)
{
    acc->initFn = fakeAccInit;
    acc->readFn = fakeAccRead;
    acc->accAlign = 0;
    return true;
}
#endif 

