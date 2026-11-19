/*
 * This file is part of INAV Project.
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

/*
 * machina's simulator frames against the firmware's state: the output frame both backends send,
 * and the firmware half of hardware-in-the-loop (MSP2_MACHINA_SIMULATOR).
 *
 * HITL runs on the wall clock: the board's own clock and scheduler rule (closed-loop-plan.md D3,
 * C5's implementation.clock = wall_clock). Each frame replaces the board's sensors with the
 * simulator's, as INAV's MSP_SIMULATOR does, and is answered with the output frame. The frames
 * carry machina's frames and SI units; the estimator always runs (the simulator sends no
 * attitude). Frame k must carry seq k: a gap or a repeat is refused, and the board leaves
 * simulator mode and disarms, so a broken link never flies on held sensors; the simulator starts
 * again from seq 0. A frame without MACHINA_SIM_HITL_ENABLE ends HITL the same way. The board's
 * motor outputs stay disabled in HITL (fc_core.c), so a board on the bench never spins a motor.
 */

#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "config/feature.h"
#include "fc/config.h"
#include "drivers/time.h"
#include "fc/fc_core.h"
#include "fc/machina_sim_outputs.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "io/gps.h"
#include "io/gps_private.h"
#include "io/rangefinder.h"
#include "navigation/navigation.h"
#include "rx/sim.h"
#include "scheduler/scheduler.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

void machinaSimFillOutputs(machinaSimOutputFrame_t *o)
{
    const uint8_t motors = getMotorCount();
    o->motorCount = motors < MACHINA_SIM_MAX_MOTORS ? motors : MACHINA_SIM_MAX_MOTORS;
    o->servoCount = MACHINA_SIM_MAX_SERVOS;
    o->failsafePhase = (uint8_t)failsafePhase();
    for (int i = 0; i < MACHINA_SIM_MAX_MOTORS; i++) {
        o->motorRaw[i] = i < o->motorCount ? (uint16_t)motor[i] : 0;
        o->motor[i] = i < o->motorCount ? ((int)motor[i] - 1000) / 1000.0f : 0.0f;
    }
    for (int i = 0; i < MACHINA_SIM_MAX_SERVOS; i++) {
        o->servoRaw[i] = (uint16_t)servo[i];
    }
    for (int i = 0; i < 3; i++) {
        o->attitudeDecideg[i] = attitude.raw[i];
        o->ratesDps[i] = gyro.gyroADCf[i];
        o->posNeuCm[i] = getEstimatedActualPosition(i);
        o->velNeuCms[i] = getEstimatedActualVelocity(i);
        o->gyroRawDps[i] = gyro.gyroRaw[i];
        o->accG[i] = acc.accADCf[i];
#ifdef USE_MAG
        o->magAdc[i] = mag.magADC[i];
#endif
    }
    o->armingFlags = armingFlags;
    o->flightModeFlags = flightModeFlags;
    o->stateFlags = stateFlags;
    o->navState = (uint32_t)NAV_Status.state;
#ifdef USE_BARO
    o->baroPressurePa = baro.baroPressure;
#endif
#ifdef USE_GPS
    o->gnssLatE7 = gpsSol.llh.lat;
    o->gnssLonE7 = gpsSol.llh.lon;
#endif
}

#ifdef USE_SIMULATOR

static uint64_t hitlApplied = 0;

static void hitlLeave(void)
{
    if (ARMING_FLAG(SIMULATOR_MODE_HITL)) {
        DISABLE_ARMING_FLAG(SIMULATOR_MODE_HITL);
        simulatorData.flags = HITL_RESET_FLAGS;
        simulatorData.machinaFrame = false;
        disarm(DISARM_SWITCH);
    }
    hitlApplied = 0;
}

static void hitlEnter(void)
{
    if (ARMING_FLAG(SIMULATOR_MODE_HITL)) {
        return;
    }
    // As INAV's MSP_SIMULATOR does on its first frame.
#ifdef USE_BARO
    if (requestedSensors[SENSOR_INDEX_BARO] != BARO_NONE) {
        sensorsSet(SENSOR_BARO);
        setTaskEnabled(TASK_BARO, true);
        DISABLE_ARMING_FLAG(ARMING_DISABLED_HARDWARE_FAILURE);
        baroStartCalibration();
    }
#endif
#ifdef USE_MAG
    if (compassConfig()->mag_hardware != MAG_NONE) {
        sensorsSet(SENSOR_MAG);
        ENABLE_STATE(COMPASS_CALIBRATED);
        DISABLE_ARMING_FLAG(ARMING_DISABLED_HARDWARE_FAILURE);
    }
#endif
    ENABLE_ARMING_FLAG(SIMULATOR_MODE_HITL);
    ENABLE_STATE(ACCELEROMETER_CALIBRATED);
}

static void hitlApply(const machinaSimInav_t *s)
{
    // The estimator always runs on the simulated IMU (no attitude is sent).
    simulatorData.flags = HITL_ENABLE | HITL_USE_IMU;
    simulatorData.machinaFrame = true;

    if (s->flags & MACHINA_SIM_HAS_IMU) {
        for (int i = 0; i < 3; i++) {
            gyro.gyroADCf[i] = s->gyroDps[i];
            gyro.gyroRaw[i] = s->gyroDps[i];
            acc.accADCf[i] = s->accG[i];
            acc.accVibeSq[i] = 0.0f;
        }
    }
#ifdef USE_MAG
    if ((s->flags & MACHINA_SIM_HAS_MAG) && sensors(SENSOR_MAG)) {
        // In HITL INAV's compass update is bypassed: body axes go straight in.
        for (int i = 0; i < 3; i++) {
            mag.magADC[i] = s->magCounts[i];
        }
    }
#endif
#ifdef USE_BARO
    if ((s->flags & MACHINA_SIM_HAS_BARO) && sensors(SENSOR_BARO)) {
        baro.baroPressure = s->pressurePa;
        baro.baroTemperature = s->temperatureCdeg;
    }
#endif
#ifdef USE_GPS
    if ((s->flags & MACHINA_SIM_HAS_GNSS) && feature(FEATURE_GPS)) {
        simulatorData.flags |= HITL_HAS_NEW_GPS_DATA;
        gpsSolDRV.fixType = (gpsFixType_e)s->fixType;
        gpsSolDRV.numSat = s->numSat;
        gpsSolDRV.hdop = s->fixType == GPS_NO_FIX ? 9999 : s->hdop;
        gpsSolDRV.eph = s->ephCm;
        gpsSolDRV.epv = s->epvCm;
        gpsSolDRV.llh.lat = s->latE7;
        gpsSolDRV.llh.lon = s->lonE7;
        gpsSolDRV.llh.alt = s->altCm;
        gpsSolDRV.groundSpeed = s->groundSpeedCms;
        gpsSolDRV.groundCourse = s->groundCourseDecideg;
        for (int i = 0; i < 3; i++) {
            gpsSolDRV.velNED[i] = s->velNedCms[i];
        }
        const bool fix = s->fixType != GPS_NO_FIX;
        gpsSolDRV.flags.validVelNE = fix;
        gpsSolDRV.flags.validVelD = fix;
        gpsSolDRV.flags.validEPE = fix;
        gpsSolDRV.flags.validTime = false;
        gpsProcessNewDriverData();
        gpsProcessNewSolutionData(false);
    }
#endif
    if (s->flags & MACHINA_SIM_HAS_RC) {
        uint16_t pwm[MACHINA_SIM_RC_CHANNELS];
        memcpy(pwm, s->rcPwm, sizeof(pwm));
        simulatorData.flags |= HITL_SIM_RC_INPUT;
        rxSimSetChannelValue(pwm, MACHINA_SIM_RC_CHANNELS);
        rxSimSetRssi(s->rssi);
    }
    if (s->flags & (MACHINA_SIM_HAS_RC | MACHINA_SIM_RC_FAILSAFE)) {
        rxSimSetFailsafe((s->flags & MACHINA_SIM_RC_FAILSAFE) != 0);
    }
    if (s->flags & MACHINA_SIM_HAS_BATTERY) {
        simulatorData.flags |= HITL_SIMULATE_BATTERY | HITL_CURRENT_SENSOR;
        simulatorData.vbatCv = s->vbatCv;
        simulatorData.currentCa = s->currentCa;
    }
    if (s->flags & MACHINA_SIM_HAS_RANGEFINDER) {
        simulatorData.flags |= HITL_RANGEFINDER;
        fakeRangefindersSetData(s->rangeCm);
    }
    if (s->flags & MACHINA_SIM_HAS_AIRSPEED) {
        simulatorData.flags |= HITL_AIRSPEED;
        simulatorData.airSpeed = s->airspeedCms <= 0.0f ? 0 : (s->airspeedCms >= 65535.0f ? 65535 : (uint16_t)s->airspeedCms);
    }
}

void machinaSimHitlProcess(const uint8_t *payload, int len, uint8_t *reply)
{
    machinaSimOutputFrame_t o;
    machinaSimSi_t si;
    machinaSimInav_t s;
    memset(&o, 0, sizeof(o));

    machinaSimStatus_e status = len < 0 ? MACHINA_SIM_ERR_SIZE : machinaSimDecodeHitlFrame(payload, (size_t)len, &si);
    if (status == MACHINA_SIM_OK && !(si.flags & MACHINA_SIM_HITL_ENABLE)) {
        hitlLeave();
    } else {
        if (status == MACHINA_SIM_OK) {
            status = machinaSimCheckStep(&si, hitlApplied, 0, 0, false);
        }
        if (status == MACHINA_SIM_OK) {
            status = machinaSimToInav(&si, &s);
        }
        if (status == MACHINA_SIM_OK) {
            hitlEnter();
            hitlApply(&s);
            hitlApplied++;
        } else {
            o.seq = hitlApplied;
            hitlLeave();
        }
    }

    machinaSimFillOutputs(&o);
    o.flags = MACHINA_SIM_OUT_HITL;
    if (status != MACHINA_SIM_OK) {
        o.flags |= MACHINA_SIM_OUT_ERROR;
        o.error = (uint8_t)status;
    } else {
        o.seq = si.seq;
        o.tNs = si.tNs;
    }
    o.clockNs = (uint64_t)micros() * 1000;
    o.applied = hitlApplied;
    machinaSimEncodeOutputs(&o, reply, sizeof(o));
}

#endif
