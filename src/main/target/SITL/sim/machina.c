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
 * machina's native binary simulator backend (--sim=machina), in place of the X-Plane emulation.
 *
 * One UDP socket to the simulator (--simip, --simport). The firmware sends one hello frame;
 * the simulator answers with sensor frame 0, and from then on every sensor frame k is answered
 * by output frame k (fc/machina_sim.h). Frames carry machina's frames and SI units; this file
 * hands them to INAV's simulated sensors.
 *
 * Lockstep (--lockstep): the receive blocks with no timeout, so nothing runs on held sensors.
 * Frame k must carry seq k and the tick t0 + clock, where t0 is frame 0's tick and the clock is
 * the firmware's virtual time since process start (target.c); its step advances the clock. Each
 * step: sensors(k), clock to the end of the step, the main loop until nothing is due, motors(k).
 * A gap, a repeat, a wrong tick or a malformed frame is answered with an error frame, and the
 * firmware exits with status 3: the bridge restarts the run (closed-loop-plan.md D3).
 *
 * Wall clock (no --lockstep): frames are applied as they arrive and answered at once; seq must
 * still count up by one.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "platform.h"
#include "target.h"

#include "build/version.h"
#include "common/utils.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/pitotmeter/pitotmeter_fake.h"
#include "drivers/time.h"
#include "fc/machina_sim.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "io/gps.h"
#include "io/rangefinder.h"
#include "navigation/navigation.h"
#include "rx/sim.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery_sensor_fake.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#include "target/SITL/sim/machina.h"

#define MACHINA_DEFAULT_PORT 49100

static int sockFd = -1;
static struct sockaddr_storage simAddr;
static socklen_t simAddrLen;
static pthread_t worker;
static bool workerStarted = false;
static bool lockstep = false;
static volatile bool closing = false;
static bool connected = false;              // frame 0 applied
static pthread_mutex_t connectMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t connectCond = PTHREAD_COND_INITIALIZER;
static uint64_t applied = 0;                 // frames applied
static uint64_t t0Ns = 0;                    // frame 0's tick

static void sendFrame(const void *buf, size_t len)
{
    if (sendto(sockFd, buf, len, 0, (struct sockaddr *)&simAddr, simAddrLen) < 0 && !closing) {
        fprintf(stderr, "[SIM] machina: send failed (%s)\n", strerror(errno));
    }
}

static void sendHello(void)
{
    machinaSimHelloFrame_t h;
    memset(&h, 0, sizeof(h));
    h.magic = MACHINA_SIM_MAGIC_HELLO;
    h.version = MACHINA_SIM_VERSION;
    h.flags = lockstep ? MACHINA_SIM_OUT_LOCKSTEP : 0;
    h.fcVersion[0] = FC_VERSION_MAJOR;
    h.fcVersion[1] = FC_VERSION_MINOR;
    h.fcVersion[2] = FC_VERSION_PATCH_LEVEL;
    strncpy(h.gitRevision, shortGitRevision, sizeof(h.gitRevision) - 1);
    sendFrame(&h, sizeof(h));
}

static uint64_t clockNs(void)
{
    return lockstep ? sitlLockstepNowNs() : (uint64_t)micros() * 1000;
}

static void fillOutputs(machinaSimOutputFrame_t *o, uint64_t seq, uint64_t tNs)
{
    memset(o, 0, sizeof(*o));
    o->flags = lockstep ? MACHINA_SIM_OUT_LOCKSTEP : 0;
    o->seq = seq;
    o->tNs = tNs;
    o->clockNs = clockNs();
    o->applied = applied;
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
        o->magAdc[i] = mag.magADC[i];
    }
    o->armingFlags = armingFlags;
    o->flightModeFlags = flightModeFlags;
    o->stateFlags = stateFlags;
    o->navState = (uint32_t)NAV_Status.state;
    o->baroPressurePa = baro.baroPressure;
    o->gnssLatE7 = gpsSol.llh.lat;
    o->gnssLonE7 = gpsSol.llh.lon;
}

static void sendOutputs(uint64_t seq, uint64_t tNs)
{
    machinaSimOutputFrame_t o;
    uint8_t buf[sizeof(o)];
    fillOutputs(&o, seq, tNs);
    sendFrame(buf, machinaSimEncodeOutputs(&o, buf, sizeof(buf)));
}

// A refused frame: answer with the error and the expected packet number, then stop.
static void fail(machinaSimStatus_e status, uint64_t seq, uint64_t tNs)
{
    machinaSimOutputFrame_t o;
    uint8_t buf[sizeof(o)];
    memset(&o, 0, sizeof(o));
    o.flags = MACHINA_SIM_OUT_ERROR | (lockstep ? MACHINA_SIM_OUT_LOCKSTEP : 0);
    o.error = (uint8_t)status;
    o.seq = applied;
    o.tNs = tNs;
    o.clockNs = clockNs();
    o.applied = applied;
    sendFrame(buf, machinaSimEncodeOutputs(&o, buf, sizeof(buf)));
    fprintf(stderr, "[SIM] machina: frame refused (%s): seq %llu, expected %llu; clock %llu ns\n",
            machinaSimStatusName(status), (unsigned long long)seq, (unsigned long long)applied,
            (unsigned long long)o.clockNs);
    fflush(stderr);
    _exit(3);
}

static void applySensors(const machinaSimInav_t *s)
{
    if (s->flags & MACHINA_SIM_HAS_IMU) {
        fakeGyroSetDps(s->gyroDps[0], s->gyroDps[1], s->gyroDps[2]);
        fakeAccSetG(s->accG[0], s->accG[1], s->accG[2]);
    }
    if (s->flags & MACHINA_SIM_HAS_MAG) {
        fakeMagSetBody(s->magCounts[0], s->magCounts[1], s->magCounts[2]);
    }
    if (s->flags & MACHINA_SIM_HAS_BARO) {
        fakeBaroSet(s->pressurePa, s->temperatureCdeg);
    }
    if (s->flags & MACHINA_SIM_HAS_GNSS) {
        gpsFakeSetAccuracy(s->ephCm, s->epvCm, s->hdop, true);
        gpsFakeSet((gpsFixType_e)s->fixType, s->numSat, s->latE7, s->lonE7, s->altCm,
                   s->groundSpeedCms, s->groundCourseDecideg,
                   s->velNedCms[0], s->velNedCms[1], s->velNedCms[2], (time_t)s->gnssUnixS);
    }
    if (s->flags & MACHINA_SIM_HAS_RC) {
        uint16_t pwm[MACHINA_SIM_RC_CHANNELS];
        memcpy(pwm, s->rcPwm, sizeof(pwm));
        rxSimSetChannelValue(pwm, MACHINA_SIM_RC_CHANNELS);
        rxSimSetRssi(s->rssi);
    }
    if (s->flags & (MACHINA_SIM_HAS_RC | MACHINA_SIM_RC_FAILSAFE)) {
        rxSimSetFailsafe((s->flags & MACHINA_SIM_RC_FAILSAFE) != 0);
    }
    if (s->flags & MACHINA_SIM_HAS_BATTERY) {
        fakeBattSensorSetVbat(s->vbatCv);
        fakeBattSensorSetAmperage(s->currentCa);
    }
    if (s->flags & MACHINA_SIM_HAS_RANGEFINDER) {
        fakeRangefindersSetData(s->rangeCm);
    }
    if (s->flags & MACHINA_SIM_HAS_AIRSPEED) {
        fakePitotSetAirspeed(s->airspeedCms);
    }
}

static void *machinaWorker(void *arg)
{
    UNUSED(arg);
    uint8_t buf[512];

    while (!closing) {
        struct sockaddr_storage from;
        socklen_t fromLen = sizeof(from);
        const ssize_t len = recvfrom(sockFd, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromLen);
        if (len < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (!closing) {
                fprintf(stderr, "[SIM] machina: receive failed (%s)\n", strerror(errno));
            }
            break;
        }

        machinaSimSi_t si;
        machinaSimInav_t s;
        machinaSimStatus_e status = machinaSimDecodeSensorFrame(buf, (size_t)len, &si);
        if (status != MACHINA_SIM_OK) {
            fail(status, 0, 0);
        }
        if (applied == 0) {
            t0Ns = si.tNs;
        }
        status = machinaSimCheckStep(&si, applied, t0Ns, lockstep ? sitlLockstepNowNs() : 0, lockstep);
        if (status == MACHINA_SIM_OK) {
            status = machinaSimToInav(&si, &s);
        }
        if (status != MACHINA_SIM_OK) {
            fail(status, si.seq, si.tNs);
        }

        if (lockstep) {
            sitlLockstepWaitParked();
        }
        applySensors(&s);
        applied++;
        if (!connected) {
            ENABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
            ENABLE_STATE(ACCELEROMETER_CALIBRATED);
            pthread_mutex_lock(&connectMutex);
            connected = true;
            pthread_cond_broadcast(&connectCond);
            pthread_mutex_unlock(&connectMutex);
        }
        unlockMainPID();
        if (lockstep) {
            sitlLockstepAdvanceTo(sitlLockstepNowNs() + si.stepNs);
        }
        sendOutputs(si.seq, si.tNs);
    }
    return NULL;
}

bool simMachinaInit(char *ip, int port, bool lockstepMode)
{
    lockstep = lockstepMode;
    if (port == 0) {
        port = MACHINA_DEFAULT_PORT;
    }
    if (lookupAddress(ip, port, SOCK_DGRAM, (struct sockaddr *)&simAddr, &simAddrLen) != 0) {
        return false;
    }
    sockFd = socket(((struct sockaddr *)&simAddr)->sa_family, SOCK_DGRAM, IPPROTO_UDP);
    if (sockFd < 0) {
        return false;
    }
    char addrbuf[IPADDRESS_PRINT_BUFLEN];
    char *name = prettyPrintAddress((struct sockaddr *)&simAddr, addrbuf, IPADDRESS_PRINT_BUFLEN);
    fprintf(stderr, "[SOCKET] machina simulator address = %s, fd=%d\n", name ? name : "?", sockFd);

    // No receive timeout: in lockstep a timeout would step the firmware on held sensors.
    sendHello();
    if (pthread_create(&worker, NULL, machinaWorker, NULL) != 0) {
        return false;
    }
    workerStarted = true;

    // Init continues once frame 0 is applied, so it runs on the simulator's sensors.
    if (lockstep) {
        while (!__atomic_load_n(&connected, __ATOMIC_ACQUIRE)) {
            sitlLockstepIdle();
        }
    } else {
        pthread_mutex_lock(&connectMutex);
        while (!connected) {
            pthread_cond_wait(&connectCond, &connectMutex);
        }
        pthread_mutex_unlock(&connectMutex);
    }
    return true;
}

void simMachinaClose(void)
{
    closing = true;
    if (sockFd >= 0) {
        shutdown(sockFd, SHUT_RDWR);
        close(sockFd);
        sockFd = -1;
    }
    if (workerStarted && !pthread_equal(pthread_self(), worker)) {
        pthread_join(worker, NULL);
        workerStarted = false;
    }
    DISABLE_ARMING_FLAG(SIMULATOR_MODE_SITL);
}
