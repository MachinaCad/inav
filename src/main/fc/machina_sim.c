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

// machina's simulator frames: decoding, conversion into INAV's units, encoding (machina_sim.h).

#include <math.h>
#include <string.h>

#include "fc/machina_sim.h"

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "machina's simulator frames are little-endian; this host is not"
#endif

_Static_assert(sizeof(machinaSimSensorFrame_t) == 272, "SITL sensor frame layout");
_Static_assert(sizeof(machinaSimHitlFrame_t) == 174, "HITL sensor frame layout");
_Static_assert(sizeof(machinaSimHitlFrame_t) <= 192 - 2, "HITL frame must fit INAV's MSP receive buffer");
_Static_assert(sizeof(machinaSimOutputFrame_t) == 214, "output frame layout");
_Static_assert(sizeof(machinaSimHelloFrame_t) == 24, "hello frame layout");

#define RAD_TO_DEG_F (57.29577951308232f)

static bool finite3(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

// Round to an integer within [lo, hi]; false when not finite or out of range. Integer bounds
// keep the check exact under INAV's -fsingle-precision-constant.
static bool roundToRange(double x, long long lo, long long hi, long long *out)
{
    if (!isfinite(x) || fabs(x) > (double)9000000000000000LL) {
        return false;
    }
    const long long r = llround(x);
    if (r < lo || r > hi) {
        return false;
    }
    *out = r;
    return true;
}

static bool roundfToRange(float x, long lo, long hi, long *out)
{
    if (!isfinite(x) || fabsf(x) > 1.0e9f) {
        return false;
    }
    const long r = lroundf(x);
    if (r < lo || r > hi) {
        return false;
    }
    *out = r;
    return true;
}

static uint16_t roundfToUint16Saturated(float x)
{
    if (!(x > 0.0f)) {
        return 0;
    }
    return x >= 65535.0f ? 65535 : (uint16_t)lroundf(x);
}

static int16_t roundfToInt16Saturated(float x)
{
    if (x <= -32768.0f) {
        return -32768;
    }
    return x >= 32767.0f ? 32767 : (int16_t)lroundf(x);
}

machinaSimStatus_e machinaSimDecodeSensorFrame(const uint8_t *buf, size_t len, machinaSimSi_t *out)
{
    machinaSimSensorFrame_t f;
    if (len != sizeof(f)) {
        return MACHINA_SIM_ERR_SIZE;
    }
    memcpy(&f, buf, sizeof(f));
    if (f.magic != MACHINA_SIM_MAGIC_SENSORS) {
        return MACHINA_SIM_ERR_MAGIC;
    }
    if (f.version != MACHINA_SIM_VERSION) {
        return MACHINA_SIM_ERR_VERSION;
    }
    memset(out, 0, sizeof(*out));
    out->flags = f.flags;
    out->seq = f.seq;
    out->tNs = f.tNs;
    out->stepNs = f.stepNs;
    for (int i = 0; i < 3; i++) {
        out->gyroRads[i] = (float)f.gyroRads[i];
        out->specificForceMps2[i] = (float)f.specificForceMps2[i];
        out->fieldT[i] = (float)f.fieldT[i];
        out->velEnuMps[i] = (float)f.velEnuMps[i];
    }
    out->pressurePa = (float)f.pressurePa;
    out->temperatureK = (float)f.temperatureK;
    if (f.flags & MACHINA_SIM_HAS_GNSS) {
        // Degrees to INAV's 1e-7 deg in double precision: float would lose up to 0.4 m.
        long long latE7, lonE7, altCm;
        if (!(fabs(f.latDeg) <= 90) || !(fabs(f.lonDeg) <= 180)
                || !roundToRange(f.latDeg * 10000000, -900000000LL, 900000000LL, &latE7)
                || !roundToRange(f.lonDeg * 10000000, -1800000000LL, 1800000000LL, &lonE7)
                || !roundToRange(f.altM * 100, INT32_MIN, INT32_MAX, &altCm)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        out->latE7 = (int32_t)latE7;
        out->lonE7 = (int32_t)lonE7;
        out->altCm = (int32_t)altCm;
    }
    out->ephM = f.ephM;
    out->epvM = f.epvM;
    out->fixType = f.fixType;
    out->numSat = f.numSat;
    out->hdop = f.hdop;
    out->gnssUnixS = f.gnssUnixS;
    for (int i = 0; i < MACHINA_SIM_RC_CHANNELS; i++) {
        out->rc[i] = f.rc[i];
    }
    out->rssi = f.rssi;
    out->batteryV = f.batteryV;
    out->batteryA = f.batteryA;
    out->rangeM = f.rangeM;
    out->airspeedMps = f.airspeedMps;
    return MACHINA_SIM_OK;
}

machinaSimStatus_e machinaSimDecodeHitlFrame(const uint8_t *buf, size_t len, machinaSimSi_t *out)
{
    machinaSimHitlFrame_t f;
    if (len != sizeof(f)) {
        return MACHINA_SIM_ERR_SIZE;
    }
    memcpy(&f, buf, sizeof(f));
    if (f.version != MACHINA_SIM_VERSION) {
        return MACHINA_SIM_ERR_VERSION;
    }
    memset(out, 0, sizeof(*out));
    out->flags = f.flags;
    out->seq = f.seq;
    out->tNs = f.tNs;
    for (int i = 0; i < 3; i++) {
        out->gyroRads[i] = f.gyroRads[i];
        out->specificForceMps2[i] = f.specificForceMps2[i];
        out->fieldT[i] = f.fieldT[i];
        out->velEnuMps[i] = f.velEnuMps[i];
    }
    out->pressurePa = f.pressurePa;
    out->temperatureK = f.temperatureK;
    out->latE7 = f.latE7;
    out->lonE7 = f.lonE7;
    out->altCm = f.altCm;
    out->ephM = f.ephCm / 100.0f;
    out->epvM = f.epvCm / 100.0f;
    out->fixType = f.fixType;
    out->numSat = f.numSat;
    out->hdop = f.hdop;
    for (int i = 0; i < MACHINA_SIM_RC_CHANNELS; i++) {
        out->rc[i] = f.rc[i];
    }
    out->rssi = f.rssi;
    out->batteryV = f.batteryV;
    out->batteryA = f.batteryA;
    out->rangeM = f.rangeM;
    out->airspeedMps = f.airspeedMps;
    return MACHINA_SIM_OK;
}

machinaSimStatus_e machinaSimToInav(const machinaSimSi_t *si, machinaSimInav_t *out)
{
    memset(out, 0, sizeof(*out));
    out->flags = si->flags;
    out->rangeCm = -1;

    if (si->flags & MACHINA_SIM_HAS_IMU) {
        if (!finite3(si->gyroRads) || !finite3(si->specificForceMps2)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        for (int i = 0; i < 3; i++) {
            // FLU is INAV's body frame for the IMU (target/SITL/sim/xplane.c feeds (p, -q, -r)).
            out->gyroDps[i] = si->gyroRads[i] * RAD_TO_DEG_F;
            out->accG[i] = si->specificForceMps2[i] / MACHINA_SIM_G0;
        }
    }

    if (si->flags & MACHINA_SIM_HAS_MAG) {
        if (!finite3(si->fieldT)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        for (int i = 0; i < 3; i++) {
            // Saturates at +-1.6 mT, beyond the range of any compass INAV supports.
            out->magCounts[i] = roundfToInt16Saturated(si->fieldT[i] * (1024.0f / MACHINA_SIM_MAG_T_PER_1024));
        }
    }

    if (si->flags & MACHINA_SIM_HAS_BARO) {
        long pa, cdeg;
        if (!roundfToRange(si->pressurePa, 0, 2000000L, &pa)
                || !roundfToRange((si->temperatureK - 273.15f) * 100.0f, -27315L, 100000L, &cdeg)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        out->pressurePa = (int32_t)pa;
        out->temperatureCdeg = (int32_t)cdeg;
    }

    if (si->flags & MACHINA_SIM_HAS_GNSS) {
        if (!finite3(si->velEnuMps) || !isfinite(si->ephM) || !isfinite(si->epvM)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        const float vnCms = si->velEnuMps[1] * 100.0f;
        const float veCms = si->velEnuMps[0] * 100.0f;
        const float vdCms = -si->velEnuMps[2] * 100.0f;
        long vn, ve, vd, gs;
        if (!roundfToRange(vnCms, INT16_MIN, INT16_MAX, &vn) || !roundfToRange(veCms, INT16_MIN, INT16_MAX, &ve)
                || !roundfToRange(vdCms, INT16_MIN, INT16_MAX, &vd)
                || !roundfToRange(sqrtf(vnCms * vnCms + veCms * veCms), 0, INT16_MAX, &gs)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        float courseDecideg = atan2f(veCms, vnCms) * RAD_TO_DEG_F * 10.0f;
        if (courseDecideg < 0.0f) {
            courseDecideg += 3600.0f;
        }
        out->latE7 = si->latE7;
        out->lonE7 = si->lonE7;
        out->altCm = si->altCm;
        out->velNedCms[0] = (int16_t)vn;
        out->velNedCms[1] = (int16_t)ve;
        out->velNedCms[2] = (int16_t)vd;
        out->groundSpeedCms = (int16_t)gs;
        out->groundCourseDecideg = (int16_t)(lroundf(courseDecideg) % 3600);
        // u-blox numbering (3 = 3D) to INAV gpsFixType_e (GPS_NO_FIX 0, GPS_FIX_2D 1, GPS_FIX_3D 2)
        out->fixType = si->fixType >= 3 ? 2 : (si->fixType == 2 ? 1 : 0);
        out->numSat = si->numSat;
        out->hdop = si->hdop;
        out->ephCm = roundfToUint16Saturated(si->ephM * 100.0f);
        out->epvCm = roundfToUint16Saturated(si->epvM * 100.0f);
        out->gnssUnixS = si->gnssUnixS;
    }

    if (si->flags & MACHINA_SIM_HAS_RC) {
        for (int i = 0; i < MACHINA_SIM_RC_CHANNELS; i++) {
            if (!(fabsf(si->rc[i]) <= 1.0f)) {
                return MACHINA_SIM_ERR_VALUE;
            }
            out->rcPwm[i] = (uint16_t)lroundf(1500.0f + 500.0f * si->rc[i]);
        }
        out->rssi = si->rssi > 1023 ? 1023 : si->rssi;
    }

    if (si->flags & MACHINA_SIM_HAS_BATTERY) {
        if (!isfinite(si->batteryV) || !isfinite(si->batteryA)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        out->vbatCv = roundfToUint16Saturated(si->batteryV * 100.0f);
        out->currentCa = roundfToUint16Saturated(si->batteryA * 100.0f);  // INAV clamps current at 0
    }

    if (si->flags & MACHINA_SIM_HAS_RANGEFINDER) {
        if (isnan(si->rangeM)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        long cm;
        if (si->rangeM >= 0.0f) {
            if (!roundfToRange(si->rangeM * 100.0f, 0, INT32_MAX, &cm)) {
                return MACHINA_SIM_ERR_VALUE;
            }
            out->rangeCm = (int32_t)cm;
        }
    }

    if (si->flags & MACHINA_SIM_HAS_AIRSPEED) {
        if (!isfinite(si->airspeedMps)) {
            return MACHINA_SIM_ERR_VALUE;
        }
        out->airspeedCms = si->airspeedMps * 100.0f;
    }
    return MACHINA_SIM_OK;
}

machinaSimStatus_e machinaSimCheckStep(const machinaSimSi_t *si, uint64_t expectedSeq, uint64_t t0Ns, uint64_t clockNs, bool lockstep)
{
    if (si->seq != expectedSeq) {
        return MACHINA_SIM_ERR_SEQUENCE;
    }
    if (lockstep) {
        if (si->stepNs == 0 || si->stepNs > MACHINA_SIM_MAX_STEP_NS) {
            return MACHINA_SIM_ERR_TIME;
        }
        if (si->tNs < t0Ns || si->tNs - t0Ns != clockNs) {
            return MACHINA_SIM_ERR_TIME;
        }
    }
    return MACHINA_SIM_OK;
}

void machinaSimUnalign(const int16_t flu[3], uint8_t align, int16_t raw[3])
{
    // The inverse of boardalignment.c's applySensorAlignment: each alignment is a signed
    // permutation, so its inverse is its transpose. CW90 and CW270 are each other's inverse;
    // the others are their own.
    const int16_t x = flu[0];
    const int16_t y = flu[1];
    const int16_t z = flu[2];
    switch (align) {
    case 2:     // CW90_DEG: (y, -x, z); inverse CW270
        raw[0] = -y; raw[1] = x; raw[2] = z;
        break;
    case 3:     // CW180_DEG
        raw[0] = -x; raw[1] = -y; raw[2] = z;
        break;
    case 4:     // CW270_DEG: (-y, x, z); inverse CW90
        raw[0] = y; raw[1] = -x; raw[2] = z;
        break;
    case 5:     // CW0_DEG_FLIP
        raw[0] = -x; raw[1] = y; raw[2] = -z;
        break;
    case 6:     // CW90_DEG_FLIP
        raw[0] = y; raw[1] = x; raw[2] = -z;
        break;
    case 7:     // CW180_DEG_FLIP
        raw[0] = x; raw[1] = -y; raw[2] = -z;
        break;
    case 8:     // CW270_DEG_FLIP
        raw[0] = -y; raw[1] = -x; raw[2] = -z;
        break;
    default:    // ALIGN_DEFAULT, CW0_DEG
        raw[0] = x; raw[1] = y; raw[2] = z;
        break;
    }
}

size_t machinaSimEncodeOutputs(const machinaSimOutputFrame_t *in, uint8_t *buf, size_t cap)
{
    if (cap < sizeof(*in)) {
        return 0;
    }
    machinaSimOutputFrame_t f = *in;
    f.magic = MACHINA_SIM_MAGIC_OUTPUTS;
    f.version = MACHINA_SIM_VERSION;
    memcpy(buf, &f, sizeof(f));
    return sizeof(f);
}

const char *machinaSimStatusName(machinaSimStatus_e status)
{
    switch (status) {
    case MACHINA_SIM_OK: return "ok";
    case MACHINA_SIM_ERR_SIZE: return "size";
    case MACHINA_SIM_ERR_MAGIC: return "magic";
    case MACHINA_SIM_ERR_VERSION: return "version";
    case MACHINA_SIM_ERR_VALUE: return "value";
    case MACHINA_SIM_ERR_SEQUENCE: return "sequence";
    case MACHINA_SIM_ERR_TIME: return "time";
    }
    return "unknown";
}
