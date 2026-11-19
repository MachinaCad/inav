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
 * machina's simulator frames (machina/README.md): the native SITL backend's packets over UDP
 * (--sim=machina) and the HITL frame over MSP (MSP2_MACHINA_SIMULATOR). Both carry the same
 * sensors in machina's frames and SI units (closed-loop-plan.md C2): FLU body, ENU world,
 * rad/s, m/s^2, T, Pa, K, m, m/s, V, A. This module only decodes, converts and encodes; it
 * holds no firmware state, so the unit tests link it alone.
 *
 * Every frame is little-endian and packed. Conversions into INAV's own units add no clipping and
 * no quantisation beyond INAV's own representation of real sensors: gyro and accelerometer stay
 * floating point (the fake IMU takes floats), the barometer is integer Pa as INAV's drivers give
 * it, GNSS uses the u-blox units INAV keeps (1e-7 deg, cm, cm/s), and the magnetometer keeps
 * 1024 counts per 50 uT (backend.py's INAV_MAG_T_PER_UNIT), finer than the usual compasses.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MACHINA_SIM_VERSION          1
#define MACHINA_SIM_MAGIC_SENSORS    0x3153434Du   // "MCS1"
#define MACHINA_SIM_MAGIC_OUTPUTS    0x314F434Du   // "MCO1"
#define MACHINA_SIM_MAGIC_HELLO      0x3148434Du   // "MCH1"

#define MACHINA_SIM_RC_CHANNELS      16
#define MACHINA_SIM_MAX_MOTORS       8
#define MACHINA_SIM_MAX_SERVOS       8

#define MACHINA_SIM_G0               9.80665f      // m/s^2 per g (INAV's GRAVITY_MSS)
#define MACHINA_SIM_MAG_T_PER_1024   5.0e-5f       // T per 1024 magnetometer counts
#define MACHINA_SIM_MAX_STEP_NS      1000000000ULL // a step longer than 1 s is refused

// What a sensor frame carries (flags).
typedef enum {
    MACHINA_SIM_HAS_IMU          = 1 << 0,   // gyro and accelerometer: a new IMU sample
    MACHINA_SIM_HAS_MAG          = 1 << 1,   // a new magnetometer sample
    MACHINA_SIM_HAS_BARO         = 1 << 2,   // a new barometer conversion
    MACHINA_SIM_HAS_GNSS         = 1 << 3,   // a new GNSS solution
    MACHINA_SIM_HAS_RC           = 1 << 4,   // receiver channels
    MACHINA_SIM_RC_FAILSAFE      = 1 << 5,   // the receiver reports signal loss
    MACHINA_SIM_HAS_BATTERY      = 1 << 6,   // battery voltage and current
    MACHINA_SIM_HAS_RANGEFINDER  = 1 << 7,   // a rangefinder reading (negative: out of range)
    MACHINA_SIM_HAS_AIRSPEED     = 1 << 8,   // a pitot reading
    MACHINA_SIM_HITL_ENABLE      = 1 << 15,  // HITL only: simulator mode on (off: leave it)
} machinaSimSensorFlags_e;

// What an output frame reports (flags).
typedef enum {
    MACHINA_SIM_OUT_LOCKSTEP     = 1 << 0,   // the firmware's clock is virtual (lockstep)
    MACHINA_SIM_OUT_ERROR        = 1 << 1,   // the sensor frame was refused (see error)
    MACHINA_SIM_OUT_HITL         = 1 << 2,   // a board in HITL mode answered (wall clock)
} machinaSimOutputFlags_e;

typedef enum {
    MACHINA_SIM_OK = 0,
    MACHINA_SIM_ERR_SIZE,          // the frame is shorter or longer than its layout
    MACHINA_SIM_ERR_MAGIC,
    MACHINA_SIM_ERR_VERSION,
    MACHINA_SIM_ERR_VALUE,         // a value is not finite or is out of its representable range
    MACHINA_SIM_ERR_SEQUENCE,      // a gap or a repeat
    MACHINA_SIM_ERR_TIME,          // the packet's tick is not the firmware's clock, or a bad step
} machinaSimStatus_e;

// The SITL sensor frame (UDP, simulator to firmware): 272 bytes.
typedef struct __attribute__((packed)) {
    uint32_t magic;                 // MACHINA_SIM_MAGIC_SENSORS
    uint16_t version;               // MACHINA_SIM_VERSION
    uint16_t flags;                 // machinaSimSensorFlags_e
    uint64_t seq;                   // 0, 1, 2, ... in each attempt
    uint64_t tNs;                   // this sample's tick (the bridge's SensorPacket.t_ns)
    uint64_t stepNs;                // to the next tick: lockstep advances the clock by this
    double gyroRads[3];             // body rates, FLU
    double specificForceMps2[3];    // specific force, FLU
    double fieldT[3];               // magnetic field, FLU
    double pressurePa;              // static pressure
    double temperatureK;            // barometer temperature
    double latDeg;                  // WGS-84
    double lonDeg;
    double altM;                    // above mean sea level
    double velEnuMps[3];            // GNSS velocity, ENU
    float ephM;                     // GNSS horizontal and vertical accuracy (1 sigma)
    float epvM;
    uint8_t fixType;                // 0 or 1 none, 2 2D, 3 3D (u-blox and MAVLink numbering)
    uint8_t numSat;
    uint16_t hdop;                  // 0.01 units
    int64_t gnssUnixS;              // GNSS time, Unix seconds (0: none)
    float rc[MACHINA_SIM_RC_CHANNELS];  // normalised, -1 to 1 (1000 to 2000 us)
    uint16_t rssi;                  // 0 to 1023
    uint16_t reserved0;
    float batteryV;
    float batteryA;
    float rangeM;                   // rangefinder distance (negative: out of range)
    float airspeedMps;
} machinaSimSensorFrame_t;

// The HITL sensor frame (MSP2_MACHINA_SIMULATOR payload, host to board): 174 bytes, inside
// INAV's 192-byte MSP receive buffer. Same fields, in float32 and integer GNSS position.
typedef struct __attribute__((packed)) {
    uint8_t version;                // MACHINA_SIM_VERSION
    uint8_t reserved0;
    uint16_t flags;                 // machinaSimSensorFlags_e, with MACHINA_SIM_HITL_ENABLE
    uint32_t seq;                   // 0, 1, 2, ... in each attempt
    uint64_t tNs;                   // the host's sample tick (recorded; the board's clock rules)
    float gyroRads[3];
    float specificForceMps2[3];
    float fieldT[3];
    float pressurePa;
    float temperatureK;
    int32_t latE7;                  // 1e-7 deg
    int32_t lonE7;
    int32_t altCm;                  // above mean sea level
    float velEnuMps[3];
    uint8_t fixType;
    uint8_t numSat;
    uint16_t hdop;                  // 0.01 units
    uint16_t ephCm;
    uint16_t epvCm;
    float rc[MACHINA_SIM_RC_CHANNELS];
    uint16_t rssi;
    float batteryV;
    float batteryA;
    float rangeM;
    float airspeedMps;
} machinaSimHitlFrame_t;

// The output frame (firmware to simulator; the HITL reply has the same layout): 214 bytes.
typedef struct __attribute__((packed)) {
    uint32_t magic;                 // MACHINA_SIM_MAGIC_OUTPUTS
    uint16_t version;
    uint16_t flags;                 // machinaSimOutputFlags_e
    uint64_t seq;                   // the answered packet's number (on an error: the expected one)
    uint64_t tNs;                   // the answered packet's tick (echo)
    uint64_t clockNs;               // the firmware's clock after the step, ns since its start
    uint64_t applied;               // sensor frames applied so far
    uint8_t motorCount;
    uint8_t servoCount;
    uint8_t error;                  // machinaSimStatus_e when MACHINA_SIM_OUT_ERROR
    uint8_t failsafePhase;          // INAV's failsafePhase_e
    float motor[MACHINA_SIM_MAX_MOTORS];        // (motor - 1000) / 1000, as the X-Plane MOTR
    uint16_t motorRaw[MACHINA_SIM_MAX_MOTORS];  // INAV's motor[] (us)
    uint16_t servoRaw[MACHINA_SIM_MAX_SERVOS];  // INAV's servo[] (us)
    int16_t attitudeDecideg[3];     // INAV: roll (right down +), pitch (nose down +), heading (cw from N)
    float ratesDps[3];              // INAV's filtered gyro, FLU
    float posNeuCm[3];              // INAV's position estimate from its origin, north east up
    float velNeuCms[3];
    uint32_t armingFlags;
    uint32_t flightModeFlags;
    uint32_t stateFlags;
    uint32_t navState;
    // The sensors as INAV holds them after the step, for conformance checks.
    float gyroRawDps[3];            // gyro.gyroRaw: unfiltered, FLU
    float accG[3];                  // acc.accADCf, FLU
    float magAdc[3];                // mag.magADC after INAV's alignment and calibration, FLU
    int32_t baroPressurePa;
    int32_t gnssLatE7;
    int32_t gnssLonE7;
} machinaSimOutputFrame_t;

// The hello frame (firmware to simulator, once, before the first sensor frame): 24 bytes.
typedef struct __attribute__((packed)) {
    uint32_t magic;                 // MACHINA_SIM_MAGIC_HELLO
    uint16_t version;
    uint16_t flags;                 // MACHINA_SIM_OUT_LOCKSTEP
    uint8_t fcVersion[3];           // major, minor, patch
    uint8_t reserved0;
    char gitRevision[12];           // NUL-padded
} machinaSimHelloFrame_t;

// Sensors in machina's frames and SI units, whichever frame carried them. Single precision,
// except the GNSS position, kept in INAV's integer units: the code runs on flight controllers.
typedef struct {
    uint16_t flags;
    uint64_t seq;
    uint64_t tNs;
    uint64_t stepNs;
    float gyroRads[3];
    float specificForceMps2[3];
    float fieldT[3];
    float pressurePa;
    float temperatureK;
    int32_t latE7;
    int32_t lonE7;
    int32_t altCm;
    float velEnuMps[3];
    float ephM;
    float epvM;
    uint8_t fixType;
    uint8_t numSat;
    uint16_t hdop;
    int64_t gnssUnixS;
    float rc[MACHINA_SIM_RC_CHANNELS];
    uint16_t rssi;
    float batteryV;
    float batteryA;
    float rangeM;
    float airspeedMps;
} machinaSimSi_t;

// The same sensors in INAV's units and axes, ready for its sensor layer.
typedef struct {
    uint16_t flags;
    float gyroDps[3];               // FLU (INAV's body axes), deg/s
    float accG[3];                  // FLU, g
    int16_t magCounts[3];           // FLU, 1024 counts per 50 uT (before INAV's mag alignment)
    int32_t pressurePa;
    int32_t temperatureCdeg;
    uint8_t fixType;                // INAV gpsFixType_e: GPS_NO_FIX 0, GPS_FIX_2D 1, GPS_FIX_3D 2
    uint8_t numSat;
    uint16_t hdop;
    int32_t latE7;
    int32_t lonE7;
    int32_t altCm;
    int16_t groundSpeedCms;
    int16_t groundCourseDecideg;    // 0 to 3599
    int16_t velNedCms[3];
    uint16_t ephCm;
    uint16_t epvCm;
    int64_t gnssUnixS;
    uint16_t rcPwm[MACHINA_SIM_RC_CHANNELS];   // us
    uint16_t rssi;
    uint16_t vbatCv;                // 0.01 V
    uint16_t currentCa;             // 0.01 A
    int32_t rangeCm;                // -1: out of range
    float airspeedCms;
} machinaSimInav_t;

machinaSimStatus_e machinaSimDecodeSensorFrame(const uint8_t *buf, size_t len, machinaSimSi_t *out);
machinaSimStatus_e machinaSimDecodeHitlFrame(const uint8_t *buf, size_t len, machinaSimSi_t *out);
machinaSimStatus_e machinaSimToInav(const machinaSimSi_t *si, machinaSimInav_t *out);
// Lockstep sequence and timing: packet seq must be expectedSeq, and its tick minus the first
// packet's tick t0Ns must be the firmware's clock; its step must lie in (0, 1 s].
machinaSimStatus_e machinaSimCheckStep(const machinaSimSi_t *si, uint64_t expectedSeq, uint64_t t0Ns, uint64_t clockNs, bool lockstep);
// The raw values that make INAV's sensor alignment `align` (sensor_align_e) yield `flu`.
void machinaSimUnalign(const int16_t flu[3], uint8_t align, int16_t raw[3]);
size_t machinaSimEncodeOutputs(const machinaSimOutputFrame_t *in, uint8_t *buf, size_t cap);
const char *machinaSimStatusName(machinaSimStatus_e status);
