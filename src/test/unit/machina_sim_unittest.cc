/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * machina's simulator frames (fc/machina_sim.c): the SITL and HITL decoders, the conversions into
 * INAV's units (every value against a closed form), the lockstep sequence and timing checks, and
 * the inverse of INAV's sensor alignments (against boardalignment.c's own applySensorAlignment).
 */

#include <cmath>
#include <cstring>
#include <limits>

extern "C" {
    #include "platform.h"
    #include "drivers/sensor.h"
    #include "fc/machina_sim.h"
    #include "sensors/boardalignment.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

machinaSimSensorFrame_t baseFrame(uint16_t flags)
{
    machinaSimSensorFrame_t f;
    memset(&f, 0, sizeof(f));
    f.magic = MACHINA_SIM_MAGIC_SENSORS;
    f.version = MACHINA_SIM_VERSION;
    f.flags = flags;
    f.seq = 4;
    f.tNs = 5000000;
    f.stepNs = 1000000;
    return f;
}

machinaSimStatus_e convert(const machinaSimSensorFrame_t &f, machinaSimInav_t *out)
{
    machinaSimSi_t si;
    const machinaSimStatus_e st = machinaSimDecodeSensorFrame((const uint8_t *)&f, sizeof(f), &si);
    if (st != MACHINA_SIM_OK) {
        return st;
    }
    return machinaSimToInav(&si, out);
}

}  // namespace

TEST(MachinaSimTest, FrameSizesMatchTheWireLayouts)
{
    EXPECT_EQ(272u, sizeof(machinaSimSensorFrame_t));
    EXPECT_EQ(174u, sizeof(machinaSimHitlFrame_t));
    EXPECT_EQ(214u, sizeof(machinaSimOutputFrame_t));
    EXPECT_EQ(24u, sizeof(machinaSimHelloFrame_t));
}

TEST(MachinaSimTest, DecoderRefusesBadSizeMagicAndVersion)
{
    machinaSimSensorFrame_t f = baseFrame(0);
    machinaSimSi_t si;
    EXPECT_EQ(MACHINA_SIM_ERR_SIZE, machinaSimDecodeSensorFrame((const uint8_t *)&f, sizeof(f) - 1, &si));
    f.magic = MACHINA_SIM_MAGIC_OUTPUTS;
    EXPECT_EQ(MACHINA_SIM_ERR_MAGIC, machinaSimDecodeSensorFrame((const uint8_t *)&f, sizeof(f), &si));
    f = baseFrame(0);
    f.version = 2;
    EXPECT_EQ(MACHINA_SIM_ERR_VERSION, machinaSimDecodeSensorFrame((const uint8_t *)&f, sizeof(f), &si));
    f = baseFrame(0);
    EXPECT_EQ(MACHINA_SIM_OK, machinaSimDecodeSensorFrame((const uint8_t *)&f, sizeof(f), &si));
    EXPECT_EQ(4u, si.seq);
    EXPECT_EQ(5000000u, si.tNs);
    EXPECT_EQ(1000000u, si.stepNs);
}

TEST(MachinaSimTest, ImuIsFluInDegreesPerSecondAndGUnclipped)
{
    machinaSimSensorFrame_t f = baseFrame(MACHINA_SIM_HAS_IMU);
    f.gyroRads[0] = 1.0;                 // 57.29578 deg/s
    f.gyroRads[1] = -50.0;               // -2864.8 deg/s: beyond an int16 count at 1/16 deg/s
    f.gyroRads[2] = 1.0e-6;              // 5.7e-5 deg/s: below a 1/16 deg/s count
    f.specificForceMps2[0] = 5.0 * 9.80665;    // 5 g: the X-Plane path clips at 3.34 g
    f.specificForceMps2[1] = -0.001;
    f.specificForceMps2[2] = 9.80665;
    machinaSimInav_t s;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_NEAR(57.2957795, s.gyroDps[0], 1e-5);
    EXPECT_NEAR(-2864.78898, s.gyroDps[1], 1e-3);
    EXPECT_NEAR(5.72957795e-5, s.gyroDps[2], 1e-11);
    EXPECT_NEAR(5.0, s.accG[0], 1e-6);
    EXPECT_NEAR(-0.001 / 9.80665, s.accG[1], 1e-9);
    EXPECT_NEAR(1.0, s.accG[2], 1e-7);
}

TEST(MachinaSimTest, MagnetometerIs1024CountsPer50MicroteslaAndSaturates)
{
    machinaSimSensorFrame_t f = baseFrame(MACHINA_SIM_HAS_MAG);
    f.fieldT[0] = 50e-6;
    f.fieldT[1] = -25e-6;
    f.fieldT[2] = 5e-3;                  // 5 mT: beyond the int16 range (1.6 mT)
    machinaSimInav_t s;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(1024, s.magCounts[0]);
    EXPECT_EQ(-512, s.magCounts[1]);
    EXPECT_EQ(32767, s.magCounts[2]);
}

TEST(MachinaSimTest, BarometerIsIntegerPascalAndCentidegrees)
{
    machinaSimSensorFrame_t f = baseFrame(MACHINA_SIM_HAS_BARO);
    f.pressurePa = 95000.75;
    f.temperatureK = 288.15;
    machinaSimInav_t s;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(95001, s.pressurePa);
    EXPECT_EQ(1500, s.temperatureCdeg);
    f.pressurePa = -1.0;
    EXPECT_EQ(MACHINA_SIM_ERR_VALUE, convert(f, &s));
}

TEST(MachinaSimTest, GnssKeepsUbloxResolutionAtARealHome)
{
    machinaSimSensorFrame_t f = baseFrame(MACHINA_SIM_HAS_GNSS);
    f.latDeg = 47.3977421;               // float32 would land 0.2 m away
    f.lonDeg = -8.5455938;
    f.altM = 488.254;
    f.velEnuMps[0] = 1.0;                // east
    f.velEnuMps[1] = 2.0;                // north
    f.velEnuMps[2] = -0.5;               // up
    f.ephM = 0.5f;
    f.epvM = 0.75f;
    f.fixType = 3;
    f.numSat = 14;
    f.hdop = 80;
    machinaSimInav_t s;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(473977421, s.latE7);
    EXPECT_EQ(-85455938, s.lonE7);
    EXPECT_EQ(48825, s.altCm);
    EXPECT_EQ(200, s.velNedCms[0]);
    EXPECT_EQ(100, s.velNedCms[1]);
    EXPECT_EQ(50, s.velNedCms[2]);
    EXPECT_EQ(224, s.groundSpeedCms);                         // hypot(100, 200) = 223.6
    EXPECT_EQ(266, s.groundCourseDecideg);                    // atan2(1, 2) = 26.57 deg
    EXPECT_EQ(2, s.fixType);                                  // GPS_FIX_3D
    EXPECT_EQ(50, s.ephCm);
    EXPECT_EQ(75, s.epvCm);
    f.velEnuMps[0] = -1.0;
    f.velEnuMps[1] = 0.0;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(2700, s.groundCourseDecideg);                   // due west
    f.fixType = 2;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(1, s.fixType);                                  // GPS_FIX_2D
    f.latDeg = 90.5;
    EXPECT_EQ(MACHINA_SIM_ERR_VALUE, convert(f, &s));
}

TEST(MachinaSimTest, RcIsNormalisedToMicroseconds)
{
    machinaSimSensorFrame_t f = baseFrame(MACHINA_SIM_HAS_RC);
    f.rc[0] = -1.0f;
    f.rc[1] = 0.0f;
    f.rc[2] = 1.0f;
    f.rc[3] = 0.3f;
    f.rssi = 2000;
    machinaSimInav_t s;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(1000, s.rcPwm[0]);
    EXPECT_EQ(1500, s.rcPwm[1]);
    EXPECT_EQ(2000, s.rcPwm[2]);
    EXPECT_EQ(1650, s.rcPwm[3]);
    EXPECT_EQ(1023, s.rssi);
    f.rc[5] = 1.01f;
    EXPECT_EQ(MACHINA_SIM_ERR_VALUE, convert(f, &s));
}

TEST(MachinaSimTest, BatteryRangefinderAirspeed)
{
    machinaSimSensorFrame_t f = baseFrame(MACHINA_SIM_HAS_BATTERY | MACHINA_SIM_HAS_RANGEFINDER | MACHINA_SIM_HAS_AIRSPEED);
    f.batteryV = 33.6f;                  // 8S: beyond MSP_SIMULATOR's 25.5 V byte
    f.batteryA = -2.0f;                  // regenerating: INAV clamps current at 0
    f.rangeM = 1.234f;
    f.airspeedMps = 12.5f;
    machinaSimInav_t s;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(3360, s.vbatCv);
    EXPECT_EQ(0, s.currentCa);
    EXPECT_EQ(123, s.rangeCm);
    EXPECT_NEAR(1250.0f, s.airspeedCms, 1e-3f);
    f.rangeM = -1.0f;
    ASSERT_EQ(MACHINA_SIM_OK, convert(f, &s));
    EXPECT_EQ(-1, s.rangeCm);
}

TEST(MachinaSimTest, NonFiniteValuesAreRefusedOnlyWhereFlagged)
{
    machinaSimSensorFrame_t f = baseFrame(0);
    f.gyroRads[0] = std::numeric_limits<double>::quiet_NaN();
    machinaSimInav_t s;
    EXPECT_EQ(MACHINA_SIM_OK, convert(f, &s));               // no IMU in this frame
    f.flags = MACHINA_SIM_HAS_IMU;
    EXPECT_EQ(MACHINA_SIM_ERR_VALUE, convert(f, &s));
    f = baseFrame(MACHINA_SIM_HAS_MAG);
    f.fieldT[1] = std::numeric_limits<double>::infinity();
    EXPECT_EQ(MACHINA_SIM_ERR_VALUE, convert(f, &s));
}

TEST(MachinaSimTest, LockstepRefusesGapsRepeatsAndWrongTicks)
{
    machinaSimSi_t si;
    memset(&si, 0, sizeof(si));
    si.seq = 10;
    si.tNs = 7000000000ULL + 10000000ULL;    // t0 = 7 s, ten 1-ms steps in
    si.stepNs = 1000000;
    EXPECT_EQ(MACHINA_SIM_OK, machinaSimCheckStep(&si, 10, 7000000000ULL, 10000000ULL, true));
    EXPECT_EQ(MACHINA_SIM_ERR_SEQUENCE, machinaSimCheckStep(&si, 11, 7000000000ULL, 10000000ULL, true));   // repeat
    EXPECT_EQ(MACHINA_SIM_ERR_SEQUENCE, machinaSimCheckStep(&si, 9, 7000000000ULL, 10000000ULL, true));    // gap
    EXPECT_EQ(MACHINA_SIM_ERR_TIME, machinaSimCheckStep(&si, 10, 7000000000ULL, 10000001ULL, true));       // off tick by 1 ns
    EXPECT_EQ(MACHINA_SIM_ERR_TIME, machinaSimCheckStep(&si, 10, 7100000000ULL, 10000000ULL, true));       // before t0
    si.stepNs = 0;
    EXPECT_EQ(MACHINA_SIM_ERR_TIME, machinaSimCheckStep(&si, 10, 7000000000ULL, 10000000ULL, true));
    si.stepNs = MACHINA_SIM_MAX_STEP_NS + 1;
    EXPECT_EQ(MACHINA_SIM_ERR_TIME, machinaSimCheckStep(&si, 10, 7000000000ULL, 10000000ULL, true));
    // Wall clock: the sequence is still checked, time is not.
    EXPECT_EQ(MACHINA_SIM_OK, machinaSimCheckStep(&si, 10, 0, 0, false));
    EXPECT_EQ(MACHINA_SIM_ERR_SEQUENCE, machinaSimCheckStep(&si, 11, 0, 0, false));
}

TEST(MachinaSimTest, UnalignInvertsEverySensorAlignment)
{
    const int16_t v[3] = { 123, -456, 789 };
    for (uint8_t align = ALIGN_DEFAULT; align <= CW270_DEG_FLIP; align++) {
        int16_t raw[3];
        machinaSimUnalign(v, align, raw);
        float in[3] = { (float)raw[0], (float)raw[1], (float)raw[2] };
        float out[3];
        applySensorAlignment(out, in, align);
        EXPECT_EQ(v[0], out[0]) << "alignment " << (int)align;
        EXPECT_EQ(v[1], out[1]) << "alignment " << (int)align;
        EXPECT_EQ(v[2], out[2]) << "alignment " << (int)align;
    }
}

TEST(MachinaSimTest, XplaneMagMappingIsTheInverseOfTheDefaultExternalCompassAlignment)
{
    // machina's G0 probe measured the X-Plane path's mapping in flight: drefs 34 to 36 carry
    // (-m_y, -m_x, -m_z) of the FLU field (backend.py InavXplane). It is INAV's own chain: the
    // fake compass has no driver alignment, so compass.c applies CW270_DEG_FLIP, the default for
    // external compasses, and that alignment is its own inverse.
    const int16_t flu[3] = { 300, -500, 800 };
    int16_t raw[3];
    machinaSimUnalign(flu, CW270_DEG_FLIP, raw);
    EXPECT_EQ(500, raw[0]);
    EXPECT_EQ(-300, raw[1]);
    EXPECT_EQ(-800, raw[2]);
}

TEST(MachinaSimTest, HitlFrameKeepsIntegerGnssAndCarriesTheSameSensors)
{
    machinaSimHitlFrame_t h;
    memset(&h, 0, sizeof(h));
    h.version = MACHINA_SIM_VERSION;
    h.flags = MACHINA_SIM_HITL_ENABLE | MACHINA_SIM_HAS_IMU | MACHINA_SIM_HAS_GNSS;
    h.seq = 3;
    h.tNs = 123456789;
    h.gyroRads[2] = 0.5f;
    h.specificForceMps2[2] = 9.80665f;
    h.latE7 = 473977421;
    h.lonE7 = 85455938;
    h.altCm = 48825;
    h.ephCm = 150;
    h.fixType = 3;
    machinaSimSi_t si;
    ASSERT_EQ(MACHINA_SIM_OK, machinaSimDecodeHitlFrame((const uint8_t *)&h, sizeof(h), &si));
    EXPECT_EQ(MACHINA_SIM_ERR_SIZE, machinaSimDecodeHitlFrame((const uint8_t *)&h, sizeof(h) + 1, &si));
    machinaSimInav_t s;
    ASSERT_EQ(MACHINA_SIM_OK, machinaSimToInav(&si, &s));
    EXPECT_EQ(3u, si.seq);
    EXPECT_EQ(123456789u, si.tNs);
    EXPECT_EQ(473977421, s.latE7);
    EXPECT_EQ(85455938, s.lonE7);
    EXPECT_EQ(48825, s.altCm);
    EXPECT_EQ(150, s.ephCm);
    EXPECT_NEAR(28.6478898f, s.gyroDps[2], 1e-4f);
    EXPECT_NEAR(1.0f, s.accG[2], 1e-6f);
    h.version = 9;
    EXPECT_EQ(MACHINA_SIM_ERR_VERSION, machinaSimDecodeHitlFrame((const uint8_t *)&h, sizeof(h), &si));
}

TEST(MachinaSimTest, OutputEncoderStampsMagicAndVersion)
{
    machinaSimOutputFrame_t o;
    memset(&o, 0, sizeof(o));
    o.seq = 42;
    uint8_t buf[sizeof(o)];
    EXPECT_EQ(0u, machinaSimEncodeOutputs(&o, buf, sizeof(buf) - 1));
    ASSERT_EQ(sizeof(o), machinaSimEncodeOutputs(&o, buf, sizeof(buf)));
    machinaSimOutputFrame_t back;
    memcpy(&back, buf, sizeof(back));
    EXPECT_EQ(MACHINA_SIM_MAGIC_OUTPUTS, back.magic);
    EXPECT_EQ(MACHINA_SIM_VERSION, back.version);
    EXPECT_EQ(42u, back.seq);
}

// boardalignment.c's dependencies, not exercised here.
extern "C" {
    uint32_t stateFlags = 0;
}
