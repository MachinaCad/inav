# machina's INAV fork

INAV 9.1.0 (tag `9.1.0`, commit `e519b69b02e27c8bdc03b4a0889f1baaae211a54`) with the changes that
machina's closed loop needs to run INAV's own firmware as the flight controller of a simulation:
deterministic lockstep, a native binary simulator interface, and the firmware half of
hardware-in-the-loop (HITL). Branch `machina-9.1.0`; this folder holds the build, the harness and the
fast checks. The evidence (commit hashes, commands, results) is in machina's repository,
`docs/research/rig/firmware/inav-9.1.0.md`; the requirements are its `closed-loop-plan.md` D3.

## What the fork changes, and why

| Commit | Change | Why |
|---|---|---|
| `804a7b7e8` | X-Plane path: a `MOTR` datagram per exchange (every motor, INAV's attitude, position and velocity, flags, navigation state); fake GNSS velocity in NED | Imported unchanged from machina's G0 probe P1d (`spikes/cl_p01d_inav/patches/0001`): the stock bridge exports one throttle and zero GNSS velocity |
| `186fed8f4` | `--lockstep[=dt_us]` prototype | Imported unchanged from P1d (`patches/0002`). Its clock started from `realMicros()` after a real-time boot, and a 1-s receive timeout could step the firmware on held sensors: 12 % of identical-seed pairs agreed |
| `3e8ba1e8a` | `machina/build.sh`, `machina/Containerfile` | Builds the SITL binary and INAV's unit tests in rootless Podman from this tree |
| `52887917d` | Lockstep core: virtual time from process start in integer ns; init driven by sensor packets; X-Plane path with a blocking receive, every packet counted and optionally numbered | D3's production lockstep, for both interfaces |
| `10260058e` | The native backend, `--sim=machina` | Replaces X-Plane emulation, which clips the accelerometer at 3.34 g, quantises the gyro to 1/16 deg/s, carries latitude and longitude as float32 (about 0.4 m at 47 deg) and needs XITL version tricks |
| `432581a00` | `MSP2_MACHINA_SIMULATOR`: machina's HITL frame | The firmware half of HITL on a real board, on the wall clock |
| `b44a14bf2` | `machina_sim_unittest` (14 cases); ctest run from `build/src/test` | INAV's own unit-test framework for the frame codec (55 of 55 pass) |
| the commit that adds this file | `machina/harness/`, `machina/tests/`, this README | The harness ported from the G0 probe, and the fast checks below |
| `436b3da6e` | The X-Plane path in the bridge's form beside FW2's: `--simip=unix:<path>`, packets with their tick, `MCH1` replies, the clock at each tick, the first packet counted when paced | Machina's merged bridge (`w1-s7-inav-bridge`) speaks this form; with it the bridge runs this fork unchanged |

`git log e519b69b0..machina-9.1.0` lists them all.

### Lockstep (`--lockstep`)

- The firmware's clock (`micros()`) is virtual from process start, kept as an integer number of
  nanoseconds, and moves only when the simulator thread applies a sensor packet. It never reads the
  host's clock.
- The main thread runs only while the clock stands still. The simulator thread changes the clock and
  the simulated sensors only while the main thread is parked, in the scheduler with nothing due or
  in `delay()`. Init is driven by packets too: its delays park the main thread until the simulator
  has stepped past them.
- Each step runs in a fixed order: sensors(k), then the clock to the end of the step, then the main
  loop until nothing is due, then motors(k). Nothing depends on when a packet arrives, only on its
  contents and order, so identical inputs give bit-identical outputs (`machina/tests/test_determinism.py`).
- The receive blocks with no timeout: the firmware never runs a step without a packet.

### The native backend (`--sim=machina`)

One UDP socket to `--simip`:`--simport` (default port 49100). The firmware sends one hello frame;
the simulator answers with sensor frame 0, and from then on sensor frame k is answered by output
frame k. The layouts are packed little-endian structs in `src/main/fc/machina_sim.h`, mirrored in
`machina/harness/frames.py`:

| Frame | Bytes | Direction | Content |
|---|---|---|---|
| hello `MCH1` | 24 | firmware to simulator | version, lockstep flag, INAV version, git revision |
| sensors `MCS1` | 272 | simulator to firmware | seq, tick `t_ns`, step `step_ns`; gyro (rad/s), specific force (m/s^2), field (T), all FLU; pressure (Pa) and temperature (K); WGS-84 latitude and longitude (float64 deg), altitude (m), ENU velocity (m/s), accuracy, fix and satellites, GNSS time; 16 RC channels (-1 to 1), RSSI; battery V and A; rangefinder (m); airspeed (m/s); flags saying which sensors are new |
| outputs `MCO1` | 214 | firmware to simulator | the echoed seq and tick, the clock after the step (ns since process start), frames applied; 8 motors (normalised and raw), 8 servos; INAV's attitude, filtered rates, position and velocity estimate; arming, flight-mode and state flags, navigation state, failsafe phase; the sensors as INAV holds them (raw gyro, accelerometer, compass, pressure, GNSS position), for conformance checks |

- **Frames.** Sensors are in machina's frames and SI (closed-loop-plan.md C2): FLU body, ENU world.
  INAV's own body frame is FLU for the IMU, so the gyro and accelerometer go in unchanged; the field
  goes in as FLU and the fake compass reads it in the sensor's own axes, through the inverse of the
  configured magnetometer alignment (the default for an external compass is `CW270_DEG_FLIP`).
  This is also why the X-Plane path needed (-m_y, -m_x, -m_z): `machina_sim_unittest.cc` derives that
  mapping from INAV's alignment code.
- **Resolution.** No clipping or quantisation beyond INAV's own representation of real sensors: the
  fake IMU now takes floats; the barometer is integer Pa, as every INAV baro driver delivers; GNSS
  position in 1e-7 deg and cm and velocity in cm/s, as from a u-blox receiver; the compass at 1024
  counts per 50 uT (finer than the usual compasses), saturating at 1.6 mT. Saturation and
  quantisation of the real parts belong to machina's sensor models, not to this interface.
- **Compass calibration.** INAV weights its compass by how close the field's magnitude comes to
  1024 counts after its calibration gains (`maggain_x/y/z`). A bridge sets the gains that
  calibration would leave at the flight's site (`sitl.compass_calibration`): left at 50 uT's, a
  25 uT site cuts the compass weight to e^-2 and the heading does not converge before arming.
- **Timing.** Each slow sensor (compass, barometer, GNSS) is flagged new only at its own tick, and
  the fake GNSS hands each solution to navigation once, as a real receiver does (stock SITL feeds
  every GPS task run).
- **Sequence and time, lockstep.** Frame k must carry seq k and the tick t0 + clock, where t0 is
  frame 0's tick (so the firmware clock starts at the scenario's start, whatever its time), with a
  step in (0, 1 s]. A gap, a repeat, a wrong tick or a malformed frame is answered with an error
  frame (flag `ERROR`, the reason, the expected seq) and the firmware exits with status 3: the bridge
  restarts the run (D3's restart recovery).
- **Wall clock** (no `--lockstep`, C5's `implementation.clock = wall_clock`): frames apply as they
  arrive and are answered at once; hello and every answer say the clock is not lockstep; seq must
  still count up by one.

### The X-Plane path (`--sim=xp`), kept for `w1-s7-inav-bridge`

machina's bridge card is written against this interface and backend.py's frozen `inav_xplane`
convention. Paced, it is unchanged. In lockstep (`--lockstep[=dt_us]`, default 1000) it now shares
the core above: the clock starts at 0 at process start and advances dt per packet; the receive blocks;
the first packet is a step like every other (the connection probe no longer consumes one uncounted,
so a bridge waits for `MOTR.rx == packets sent`); the reply follows the step. A bridge may number its
packets in drefs 42 and 43 (the low and high 24 bits, exact in float32): `MOTR` then echoes the number
(appended u64), and a gap or a repeat stops the firmware with status 3. `MOTR` also appends the
clock in ns. The parser in `machina/harness/frames.py` reads both the 89-byte probe layout and the
extended one.

**The bridge's form.** Machina's merged bridge (`w1-s7-inav-bridge`, its `patches/0002`) launches
`--sim=xp --simip=unix:<path> --lockstep` (paced: without `--lockstep`) and numbers every packet with
its tick: drefs 42 and 43 carry the number in two 16-bit words and 44 to 47 the tick in integer ns
in four. A run whose first packet carries a tick is in this form: packet k must be number k at a
tick after the last, and a departure sets the sequence flag (0x02) in this and every later reply
instead of stopping the firmware; the clock is set to each packet's tick (never back), and a delay
takes no virtual time, so init runs whole at the first packet's tick; the reply is `MCH1`
(`frames.MCH1`, 142 bytes: rx, the echoed number and tick, the clock, flags, motors, INAV's
attitude, position and velocity, its gyro and accelerometer, arming, mode, navigation and failsafe
state, the GNSS origin), alone in lockstep. The Unix socket (the bridge listens at `<path>`, the
firmware binds `<path>.fw`) also works for FW2's form. Paced, the connection probe only peeks, so
the first packet is applied and counted in both forms.

### HITL (`MSP2_MACHINA_SIMULATOR`, 0x2F01)

The firmware half of hardware-in-the-loop on a real board, on the wall clock (the board's clock and
scheduler rule). The payload (174 bytes, inside INAV's 192-byte MSP buffer) carries the same sensors
in float32, with the GNSS position in 1e-7 deg and cm; the answer is the output frame above with the
`HITL` flag and the board's clock. The estimator always runs (no attitude is sent). Frame k must carry
seq k: a gap or a repeat is refused, and the board leaves simulator mode and disarms, so a broken link
never flies on held sensors; the host starts again from seq 0. A frame without `HITL_ENABLE` ends HITL
the same way. In HITL INAV never writes its motor outputs (`fc_core.c`), so a board on the bench does
not spin a motor. Battery voltage and current go in at 0.01 V and 0.01 A (stock `MSP_SIMULATOR`
stops at 25.5 V). The board needs `receiver_type = SIM` for simulated RC. The checks run against this
fork's SITL binary in configurator mode (no `--sim`), whose UART1 is a TCP socket and whose clock is
the wall clock; the bench (machina's task FW5) runs them on a board.

## Build and test

Requirements: rootless Podman; for the harness, Python 3.11 or later with numpy, mujoco and pytest.

```bash
machina/build.sh sitl                       # $MACHINA_CACHE/inav-fork/inav-sitl (about a minute)
machina/build.sh unit                       # INAV's unit tests, machina_sim_unittest among them
cd machina && python -m pytest tests -q     # the fast checks (about 10 minutes on a loaded host)
python -m harness fly rth                   # one flight; see python -m harness --help
python -m harness campaign ls-rth --runs 20 --jobs 4 --scenario rth --hold 5
```

`MACHINA_CACHE` defaults to `~/.cache/machina`; `INAV_SITL` points the harness at another binary.
Every container runs with `--memory 6g --cpus 4 --oom-score-adj 1000` and a hard time limit.

The fast checks (`machina/tests/`):

| File | Checks |
|---|---|
| `test_frames.py` | Frame layouts and round trips, MSP v2 framing, integer-ns ticks without drift (rational rates), slow sensors new only at their ticks, the plant's sensors in C2, the X-Plane drefs against backend.py's `InavXplane` |
| `test_lockstep.py` | Hello; the clock from 0 at process start and t0 + clock at every tick (also at 2,333.33 Hz and a nonzero t0); floor ticks at 3 kHz, exactly 1 s after 3,000; no step while a packet is held back 2.5 s; gaps, repeats, a 1-ns tick error, a zero step and a malformed frame each stop the firmware with status 3; wall-clock mode declares itself; the X-Plane path's numbering, echo and gap |
| `test_sensors.py` | Through INAV's own sensor code: gyro 2,500 deg/s and 0.0004 deg/s, accelerometer 5 g and 20 g, the compass in body axes after INAV's calibration and alignment, the barometer rounded to Pa, GNSS exact in 1e-7 deg at a real home |
| `test_determinism.py` | Three identical-seed flights side by side are bit-identical (every output frame and the trajectory); another seed differs |
| `test_scenarios.py` | Hover (altitude within 0.2 m over 10 s), return-to-home (lands within 1 m), receiver loss enters INAV's failsafe, the X-Plane path hovers in lockstep |
| `test_hitl.py` | HITL frames drive the board on its own clock with the sequence echoed; a gap, a frame without enable and a malformed frame each end simulator mode |
| `test_bridge_form.py` | The bridge's form on its Unix socket: `MCH1` counts k + 1 packets, echoes number k and tick k, and the clock reads the tick, from reply 0 with four motors; a gap is flagged in that and every later reply; 65,600 packets pass 2^16 with no error; paced, the first packet is counted |

## The heading offset of the G0 probe

P1d saw INAV's heading 1.2 to 1.6 deg off the truth with its estimator running. It is INAV's own
estimator, not the interface: the yaw drift integrator (`ahrs_dcm_ki_mag`) winds up while the
estimate turns from its initial north to the true heading under the 10-times boot gains, and when
those end (at arming, or 20 s after boot) the stored bias shows as an offset that decays with the
integrator's slow time constant, about 34 s for the default gains. machina's evidence document gives
the measurements. A real board does the same after power-up; studies that compare headings should
start facing north or leave the firmware about two minutes before arming.

## Licence and the written source offer

INAV is free software under the GNU General Public License, version 3 or (at your option) any later
version (`LICENSE`); `src/main/target/SITL/sim/xplane.c` is also under the Mozilla Public License 2.0.
machina's changes in this fork, this folder included, are under the same terms.

machina builds this firmware from source on the user's or the organisation's own compute, so
normally nothing is conveyed. Whenever MachinaCad conveys a binary of this fork (in an image, a
bundle or on a board) it does so under GPL-3.0 section 6, with the Corresponding Source on the same
medium or this written offer:

> For at least three years from the date we conveyed the binary to you, and for as long as we offer
> spare parts or customer support for the product it came with, MachinaCad will give anyone who
> holds it the complete Corresponding Source of this INAV build, under the GNU GPL version 3, either
> on a durable physical medium for no more than our cost of making and sending it, or by download
> at no charge from the public repository of this fork. To ask for it, quote the binary's version
> string (`INAV 9.1.0 SITL (<git revision>)`) or its `machina/<version>` tag.

The public repository and the postal and email contact for requests are recorded when the fork is
published under MachinaCad (see "Before a tag").

## Tags, and how machina's bridges take one

A build is qualified for machina by a tag in this fork, `machina/<upstream tag>-m<n>`, for example
`machina/9.1.0-m1`, on a commit of `machina-9.1.0` whose evidence is in machina's
`docs/research/rig/firmware/inav-9.1.0.md`. A bridge pins the tag and the commit it points to:
`plugins/core_autopilot/inav/build.sh fork` clones the fork at the tag (instead of the G0 probe's
clone of upstream plus patches), checks the commit hash, and builds with `machina/build.sh sitl`.
The binary's version string carries the commit (`INAV 9.1.0 SITL (<hash>)`), and the hello frame
carries it again, so a bridge can refuse a binary that is not the tagged build.

## Before a tag

- P1's full criteria through `w1-s7-inav-bridge` once it merges: 1,000 runs per scenario, sequence
  integrity, exact sensor timing, and the pre-registered spread or bit-identical evidence.
- machina's backend.py names INAV's convention `inav_xplane` only; the bridge needs the native
  convention (machina's frames go through unchanged, `enu_flu`) to use `--sim=machina`.
- Publication under MachinaCad with CI (`machina/build.sh sitl` and `unit`, then the fast checks),
  and the contact for source requests.
- An ARM build of a real target (the HITL frame is compiled into every target with `USE_SIMULATOR`;
  only the SITL and unit-test builds are checked here) and the HITL checks on a board (FW5).
- RPM filtering on simulated eRPM (D3): the frames carry no eRPM yet.
