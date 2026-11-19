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

#pragma once

#include "fc/machina_sim.h"

// Fill an output frame from the firmware's state (motors, servos, attitude, estimate, flags
// and the sensors as INAV holds them); the caller sets flags, seq, tNs, clockNs and applied.
void machinaSimFillOutputs(machinaSimOutputFrame_t *o);

#ifdef USE_SIMULATOR
// MSP2_MACHINA_SIMULATOR (HITL, wall clock): apply one HITL frame of `len` bytes and write the
// output frame into `reply` (sizeof(machinaSimOutputFrame_t) bytes).
void machinaSimHitlProcess(const uint8_t *payload, int len, uint8_t *reply);
#endif
