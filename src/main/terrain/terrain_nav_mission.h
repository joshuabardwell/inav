/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
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

#include <stdbool.h>
#include <stdint.h>

#include "drivers/time.h"

#include "terrain/terrain_nav.h"
#include "terrain/terrain_nav_hold_core.h"

/*
 * Terrain options of waypoint missions (fixed wing).
 *
 * A waypoint can ask for the leg flown toward it to keep a floor above the
 * terrain, or to follow the terrain at a set height (see
 * terrain_nav_mission_logic.h for the P3 bits and the rules). The decisions
 * are made by the TERRAIN AGL HOLD core, driven from the same gate as the
 * cruise hold (terrain_nav_hold.c); this file tracks the leg and supplies
 * the mission-specific inputs.
 *
 * Unlike the cruise hold, the gate does not write the altitude target here.
 * The waypoint state machine stays the only writer: it passes its own stock
 * target through terrainNavMissionAdjustTargetZ(). With terrain data unusable
 * the stock target comes straight back, so the mission flies as stock.
 *
 * The TERRAIN AGL HOLD mode switch plays no part: the waypoint's own option
 * is the opt-in, and terrain_enabled must be ON.
 */

// --- Called from the gate (terrain_nav_hold.c), once per navigation cycle ---

// Tracks the active waypoint. True while a leg with a terrain mode is flown
bool terrainNavMissionUpdate(timeMs_t currentTimeMs);

// True in the cycle the tracked leg changed (the hold restarts its capture)
bool terrainNavMissionLegChanged(void);

// Asks the map for the terrain height at the waypoint when the leg needs it
// and does not have it yet. True when a query was made this cycle
bool terrainNavMissionResolveWaypoint(timeMs_t currentTimeMs);

// The look-ahead along the mission route: from the course over ground, turning
// toward the waypoint as the aircraft will at this ground speed, then along the
// next leg when that is a terrain leg too, with the next leg's minimum
bool terrainNavMissionLookahead(float distanceM, float groundSpeedCmS, float climbRatio, float escapeRatio, terrainNavLookaheadResult_t *result);

// Fills the mission-specific inputs of the hold core
void terrainNavMissionApplyInput(terrainNavHoldInput_t *in);

// Takes what the hold core decided this cycle
void terrainNavMissionStoreOutput(const terrainNavHoldOutput_t *out);

// Bits for debug word 0 of debug_mode = TERRAIN_NAV: bit 3 terrain leg being
// flown, bits 4-5 its mode, bit 6 follow clearance known, bit 7 terrain
// height at the waypoint known
uint32_t terrainNavMissionDebugFlags(void);

// --- Called from the waypoint state machine (navigation.c) ---

// The altitude target for the leg in progress: stockZCm adjusted for the
// leg's terrain mode, or stockZCm itself on a stock leg or without data
float terrainNavMissionAdjustTargetZ(float stockZCm);

// The altitude the leg in progress ends at, for the "waypoint altitude
// reached" test and the loiter that waits for it (nav_wp_enforce_altitude)
float terrainNavMissionWaypointZ(float stockWpZCm);
