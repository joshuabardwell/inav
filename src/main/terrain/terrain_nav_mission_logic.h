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

/*
 * Terrain options of waypoint missions - the decisions, with no firmware
 * dependencies, so they can be unit tested on their own.
 *
 * A waypoint the aircraft flies to (WAYPOINT, POSHOLD_TIME) carries two
 * options for the leg flown TOWARD it, in P3 bits stock INAV leaves unused:
 *
 *   bits 6-12   minimum height above ground for the leg, whole metres;
 *               0 (or a value outside 50-120) = the global terrain_nav_min_agl
 *   bits 13-14  terrain mode: 0 off, 1 floor, 2 follow (3 reads as off)
 *
 * Bits 0-4 are stock (altitude datum, user actions). Bit 5 is left free for
 * upstream and bit 15 is the sign bit of the int16.
 *
 *   floor   the stock altitude path, except that the target never drops
 *           below (terrain + minimum + margin)
 *   follow  the target is terrain + the waypoint's own ground clearance,
 *           the clearance being what the stored altitude leaves above the
 *           terrain at the waypoint
 *
 * The Configurator mirrors these rules to predict the flown altitude
 * (js/mission_terrain.js) - change both together.
 */

#define TERRAIN_NAV_MISSION_P3_MIN_SHIFT    6
#define TERRAIN_NAV_MISSION_P3_MIN_MASK     (0x7F << TERRAIN_NAV_MISSION_P3_MIN_SHIFT)
#define TERRAIN_NAV_MISSION_P3_MODE_SHIFT   13
#define TERRAIN_NAV_MISSION_P3_MODE_MASK    (0x3 << TERRAIN_NAV_MISSION_P3_MODE_SHIFT)

// Limits of a per-waypoint minimum - the limits of terrain_nav_min_agl
#define TERRAIN_NAV_MISSION_MIN_AGL_LOWEST_M    50
#define TERRAIN_NAV_MISSION_MIN_AGL_HIGHEST_M   120

typedef enum {
    TERRAIN_NAV_MISSION_OFF = 0,
    TERRAIN_NAV_MISSION_FLOOR = 1,
    TERRAIN_NAV_MISSION_FOLLOW = 2,
} terrainNavMissionMode_e;

// actionHasOptions: the waypoint is one the aircraft flies to (WAYPOINT or
// POSHOLD_TIME). JUMP keeps its repeat counter in P3 and LAND descends on
// purpose, so their P3 is never read as terrain options
static inline terrainNavMissionMode_e terrainNavMissionModeFromP3(bool actionHasOptions, int16_t p3)
{
    if (!actionHasOptions) {
        return TERRAIN_NAV_MISSION_OFF;
    }

    const uint16_t mode = (((uint16_t)p3) & TERRAIN_NAV_MISSION_P3_MODE_MASK) >> TERRAIN_NAV_MISSION_P3_MODE_SHIFT;
    if (mode == TERRAIN_NAV_MISSION_FLOOR || mode == TERRAIN_NAV_MISSION_FOLLOW) {
        return (terrainNavMissionMode_e)mode;
    }
    return TERRAIN_NAV_MISSION_OFF;
}

// Minimum height above ground for the leg flown to a waypoint, in cm
static inline int32_t terrainNavMissionMinAglCm(bool actionHasOptions, int16_t p3, int32_t globalMinAglCm)
{
    if (!actionHasOptions) {
        return globalMinAglCm;
    }

    const int32_t ownM = (((uint16_t)p3) & TERRAIN_NAV_MISSION_P3_MIN_MASK) >> TERRAIN_NAV_MISSION_P3_MIN_SHIFT;
    if (ownM >= TERRAIN_NAV_MISSION_MIN_AGL_LOWEST_M && ownM <= TERRAIN_NAV_MISSION_MIN_AGL_HIGHEST_M) {
        return ownM * 100;
    }
    return globalMinAglCm;
}

// Ground clearance a follow leg holds: what the waypoint's stored altitude
// leaves above the terrain at the waypoint.
//  - sea-level altitude: altitude minus terrain height, both on the map
//    datum - independent of where the aircraft took off
//  - altitude relative to home: the waypoint's height in the local frame
//    minus the terrain rise from home to the waypoint
static inline int32_t terrainNavMissionLegClearanceCm(bool altIsAmsl, int32_t wpAltCm, float wpLocalZCm,
                                                      float wpTerrainM, float homeTerrainM)
{
    if (altIsAmsl) {
        return wpAltCm - (int32_t)(wpTerrainM * 100.0f);
    }
    return (int32_t)(wpLocalZCm - (wpTerrainM - homeTerrainM) * 100.0f);
}

// The height above ground the hold is asked to keep on a terrain leg. The
// margin keeps the aircraft off the edge of the alarm band, exactly as a
// cruise engagement below the floor does. A follow leg whose clearance is
// not known yet (terrain at the waypoint still loading) holds the floor
static inline int32_t terrainNavMissionCommandedAglCm(terrainNavMissionMode_e mode, bool clearanceValid,
                                                      int32_t clearanceCm, int32_t minAglCm, int32_t marginCm)
{
    const int32_t floorCm = minAglCm + marginCm;
    if (mode == TERRAIN_NAV_MISSION_FOLLOW && clearanceValid && clearanceCm > floorCm) {
        return clearanceCm;
    }
    return floorCm;
}

// The altitude target of a terrain leg, in the local frame.
//   holdActive     the hold is producing terrain targets this cycle
//   haveTerrainZ   terrainZCm holds a target (this cycle's, or the last one
//                  produced on this run of terrain legs)
//  - active follow leg with a known clearance: the terrain target
//  - active otherwise (floor, or follow still resolving): the higher of the
//    stock target and the terrain target
//  - not active (no data yet, data lost): the stock target, but never below
//    the last terrain target - never descend on dead data
static inline float terrainNavMissionResolveTargetZ(terrainNavMissionMode_e mode, bool followResolved, bool holdActive,
                                                    bool haveTerrainZ, float terrainZCm, float stockZCm)
{
    if (mode == TERRAIN_NAV_MISSION_OFF || !haveTerrainZ) {
        return stockZCm;
    }
    if (holdActive && mode == TERRAIN_NAV_MISSION_FOLLOW && followResolved) {
        return terrainZCm;
    }
    return (terrainZCm > stockZCm) ? terrainZCm : stockZCm;
}

// The altitude a terrain leg ends at, in the local frame: what "waypoint
// altitude reached" and the loiter at the waypoint have to use in place of
// the stored altitude, which the leg may not be allowed to descend to.
//   wpRiseCm   terrain rise from home to the waypoint
static inline float terrainNavMissionLegEndZ(terrainNavMissionMode_e mode, bool wpTerrainValid, float wpRiseCm,
                                             int32_t commandedAglCm, float stockWpZCm)
{
    if (mode == TERRAIN_NAV_MISSION_OFF || !wpTerrainValid) {
        return stockWpZCm;
    }

    const float terrainWpZCm = wpRiseCm + (float)commandedAglCm;
    if (mode == TERRAIN_NAV_MISSION_FOLLOW) {
        return terrainWpZCm;
    }
    return (terrainWpZCm > stockWpZCm) ? terrainWpZCm : stockWpZCm;
}

// Extra rise the route look-ahead adds to terrain past the next waypoint,
// where the next leg's minimum applies. Where that minimum is higher, the
// height that leg will hold (minimum + margin) is the aim, so the aircraft
// arrives at the waypoint already above the new minimum
static inline float terrainNavMissionTurnExtraRiseM(int32_t minAglCm, int32_t nextMinAglCm, int32_t marginCm)
{
    float extraCm = (float)(nextMinAglCm - minAglCm);
    if (nextMinAglCm > minAglCm) {
        extraCm += (float)marginCm;
    }
    return extraCm * 0.01f;
}
