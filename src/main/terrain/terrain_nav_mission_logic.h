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

#include <math.h>
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

/*
 * The band the mission look-ahead scans (terrainNavLookaheadRoute).
 *
 * A lane is one path the aircraft may fly. It starts at the aircraft on its course over
 * ground and turns toward the active waypoint no faster than the lane's turn rate. Where the
 * navigation controller will switch to the next waypoint (the fly-by lead before the active
 * one) the lane switches too, and turns toward the next waypoint the same way. So the turn
 * at a waypoint is scanned before the aircraft gets there, and the turn it is still flying
 * after the switch is scanned where it will be flown.
 *
 * Lane 0 turns at the bank the navigation controller plans its turns for. The last lane
 * turns at half that bank, the lanes between at even steps of curvature: an aircraft that
 * banks less than planned (the energy guard, a climb at full throttle, wind) flies a wider
 * turn, somewhere inside the band. Between neighbouring lanes the band is sampled across
 * its width, every step, so ground between two lanes counts as well. While the lanes
 * coincide - a straight leg flown on course - the band is lane 0 alone.
 *
 * The scan takes the highest ground of the whole band. Each lane counts on its own
 * achievable climb (less while it turns); a point between two lanes counts on the lower.
 *
 * Budgets of one scan: lane 0 keeps the limits of the straight scan (distance, sample
 * count). The rest of the band adds at most TERRAIN_NAV_BAND_MAX_SAMPLES samples, nearest
 * steps first, and may not bring the blocks touched by the scan above the block budget of
 * a query cycle: a band is wider than a line, so the "one block per 540 m" bound of a path
 * does not hold for it and the blocks are counted instead.
 */

// The scan's step: TERRAIN_NAV_STEP_M of terrain_nav.c, the grid spacing
#define TERRAIN_NAV_MISSION_PATH_STEP_M         30.0f
// Below one step the turn is as tight as the scan can draw it
#define TERRAIN_NAV_MISSION_TURN_RADIUS_MIN_M   30.0f
// A step whose course still differs from the wanted one by more than this is flown banked
#define TERRAIN_NAV_MISSION_TURNING_DEG         10.0f
// While banked in a turn the aircraft climbs worse: the achievable climb of a turning
// step is this fraction of the usual (HITL 2026-11-23: about 5 m/s in a 55 degree bank,
// 11-16 m/s once level)
#define TERRAIN_NAV_MISSION_TURN_CLIMB_FACTOR   0.5f
// The fly-by lead is R tan(turn / 2) with the tangent capped (NAV_FW_TURN_LEAD_TAN_MAX),
// for turns between these angles (navigation_fixedwing.c)
#define TERRAIN_NAV_MISSION_LEAD_TAN_MAX        3.7f
#define TERRAIN_NAV_MISSION_FLYBY_MIN_DEG       30.0f
#define TERRAIN_NAV_MISSION_FLYBY_MAX_DEG       160.0f
// The controller plans no turn wider than this (NAV_FW_TURN_RADIUS_MAX)
#define TERRAIN_NAV_MISSION_PLAN_RADIUS_MAX_M   300.0f

// Lanes of the band. Sized for a corridor on straight steps to add lanes later
#define TERRAIN_NAV_BAND_MAX_LANES              3
// Samples one scan may add beside lane 0
#define TERRAIN_NAV_BAND_MAX_SAMPLES            64
// A lane closer than this to its neighbour adds nothing to it
#define TERRAIN_NAV_BAND_LANE_MERGE_M           15.0f
// Blocks one scan can keep count of (the largest block cache)
#define TERRAIN_NAV_BLOCK_SET_MAX               8

// Radius of a turn, in metres: v^2 / (g tan(bank)) at the ground speed
static inline float terrainNavMissionTurnRadiusM(float groundSpeedCmS, float bankDeg)
{
    const float speedMS = groundSpeedCmS * 0.01f;
    const float tanBank = tanf(bankDeg * 0.017453293f);
    if (!(tanBank > 0.01f)) {
        return 1.0e6f;      // no usable bank: as good as straight
    }
    const float radiusM = speedMS * speedMS / (9.80665f * tanBank);
    return (radiusM > TERRAIN_NAV_MISSION_TURN_RADIUS_MIN_M) ? radiusM : TERRAIN_NAV_MISSION_TURN_RADIUS_MIN_M;
}

static inline float terrainNavMissionWrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg <= -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

// Most the scan's course may change in one step for a turn of radiusM
static inline float terrainNavMissionMaxTurnPerStepDeg(float radiusM)
{
    return TERRAIN_NAV_MISSION_PATH_STEP_M / radiusM * 57.29578f;
}

// How far before the active waypoint the navigation controller switches to the next one.
//   flyBy          nav_fw_wp_turn_mode is COORD_FLYBY and the waypoint is not a hold
//   turnAngleDeg   change of course at the waypoint
//   radiusM        the turn radius at the controller's planning bank
// FLY_BY starts the turn R tan(turn / 2) early, never more than maxLeadMs of flight (the
// controller adds a roll-in lead of a few tens of metres, left out here: the band is wider
// than that). Every other case switches at the waypoint radius
static inline float terrainNavMissionSwitchDistM(bool flyBy, float turnAngleDeg, float radiusM, float groundSpeedCmS,
                                                 float maxLeadMs, float wpRadiusM)
{
    float distM = wpRadiusM;
    const float angleDeg = fabsf(turnAngleDeg);
    if (flyBy && angleDeg > TERRAIN_NAV_MISSION_FLYBY_MIN_DEG && angleDeg < TERRAIN_NAV_MISSION_FLYBY_MAX_DEG) {
        float halfTan = tanf(angleDeg * 0.5f * 0.017453293f);
        if (halfTan > TERRAIN_NAV_MISSION_LEAD_TAN_MAX) {
            halfTan = TERRAIN_NAV_MISSION_LEAD_TAN_MAX;
        }
        float leadM = ((radiusM < TERRAIN_NAV_MISSION_PLAN_RADIUS_MAX_M) ? radiusM : TERRAIN_NAV_MISSION_PLAN_RADIUS_MAX_M) * halfTan;
        const float maxLeadM = groundSpeedCmS * 0.01f * maxLeadMs * 0.001f;
        if (leadM > maxLeadM) {
            leadM = maxLeadM;
        }
        if (leadM > distM) {
            distM = leadM;
        }
    }
    return distM;
}

typedef struct {
    float wpNorthM;             // the active waypoint, from where the scan starts
    float wpEastM;
    float switchDistM;          // the lanes switch to the next waypoint this far before the active one
    bool nextLegValid;          // the route continues past the waypoint
    float nextWpNorthM;         // the next waypoint, from where the scan starts
    float nextWpEastM;
} terrainNavMissionPath_t;

typedef struct {
    float northM;               // the last sample, from where the scan starts
    float eastM;
    float courseDeg;
    bool pastWaypoint;          // flying to the next waypoint
    bool turning;               // the last step was flown banked
    float wantedTurnDeg;        // course change the last step asked for, before the lane's limit (>= 0)
} terrainNavMissionPathState_t;

static inline void terrainNavMissionPathStart(terrainNavMissionPathState_t *state, float courseDeg)
{
    state->northM = 0.0f;
    state->eastM = 0.0f;
    state->courseDeg = courseDeg;
    state->pastWaypoint = false;
    state->turning = false;
    state->wantedTurnDeg = 0.0f;
}

// Moves a lane one step on, its course changing by at most maxTurnDeg. Returns false,
// without moving, when the lane ends: at the active waypoint when no leg follows, else at
// the next waypoint (the scan knows the route no further). *stepNorthM, *stepEastM: the move
static inline bool terrainNavMissionPathStep(terrainNavMissionPathState_t *state, const terrainNavMissionPath_t *path,
                                             float maxTurnDeg, float *stepNorthM, float *stepEastM)
{
    const float stepSq = TERRAIN_NAV_MISSION_PATH_STEP_M * TERRAIN_NAV_MISSION_PATH_STEP_M;

    state->turning = false;

    if (!state->pastWaypoint) {
        const float toWpNorthM = path->wpNorthM - state->northM;
        const float toWpEastM = path->wpEastM - state->eastM;
        // Without a leg to follow the lane runs up to the waypoint itself
        const float reachM = (path->nextLegValid && path->switchDistM > TERRAIN_NAV_MISSION_PATH_STEP_M)
            ? path->switchDistM : TERRAIN_NAV_MISSION_PATH_STEP_M;
        if (toWpNorthM * toWpNorthM + toWpEastM * toWpEastM < reachM * reachM) {
            if (!path->nextLegValid) {
                return false;
            }
            state->pastWaypoint = true;
        }
    }

    const float toNorthM = (state->pastWaypoint ? path->nextWpNorthM : path->wpNorthM) - state->northM;
    const float toEastM = (state->pastWaypoint ? path->nextWpEastM : path->wpEastM) - state->eastM;
    if (state->pastWaypoint && toNorthM * toNorthM + toEastM * toEastM < stepSq) {
        return false;
    }

    const float wantDeg = atan2f(toEastM, toNorthM) * 57.29578f;
    const float errorDeg = terrainNavMissionWrap180(wantDeg - state->courseDeg);
    float turnDeg = errorDeg;
    if (turnDeg > maxTurnDeg) {
        turnDeg = maxTurnDeg;
    } else if (turnDeg < -maxTurnDeg) {
        turnDeg = -maxTurnDeg;
    }
    state->wantedTurnDeg = fabsf(errorDeg);
    state->turning = state->wantedTurnDeg > TERRAIN_NAV_MISSION_TURNING_DEG;
    state->courseDeg = terrainNavMissionWrap180(state->courseDeg + turnDeg);

    const float courseRad = state->courseDeg * 0.017453293f;
    *stepNorthM = cosf(courseRad) * TERRAIN_NAV_MISSION_PATH_STEP_M;
    *stepEastM = sinf(courseRad) * TERRAIN_NAV_MISSION_PATH_STEP_M;
    state->northM += *stepNorthM;
    state->eastM += *stepEastM;
    return true;
}

typedef struct {
    uint8_t laneCount;
    float maxTurnDeg[TERRAIN_NAV_BAND_MAX_LANES];   // per step; lane 0 is the tightest
} terrainNavBand_t;

// The band of turns for the bank the navigation controller plans with: lane 0 at that
// bank, the last lane at half of it, even steps of curvature between
static inline void terrainNavBandFromBank(float groundSpeedCmS, float bankDeg, terrainNavBand_t *band)
{
    const float tightDeg = terrainNavMissionMaxTurnPerStepDeg(terrainNavMissionTurnRadiusM(groundSpeedCmS, bankDeg));
    const float wideDeg = terrainNavMissionMaxTurnPerStepDeg(terrainNavMissionTurnRadiusM(groundSpeedCmS, bankDeg * 0.5f));

    band->laneCount = TERRAIN_NAV_BAND_MAX_LANES;
    for (int lane = 0; lane < TERRAIN_NAV_BAND_MAX_LANES; lane++) {
        band->maxTurnDeg[lane] = tightDeg + (wideDeg - tightDeg) * (float)lane / (float)(TERRAIN_NAV_BAND_MAX_LANES - 1);
    }
}

// The cache blocks one scan has touched
typedef struct {
    uint8_t count;
    int32_t lat[TERRAIN_NAV_BLOCK_SET_MAX];
    int32_t lon[TERRAIN_NAV_BLOCK_SET_MAX];
} terrainNavBlockSet_t;

// True when the block is in the set: it was already, or it was added because the set
// held fewer than budget blocks
static inline bool terrainNavBlockSetAdd(terrainNavBlockSet_t *set, int32_t lat, int32_t lon, uint8_t budget)
{
    for (uint8_t i = 0; i < set->count; i++) {
        if (set->lat[i] == lat && set->lon[i] == lon) {
            return true;
        }
    }
    if (set->count >= budget || set->count >= TERRAIN_NAV_BLOCK_SET_MAX) {
        return false;
    }
    set->lat[set->count] = lat;
    set->lon[set->count] = lon;
    set->count++;
    return true;
}

// How the walk reads the terrain. Points are metres north and east of the scan's start
typedef struct {
    void *ctx;
    // The cache block that holds the point, as two numbers that identify it
    void (*block)(void *ctx, float northM, float eastM, int32_t *blockLat, int32_t *blockLon);
    // Terrain height at the point block() was last called for; false when there is no
    // data (the read gets scheduled)
    bool (*height)(void *ctx, float *heightM);
} terrainNavBandTerrain_t;

typedef struct {
    float climbNeededM;         // worst deficit against the planned climb (>= 0)
    float escapeDeficitM;       // worst deficit against the escape slope (>= 0)
    uint8_t samplesTotal;       // lane 0: points sampled
    uint8_t samplesMissed;      // lane 0: points without data
    uint8_t bandSamples;        // points sampled beside lane 0
    uint8_t bandMissed;         // of those, points without data
    uint16_t bandSkipped;       // band points left out: over the sample or the block budget
} terrainNavBandResult_t;

typedef struct {
    const terrainNavBandTerrain_t *terrain;
    float baseHeightM;
    terrainNavBlockSet_t blocks;
    terrainNavBandResult_t *result;
} terrainNavBandWalk_t;

// One sample: riseExtraM is added to the terrain rise, climbM and escapeM are the heights
// the aircraft can have gained by then on the two slopes
static inline void terrainNavBandSample(terrainNavBandWalk_t *walk, float northM, float eastM, bool lane0, uint8_t blockBudget,
                                        float riseExtraM, float climbM, float escapeM)
{
    terrainNavBandResult_t *result = walk->result;
    int32_t blockLat;
    int32_t blockLon;

    if (!lane0 && result->bandSamples >= TERRAIN_NAV_BAND_MAX_SAMPLES) {
        result->bandSkipped++;
        return;
    }
    walk->terrain->block(walk->terrain->ctx, northM, eastM, &blockLat, &blockLon);
    if (lane0) {
        // Lane 0 is bounded by its length, as the straight scan is: always sampled
        terrainNavBlockSetAdd(&walk->blocks, blockLat, blockLon, TERRAIN_NAV_BLOCK_SET_MAX);
        result->samplesTotal++;
    } else {
        if (!terrainNavBlockSetAdd(&walk->blocks, blockLat, blockLon, blockBudget)) {
            result->bandSkipped++;
            return;
        }
        result->bandSamples++;
    }

    float heightM;
    if (!walk->terrain->height(walk->terrain->ctx, &heightM)) {
        if (lane0) {
            result->samplesMissed++;
        } else {
            result->bandMissed++;
        }
        return;
    }

    const float riseM = heightM - walk->baseHeightM + riseExtraM;
    if (riseM - climbM > result->climbNeededM) {
        result->climbNeededM = riseM - climbM;
    }
    if (riseM - escapeM > result->escapeDeficitM) {
        result->escapeDeficitM = riseM - escapeM;
    }
}

// The scan. Walks lane 0 first, exactly as a single path, so its blocks are counted before
// the rest of the band may claim any; then walks all lanes together and samples the other
// lanes and the points between neighbours.
//   distanceM, maxLaneSamples   limits of lane 0 (and so of the band's length)
//   climbRatio, escapeRatio     metres of climb per metre flown on the two slopes
//   nextExtraRiseM              added to the terrain rise on the way to the next waypoint
//   baseHeightM                 terrain height at the start
//   blockBudget                 most blocks the scan may touch, the start's block included
static inline void terrainNavBandWalk(const terrainNavMissionPath_t *path, const terrainNavBand_t *band, float courseDeg,
                                      float distanceM, uint8_t maxLaneSamples, float climbRatio, float escapeRatio,
                                      float nextExtraRiseM, float baseHeightM, uint8_t blockBudget,
                                      const terrainNavBandTerrain_t *terrain, terrainNavBandResult_t *result)
{
    const float climbPerStepM = climbRatio * TERRAIN_NAV_MISSION_PATH_STEP_M;
    const float escapePerStepM = escapeRatio * TERRAIN_NAV_MISSION_PATH_STEP_M;
    float stepNorthM;
    float stepEastM;

    result->climbNeededM = 0.0f;
    result->escapeDeficitM = 0.0f;
    result->samplesTotal = 0;
    result->samplesMissed = 0;
    result->bandSamples = 0;
    result->bandMissed = 0;
    result->bandSkipped = 0;

    terrainNavBandWalk_t walk = { .terrain = terrain, .baseHeightM = baseHeightM, .result = result };
    walk.blocks.count = 0;

    // The block under the aircraft is in use by the height-above-ground query
    int32_t blockLat;
    int32_t blockLon;
    terrain->block(terrain->ctx, 0.0f, 0.0f, &blockLat, &blockLon);
    terrainNavBlockSetAdd(&walk.blocks, blockLat, blockLon, TERRAIN_NAV_BLOCK_SET_MAX);

    // Lane 0
    terrainNavMissionPathState_t lane0;
    terrainNavMissionPathStart(&lane0, courseDeg);
    float climbM = 0.0f;
    float escapeM = 0.0f;
    float sharpestTurnDeg = 0.0f;
    for (float travelledM = TERRAIN_NAV_MISSION_PATH_STEP_M; travelledM <= distanceM; travelledM += TERRAIN_NAV_MISSION_PATH_STEP_M) {
        if (result->samplesTotal >= maxLaneSamples
            || !terrainNavMissionPathStep(&lane0, path, band->maxTurnDeg[0], &stepNorthM, &stepEastM)) {
            break;
        }
        if (lane0.wantedTurnDeg > sharpestTurnDeg) {
            sharpestTurnDeg = lane0.wantedTurnDeg;
        }
        climbM += lane0.turning ? climbPerStepM * TERRAIN_NAV_MISSION_TURN_CLIMB_FACTOR : climbPerStepM;
        escapeM += escapePerStepM;
        terrainNavBandSample(&walk, lane0.northM, lane0.eastM, true, blockBudget,
                             lane0.pastWaypoint ? nextExtraRiseM : 0.0f, climbM, escapeM);
    }

    if (band->laneCount < 2) {
        return;
    }

    // The rest of the band
    const int laneCount = (band->laneCount < TERRAIN_NAV_BAND_MAX_LANES) ? band->laneCount : TERRAIN_NAV_BAND_MAX_LANES;
    // Lane 0 never wanted to turn more in a step than the widest lane can: every lane
    // flies the same path, there is no band to scan (a leg flown on course)
    if (sharpestTurnDeg <= band->maxTurnDeg[laneCount - 1]) {
        return;
    }
    terrainNavMissionPathState_t lanes[TERRAIN_NAV_BAND_MAX_LANES];
    bool alive[TERRAIN_NAV_BAND_MAX_LANES];
    float laneClimbM[TERRAIN_NAV_BAND_MAX_LANES];
    for (int lane = 0; lane < laneCount; lane++) {
        terrainNavMissionPathStart(&lanes[lane], courseDeg);
        alive[lane] = true;
        laneClimbM[lane] = 0.0f;
    }

    escapeM = 0.0f;
    uint8_t steps = 0;
    for (float travelledM = TERRAIN_NAV_MISSION_PATH_STEP_M; travelledM <= distanceM; travelledM += TERRAIN_NAV_MISSION_PATH_STEP_M) {
        if (steps >= maxLaneSamples) {
            break;
        }
        if (result->bandSamples >= TERRAIN_NAV_BAND_MAX_SAMPLES) {
            result->bandSkipped++;      // the far steps of the band stay unscanned this cycle
            break;
        }
        steps++;

        bool anyAlive = false;
        for (int lane = 0; lane < laneCount; lane++) {
            if (alive[lane] && !terrainNavMissionPathStep(&lanes[lane], path, band->maxTurnDeg[lane], &stepNorthM, &stepEastM)) {
                alive[lane] = false;
            }
            if (alive[lane]) {
                anyAlive = true;
                laneClimbM[lane] += lanes[lane].turning ? climbPerStepM * TERRAIN_NAV_MISSION_TURN_CLIMB_FACTOR : climbPerStepM;
            }
        }
        if (!anyAlive) {
            break;
        }
        escapeM += escapePerStepM;

        // The lanes themselves first, then the points between them: when the sample
        // budget runs out it is the cross-fill of the far steps that goes
        for (int lane = 1; lane < laneCount; lane++) {
            if (!alive[lane]) {
                continue;
            }
            if (alive[lane - 1]) {
                const float dNorthM = lanes[lane].northM - lanes[lane - 1].northM;
                const float dEastM = lanes[lane].eastM - lanes[lane - 1].eastM;
                if (dNorthM * dNorthM + dEastM * dEastM < TERRAIN_NAV_BAND_LANE_MERGE_M * TERRAIN_NAV_BAND_LANE_MERGE_M) {
                    continue;
                }
            }
            terrainNavBandSample(&walk, lanes[lane].northM, lanes[lane].eastM, false, blockBudget,
                                 lanes[lane].pastWaypoint ? nextExtraRiseM : 0.0f, laneClimbM[lane], escapeM);
        }
        for (int lane = 1; lane < laneCount; lane++) {
            if (!alive[lane] || !alive[lane - 1]) {
                continue;
            }
            const float dNorthM = lanes[lane].northM - lanes[lane - 1].northM;
            const float dEastM = lanes[lane].eastM - lanes[lane - 1].eastM;
            const int pieces = (int)ceilf(sqrtf(dNorthM * dNorthM + dEastM * dEastM) / TERRAIN_NAV_MISSION_PATH_STEP_M);
            if (pieces < 2) {
                continue;
            }
            // Between two lanes: the lower climb, the higher extra rise
            const float extraA = lanes[lane - 1].pastWaypoint ? nextExtraRiseM : 0.0f;
            const float extraB = lanes[lane].pastWaypoint ? nextExtraRiseM : 0.0f;
            const float extraM = (extraA > extraB) ? extraA : extraB;
            const float betweenClimbM = (laneClimbM[lane - 1] < laneClimbM[lane]) ? laneClimbM[lane - 1] : laneClimbM[lane];
            for (int piece = 1; piece < pieces; piece++) {
                const float f = (float)piece / (float)pieces;
                terrainNavBandSample(&walk, lanes[lane - 1].northM + dNorthM * f, lanes[lane - 1].eastM + dEastM * f, false,
                                     blockBudget, extraM, betweenClimbM, escapeM);
            }
        }
    }
}
