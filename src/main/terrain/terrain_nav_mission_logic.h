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
// where the next leg's minimum applies: the difference of the two minimums.
// The look-ahead aims at minimum + margin on either leg (the hold core adds
// the margin, lookaheadMarginCm), the height a floor leg holds, so an
// aircraft that follows the climb line exactly still clears every point by
// the margin
static inline float terrainNavMissionTurnExtraRiseM(int32_t minAglCm, int32_t nextMinAglCm)
{
    return (float)(nextMinAglCm - minAglCm) * 0.01f;
}

/*
 * The altitude target a terrain leg hands to the altitude controller.
 *
 * The fixed-wing altitude controller commands a climb rate in proportion to the altitude
 * error: rate = response * error / 100 (nav_fw_alt_control_response). An aircraft exactly
 * on a rising target is asked for no climb, so it settles under the target by
 * rate * 100 / response: 25 m at 10 m/s with a response of 40. The terrain target (the
 * height the leg holds, or the look-ahead's climb line) is therefore raised by a lead:
 *
 *   planned-climb lead   while the look-ahead asks for a climb: the lag of the climb rate
 *                        the look-ahead plans with, so that on the climb line the aircraft
 *                        is commanded that rate. Fades in over the first metres the scan
 *                        asks for
 *   measured lead        the lag of the rate the terrain target itself rose at over the
 *                        last second: a steady slope under a follow leg, and the last
 *                        second before a ridge, where the held height takes over from the
 *                        climb line and rises faster than the plan
 *
 * The commanded descent is limited to the planned climb rate: the look-ahead plans every
 * climb from level flight, and an aircraft diving at the controller's full rate meets the
 * next rise later than planned.
 *
 * On the first cycles of a new leg the leg has no answers of its own (its look-ahead scan,
 * a follow leg's clearance). Until it has, the target of the leg before stands as a floor
 * under the new one (the carry).
 */

// The planned-climb lead is full once the look-ahead asks for this much climb
#define TERRAIN_NAV_MISSION_LEAD_FADE_CM        1000.0f
// The measured lead: the rise over this many samples of the terrain target (one a query
// cycle, 100 ms)
#define TERRAIN_NAV_MISSION_RISE_SAMPLES        10
// Samples further apart than this do not belong to one rise
#define TERRAIN_NAV_MISSION_RISE_GAP_MS         500
// The carry ends by itself after this long (no look-ahead: too slow, switched off)
#define TERRAIN_NAV_MISSION_CARRY_MAX_MS        2000

// How far under a target rising at rateCmS the altitude controller lets the aircraft settle
static inline float terrainNavMissionLagCm(float rateCmS, float responseFactor)
{
    return rateCmS * 100.0f / ((responseFactor > 1.0f) ? responseFactor : 1.0f);
}

// The climb a scan asks for is measured from the ground under the aircraft where the scan
// started. The hold core adds it to the ground under the aircraft now, which is other
// ground a moment later (and, between two terrain queries, the same ground read against a
// newer altitude). This returns the climb to hand the core so that its line stays where the
// scan put it: scan ground + minimum + margin + climb, until the next scan's answer
static inline float terrainNavMissionRebaseClimbCm(float climbCm, float scanGroundZCm, float groundZCm)
{
    const float rebasedCm = climbCm + scanGroundZCm - groundZCm;
    return (rebasedCm > 0.0f) ? rebasedCm : 0.0f;
}

// The look-ahead's climb line, as the hold core computes it: ground under the aircraft +
// the minimum + the margin + the climb the scan asks for
static inline float terrainNavMissionClimbLineZ(float currentZCm, float aglCm, float minAglCm, float marginCm, float lookaheadClimbCm)
{
    return currentZCm - aglCm + minAglCm + marginCm + lookaheadClimbCm;
}

static inline float terrainNavMissionPlannedLeadCm(float lookaheadClimbCm, float plannedClimbCmS, float responseFactor)
{
    float fade = lookaheadClimbCm / TERRAIN_NAV_MISSION_LEAD_FADE_CM;
    if (fade > 1.0f) {
        fade = 1.0f;
    } else if (!(fade > 0.0f)) {
        return 0.0f;
    }
    return fade * terrainNavMissionLagCm(plannedClimbCmS, responseFactor);
}

// The rise of the terrain target over the last samples
typedef struct {
    bool havePrev;
    float prevZCm;
    uint32_t prevMs;
    uint8_t count;
    uint8_t next;
    float riseCm[TERRAIN_NAV_MISSION_RISE_SAMPLES];
    uint16_t spanMs[TERRAIN_NAV_MISSION_RISE_SAMPLES];
} terrainNavMissionRise_t;

static inline void terrainNavMissionRiseReset(terrainNavMissionRise_t *rise)
{
    rise->havePrev = false;
    rise->count = 0;
    rise->next = 0;
}

// One sample of the terrain target. comparable: the target was computed as the sample
// before was - same leg, same held height, same minimum, not under the carry. A sample that
// is not comparable is no rise of the terrain: nothing is recorded, the rate measured so
// far stands, and the next sample is measured from this one
static inline void terrainNavMissionRiseAdd(terrainNavMissionRise_t *rise, uint32_t nowMs, float targetZCm, bool comparable)
{
    if (rise->havePrev && nowMs - rise->prevMs > TERRAIN_NAV_MISSION_RISE_GAP_MS) {
        terrainNavMissionRiseReset(rise);
    }
    if (rise->havePrev && comparable && nowMs != rise->prevMs) {
        rise->riseCm[rise->next] = targetZCm - rise->prevZCm;
        rise->spanMs[rise->next] = (uint16_t)(nowMs - rise->prevMs);
        rise->next = (uint8_t)((rise->next + 1) % TERRAIN_NAV_MISSION_RISE_SAMPLES);
        if (rise->count < TERRAIN_NAV_MISSION_RISE_SAMPLES) {
            rise->count++;
        }
    }
    rise->havePrev = true;
    rise->prevZCm = targetZCm;
    rise->prevMs = nowMs;
}

// Rate the terrain target rose at over the samples held, cm/s (negative: it fell)
static inline float terrainNavMissionRiseRateCmS(const terrainNavMissionRise_t *rise)
{
    float riseCm = 0.0f;
    uint32_t spanMs = 0;
    for (uint8_t i = 0; i < rise->count; i++) {
        riseCm += rise->riseCm[i];
        spanMs += rise->spanMs[i];
    }
    return (spanMs > 0) ? riseCm * 1000.0f / (float)spanMs : 0.0f;
}

// Only a rise leads, and no faster than the altitude controller may command a climb
static inline float terrainNavMissionMeasuredLeadCm(float riseRateCmS, float maxClimbCmS, float responseFactor)
{
    if (!(riseRateCmS > 0.0f)) {
        return 0.0f;
    }
    return terrainNavMissionLagCm((riseRateCmS < maxClimbCmS) ? riseRateCmS : maxClimbCmS, responseFactor);
}

// The terrain target with its leads.
//   holdTargetZCm   the hold core's target: the higher of the held height and the climb line
//   climbing        the look-ahead asks for a climb (lineZCm is the climb line)
static inline float terrainNavMissionLeadTargetZ(float holdTargetZCm, bool climbing, float lineZCm,
                                                 float plannedLeadCm, float measuredLeadCm)
{
    float targetZCm = holdTargetZCm + measuredLeadCm;
    if (climbing && lineZCm + plannedLeadCm > targetZCm) {
        targetZCm = lineZCm + plannedLeadCm;
    }
    return targetZCm;
}

// The target no lower than the altitude controller answers with a descent of maxDescentCmS
static inline float terrainNavMissionLimitDescentZ(float targetZCm, float currentZCm, float maxDescentCmS, float responseFactor)
{
    const float lowestZCm = currentZCm - terrainNavMissionLagCm(maxDescentCmS, responseFactor);
    return (targetZCm > lowestZCm) ? targetZCm : lowestZCm;
}

// True when the target of the leg before no longer has to stand under the new leg's: the
// leg's own look-ahead has answered and, on a follow leg, its clearance is known
static inline bool terrainNavMissionCarryEnds(terrainNavMissionMode_e mode, bool clearanceValid, bool lookaheadValid, uint32_t elapsedMs)
{
    if (elapsedMs >= TERRAIN_NAV_MISSION_CARRY_MAX_MS) {
        return true;
    }
    return lookaheadValid && (mode != TERRAIN_NAV_MISSION_FOLLOW || clearanceValid);
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
 *
 * A scan can be run in pieces (terrainNavBandScanStart, terrainNavBandScanRun): the
 * firmware does a fixed amount of it in each cycle of its control loop, because a whole
 * scan takes longer than one cycle may. The scan works on what it was started with, and
 * samples the same points in the same order however it is cut up.
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

// Work one run of the scan may do (terrainNavBandScanRun): a sampled point counts
// TERRAIN_NAV_BAND_WORK_SAMPLE, a lane moved one step TERRAIN_NAV_BAND_WORK_STEP. A point
// costs about twice a step (two grid lookups against one atan2f, cosf, sinf)
#define TERRAIN_NAV_BAND_WORK_SAMPLE            2
#define TERRAIN_NAV_BAND_WORK_STEP              1

// Where a scan stands between two runs
typedef enum {
    TERRAIN_NAV_BAND_SCAN_LANE0 = 0,    // walking lane 0 alone
    TERRAIN_NAV_BAND_SCAN_BAND_STEP,    // the band: about to move the lanes one step
    TERRAIN_NAV_BAND_SCAN_BAND_LANES,   // the band: sampling the lanes of this step
    TERRAIN_NAV_BAND_SCAN_BAND_FILL,    // the band: sampling between neighbouring lanes
    TERRAIN_NAV_BAND_SCAN_DONE,
} terrainNavBandScanPhase_e;

// One scan: what it was started with, where it stands and what it has found. The scan can
// be run in pieces (terrainNavBandScanRun), so that no single call does the whole of it;
// the points it samples and their order do not depend on how it is cut up
typedef struct {
    // What the scan was started with
    terrainNavMissionPath_t path;
    terrainNavBand_t band;
    terrainNavBandTerrain_t terrain;
    float courseDeg;
    float distanceM;
    float climbPerStepM;
    float escapePerStepM;
    float nextExtraRiseM;
    float baseHeightM;
    uint8_t maxLaneSamples;
    uint8_t blockBudget;

    // Where it stands
    uint8_t phase;                      // terrainNavBandScanPhase_e
    uint8_t laneCount;
    uint8_t steps;                      // band steps begun
    uint8_t lane;                       // the lane the band phase is at
    uint8_t piece;                      // the point between lane - 1 and lane it is at (0: none begun)
    uint8_t pieces;
    float travelledM;
    float climbM;                       // lane 0
    float escapeM;
    float sharpestTurnDeg;
    terrainNavMissionPathState_t lane0;
    terrainNavMissionPathState_t lanes[TERRAIN_NAV_BAND_MAX_LANES];
    bool alive[TERRAIN_NAV_BAND_MAX_LANES];
    float laneClimbM[TERRAIN_NAV_BAND_MAX_LANES];
    terrainNavBlockSet_t blocks;

    terrainNavBandResult_t result;
} terrainNavBandScan_t;

// One sample: riseExtraM is added to the terrain rise, climbM and escapeM are the heights
// the aircraft can have gained by then on the two slopes. Returns the work done
static inline uint8_t terrainNavBandSample(terrainNavBandScan_t *scan, float northM, float eastM, bool lane0,
                                           float riseExtraM, float climbM, float escapeM)
{
    terrainNavBandResult_t *result = &scan->result;
    int32_t blockLat;
    int32_t blockLon;

    if (!lane0 && result->bandSamples >= TERRAIN_NAV_BAND_MAX_SAMPLES) {
        result->bandSkipped++;
        return 0;
    }
    scan->terrain.block(scan->terrain.ctx, northM, eastM, &blockLat, &blockLon);
    if (lane0) {
        // Lane 0 is bounded by its length, as the straight scan is: always sampled
        terrainNavBlockSetAdd(&scan->blocks, blockLat, blockLon, TERRAIN_NAV_BLOCK_SET_MAX);
        result->samplesTotal++;
    } else {
        if (!terrainNavBlockSetAdd(&scan->blocks, blockLat, blockLon, scan->blockBudget)) {
            result->bandSkipped++;
            return TERRAIN_NAV_BAND_WORK_SAMPLE;
        }
        result->bandSamples++;
    }

    float heightM;
    if (!scan->terrain.height(scan->terrain.ctx, &heightM)) {
        if (lane0) {
            result->samplesMissed++;
        } else {
            result->bandMissed++;
        }
        return TERRAIN_NAV_BAND_WORK_SAMPLE;
    }

    const float riseM = heightM - scan->baseHeightM + riseExtraM;
    if (riseM - climbM > result->climbNeededM) {
        result->climbNeededM = riseM - climbM;
    }
    if (riseM - escapeM > result->escapeDeficitM) {
        result->escapeDeficitM = riseM - escapeM;
    }
    return TERRAIN_NAV_BAND_WORK_SAMPLE;
}

// Starts a scan. It walks lane 0 first, exactly as a single path, so its blocks are counted
// before the rest of the band may claim any; then it walks all lanes together and samples
// the other lanes and the points between neighbours.
//   distanceM, maxLaneSamples   limits of lane 0 (and so of the band's length)
//   climbRatio, escapeRatio     metres of climb per metre flown on the two slopes
//   nextExtraRiseM              added to the terrain rise on the way to the next waypoint
//   baseHeightM                 terrain height at the start
//   blockBudget                 most blocks the scan may touch, the start's block included
// Everything is copied: the scan goes on from where the aircraft was when it started
static inline void terrainNavBandScanStart(terrainNavBandScan_t *scan, const terrainNavMissionPath_t *path,
                                           const terrainNavBand_t *band, float courseDeg, float distanceM,
                                           uint8_t maxLaneSamples, float climbRatio, float escapeRatio, float nextExtraRiseM,
                                           float baseHeightM, uint8_t blockBudget, const terrainNavBandTerrain_t *terrain)
{
    scan->path = *path;
    scan->band = *band;
    scan->terrain = *terrain;
    scan->courseDeg = courseDeg;
    scan->distanceM = distanceM;
    scan->climbPerStepM = climbRatio * TERRAIN_NAV_MISSION_PATH_STEP_M;
    scan->escapePerStepM = escapeRatio * TERRAIN_NAV_MISSION_PATH_STEP_M;
    scan->nextExtraRiseM = nextExtraRiseM;
    scan->baseHeightM = baseHeightM;
    scan->maxLaneSamples = maxLaneSamples;
    scan->blockBudget = blockBudget;

    scan->result.climbNeededM = 0.0f;
    scan->result.escapeDeficitM = 0.0f;
    scan->result.samplesTotal = 0;
    scan->result.samplesMissed = 0;
    scan->result.bandSamples = 0;
    scan->result.bandMissed = 0;
    scan->result.bandSkipped = 0;

    // The block under the aircraft is in use by the height-above-ground query
    int32_t blockLat;
    int32_t blockLon;
    scan->blocks.count = 0;
    scan->terrain.block(scan->terrain.ctx, 0.0f, 0.0f, &blockLat, &blockLon);
    terrainNavBlockSetAdd(&scan->blocks, blockLat, blockLon, TERRAIN_NAV_BLOCK_SET_MAX);

    terrainNavMissionPathStart(&scan->lane0, courseDeg);
    scan->travelledM = TERRAIN_NAV_MISSION_PATH_STEP_M;
    scan->climbM = 0.0f;
    scan->escapeM = 0.0f;
    scan->sharpestTurnDeg = 0.0f;
    scan->laneCount = 0;
    scan->steps = 0;
    scan->lane = 0;
    scan->piece = 0;
    scan->pieces = 0;
    scan->phase = TERRAIN_NAV_BAND_SCAN_LANE0;
}

static inline bool terrainNavBandScanDone(const terrainNavBandScan_t *scan)
{
    return scan->phase == TERRAIN_NAV_BAND_SCAN_DONE;
}

// Lane 0 is walked: the rest of the band begins, or the scan ends there
static inline void terrainNavBandScanEnterBand(terrainNavBandScan_t *scan)
{
    scan->phase = TERRAIN_NAV_BAND_SCAN_DONE;
    if (scan->band.laneCount < 2) {
        return;
    }
    scan->laneCount = (scan->band.laneCount < TERRAIN_NAV_BAND_MAX_LANES) ? scan->band.laneCount : TERRAIN_NAV_BAND_MAX_LANES;
    // Lane 0 never wanted to turn more in a step than the widest lane can: every lane
    // flies the same path, there is no band to scan (a leg flown on course)
    if (scan->sharpestTurnDeg <= scan->band.maxTurnDeg[scan->laneCount - 1]) {
        return;
    }
    for (int lane = 0; lane < scan->laneCount; lane++) {
        terrainNavMissionPathStart(&scan->lanes[lane], scan->courseDeg);
        scan->alive[lane] = true;
        scan->laneClimbM[lane] = 0.0f;
    }
    scan->escapeM = 0.0f;
    scan->steps = 0;
    scan->travelledM = TERRAIN_NAV_MISSION_PATH_STEP_M;
    scan->phase = TERRAIN_NAV_BAND_SCAN_BAND_STEP;
}

// Runs the scan on, until it is done or it has done workBudget of work in this call
// (TERRAIN_NAV_BAND_WORK_*; 0: no limit). True once the scan is done; scan->result is
// the answer then, and a lower bound of it before
static inline bool terrainNavBandScanRun(terrainNavBandScan_t *scan, uint16_t workBudget)
{
    terrainNavBandResult_t *result = &scan->result;
    uint16_t work = 0;
    float stepNorthM;
    float stepEastM;

    while (scan->phase != TERRAIN_NAV_BAND_SCAN_DONE) {
        if (workBudget != 0 && work >= workBudget) {
            return false;
        }

        switch (scan->phase) {
        case TERRAIN_NAV_BAND_SCAN_LANE0:
            // One step of lane 0 and its sample
            if (!(scan->travelledM <= scan->distanceM) || result->samplesTotal >= scan->maxLaneSamples
                || !terrainNavMissionPathStep(&scan->lane0, &scan->path, scan->band.maxTurnDeg[0], &stepNorthM, &stepEastM)) {
                terrainNavBandScanEnterBand(scan);
                break;
            }
            if (scan->lane0.wantedTurnDeg > scan->sharpestTurnDeg) {
                scan->sharpestTurnDeg = scan->lane0.wantedTurnDeg;
            }
            scan->climbM += scan->lane0.turning ? scan->climbPerStepM * TERRAIN_NAV_MISSION_TURN_CLIMB_FACTOR : scan->climbPerStepM;
            scan->escapeM += scan->escapePerStepM;
            work += TERRAIN_NAV_BAND_WORK_STEP;
            work += terrainNavBandSample(scan, scan->lane0.northM, scan->lane0.eastM, true,
                                         scan->lane0.pastWaypoint ? scan->nextExtraRiseM : 0.0f, scan->climbM, scan->escapeM);
            scan->travelledM += TERRAIN_NAV_MISSION_PATH_STEP_M;
            break;

        case TERRAIN_NAV_BAND_SCAN_BAND_STEP: {
            // All lanes one step on
            if (!(scan->travelledM <= scan->distanceM) || scan->steps >= scan->maxLaneSamples) {
                scan->phase = TERRAIN_NAV_BAND_SCAN_DONE;
                break;
            }
            if (result->bandSamples >= TERRAIN_NAV_BAND_MAX_SAMPLES) {
                result->bandSkipped++;      // the far steps of the band stay unscanned this scan
                scan->phase = TERRAIN_NAV_BAND_SCAN_DONE;
                break;
            }
            scan->steps++;

            bool anyAlive = false;
            for (int lane = 0; lane < scan->laneCount; lane++) {
                if (scan->alive[lane]) {
                    work += TERRAIN_NAV_BAND_WORK_STEP;
                    if (!terrainNavMissionPathStep(&scan->lanes[lane], &scan->path, scan->band.maxTurnDeg[lane], &stepNorthM, &stepEastM)) {
                        scan->alive[lane] = false;
                    }
                }
                if (scan->alive[lane]) {
                    anyAlive = true;
                    scan->laneClimbM[lane] += scan->lanes[lane].turning
                        ? scan->climbPerStepM * TERRAIN_NAV_MISSION_TURN_CLIMB_FACTOR : scan->climbPerStepM;
                }
            }
            if (!anyAlive) {
                scan->phase = TERRAIN_NAV_BAND_SCAN_DONE;
                break;
            }
            scan->escapeM += scan->escapePerStepM;
            // The lanes themselves first, then the points between them: when the sample
            // budget runs out it is the cross-fill of the far steps that goes
            scan->lane = 1;
            scan->phase = TERRAIN_NAV_BAND_SCAN_BAND_LANES;
            break;
        }

        case TERRAIN_NAV_BAND_SCAN_BAND_LANES: {
            // One lane of this step
            if (scan->lane >= scan->laneCount) {
                scan->lane = 1;
                scan->piece = 0;
                scan->phase = TERRAIN_NAV_BAND_SCAN_BAND_FILL;
                break;
            }
            const int lane = scan->lane++;
            if (!scan->alive[lane]) {
                break;
            }
            if (scan->alive[lane - 1]) {
                const float dNorthM = scan->lanes[lane].northM - scan->lanes[lane - 1].northM;
                const float dEastM = scan->lanes[lane].eastM - scan->lanes[lane - 1].eastM;
                if (dNorthM * dNorthM + dEastM * dEastM < TERRAIN_NAV_BAND_LANE_MERGE_M * TERRAIN_NAV_BAND_LANE_MERGE_M) {
                    break;
                }
            }
            work += terrainNavBandSample(scan, scan->lanes[lane].northM, scan->lanes[lane].eastM, false,
                                         scan->lanes[lane].pastWaypoint ? scan->nextExtraRiseM : 0.0f,
                                         scan->laneClimbM[lane], scan->escapeM);
            break;
        }

        case TERRAIN_NAV_BAND_SCAN_BAND_FILL: {
            // One point between two neighbouring lanes of this step
            if (scan->lane >= scan->laneCount) {
                scan->travelledM += TERRAIN_NAV_MISSION_PATH_STEP_M;
                scan->phase = TERRAIN_NAV_BAND_SCAN_BAND_STEP;
                break;
            }
            const int lane = scan->lane;
            if (!scan->alive[lane] || !scan->alive[lane - 1]) {
                scan->lane++;
                break;
            }
            const float dNorthM = scan->lanes[lane].northM - scan->lanes[lane - 1].northM;
            const float dEastM = scan->lanes[lane].eastM - scan->lanes[lane - 1].eastM;
            if (scan->piece == 0) {
                const int pieces = (int)ceilf(sqrtf(dNorthM * dNorthM + dEastM * dEastM) / TERRAIN_NAV_MISSION_PATH_STEP_M);
                if (pieces < 2) {
                    scan->lane++;
                    break;
                }
                scan->pieces = (pieces < 255) ? (uint8_t)pieces : 255;
                scan->piece = 1;
            }
            // Between two lanes: the lower climb, the higher extra rise
            const float extraA = scan->lanes[lane - 1].pastWaypoint ? scan->nextExtraRiseM : 0.0f;
            const float extraB = scan->lanes[lane].pastWaypoint ? scan->nextExtraRiseM : 0.0f;
            const float extraM = (extraA > extraB) ? extraA : extraB;
            const float betweenClimbM = (scan->laneClimbM[lane - 1] < scan->laneClimbM[lane])
                ? scan->laneClimbM[lane - 1] : scan->laneClimbM[lane];
            const float f = (float)scan->piece / (float)scan->pieces;
            work += terrainNavBandSample(scan, scan->lanes[lane - 1].northM + dNorthM * f, scan->lanes[lane - 1].eastM + dEastM * f,
                                         false, extraM, betweenClimbM, scan->escapeM);
            scan->piece++;
            if (scan->piece >= scan->pieces) {
                scan->piece = 0;
                scan->lane++;
            }
            break;
        }

        default:
            scan->phase = TERRAIN_NAV_BAND_SCAN_DONE;
            break;
        }
    }
    return true;
}

// The whole scan in one call
static inline void terrainNavBandWalk(const terrainNavMissionPath_t *path, const terrainNavBand_t *band, float courseDeg,
                                      float distanceM, uint8_t maxLaneSamples, float climbRatio, float escapeRatio,
                                      float nextExtraRiseM, float baseHeightM, uint8_t blockBudget,
                                      const terrainNavBandTerrain_t *terrain, terrainNavBandResult_t *result)
{
    terrainNavBandScan_t scan;
    terrainNavBandScanStart(&scan, path, band, courseDeg, distanceM, maxLaneSamples, climbRatio, escapeRatio, nextExtraRiseM,
                            baseHeightM, blockBudget, terrain);
    terrainNavBandScanRun(&scan, 0);
    *result = scan.result;
}
