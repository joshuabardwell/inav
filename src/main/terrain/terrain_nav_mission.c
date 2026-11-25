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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TERRAIN

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/pid.h"

#include "io/gps.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "terrain/terrain_nav.h"
#include "terrain/terrain_nav_hold.h"
#include "terrain/terrain_nav_mission.h"
#include "terrain/terrain_nav_mission_logic.h"

// The terrain height at the waypoint is usually many blocks away from the
// aircraft, so it is asked for sparingly: quickly at first (the block only
// has to be read once), then slowly, then not at all for this leg
#define TERRAIN_NAV_MISSION_WP_QUERY_FAST_MS    200
#define TERRAIN_NAV_MISSION_WP_QUERY_FAST_TRIES 10
#define TERRAIN_NAV_MISSION_WP_QUERY_SLOW_MS    1000
#define TERRAIN_NAV_MISSION_WP_QUERY_MAX_TRIES  20

typedef struct {
    bool active;                // a leg with a terrain mode is being flown

    // Identity of the leg: the waypoint it flies to, as stored
    int8_t wpIndex;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int16_t p3;
    uint8_t action;

    terrainNavMissionMode_e mode;
    int32_t minAglCm;           // the leg's minimum height above ground

    bool wpTerrainValid;        // terrain at the waypoint known
    float wpRiseCm;             // terrain rise from home to the waypoint
    bool clearanceValid;        // follow: the clearance to hold is known
    int32_t clearanceCm;
    uint8_t wpQueryTries;
    timeMs_t wpQueryNextMs;

    bool nextRouteValid;        // the mission flies on to another waypoint after this one
    int8_t nextIndex;           // that waypoint
    float nextNorthCm;          // in the local frame
    float nextEastCm;
    bool nextLegValid;          // and the leg to it is a terrain leg too
    int32_t nextMinAglCm;

    // The turn guard (terrain_nav_mission_logic.h): the leg flown is not a terrain leg, but
    // the turn onto it from a terrain leg is still being flown. It is tracked as a floor
    // leg with the minimum of the terrain leg before
    bool guard;
    bool guardOnCourse;         // the aircraft is on course for the leg, since guardOnCourseMs
    timeMs_t guardStartMs;
    timeMs_t guardOnCourseMs;

    bool legChanged;            // set for the one cycle in which the leg changed

    bool holdActive;            // the hold produced (or is briefly pausing) terrain targets
    bool haveTerrainZ;          // terrainZCm is a target from this run of terrain legs
    float terrainZCm;           // the line with its lead

    // The line the leg flies (terrain_nav_mission_logic.h). It goes on from one terrain leg
    // to the next
    bool lineValid;             // lineZCm holds a line of this run of terrain legs
    float lineZCm;
    float lineRateCmS;          // the rate it moves at until the next terrain query
    float lineRiseCmS;          // its rise, smoothed, for the lead
    float lineFreshZCm;         // where it stood at the last terrain query
    float lineGroundZCm;        // the ground under the aircraft at that query
    timeMs_t lineTimeMs;        // when it last moved
    timeMs_t lineFreshMs;       // the last terrain query
    bool lineAnswerValid;       // the look-ahead's answer for the line
    bool lineAnswerComplete;    // every point in view had data
    float lineSlope;
    float lineTopZCm;           // the highest ground in view at the height the leg holds, local frame
    float lineSpeedCmS;         // the speed the scan that answered planned with
    float lineScanSpeedCmS;     // the same of the scan that is running
    float lineScanHeldZCm;      // the ground under the aircraft + the held height where that scan started

    bool carryActive;           // new leg: the line may not come down before the leg has its own answers
    timeMs_t carryStartMs;
} terrainNavMissionState_t;

static terrainNavMissionState_t mission;

static bool waypointHasTerrainOptions(const navWaypoint_t *wp)
{
    return wp->action == NAV_WP_ACTION_WAYPOINT || wp->action == NAV_WP_ACTION_HOLD_TIME;
}

// The states in which the aircraft flies to, or waits at, the active waypoint
static bool flyingToActiveWaypoint(void)
{
    switch (posControl.navState) {
        case NAV_STATE_WAYPOINT_IN_PROGRESS:
        case NAV_STATE_WAYPOINT_REACHED:
        case NAV_STATE_WAYPOINT_HOLD_TIME:
            return true;
        default:
            return false;
    }
}

static bool matchesActiveWaypoint(const navWaypoint_t *wp)
{
    return mission.wpIndex == posControl.activeWaypointIndex
        && mission.lat == wp->lat
        && mission.lon == wp->lon
        && mission.alt == wp->alt
        && mission.p3 == wp->p3
        && mission.action == wp->action;
}

// The last terrain target survives only across legs that follow each other
// inside one mission run: "never descend on dead data" must hold over a
// waypoint, and must not leak into a stock leg or another flight
static void deactivate(bool keepTerrainTarget)
{
    mission.active = false;
    mission.holdActive = false;
    if (!keepTerrainTarget) {
        mission.haveTerrainZ = false;
        mission.lineValid = false;
        mission.nextRouteValid = false;
        mission.nextLegValid = false;
        mission.guard = false;
    }
}

// The waypoint the mission flies to after the active one, as the navigation controller works
// it out for its own turns (getLocalPosNextWaypoint in navigation.c: a JUMP is followed to
// its target, or stepped over once its count is used up). -1: there is none, or what comes
// next is not a waypoint the aircraft flies to as a leg (RTH, the end of the mission)
static int nextFlownWaypointIndex(const navWaypoint_t *wp)
{
    const int last = posControl.startWpIndex + posControl.waypointCount - 1;
    int index = posControl.activeWaypointIndex + 1;
    if (wp->flag == NAV_WP_FLAG_LAST || index > last || index >= NAV_MAX_WAYPOINTS) {
        return -1;
    }

    const navWaypoint_t *next = &posControl.waypointList[index];
    if (next->action == NAV_WP_ACTION_JUMP) {
        if (next->p3 != 0 || next->p2 == -1) {
            index = next->p1 + posControl.startWpIndex;
        } else if (index + 1 <= last && posControl.waypointList[index + 1].action != NAV_WP_ACTION_JUMP) {
            index++;
        } else {
            return -1;
        }
        if (index < 0 || index > last || index >= NAV_MAX_WAYPOINTS) {
            return -1;
        }
    }

    switch (posControl.waypointList[index].action) {
        case NAV_WP_ACTION_WAYPOINT:
        case NAV_WP_ACTION_HOLD_TIME:
        case NAV_WP_ACTION_LAND:
            return index;
        default:
            return -1;
    }
}

static void setUpNextLeg(const navWaypoint_t *wp)
{
    mission.nextRouteValid = false;
    mission.nextLegValid = false;

    const int nextIndex = nextFlownWaypointIndex(wp);
    if (nextIndex < 0) {
        return;
    }

    const navWaypoint_t *next = &posControl.waypointList[nextIndex];
    // A waypoint without coordinates stands for the home position: not a
    // route the scan can follow
    if (next->lat == 0 && next->lon == 0) {
        return;
    }

    const gpsLocation_t nextLlh = { .lat = next->lat, .lon = next->lon, .alt = 0 };
    fpVector3_t nextPos;
    if (!geoConvertGeodeticToLocal(&nextPos, &posControl.gpsOrigin, &nextLlh, GEO_ALT_RELATIVE)) {
        return;
    }

    const float deltaNorth = nextPos.x - posControl.activeWaypoint.pos.x;
    const float deltaEast = nextPos.y - posControl.activeWaypoint.pos.y;
    if (fabsf(deltaNorth) < 1.0f && fabsf(deltaEast) < 1.0f) {
        return;
    }

    mission.nextNorthCm = nextPos.x;
    mission.nextEastCm = nextPos.y;
    mission.nextIndex = nextIndex;
    mission.nextRouteValid = true;

    const bool nextHasOptions = waypointHasTerrainOptions(next);
    if (terrainNavMissionModeFromP3(nextHasOptions, next->p3) != TERRAIN_NAV_MISSION_OFF) {
        mission.nextMinAglCm = terrainNavMissionMinAglCm(nextHasOptions, next->p3, terrainNavConfig()->minAglCm);
        mission.nextLegValid = true;
    }
}

// True when the leg to the active waypoint, not a terrain leg itself, starts with the turn
// guard: it is the waypoint the terrain leg just flown was followed by, and that run of
// terrain legs had terrain targets (without terrain data the mission flies as stock)
static bool guardStartsAtActiveWaypoint(void)
{
    return mission.haveTerrainZ && !mission.guard && mission.nextRouteValid
        && mission.nextIndex == posControl.activeWaypointIndex;
}

// True when the turn guard is over: the aircraft has been on course for the leg for
// TERRAIN_NAV_MISSION_ESTABLISHED_MS, or the guard has lasted as long as a turn can
static bool guardEnds(timeMs_t currentTimeMs)
{
    const fpVector3_t *here = &navGetCurrentActualPositionAndVelocity()->pos;
    const float toWpDeg = RADIANS_TO_DEGREES(atan2f(posControl.activeWaypoint.pos.y - here->y, posControl.activeWaypoint.pos.x - here->x));
    navFwTurnArc_t arc;
    navFwGetTurnArc(&arc);

    if (terrainNavMissionOnCourse((float)posControl.actualState.cog * 0.01f, toWpDeg, navConfig()->fw.wp_tracking_accuracy > 0,
                                  (float)posControl.activeWaypoint.bearing * 0.01f, arc.engaged)) {
        if (!mission.guardOnCourse) {
            mission.guardOnCourse = true;
            mission.guardOnCourseMs = currentTimeMs;
        } else if (currentTimeMs - mission.guardOnCourseMs >= TERRAIN_NAV_MISSION_ESTABLISHED_MS) {
            return true;
        }
    } else {
        mission.guardOnCourse = false;
    }

    return currentTimeMs - mission.guardStartMs >= terrainNavMissionGuardMaxMs(posControl.actualState.velXY, navFwGetPlanningBankDeg());
}

static void startLeg(const navWaypoint_t *wp, timeMs_t currentTimeMs)
{
    mission.active = true;
    mission.wpIndex = posControl.activeWaypointIndex;
    mission.lat = wp->lat;
    mission.lon = wp->lon;
    mission.alt = wp->alt;
    mission.p3 = wp->p3;
    mission.action = wp->action;
    mission.wpTerrainValid = false;
    mission.clearanceValid = false;
    mission.wpQueryTries = 0;
    mission.wpQueryNextMs = currentTimeMs;
    mission.holdActive = false;
    mission.legChanged = true;
    // After a terrain leg of the same run the line goes on, and does not
    // come down until this leg has its own answers: its look-ahead scan
    // and, on a follow leg, its clearance (until then the leg holds
    // minimum + margin). The answer of the leg before was measured from
    // that leg's held height
    mission.carryActive = mission.haveTerrainZ;
    mission.carryStartMs = currentTimeMs;
    mission.lineAnswerValid = false;
}

bool terrainNavMissionUpdate(timeMs_t currentTimeMs)
{
    mission.legChanged = false;

    if (!ARMING_FLAG(ARMED) || !STATE(AIRPLANE) || !(navGetCurrentStateFlags() & NAV_AUTO_WP)) {
        deactivate(false);
        return false;
    }

    if (!flyingToActiveWaypoint() || posControl.activeWaypointIndex < 0 || posControl.activeWaypointIndex >= NAV_MAX_WAYPOINTS) {
        // Between two waypoints of the same mission
        deactivate(true);
        return false;
    }

    const navWaypoint_t *wp = &posControl.waypointList[posControl.activeWaypointIndex];
    const bool hasOptions = waypointHasTerrainOptions(wp);
    const terrainNavMissionMode_e mode = terrainNavMissionModeFromP3(hasOptions, wp->p3);
    if (mode == TERRAIN_NAV_MISSION_OFF) {
        // Not a terrain leg. The turn onto it from a terrain leg is guarded: the floor of
        // the leg before stays until the aircraft is on course for this one
        if (mission.guard && matchesActiveWaypoint(wp)) {
            if (guardEnds(currentTimeMs)) {
                deactivate(false);
                return false;
            }
            mission.active = true;
            return true;
        }
        if (guardStartsAtActiveWaypoint()) {
            startLeg(wp, currentTimeMs);
            mission.mode = TERRAIN_NAV_MISSION_FLOOR;       // the minimum stays that of the terrain leg before
            mission.guard = true;
            mission.guardOnCourse = false;
            mission.guardStartMs = currentTimeMs;
            mission.nextRouteValid = false;                 // what follows this leg is not the guard's to scan
            mission.nextLegValid = false;
            return true;
        }
        deactivate(false);
        return false;
    }

    if (!mission.active || mission.guard || !matchesActiveWaypoint(wp)) {
        startLeg(wp, currentTimeMs);
        mission.mode = mode;
        mission.minAglCm = terrainNavMissionMinAglCm(hasOptions, wp->p3, terrainNavConfig()->minAglCm);
        mission.guard = false;
        setUpNextLeg(wp);
    }

    return true;
}

bool terrainNavMissionLegChanged(void)
{
    return mission.legChanged;
}

bool terrainNavMissionResolveWaypoint(timeMs_t currentTimeMs)
{
    // The turn guard ends long before its waypoint: the terrain there is not needed
    if (!mission.active || mission.wpTerrainValid || mission.guard) {
        return false;
    }

    // A floor leg needs the terrain at the waypoint only for the altitude
    // test of nav_wp_enforce_altitude; a follow leg always needs it
    if (mission.mode != TERRAIN_NAV_MISSION_FOLLOW && navConfig()->general.waypoint_enforce_altitude == 0) {
        return false;
    }

    if (mission.wpQueryTries >= TERRAIN_NAV_MISSION_WP_QUERY_MAX_TRIES || (int32_t)(currentTimeMs - mission.wpQueryNextMs) < 0) {
        return false;
    }

    mission.wpQueryTries++;
    mission.wpQueryNextMs = currentTimeMs + ((mission.wpQueryTries < TERRAIN_NAV_MISSION_WP_QUERY_FAST_TRIES)
        ? TERRAIN_NAV_MISSION_WP_QUERY_FAST_MS
        : TERRAIN_NAV_MISSION_WP_QUERY_SLOW_MS);

    // A waypoint without coordinates stands for the home position
    const bool atHome = (mission.lat == 0 && mission.lon == 0);
    const gpsLocation_t wpLlh = { .lat = mission.lat, .lon = mission.lon, .alt = 0 };

    float homeTerrainM;
    float wpTerrainM;
    if (!terrainNavGetLatchedHomeHeight(&homeTerrainM)) {
        return true;
    }
    if (atHome) {
        wpTerrainM = homeTerrainM;
    } else if (!terrainNavGetHeightAtLocation(&wpLlh, &wpTerrainM)) {
        return true;    // the block is being read; asked again later
    }

    mission.wpTerrainValid = true;
    mission.wpRiseCm = (wpTerrainM - homeTerrainM) * 100.0f;

    if (mission.mode == TERRAIN_NAV_MISSION_FOLLOW) {
        const bool altIsAmsl = (mission.p3 & NAV_WP_ALTMODE) == NAV_WP_ALTMODE;
        mission.clearanceCm = terrainNavMissionLegClearanceCm(altIsAmsl, mission.alt, posControl.activeWaypoint.pos.z,
                                                              wpTerrainM, homeTerrainM);
        mission.clearanceValid = true;
    }

    return true;
}

// The height above ground the leg holds
static int32_t commandedAglCm(void)
{
    return terrainNavMissionCommandedAglCm(mission.mode, mission.clearanceValid, mission.clearanceCm,
                                           mission.minAglCm, TERRAIN_NAV_HOLD_RECOVER_MARGIN_CM);
}

bool terrainNavMissionLookaheadStart(float distanceM, float groundSpeedCmS, float climbRatio, float escapeRatio,
                                     bool groundValid, float groundZCm)
{
    const fpVector3_t *here = &navGetCurrentActualPositionAndVelocity()->pos;

    // The scan is a band of lanes (terrain_nav_mission_logic.h). They start on the
    // course over ground and turn toward the waypoint, the tightest at the bank the
    // navigation controller plans its turns for right now (nav_fw_bank_angle under
    // max_angle_inclination_rll and the energy guard), the widest at half of it
    const float planningBankDeg = navFwGetPlanningBankDeg();
    terrainNavBand_t band;
    terrainNavBandFromBank(groundSpeedCmS, planningBankDeg, &band);

    terrainNavMissionPath_t path = { 0 };
    // (CENTIMETERS_TO_METERS does not bracket its argument: take the difference first)
    const float toWpNorthCm = posControl.activeWaypoint.pos.x - here->x;
    const float toWpEastCm = posControl.activeWaypoint.pos.y - here->y;
    path.wpNorthM = CENTIMETERS_TO_METERS(toWpNorthCm);
    path.wpEastM = CENTIMETERS_TO_METERS(toWpEastCm);
    // Where the mission flies on to another waypoint the lanes go round the corner. If the
    // leg to it is a terrain leg they follow it; if not they end once they are on course for
    // it: the turn is this leg's, what the next leg crosses is not (the turn zone). Past a
    // waypoint the mission does not fly on from (RTH, its end) the scan stops at the waypoint
    path.nextLegValid = mission.nextRouteValid;
    path.turnZoneOnly = mission.guard || (mission.nextRouteValid && !mission.nextLegValid);
    path.guard = mission.guard;
    const float toNextNorthCm = mission.nextNorthCm - here->x;
    const float toNextEastCm = mission.nextEastCm - here->y;
    path.nextWpNorthM = CENTIMETERS_TO_METERS(toNextNorthCm);
    path.nextWpEastM = CENTIMETERS_TO_METERS(toNextEastCm);

    // The lanes fly the turn as the controller will. With the waypoint tracker on, the
    // aircraft heads for the leg's line, not for the waypoint
    const bool tracking = navConfig()->fw.wp_tracking_accuracy > 0;
    const float legCourseDeg = (float)posControl.activeWaypoint.bearing * 0.01f;
    path.speedMS = groundSpeedCmS * 0.01f;
    path.trackMaxAngleDeg = navConfig()->fw.wp_tracking_max_angle;
    path.trackLine = tracking;
    path.lineCourseDeg = legCourseDeg;
    if (mission.nextRouteValid) {
        path.nextTrackLine = tracking;
        path.nextLineCourseDeg = RADIANS_TO_DEGREES(atan2f(mission.nextEastCm - posControl.activeWaypoint.pos.y,
                                                           mission.nextNorthCm - posControl.activeWaypoint.pos.x));
    }

    // COORD_FLYINTO swings away from the corner ahead of the waypoint. While its S is flown
    // the lanes take the controller's own two arcs; before, the plan it will make
    navFwTurnArc_t arc;
    navFwGetTurnArc(&arc);
    const float turnAngleDeg = (float)posControl.activeWaypoint.nextTurnAngle * 0.01f;
    if (navConfig()->fw.wp_turn_mode == NAV_FW_WP_TURN_COORD_FLY_INTO && !mission.guard
        && mission.action != NAV_WP_ACTION_HOLD_TIME && posControl.activeWaypoint.nextTurnAngle != -1) {
        const bool onMainArc = arc.engaged && !arc.secondStaged
            && fabsf(terrainNavMissionWrap180((float)arc.outBearing * 0.01f - (legCourseDeg + turnAngleDeg))) < 5.0f;
        if (arc.engaged && arc.secondStaged) {
            path.flyInto = true;
            path.flyIntoDir = arc.secondDir;
            path.flyIntoGapSteps = 0;
            path.flyIntoTriggerM = 1.0e9f;      // begun
            path.flyIntoCounterEndDeg = terrainNavMissionWrap180((float)arc.outBearing * 0.01f);
            path.flyIntoOutCourseDeg = terrainNavMissionWrap180((float)arc.secondOutBearing * 0.01f);
        } else if (!onMainArc) {
            terrainNavMissionFlyIntoPlan(&path, legCourseDeg, turnAngleDeg, terrainNavMissionTurnRadiusM(groundSpeedCmS, planningBankDeg),
                                         path.speedMS, navFwGetTurnEaseMs() * 0.001f);
        }
    }
    // An arc the turn coordinator is flying onto this leg is flown on, the way it goes round
    if (arc.engaged && !(path.flyInto && arc.secondStaged)) {
        path.arc = true;
        path.arcDir = arc.dir;
        path.arcOutCourseDeg = terrainNavMissionWrap180((float)arc.outBearing * 0.01f);
    }
    // The lanes leave for the next waypoint where the controller will switch to it: the
    // fly-by lead before the waypoint. A waypoint the aircraft holds at is flown to
    const bool flyBy = navConfig()->fw.wp_turn_mode == NAV_FW_WP_TURN_COORD_FLY_BY
        && mission.action != NAV_WP_ACTION_HOLD_TIME
        && posControl.activeWaypoint.nextTurnAngle != -1;
    // From its switch on COORD_FLYBY flies an arc onto the next leg's course, for any turn
    // its coordinator flies an arc for
    if (flyBy && mission.nextRouteValid && fabsf(turnAngleDeg) > TERRAIN_NAV_MISSION_FLYINTO_MIN_DEG) {
        path.nextArc = true;
        path.nextArcDir = (turnAngleDeg > 0.0f) ? 1 : -1;
        path.nextArcCourseDeg = terrainNavMissionWrap180(legCourseDeg + turnAngleDeg);
    }
    path.switchDistM = terrainNavMissionSwitchDistEaseM(flyBy, turnAngleDeg,
                                                        terrainNavMissionTurnRadiusM(groundSpeedCmS, planningBankDeg), groundSpeedCmS,
                                                        (float)navConfig()->fw.wp_turn_max_lead_time,
                                                        CENTIMETERS_TO_METERS((float)navConfig()->general.waypoint_radius),
                                                        navFwGetTurnEaseMs());

    const float nextExtraRiseM = mission.nextLegValid
        ? terrainNavMissionTurnExtraRiseM(mission.minAglCm, mission.nextMinAglCm)
        : 0.0f;

    // The line the leg flies is aimed over the ground from where it stands now, at the
    // height this leg holds; past the waypoint, at the next leg's minimum + margin. It
    // looks TERRAIN_NAV_MISSION_VIEW_S of flight ahead, at the speed the scan plans with
    float lineStartM = 0.0f;
    mission.lineScanHeldZCm = groundZCm + (float)commandedAglCm();
    if (groundValid && mission.lineValid) {
        const float lineAboveHeldCm = mission.lineZCm - mission.lineScanHeldZCm;
        lineStartM = CENTIMETERS_TO_METERS(lineAboveHeldCm);
    }
    const float lineViewCm = groundSpeedCmS * TERRAIN_NAV_MISSION_VIEW_S;
    // Past the waypoint the line is aimed at the next leg's minimum + margin; in the turn
    // zone of a leg that is not a terrain leg, at this leg's: a floor, never a follow
    // leg's clearance
    float lineNextExtraM = 0.0f;
    if (mission.nextLegValid) {
        lineNextExtraM = terrainNavMissionLineNextExtraM(commandedAglCm(), mission.nextMinAglCm, TERRAIN_NAV_HOLD_RECOVER_MARGIN_CM);
    } else if (mission.nextRouteValid) {
        lineNextExtraM = terrainNavMissionLineNextExtraM(commandedAglCm(), mission.minAglCm, TERRAIN_NAV_HOLD_RECOVER_MARGIN_CM);
    }
    mission.lineScanSpeedCmS = groundSpeedCmS;

    return terrainNavLookaheadRouteStart(&path, &band, CENTIDEGREES_TO_DEGREES((float)posControl.actualState.cog), distanceM,
                                         climbRatio, escapeRatio, nextExtraRiseM,
                                         lineStartM, CENTIMETERS_TO_METERS(lineViewCm), lineNextExtraM);
}

void terrainNavMissionLineAnswer(bool valid, bool complete, float slope, float topM)
{
    mission.lineAnswerValid = valid;
    mission.lineAnswerComplete = complete;
    mission.lineSlope = slope;
    mission.lineTopZCm = mission.lineScanHeldZCm + METERS_TO_CENTIMETERS(topM);
    mission.lineSpeedCmS = mission.lineScanSpeedCmS;
}

void terrainNavMissionApplyInput(terrainNavHoldInput_t *in)
{
    in->minAglCm = mission.minAglCm;
    in->commandedAglValid = true;
    in->commandedAglCm = commandedAglCm();
    in->lookaheadMarginCm = TERRAIN_NAV_HOLD_RECOVER_MARGIN_CM;

    // The altitude funnel does not apply nav_max_altitude in waypoint mode
    // when nav_wp_enforce_altitude is set, so there is no ceiling to warn about
    if (navConfig()->general.waypoint_enforce_altitude) {
        in->maxAltCm = 0;
    }
}

// The climb rate the look-ahead plans with; also the fastest descent a terrain
// leg commands
static float plannedClimbCmS(void)
{
    return TERRAIN_NAV_HOLD_PLANNED_CLIMB_FRACTION * (float)navConfig()->fw.max_auto_climb_rate;
}

// A longer gap than this between two targets is not flown time (a pause, lost data): the
// line does not move for it
#define TERRAIN_NAV_MISSION_LINE_GAP_MS     500

void terrainNavMissionStoreOutput(const terrainNavHoldInput_t *in, const terrainNavHoldOutput_t *out, bool freshQuery)
{
    mission.holdActive = (out->status == TERRAIN_NAV_HOLD_ACTIVE);
    if (!out->writeTarget) {
        // No target this cycle (no data, a pause): the line stands where it is
        return;
    }

    if (mission.carryActive && terrainNavMissionCarryEnds(mission.mode, mission.clearanceValid, in->lookaheadValid,
                                                          in->nowMs - mission.carryStartMs)) {
        mission.carryActive = false;
    }

    const float maxClimbCmS = (float)navConfig()->fw.max_auto_climb_rate;
    const float groundZCm = in->currentZCm - (float)in->aglCm;
    const float hardFloorZCm = groundZCm + (float)in->minAglCm + (float)in->lookaheadMarginCm;

    if (!mission.lineValid) {
        // The first target of a run of terrain legs: the line starts at the height the
        // leg holds. An aircraft above it comes down at the descent limit
        // (terrainNavMissionAdjustTargetZ), and a floor leg's stock line is not held up
        mission.lineValid = true;
        mission.lineZCm = groundZCm + (float)commandedAglCm();
        mission.lineRateCmS = 0.0f;
        mission.lineRiseCmS = 0.0f;
        mission.lineFreshZCm = mission.lineZCm;
        mission.lineGroundZCm = groundZCm;
        mission.lineTimeMs = in->nowMs;
        mission.lineFreshMs = in->nowMs;
    }

    // Between two terrain queries the line goes on at the rate of the last one
    const uint32_t movedMs = in->nowMs - mission.lineTimeMs;
    mission.lineTimeMs = in->nowMs;
    if (movedMs <= TERRAIN_NAV_MISSION_LINE_GAP_MS) {
        mission.lineZCm += mission.lineRateCmS * (float)movedMs * 0.001f;
    }

    if (freshQuery) {
        // New ground under the aircraft: what the line may never be under, its rise since
        // the last query, and the rate it moves at until the next
        const uint32_t freshMs = in->nowMs - mission.lineFreshMs;
        const float freshS = (freshMs <= TERRAIN_NAV_MISSION_LINE_GAP_MS) ? (float)freshMs * 0.001f : 0.0f;
        const bool climbing = in->lookaheadValid && in->lookaheadClimbCm > 0.0f;
        const float climbLineZCm = terrainNavMissionClimbLineZ(in->currentZCm, in->aglCm, in->minAglCm, in->lookaheadMarginCm,
                                                               in->lookaheadClimbCm);
        mission.lineZCm = terrainNavMissionLineAdvanceZ(mission.lineZCm, 0.0f, freshS, groundZCm + (float)commandedAglCm(),
                                                        plannedClimbCmS(), hardFloorZCm, climbing, climbLineZCm);
        mission.lineRiseCmS = terrainNavMissionLineRiseCmS(mission.lineRiseCmS, mission.lineFreshZCm, mission.lineZCm, freshS,
                                                           maxClimbCmS);
        mission.lineFreshZCm = mission.lineZCm;
        mission.lineGroundZCm = groundZCm;
        mission.lineFreshMs = in->nowMs;

        if (terrainNavConfig()->lookaheadDistM == 0) {
            // The look-ahead is switched off: nothing to aim at, and the only ground in
            // view is the ground under the aircraft. The line comes down to the height
            // the leg holds there and stays on it
            mission.lineRateCmS = terrainNavMissionLineRateCmS(true, -1.0f, 0.0f,
                                                               mission.lineZCm - (groundZCm + (float)commandedAglCm()),
                                                               !mission.carryActive, plannedClimbCmS(), maxClimbCmS);
        } else if (mission.carryActive && !in->lookaheadValid) {
            // A new leg whose look-ahead has not answered yet: the line goes on as it was
            // going on the leg before, but not down
            if (mission.lineRateCmS < 0.0f) {
                mission.lineRateCmS = 0.0f;
            }
        } else {
            const bool answered = in->lookaheadValid && mission.lineAnswerValid;
            mission.lineRateCmS = terrainNavMissionLineRateCmS(answered, mission.lineSlope, mission.lineSpeedCmS,
                                                               mission.lineZCm - mission.lineTopZCm,
                                                               !mission.carryActive && mission.lineAnswerComplete,
                                                               plannedClimbCmS(), maxClimbCmS);
        }
    }

    mission.haveTerrainZ = true;
    mission.terrainZCm = terrainNavMissionLineTargetZ(mission.lineZCm, mission.lineRiseCmS,
                                                      pidProfile()->fwAltControlResponseFactor);
}

bool terrainNavMissionLineAglCm(int32_t *aglCm)
{
    if (!mission.active || !mission.lineValid) {
        return false;
    }

    *aglCm = lrintf(mission.lineZCm - mission.lineGroundZCm);
    return true;
}

uint32_t terrainNavMissionDebugFlags(void)
{
    if (!mission.active) {
        return 0;
    }

    // The turn guard shows as mode 3, which no waypoint can ask for
    uint32_t flags = (1 << 3) | ((mission.guard ? 3 : ((uint32_t)mission.mode & 3)) << 4);
    if (mission.clearanceValid) {
        flags |= (1 << 6);
    }
    if (mission.wpTerrainValid) {
        flags |= (1 << 7);
    }
    return flags;
}

// The state machine runs ahead of the gate inside a navigation cycle, so on
// the first pass of a new leg the tracked leg is still the previous one: the
// stock value goes back until the gate has caught up
static bool trackingActiveWaypoint(void)
{
    if (!mission.active || !STATE(AIRPLANE) || !ARMING_FLAG(ARMED)) {
        return false;
    }
    if (posControl.activeWaypointIndex < 0 || posControl.activeWaypointIndex >= NAV_MAX_WAYPOINTS) {
        return false;
    }
    return matchesActiveWaypoint(&posControl.waypointList[posControl.activeWaypointIndex]);
}

// The active waypoint asks for a terrain mode, or its leg starts with the turn guard, whether
// or not the gate tracks it yet
static bool activeWaypointIsTerrainLeg(void)
{
    if (!STATE(AIRPLANE) || !ARMING_FLAG(ARMED)) {
        return false;
    }
    if (posControl.activeWaypointIndex < 0 || posControl.activeWaypointIndex >= NAV_MAX_WAYPOINTS) {
        return false;
    }
    const navWaypoint_t *wp = &posControl.waypointList[posControl.activeWaypointIndex];
    return terrainNavMissionModeFromP3(waypointHasTerrainOptions(wp), wp->p3) != TERRAIN_NAV_MISSION_OFF
        || guardStartsAtActiveWaypoint();
}

float terrainNavMissionAdjustTargetZ(float stockZCm)
{
    if (!trackingActiveWaypoint()) {
        // The first pass of a new leg. From one terrain leg to the next the
        // target of the leg before stands, as it does for the cycles after
        // (the carry)
        if (mission.haveTerrainZ && mission.terrainZCm > stockZCm && activeWaypointIsTerrainLeg()) {
            return mission.terrainZCm;
        }
        return stockZCm;
    }

    const float targetZCm = terrainNavMissionResolveTargetZ(mission.mode, mission.clearanceValid, mission.holdActive,
                                                            mission.haveTerrainZ, mission.terrainZCm, stockZCm);
    if (!mission.holdActive) {
        return targetZCm;
    }
    return terrainNavMissionLimitDescentZ(targetZCm, navGetCurrentActualPositionAndVelocity()->pos.z, plannedClimbCmS(),
                                          pidProfile()->fwAltControlResponseFactor);
}

float terrainNavMissionWaypointZ(float stockWpZCm)
{
    // Without usable terrain data the stored altitude is all there is. A leg under the turn
    // guard is not a terrain leg: it ends at its stored altitude
    if (!trackingActiveWaypoint() || !mission.holdActive || mission.guard) {
        return stockWpZCm;
    }

    return terrainNavMissionLegEndZ(mission.mode, mission.wpTerrainValid, mission.wpRiseCm, commandedAglCm(), stockWpZCm);
}

#endif
