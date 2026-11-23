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

#include "common/maths.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

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

    bool nextLegValid;          // the leg after this one is a terrain leg too
    float nextNorthCm;          // its waypoint, in the local frame
    float nextEastCm;
    int32_t nextMinAglCm;

    bool legChanged;            // set for the one cycle in which the leg changed

    bool holdActive;            // the hold produced (or is briefly pausing) terrain targets
    bool haveTerrainZ;          // terrainZCm is a target from this run of terrain legs
    float terrainZCm;
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
    }
}

static void setUpNextLeg(const navWaypoint_t *wp)
{
    mission.nextLegValid = false;

    const int nextIndex = posControl.activeWaypointIndex + 1;
    if (wp->flag == NAV_WP_FLAG_LAST || nextIndex > posControl.startWpIndex + posControl.waypointCount - 1 || nextIndex >= NAV_MAX_WAYPOINTS) {
        return;
    }

    const navWaypoint_t *next = &posControl.waypointList[nextIndex];
    const bool nextHasOptions = waypointHasTerrainOptions(next);
    if (terrainNavMissionModeFromP3(nextHasOptions, next->p3) == TERRAIN_NAV_MISSION_OFF) {
        return;
    }
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
    mission.nextMinAglCm = terrainNavMissionMinAglCm(nextHasOptions, next->p3, terrainNavConfig()->minAglCm);
    mission.nextLegValid = true;
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
        deactivate(false);
        return false;
    }

    if (!mission.active || !matchesActiveWaypoint(wp)) {
        mission.active = true;
        mission.wpIndex = posControl.activeWaypointIndex;
        mission.lat = wp->lat;
        mission.lon = wp->lon;
        mission.alt = wp->alt;
        mission.p3 = wp->p3;
        mission.action = wp->action;
        mission.mode = mode;
        mission.minAglCm = terrainNavMissionMinAglCm(hasOptions, wp->p3, terrainNavConfig()->minAglCm);
        mission.wpTerrainValid = false;
        mission.clearanceValid = false;
        mission.wpQueryTries = 0;
        mission.wpQueryNextMs = currentTimeMs;
        mission.holdActive = false;
        mission.legChanged = true;
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
    if (!mission.active || mission.wpTerrainValid) {
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

bool terrainNavMissionLookahead(float distanceM, float groundSpeedCmS, float climbRatio, float escapeRatio, terrainNavLookaheadResult_t *result)
{
    const fpVector3_t *here = &navGetCurrentActualPositionAndVelocity()->pos;

    // The scan is a band of lanes (terrain_nav_mission_logic.h). They start on the
    // course over ground and turn toward the waypoint, the tightest at the bank the
    // navigation controller plans its turns for right now (nav_fw_bank_angle under
    // max_angle_inclination_rll and the energy guard), the widest at half of it
    const float planningBankDeg = navFwGetPlanningBankDeg();
    terrainNavBand_t band;
    terrainNavBandFromBank(groundSpeedCmS, planningBankDeg, &band);

    terrainNavMissionPath_t path;
    // (CENTIMETERS_TO_METERS does not bracket its argument: take the difference first)
    const float toWpNorthCm = posControl.activeWaypoint.pos.x - here->x;
    const float toWpEastCm = posControl.activeWaypoint.pos.y - here->y;
    path.wpNorthM = CENTIMETERS_TO_METERS(toWpNorthCm);
    path.wpEastM = CENTIMETERS_TO_METERS(toWpEastCm);
    // What lies past the waypoint may not be a terrain leg (a stock leg, a
    // landing, RTH, a jump, the end of the mission): then the scan stops at
    // the waypoint and never reacts to ground the route does not cross
    path.nextLegValid = mission.nextLegValid;
    const float toNextNorthCm = mission.nextNorthCm - here->x;
    const float toNextEastCm = mission.nextEastCm - here->y;
    path.nextWpNorthM = CENTIMETERS_TO_METERS(toNextNorthCm);
    path.nextWpEastM = CENTIMETERS_TO_METERS(toNextEastCm);
    // The lanes leave for the next waypoint where the controller will switch to it: the
    // fly-by lead before the waypoint. A waypoint the aircraft holds at is flown to
    const bool flyBy = navConfig()->fw.wp_turn_mode == NAV_FW_WP_TURN_COORD_FLY_BY
        && mission.action != NAV_WP_ACTION_HOLD_TIME
        && posControl.activeWaypoint.nextTurnAngle != -1;
    path.switchDistM = terrainNavMissionSwitchDistM(flyBy, CENTIDEGREES_TO_DEGREES((float)posControl.activeWaypoint.nextTurnAngle),
                                                    terrainNavMissionTurnRadiusM(groundSpeedCmS, planningBankDeg), groundSpeedCmS,
                                                    (float)navConfig()->fw.wp_turn_max_lead_time,
                                                    CENTIMETERS_TO_METERS((float)navConfig()->general.waypoint_radius));

    const float nextExtraRiseM = mission.nextLegValid
        ? terrainNavMissionTurnExtraRiseM(mission.minAglCm, mission.nextMinAglCm, TERRAIN_NAV_HOLD_RECOVER_MARGIN_CM)
        : 0.0f;

    return terrainNavLookaheadRoute(&path, &band, CENTIDEGREES_TO_DEGREES((float)posControl.actualState.cog), distanceM,
                                    climbRatio, escapeRatio, nextExtraRiseM, result);
}

static int32_t commandedAglCm(void)
{
    return terrainNavMissionCommandedAglCm(mission.mode, mission.clearanceValid, mission.clearanceCm,
                                           mission.minAglCm, TERRAIN_NAV_HOLD_RECOVER_MARGIN_CM);
}

void terrainNavMissionApplyInput(terrainNavHoldInput_t *in)
{
    in->minAglCm = mission.minAglCm;
    in->commandedAglValid = true;
    in->commandedAglCm = commandedAglCm();

    // The altitude funnel does not apply nav_max_altitude in waypoint mode
    // when nav_wp_enforce_altitude is set, so there is no ceiling to warn about
    if (navConfig()->general.waypoint_enforce_altitude) {
        in->maxAltCm = 0;
    }
}

void terrainNavMissionStoreOutput(const terrainNavHoldOutput_t *out)
{
    mission.holdActive = (out->status == TERRAIN_NAV_HOLD_ACTIVE);
    if (out->writeTarget) {
        mission.haveTerrainZ = true;
        mission.terrainZCm = out->targetZCm;
    }
}

uint32_t terrainNavMissionDebugFlags(void)
{
    if (!mission.active) {
        return 0;
    }

    uint32_t flags = (1 << 3) | (((uint32_t)mission.mode & 3) << 4);
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

float terrainNavMissionAdjustTargetZ(float stockZCm)
{
    if (!trackingActiveWaypoint()) {
        return stockZCm;
    }

    return terrainNavMissionResolveTargetZ(mission.mode, mission.clearanceValid, mission.holdActive,
                                           mission.haveTerrainZ, mission.terrainZCm, stockZCm);
}

float terrainNavMissionWaypointZ(float stockWpZCm)
{
    // Without usable terrain data the stored altitude is all there is
    if (!trackingActiveWaypoint() || !mission.holdActive) {
        return stockWpZCm;
    }

    return terrainNavMissionLegEndZ(mission.mode, mission.wpTerrainValid, mission.wpRiseCm, commandedAglCm(), stockWpZCm);
}

#endif
