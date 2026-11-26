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
 * Free Software Foundation, either version 3 of the License or (at your
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

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Terrain subsystem notes:
 * This module runs in the main loop without an RTOS. It relies on synchronous SD card
 * access for terrain data, which is subject to I/O latency. Because there is no
 * background thread to handle blocking operations, increasing the update frequency
 * or adding "look-ahead" calculations for multiple GPS positions is not recommended.
 */
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>

#include "platform.h"

#ifdef USE_TERRAIN

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "terrain.h"
#include "terrain_utils.h"
#include "terrain_io.h"

#include "common/utils.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "drivers/sdcard/sdcard.h"
#include "drivers/time.h"


PG_REGISTER_WITH_RESET_TEMPLATE(terrainConfig_t, terrainConfig, PG_TERRAIN_CONFIG, 1);

PG_RESET_TEMPLATE(terrainConfig_t, terrainConfig,
                  .terrainEnabled = false,
);


static struct {
    timeMs_t lastUpdate;
    int32_t terrainAGL;     // cm
    int32_t terrainAMSL;    // cm
} terrainHeight = {
    .terrainAGL = 0,
    .lastUpdate = 0,
    .terrainAMSL = 0,
};

/*
 * The height reference: the map height that the navigation's altitude zero stands for.
 * Height above ground = estimated altitude - (map height here - reference).
 *
 * The altitude zero is set where the aircraft arms (inav_reset_altitude), which need not be
 * where the GPS origin was set by the first fix. So the map is read where the aircraft
 * stands while it is disarmed, and kept with the altitude it had there: the reference is
 * that map height minus that altitude. Arming keeps the last such point for the flight.
 *
 * The GPS altitude of the altitude zero (the GPS origin's altitude) is a second reading of
 * the same height, on the same datum as the map. The two differ by the map's error at the
 * take-off spot, the GPS altitude error and the height of the aircraft above the ground when
 * it armed. The lower of the two is used: a reference that is too high puts the ground too
 * low and the aircraft closer to it than it believes, a reference that is too low only
 * makes it fly higher. So a map that is too high at the take-off spot no longer lowers the
 * whole flight, and a GPS altitude that reads high changes nothing.
 */
typedef struct {
    bool located;           // a position and altitude have been recorded
    gpsLocation_t location;
    float altitudeCm;       // estimated altitude at that position, local frame
    bool heightFound;       // the map has answered for that position
    float heightM;
    bool gpsUsable;         // the GPS altitude was good enough there to be the second reading
} terrainGroundPoint_t;

static terrainGroundPoint_t groundDisarmed;     // follows the aircraft while it is disarmed
static terrainGroundPoint_t groundFlight;       // the point of the flight, taken at arming

/**
 * @brief Get the height above mean sea level (AMSL) in meters for a given GPS location.
 */
float getHeightAmslMeters(const gpsLocation_t *loc)
{
    if(isTerrainIoFailure()){
        return TERRAIN_STATUS_NO_AMSL_DATA;
    }

    gridInfo_t info;
    calculateGridInfo(loc, &info);

    if (info.idx_x > TERRAIN_GRID_BLOCK_SIZE_X - 2) {
        return TERRAIN_STATUS_NO_AMSL_DATA;
    }
    if (info.idx_y > TERRAIN_GRID_BLOCK_SIZE_Y - 2) {
        return TERRAIN_STATUS_NO_AMSL_DATA;
    }

    // find the grid
    gridCache_t *cache = findGridCache(&info);
    if(cache == NULL){
        return TERRAIN_STATUS_NO_AMSL_DATA;
    }
    gridBlock_t *grid = &cache->gridBlock;

    // check we have all 4 required heights; it's check if grid is loaded from SD card
    if (!checkBitmap(grid, info.idx_x,   info.idx_y) || !checkBitmap(grid, info.idx_x, info.idx_y + 1) || !checkBitmap(grid, info.idx_x + 1, info.idx_y) || !checkBitmap(grid, info.idx_x + 1, info.idx_y + 1)) {
        markGridBlockNeedRead(grid);
        return TERRAIN_STATUS_NO_AMSL_DATA;
    }

    // hXY are the heights of the 4 surrounding grid points, reconstructed from
    // the packed 10-bit offsets plus the block's heightBase
    const int16_t h00 = grid->heightBase + TERRAIN_HEIGHT_OFFSET_RESOLUTION_M * getHeightOffsetByIndex(grid, info.idx_x+0, info.idx_y+0);
    const int16_t h01 = grid->heightBase + TERRAIN_HEIGHT_OFFSET_RESOLUTION_M * getHeightOffsetByIndex(grid, info.idx_x+0, info.idx_y+1);
    const int16_t h10 = grid->heightBase + TERRAIN_HEIGHT_OFFSET_RESOLUTION_M * getHeightOffsetByIndex(grid, info.idx_x+1, info.idx_y+0);
    const int16_t h11 = grid->heightBase + TERRAIN_HEIGHT_OFFSET_RESOLUTION_M * getHeightOffsetByIndex(grid, info.idx_x+1, info.idx_y+1);

    // do a simple dual linear interpolation. We could do something
    // fancier, but it probably isn't worth it as long as the
    // grid_spacing is kept small enough
    const float avg1 = (1.0f-info.frac_x) * h00  + info.frac_x * h10;
    const float avg2 = (1.0f-info.frac_x) * h01  + info.frac_x * h11;
    const float avg  = (1.0f-info.frac_y) * avg1 + info.frac_y * avg2;

    return avg;
}

void terrainInit(void)
{
}

// Arming hands the disarmed point over to the flight. An emergency re-arm in the air keeps
// the flight's point, as the estimator keeps its altitude zero
static terrainGroundPoint_t *terrainGroundPointInUse(void)
{
    static bool wasArmed = false;

    const bool armed = ARMING_FLAG(ARMED);
    if (armed && !wasArmed && !(STATE(IN_FLIGHT_EMERG_REARM) && groundFlight.located)) {
        groundFlight = groundDisarmed;
    }
    wasArmed = armed;

    return armed ? &groundFlight : &groundDisarmed;
}

static void terrainGroundPointUpdate(void)
{
    terrainGroundPoint_t *point = terrainGroundPointInUse();

    if (!ARMING_FLAG(ARMED) && posControl.flags.estPosStatus >= EST_USABLE) {
        const fpVector3_t *pos = &navGetCurrentActualPositionAndVelocity()->pos;
        gpsLocation_t here;
        if (geoConvertLocalToGeodetic(&here, &posControl.gpsOrigin, pos)) {
            point->located = true;
            point->location = here;
            point->altitudeCm = pos->z;
            point->heightFound = false;
            point->gpsUsable = STATE(GPS_FIX) && gpsSol.fixType == GPS_FIX_3D
                               && gpsSol.epv < positionEstimationConfig()->max_eph_epv;
        }
    }

    // In flight only a point whose block was not loaded at arming is still asked for
    if (point->located && !point->heightFound) {
        const float heightM = getHeightAmslMeters(&point->location);
        if (heightM != TERRAIN_STATUS_NO_AMSL_DATA) {
            point->heightFound = true;
            point->heightM = heightM;
        }
    }
}

// The two readings of the height reference: from the map, and from the GPS altitude
static bool terrainReferenceReadings(float *mapM, bool *gpsUsable, float *gpsM)
{
    const terrainGroundPoint_t *point = terrainGroundPointInUse();
    if (!point->located || !point->heightFound) {
        return false;
    }

    *mapM = point->heightM - point->altitudeCm * 0.01f;
    *gpsUsable = point->gpsUsable && posControl.gpsOrigin.valid;
    *gpsM = posControl.gpsOrigin.alt * 0.01f;
    return true;
}

bool terrainGetReferenceHeightM(float *heightM)
{
    float mapM;
    float gpsM;
    bool gpsUsable;
    if (!terrainReferenceReadings(&mapM, &gpsUsable, &gpsM)) {
        return false;
    }

    *heightM = (gpsUsable && gpsM < mapM) ? gpsM : mapM;
    return true;
}

bool terrainReferenceReadingsDisagree(void)
{
    float mapM;
    float gpsM;
    bool gpsUsable;
    if (!terrainReferenceReadings(&mapM, &gpsUsable, &gpsM) || !gpsUsable) {
        return false;
    }

    return fabsf(mapM - gpsM) > TERRAIN_REFERENCE_DISAGREE_M;
}

/**
 * @brief Update the terrain subsystem.
 *
 * This function checks the status of the SD card and attempts to load terrain data into the cache.
 * It also keeps the height reference: the map height where the aircraft stands before it arms.
 */
void terrainUpdateTask(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if(terrainConfig()->terrainEnabled == false){
        return;
    }

    if(STATE(GPS_FIX) == false){
        return;
    }

    terrainGroundPointUpdate();

    float referenceM;
    if (ARMING_FLAG(ARMED) && !terrainGetReferenceHeightM(&referenceM)) {
        // armed without a height reference: the block under the take-off point was not
        // loaded. That point is still asked for above; nothing else is read from the card
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    if(terrainGetReferenceHeightM(&referenceM))
    {
        float heightASL = getHeightAmslMeters(&gpsSol.llh);
        if(heightASL != TERRAIN_STATUS_NO_AMSL_DATA){
            terrainHeight.terrainAMSL = (int32_t)(heightASL * 100.0f);
            terrainHeight.terrainAGL =  MAX(0, ((int32_t)getEstimatedActualPosition(Z) + (int32_t)(referenceM * 100.0f)) - terrainHeight.terrainAMSL);
            terrainHeight.lastUpdate = millis();
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////
}

int32_t terrainGetLastAMSL(void){
    if(!terrainConfig()->terrainEnabled){
        return TERRAIN_STATUS_NO_AMSL_DATA;
    }

    return terrainHeight.lastUpdate + TERRAIN_NO_DATA_DELAY_MS < millis() ? TERRAIN_STATUS_NO_AMSL_DATA : terrainHeight.terrainAMSL;
}

int32_t terrainGetLastDistanceCm(void)
{
    //start a terrain system after the first query to terrain height
    if(!terrainConfig()->terrainEnabled){
        return TERRAIN_STATUS_NO_AGL_DATA;
    }

    return terrainHeight.lastUpdate + TERRAIN_NO_DATA_DELAY_MS < millis() ? TERRAIN_STATUS_NO_AGL_DATA : terrainHeight.terrainAGL;
}

#endif