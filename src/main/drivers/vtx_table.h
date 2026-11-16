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
 * User-defined VTX band, channel and power table.
 * Data layout, CLI syntax and MSP format follow Betaflight's vtxtable
 * (originally by jflyper) so that Betaflight tables can be used unchanged.
 *
 * Unlike Betaflight, an empty table is not an error: while the table has no
 * bands (or no power levels) the VTX drivers keep using their built-in tables.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "config/parameter_group.h"

#include "drivers/vtx_common.h"

#define VTX_TABLE_MAX_BANDS             8 // Maximum number of bands
#define VTX_TABLE_MAX_CHANNELS          8 // Maximum number of channels per band
#define VTX_TABLE_MAX_POWER_LEVELS      8 // Maximum number of power levels
#define VTX_TABLE_CHANNEL_NAME_LENGTH   1
#define VTX_TABLE_BAND_NAME_LENGTH      8
#define VTX_TABLE_POWER_LABEL_LENGTH    3

// Any frequency a 4 digit field can hold is accepted. 0 marks an unused channel.
// The VTX itself decides which frequencies it can tune.
#define VTX_TABLE_MAX_FREQUENCY_MHZ     9999

#ifdef USE_VTX_TABLE

typedef struct vtxTableConfig_s {
    uint8_t  bands;
    uint8_t  channels;
    uint16_t frequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS];
    char     bandNames[VTX_TABLE_MAX_BANDS][VTX_TABLE_BAND_NAME_LENGTH + 1];
    char     bandLetters[VTX_TABLE_MAX_BANDS];
    char     channelNames[VTX_TABLE_MAX_CHANNELS][VTX_TABLE_CHANNEL_NAME_LENGTH + 1];
    bool     isFactoryBand[VTX_TABLE_MAX_BANDS];

    uint8_t  powerLevels;
    uint16_t powerValues[VTX_TABLE_MAX_POWER_LEVELS];
    char     powerLabels[VTX_TABLE_MAX_POWER_LEVELS][VTX_TABLE_POWER_LABEL_LENGTH + 1];
} vtxTableConfig_t;

PG_DECLARE(vtxTableConfig_t, vtxTableConfig);

// --- Lookups used by the VTX drivers. band, channel and power index are 1-based. ---

// True when the table defines bands and channels. When false, drivers use their built-in bands.
bool vtxTableHasBands(void);
// True when the table defines power levels. When false, drivers use their built-in power levels.
bool vtxTableHasPowerLevels(void);

// Frequency in MHz, or 0 when band/channel is outside the table or the channel is unused.
uint16_t vtxTableLookupFrequency(uint8_t band, uint8_t channel);
// FACTORY band: the VTX is told a band/channel number and uses its own frequency for it.
// CUSTOM band: the VTX is told the frequency.
bool vtxTableIsFactoryBand(uint8_t band);
// Finds the band/channel that has this frequency. When preferBand/preferChannel has that
// frequency it wins, so a frequency shared by two bands resolves to the selected one.
bool vtxTableLookupBandChannel(uint16_t freq, uint8_t preferBand, uint8_t preferChannel, uint8_t *pBand, uint8_t *pChannel);

// The value sent to the VTX for a power level (meaning depends on the protocol).
bool vtxTableLookupPowerValue(uint8_t index, uint16_t *pValue);
// Power level whose value equals the argument, or 0 when there is none.
uint8_t vtxTableLookupPowerIndex(uint16_t value);
// Best-effort milliwatts parsed from the power label ("25", "1W", "1.6"), 0 when unknown.
uint16_t vtxTableLookupPowerMilliwatt(uint8_t index);

// Replaces counts and names in a device's capability with the table's, where the table defines them.
void vtxTableOverlayCapability(vtxDeviceCapability_t *capability);
// Replaces band and channel names in OSD info with the table's, where the table defines them.
void vtxTableOverlayOsdInfo(vtxDeviceOsdInfo_t *osdInfo);

// --- Helpers for editing the stored table (CLI and MSP) ---

void vtxTableStrncpyWithPad(char *dst, const char *src, int length);
void vtxTableConfigClearBand(vtxTableConfig_t *config, int band);
void vtxTableConfigClearChannels(vtxTableConfig_t *config, int band, int channels);
void vtxTableConfigClearPowerValues(vtxTableConfig_t *config, int start);
void vtxTableConfigClearPowerLabels(vtxTableConfig_t *config, int start);

#else // !USE_VTX_TABLE

static inline bool vtxTableHasBands(void) { return false; }
static inline bool vtxTableHasPowerLevels(void) { return false; }
static inline uint16_t vtxTableLookupFrequency(uint8_t band, uint8_t channel) { (void)band; (void)channel; return 0; }
static inline bool vtxTableIsFactoryBand(uint8_t band) { (void)band; return true; }
static inline bool vtxTableLookupBandChannel(uint16_t freq, uint8_t preferBand, uint8_t preferChannel, uint8_t *pBand, uint8_t *pChannel)
{
    (void)freq; (void)preferBand; (void)preferChannel;
    *pBand = 0;
    *pChannel = 0;
    return false;
}
static inline bool vtxTableLookupPowerValue(uint8_t index, uint16_t *pValue) { (void)index; (void)pValue; return false; }
static inline uint8_t vtxTableLookupPowerIndex(uint16_t value) { (void)value; return 0; }
static inline uint16_t vtxTableLookupPowerMilliwatt(uint8_t index) { (void)index; return 0; }
static inline void vtxTableOverlayCapability(vtxDeviceCapability_t *capability) { (void)capability; }
static inline void vtxTableOverlayOsdInfo(vtxDeviceOsdInfo_t *osdInfo) { (void)osdInfo; }

#endif // USE_VTX_TABLE
