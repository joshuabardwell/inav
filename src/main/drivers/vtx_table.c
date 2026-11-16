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

/* Ported from Betaflight's vtx_table (created by jflyper). See vtx_table.h. */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_VTX_TABLE

#include "common/printf.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

PG_REGISTER_WITH_RESET_FN(vtxTableConfig_t, vtxTableConfig, PG_VTX_TABLE_CONFIG, 0);

void pgResetFn_vtxTableConfig(vtxTableConfig_t *config)
{
    config->bands = 0;
    config->channels = 0;

    for (int band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
        vtxTableConfigClearBand(config, band);
    }

    config->powerLevels = 0;
    vtxTableConfigClearPowerValues(config, 0);
    vtxTableConfigClearPowerLabels(config, 0);
}

// Name tables handed to menus through vtxDeviceCapability_t. Entry 0 is the "none" entry.
static const char *bandNamePtrs[VTX_TABLE_MAX_BANDS + 1];
static const char *channelNamePtrs[VTX_TABLE_MAX_CHANNELS + 1];
static const char *powerLabelPtrs[VTX_TABLE_MAX_POWER_LEVELS + 1];

bool vtxTableHasBands(void)
{
    const vtxTableConfig_t *config = vtxTableConfig();
    return config->bands > 0 && config->bands <= VTX_TABLE_MAX_BANDS &&
           config->channels > 0 && config->channels <= VTX_TABLE_MAX_CHANNELS;
}

bool vtxTableHasPowerLevels(void)
{
    const vtxTableConfig_t *config = vtxTableConfig();
    return config->powerLevels > 0 && config->powerLevels <= VTX_TABLE_MAX_POWER_LEVELS;
}

uint16_t vtxTableLookupFrequency(uint8_t band, uint8_t channel)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    if (!vtxTableHasBands() || band < 1 || band > config->bands || channel < 1 || channel > config->channels) {
        return 0;
    }

    return config->frequency[band - 1][channel - 1];
}

bool vtxTableIsFactoryBand(uint8_t band)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    if (!vtxTableHasBands() || band < 1 || band > config->bands) {
        return false;
    }

    return config->isFactoryBand[band - 1];
}

bool vtxTableLookupBandChannel(uint16_t freq, uint8_t preferBand, uint8_t preferChannel, uint8_t *pBand, uint8_t *pChannel)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    *pBand = 0;
    *pChannel = 0;

    if (!vtxTableHasBands() || freq == 0) {
        return false;
    }

    if (vtxTableLookupFrequency(preferBand, preferChannel) == freq) {
        *pBand = preferBand;
        *pChannel = preferChannel;
        return true;
    }

    // Reverse band order, as Betaflight does, so 5880 MHz resolves to Raceband 7 rather than Fatshark 8
    for (int band = config->bands - 1; band >= 0; band--) {
        for (int channel = 0; channel < config->channels; channel++) {
            if (config->frequency[band][channel] == freq) {
                *pBand = band + 1;
                *pChannel = channel + 1;
                return true;
            }
        }
    }

    return false;
}

bool vtxTableLookupPowerValue(uint8_t index, uint16_t *pValue)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    if (!vtxTableHasPowerLevels() || index < 1 || index > config->powerLevels) {
        return false;
    }

    *pValue = config->powerValues[index - 1];
    return true;
}

uint8_t vtxTableLookupPowerIndex(uint16_t value)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    if (!vtxTableHasPowerLevels()) {
        return 0;
    }

    for (int i = 0; i < config->powerLevels; i++) {
        if (config->powerValues[i] == value) {
            return i + 1;
        }
    }

    return 0;
}

uint16_t vtxTableLookupPowerMilliwatt(uint8_t index)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    if (!vtxTableHasPowerLevels() || index < 1 || index > config->powerLevels) {
        return 0;
    }

    // Labels are free text. Understand "25", "800", "1W", "2W5" and "1.6" (watts); anything else is unknown.
    const char *p = config->powerLabels[index - 1];
    uint32_t whole = 0;
    bool haveDigits = false;

    while (*p >= '0' && *p <= '9') {
        whole = whole * 10 + (*p - '0');
        haveDigits = true;
        p++;
    }

    if (!haveDigits) {
        return 0;
    }

    if (*p == '.' || *p == 'W' || *p == 'w') {
        uint32_t mw = whole * 1000;
        uint32_t scale = 100;
        p++;
        while (*p >= '0' && *p <= '9' && scale > 0) {
            mw += (*p - '0') * scale;
            scale /= 10;
            p++;
        }
        return mw > UINT16_MAX ? UINT16_MAX : mw;
    }

    return whole > UINT16_MAX ? UINT16_MAX : whole;
}

void vtxTableOverlayCapability(vtxDeviceCapability_t *capability)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    if (vtxTableHasBands()) {
        bandNamePtrs[0] = "--------";
        for (int band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
            bandNamePtrs[band + 1] = config->bandNames[band];
        }
        channelNamePtrs[0] = "-";
        for (int channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
            channelNamePtrs[channel + 1] = config->channelNames[channel];
        }

        capability->bandCount = config->bands;
        capability->channelCount = config->channels;
        capability->bandNames = (char **)bandNamePtrs;
        capability->channelNames = (char **)channelNamePtrs;
    }

    if (vtxTableHasPowerLevels()) {
        powerLabelPtrs[0] = "---";
        for (int level = 0; level < VTX_TABLE_MAX_POWER_LEVELS; level++) {
            powerLabelPtrs[level + 1] = config->powerLabels[level];
        }

        capability->powerCount = config->powerLevels;
        capability->powerNames = (char **)powerLabelPtrs;
    }
}

void vtxTableOverlayOsdInfo(vtxDeviceOsdInfo_t *osdInfo)
{
    const vtxTableConfig_t *config = vtxTableConfig();

    if (!vtxTableHasBands()) {
        return;
    }

    if (osdInfo->band >= 1 && osdInfo->band <= config->bands) {
        osdInfo->bandLetter = config->bandLetters[osdInfo->band - 1];
        osdInfo->bandName = config->bandNames[osdInfo->band - 1];
    } else {
        osdInfo->bandLetter = '-';
        osdInfo->bandName = "-";
    }

    if (osdInfo->channel >= 1 && osdInfo->channel <= config->channels) {
        osdInfo->channelName = config->channelNames[osdInfo->channel - 1];
    } else {
        osdInfo->channelName = "-";
    }
}

void vtxTableStrncpyWithPad(char *dst, const char *src, int length)
{
    char c;

    while (length && (c = *src++)) {
        *dst++ = c;
        length--;
    }

    while (length--) {
        *dst++ = ' ';
    }

    *dst = 0;
}

// Prune a band to "channels"
void vtxTableConfigClearChannels(vtxTableConfig_t *config, int band, int channels)
{
    for (int channel = channels; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
        config->frequency[band][channel] = 0;
    }
}

void vtxTableConfigClearBand(vtxTableConfig_t *config, int band)
{
    vtxTableConfigClearChannels(config, band, 0);
    for (int channel = 0; channel < VTX_TABLE_MAX_CHANNELS; channel++) {
        tfp_sprintf(config->channelNames[channel], "%d", channel + 1);
    }
    char tempbuf[6];
    tfp_sprintf(tempbuf, "BAND%d", band + 1);
    vtxTableStrncpyWithPad(config->bandNames[band], tempbuf, VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[band] = '1' + band;
    config->isFactoryBand[band] = false;
}

void vtxTableConfigClearPowerValues(vtxTableConfig_t *config, int start)
{
    for (int i = start; i < VTX_TABLE_MAX_POWER_LEVELS; i++) {
        config->powerValues[i] = 0;
    }
}

void vtxTableConfigClearPowerLabels(vtxTableConfig_t *config, int start)
{
    for (int i = start; i < VTX_TABLE_MAX_POWER_LEVELS; i++) {
        char tempbuf[4];
        tfp_sprintf(tempbuf, "LV%d", i);
        vtxTableStrncpyWithPad(config->powerLabels[i], tempbuf, VTX_TABLE_POWER_LABEL_LENGTH);
    }
}

#endif // USE_VTX_TABLE
