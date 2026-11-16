## VTx Setup

### IRC Tramp

#### Matek 1G3SE Setup

To use the Matek 1G3SE with IRC Tramp. You will need to enter the CLI command `set vtx_frequency_group = FREQUENCYGROUP_1G3`. You must also make sure that the initial VTx settings in the configuration tab are in a valid range. They are: 
- `vtx_band` 1 or 2
- `vtx_channel` between 1 and 8

Note: The frequencies required by the US version of the VTx are on `vtx_band` 2 (BAND B) only.

Power levels are: 
- `1` 25mW
- `2` 200mW
- `3` 800 mW

##### Matek 1G3SE frequency chart

| Band | 1    | 2    | 3    | 4    | 5    | 6    | 7    | 8    |
|------|------|------|------|------|------|------|------|------|
| A    | 1080 | 1120 | 1160 | 1200 | 1240 | 1280 | 1320 | 1360 |
| B    | 1080 | 1120 | 1160 | 1200 | 1258 | 1280 | 1320 | 1360 |

### Team BlackSheep SmartAudio

If you have problems getting SmartAudio working. There are a couple of CLI parameters you can try changing to see if they help.

- There is a workaround for early AKK VTx modules. This is enabled by default. You could try disabling this setting [`vtx_smartaudio_early_akk_workaround`](https://github.com/iNavFlight/inav/blob/master/docs/Settings.md#vtx_smartaudio_early_akk_workaround) to OFF.

- If you are using softserial, you can try using the alternate method by setting [`vtx_smartaudio_alternate_softserial_method`](https://github.com/iNavFlight/inav/blob/master/docs/Settings.md#vtx_smartaudio_alternate_softserial_method) to OFF.

- If you are using TBS Sixty9 VTX you may consider to set count of stop bits to 1, using [`set vtx_smartaudio_stopbits = 1`](https://github.com/iNavFlight/inav/blob/master/docs/Settings.md#vtx_smartaudio_stopbits)

### VTX table (`vtxtable`)

This fork adds Betaflight's `vtxtable`: your own bands, channel frequencies and power levels. The CLI syntax is the same as Betaflight's, so a `vtxtable` block written for Betaflight can be pasted into the INAV CLI unchanged. The Configurator's Configuration tab can also load a Betaflight VTX table JSON file.

With no table stored (`vtxtable bands 0` and `vtxtable powerlevels 0`, the default), INAV behaves as before: bands A, B, E, F and R, and the power levels built into each VTX driver. Bands and power levels are independent, so a table can define only one of them.

```
vtxtable bands 6
vtxtable channels 8
vtxtable band 1 BOSCAM_A A FACTORY 5865 5845 5825 5805 5785 5765 5745 5725
vtxtable band 2 BOSCAM_B B FACTORY 5733 5752 5771 5790 5809 5828 5847 5866
vtxtable band 3 BOSCAM_E E FACTORY 5705 5685 5665 5645 5885 5905 5925 5945
vtxtable band 4 FATSHARK F FACTORY 5740 5760 5780 5800 5820 5840 5860 5880
vtxtable band 5 RACEBAND R FACTORY 5658 5695 5732 5769 5806 5843 5880 5917
vtxtable band 6 LOWBAND  L CUSTOM  4990 5000 5010 5020 5030 5040 5050 5060
vtxtable powerlevels 4
vtxtable powervalues 14 20 26 30
vtxtable powerlabels 25 100 400 1W
save
```

- Up to 8 bands, 8 channels per band and 8 power levels. Band names are up to 8 characters with no spaces, power labels up to 3.
- `vtx_band`, `vtx_channel` and `vtx_power` select a row of the table. `vtx_band` accepts 1 to 8.
- A frequency of 0 marks a channel as not used. Selecting it sends nothing to the VTX.
- Frequencies are not limited to 5000-5999 MHz. Any value up to 9999 is stored and sent; the VTX decides what it can tune.
- `FACTORY` means the VTX has that band built in at the same position. SmartAudio then selects it by band and channel number, and the frequencies in the table are only used for display. `CUSTOM` means SmartAudio sets the frequency directly, which needs a VTX that allows it (usually an unlocked one). IRC Tramp and MSP VTXs are always given the frequency.
- `powervalues` are the numbers sent to the VTX: milliwatts for IRC Tramp, dBm for SmartAudio 2.1, the level number starting at 0 for SmartAudio 2.0, and the VTX's DAC values for SmartAudio 1.0. `powerlabels` are only shown in menus.
- With a SmartAudio 2.1 VTX connected, `vtxtable` with no arguments also prints the power levels the VTX reports, in dBm. Those are the values to use for `powervalues`.
- `vtxtable` appears in `diff` and `dump`.
