# JB fork of INAV 10: what is different

This is a test fork of [iNavFlight/inav](https://github.com/iNavFlight/inav), started from
`10.0.0-rc2`. It is not an official INAV release and nothing here is supported by the INAV
team. Fly it only if you can recover from a failure. The first hardware tests were on a
TBS Lucid H743 Wing.

The branch `jb/build` is the one to use. It is the upstream release plus the changes below,
and nothing else. Each change also lives on its own topic branch (`jb/wpterrain`,
`jb/vtxtable`, `jb/vtxready`) if you want to read or build one alone.

`jb/build` is rebuilt when upstream releases, so its history is rewritten and a plain
`git pull` will fail. To update a clone, run `git fetch` and then
`git reset --hard origin/jb/build` (this discards any local changes on that branch).

No firmware files are published. Build it yourself (see [Building](#building)); the commit
history on `jb/build` shows exactly what changed and when.

## What the fork adds

### Terrain handling on waypoint legs (fixed wing)

Upstream's [terrain-following cruise mode](docs/Navigation%20Terrain%20Following.md) works
only in 3D Cruise and steps aside for waypoint missions. In this fork a waypoint can ask for
terrain handling on the leg flown toward it, as a stock leg, a **floor**, or **follow**,
with its own minimum height above ground (50 to 120 m).

- What each option does, the look-ahead over turns, the turn guard and the OSD messages:
  [Waypoint missions](docs/Navigation%20Terrain%20Following.md#waypoint-missions).
- The rules the look-ahead keeps: the minimum height above ground is never traded for a
  smoother line; a terrain leg flies a line over the ground ahead, not the ground's contour.
- How take-off height is fixed at arming, and the `TERRAIN: MAP/GPS ALT DIFFER` warning:
  same page, under the OSD table.
- The option travels with the mission in the waypoint's `p3` field, so a mission file
  carries it. The bit layout is in [Navigation.md](docs/Navigation.md).

It needs what upstream's terrain feature needs: a flight controller with more than 256 KB
of RAM (H7, or an F7 such as STM32F745/F746/F765), an SD card, and `.TER` terrain files made
with the generator described in [Terrain.md](docs/Terrain.md). Set `terrain_enabled = ON`.

The fork does not change upstream's flight logic (attitude, bank and energy control).
Terrain legs work around its behaviour, and where that behaviour limits a terrain leg the
documentation says so.

### VTX table

Betaflight-style `vtxtable`: your own bands, channel frequencies and power labels, with the
same CLI syntax, so a Betaflight block pastes in unchanged. Works with SmartAudio, IRC Tramp
and MSP VTXs. SmartAudio is also reported as ready only while the VTX answers, and the OSD can
show the table's power label. See [VTx.md](docs/VTx.md#vtx-table-vtxtable).

## Use it with the fork's Configurator

Only [INAV Configurator JB](https://github.com/joshuabardwell/inav-configurator) has the
per-waypoint terrain options, the terrain-aware route check and the VTX table editor
(the VTX table can also be set in the CLI).
Use the same `jb/build` branch on both sides: new MSP commands are added to both, in the
`0x2F00` range.

## Building

Build as for upstream INAV: [Building in Linux](docs/development/Building%20in%20Linux.md),
[Windows with WSL](docs/development/Building%20in%20Windows%2010%20or%2011%20with%20Linux%20Subsystem.md),
or the other guides in [docs/development](docs/development/). Clone this repository, check out
`jb/build`, and build your target. Do not build on a cloud-synced folder.

A build is named `jb-<build date>-<commit>`, for example `jb-20261126-cf085428`. The same
commit shows in the CLI `version` line. If you report a problem, give that label.

## Telling the fork apart from upstream

The firmware still identifies itself as `INAV`, because the Configurator refuses any other
identifier. The build label is how to tell the two apart.

## Reporting results

Open an issue on this repository with the build label, the board, what you set, and what
the aircraft did. A blackbox log (with `debug_mode = TERRAIN_NAV` for terrain legs) and the
mission file help most. RF output of a VTX (frequency, power) can only be confirmed on
hardware, so say what you measured.

Problems that exist in upstream INAV itself are better reported to
[iNavFlight/inav](https://github.com/iNavFlight/inav/issues).
