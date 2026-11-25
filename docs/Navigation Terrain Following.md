# Terrain AGL Hold

Terrain AGL Hold is INAV's **terrain-following** mode for fixed wing. It is a **3D Cruise** enhancement that holds a set height **above the ground** instead of above the take-off point — *Cruise with a moving target*. As the ground rises and falls under you, the altitude target rides with it, so the aircraft keeps its clearance over hills and valleys with no stick input: downhill it descends with the valley, uphill it climbs with the ridge.

It reads the ground elevation from the terrain elevation data on the SD card (see **[Terrain](Terrain.md)** for generating and loading tiles); it never uses a physical rangefinder. With the mode switched off, the aircraft flies exactly like stock 3D Cruise — everything here is opt-in, and **the pitch stick always wins** (the mode box is an instant in-flight kill switch).

## How it works

![How Terrain AGL Hold works](assets/images/Terrain-AGL-Hold-diagram.png)

*One navigation cycle — inputs, health gate, the moving target, the alarm ladder, and the single gate into the altitude target path (the stock controller, untouched).*

## Requirements

- A flight controller with terrain support (SD card + barometer) and **more than 256 KB of RAM: STM32H7 or STM32F7 (F745 / F746 / F765)**. F4 and AT32 boards are not enabled for terrain by default.
- **Fixed-wing only.** A valid **GPS fix** and a **barometer**.
- **Terrain tiles** (`.TER`) on the SD card, **including the tile for your take-off site** — see [Terrain](Terrain.md).
- **3D Cruise must be active.** At this stage Terrain AGL Hold works **only together with NAV CRUISE** — this is a mandatory safety condition for now, not just a recommendation.

## Not the same as copter SURFACE mode

Multirotor **SURFACE** mode follows the ground *reactively* from a physical rangefinder — the instantaneous distance straight below. Terrain AGL Hold is the fixed-wing equivalent taken from the SD terrain map, and crucially it is **predictive**: it scans **up to 2 km ahead** along your track and climbs/warns *early* for terrain rising in front of you — something a downward rangefinder cannot do. The two never stack (this mode refuses to engage while SURFACE is active), and SURFACE mode itself is multirotor-only.

## What it does

- **Captures** your current height above ground when you switch it on, and holds it. Move the **pitch stick** to set a new height; centre the stick and it holds the new one.
- Engage **below the safety floor** → it climbs smoothly up to about **10 m above the floor**, so the recovery does not leave you on the edge of the warning band.
- **Looks ahead** along your track (up to 2 km, 1 km by default) with a per-airframe **escape test** — *"at this speed, with your configured climb rate, will you clear what's ahead?"* — and warns you **early, while there's still time**.
- If the terrain data is lost mid-flight, it **holds the last valid altitude target and warns** on the OSD — it never descends on dead data.
- The stock 3D Cruise altitude controller is **untouched**; this only feeds it a moving target.
- **RTH does not use it.** RTH flies stock (terrain-blind). Waypoint missions can use it leg by leg — see *Waypoint missions* below.

## Setup

1. **Prepare and enable terrain data** — see **[Terrain](Terrain.md)**: generate `.TER` tiles for your area (include the take-off tile), copy them to the SD card root, then in the CLI:
   ```
   set terrain_enabled = ON
   save
   ```
2. **Assign the mode.** In the Configurator **Modes** tab (or via CLI `aux`), put **TERRAIN AGL HOLD** on a spare, **deliberately guarded** switch — not next to ARM. It is active only inside 3D Cruise.
   > **Hint (once you are confident with the mode):** you *can* put **NAV CRUISE** and **TERRAIN AGL HOLD** on the **same switch** so one flip arms both. This is **not** the recommended default — keeping them on **separate** switches lets you enter Cruise first, confirm it, and engage terrain hold as a deliberate second step.
3. **Set the floor and look-ahead** if you want (defaults are sensible — see *Settings*).
4. **OSD:** add the **Rangefinder** element to see your height above ground (AGL) in flight. If a physical rangefinder is fitted, that element shows the rangefinder instead — Terrain AGL Hold still flies from its map and its alarms are unaffected. The flight-mode field shows **TERR** while the hold is engaged.
5. **Fly** in 3D Cruise, then flip the switch — it captures and holds your current AGL. **Panic rule: box off = stock cruise instantly; the pitch stick always wins.**

## Settings

| Setting | Default | Range | Meaning |
|---|---|---|---|
| `terrain_nav_min_agl` | 60 m | 50 – 120 m | The **safety floor** — the lowest height above ground the mode will hold. Engage below it and it climbs up to ~10 m above it. (Stored in centimetres: 6000 = 60 m.) |
| `terrain_nav_lookahead` | 1000 m | 0 – 2000 m | How far ahead along your track the **forward scan** looks for rising terrain. The scan is additionally capped at **35 s of flight at your current ground speed**, because the horizon is really *time*, not distance: the same 1000 m is nearly a minute of flying at 20 m/s but only ~18 s at 55 m/s. The time cap stops slow flight from climbing absurdly early for terrain a minute away, while fast flight still uses the full distance. `0` disables the early *TERRAIN AHEAD!* warning, leaving only the reactive floor. |

> **NB — keep your climb rates honest.** The climb authority is your existing **`nav_fw_auto_climb_rate`** — there is no separate terrain climb setting, and the *TERRAIN AHEAD!* escape warning **trusts that number**, so set it to a rate your model genuinely sustains. Keep **`nav_fw_manual_climb_rate` at least equal to `nav_fw_auto_climb_rate`** (they default to 300 / 500) — or **greater**, if you want pulling the stick to actually add climb. If `nav_fw_manual_climb_rate` is **lower** than `nav_fw_auto_climb_rate`, grabbing pitch during an auto-climb commands the lower rate: the nose visibly eases and it can even **slow** the climb.

## Behaviour in the air

- **Engage at or above the floor** → captures your current AGL and holds it.
- **Engage below the floor** → climbs to ~10 m above the floor (*TERRAIN AUTO CLIMB TO MIN*).
- **Pitch stick** → sets a new held AGL. The stick always wins: it pauses the hold the moment it moves (smooth blend, no jump); centre it and the current height is re-captured.
- **Hands off over a hill** → the target rides the terrain up and back down.
- **Sharp turns while riding the floor** clip the safety margin — bank with height in hand.

## OSD messages — the alarm ladder

The mode narrates itself on the OSD. In plain terms: **TERRAIN AHEAD!** talks about *later* — keep this up and you will end up below your safety floor (default 60 m). The other two talk about *now*, when you are already too low: **TERRAIN! PULL UP!** means climbing will still get you out of it; **TERRAIN! TURN AWAY!** means climbing is no longer enough — turn aside. Higher-priority messages take the slot; while the autopilot is auto-climbing under a warning, the two alternate so you always see both the danger and the action.

| Message | Kind | Meaning | What to do |
|---|---|---|---|
| **TERRAIN! TURN AWAY!** | Warning | Too low **and** climbing can no longer save you (full climb still loses, or you're sinking, or pinned at your altitude ceiling). | **Turn away immediately** (manual + full throttle). Don't count on pulling up. |
| **TERRAIN! PULL UP!** | Warning | Too low — below (floor − 5 m), i.e. under 55 m at the default floor — but climbing **still works**. | Release the stick — the autopilot climbs at full rate — or pull **if `nav_fw_manual_climb_rate` > `nav_fw_auto_climb_rate`**; if in doubt, turn as well. |
| **TERRAIN AHEAD!** | Caution | The forward scan says that at this speed, even a full-rate climb would leave you **below your safety floor** somewhere ahead — not necessarily hitting anything, just lower than the floor you set. **You have time now.** | Turn, slow down, or climb early — while it's cheap. |
| **TERRAIN VS MAX ALT** | Caution | The terrain needs more altitude than your `nav_max_altitude` ceiling allows (only if a ceiling is set). | Raise/remove the ceiling, or accept reduced clearance at the wall. |
| **TERRAIN AUTO CLIMB TO MIN** | Info | You engaged (or ended up) below the safety floor — the minimum height above ground the mode will hold (default 60 m) — and the autopilot is already climbing you back up; the climb levels off about 10 m above the floor. | Nothing — it's handled. |
| **TERRAIN NOT READY** | Info | Engagement refused — no usable terrain data yet. | Fly stock cruise; check tiles / SD card if it persists. |
| **TERRAIN LOST - ALT FROZEN** | Caution | The terrain data stopped coming while the mode was holding your height — the altitude target is **frozen at the last good value** (it never descends on dead data). | Never descend blind; navigate out on the frozen altitude (it resumes after a few seconds of good data). |
| **TERRAIN LOOKAHEAD OFF** | Info | The forward scan is unavailable (no heading estimate, or ground speed below 3 m/s). | Only the reactive floor guards you now — do **not** expect the early “TERRAIN AHEAD!” warning while this shows. |

> **Seeing "TERRAIN! PULL UP!" often while riding low? That is the geometry of the floor, not a fault.** The floor (default 60 m) is already the lowest safe height, so the warning band sits only 5 m under it — below a minimum there is no room to spare. And the mode does not glue AGL to the ground 1:1: the climb is rate-limited, so on rising slopes the real height sags a little under the held value, which is exactly when the warning earns its keep. **If you want quiet — hold height in hand:** engage at 80 m and it holds 80; the warning still starts only below 55 m, so you fly with 25 m of calm. Engage below the floor and it simply climbs to ~10 m above it by itself (*TERRAIN AUTO CLIMB TO MIN*), no alarm while the climb is winning.

> **About `nav_max_altitude`:** that ceiling is **barometric altitude above your Home point** — it is *not* terrain-AGL aware. So over rising ground the terrain can legitimately need more altitude than the ceiling allows; that is exactly when *TERRAIN VS MAX ALT* appears.

## Waypoint missions

*This section describes an addition of this fork; it is not in upstream INAV.*

A waypoint (WAYPOINT or POSHOLD_TIME) can ask for terrain handling on the leg flown **toward** it. The option travels with the mission in `p3`, so it is set per waypoint in the mission planner and needs no mode switch. The TERRAIN AGL HOLD switch plays no part in missions; `terrain_enabled` must be ON.

| Terrain option | What the leg does |
|---|---|
| **Off** | Stock: a straight line from the altitude at the start of the leg to the waypoint altitude. |
| **Floor** | The stock line, except that the altitude target never drops below terrain + minimum + 10 m. Once the terrain allows, the aircraft is back on the stock line. |
| **Follow** | Keeps at least a set height above the ground for the whole leg: the height the waypoint's stored altitude leaves above the terrain at the waypoint (never less than minimum + 10 m). The aircraft comes down with the terrain after an obstacle. It does not trace the ground: it stays up over a dip shorter than about 20 s of flight. |

- **A terrain leg flies a line over the ground ahead, not the ground's contour.** An aircraft answers its altitude target with a lag of seconds, so a target that follows every step of the map makes it pitch for each one. Instead the leg looks 20 s of flight ahead along its route (within `terrain_nav_lookahead`) and flies a line. Going up, the line is aimed at the point in view that needs the steepest climb from where the line is, so the climb starts before the ground rises and stairs are climbed at one steady rate. It stays level over a dip whose far side is in view. It comes down only when every point in view is lower, and then to the highest of them. The height it keeps above ground is the leg's: minimum + 10 m on a Floor leg, the Follow height on a Follow leg. Over uneven ground the aircraft is therefore higher than that height by up to the depth of the dips it does not enter.
- **The minimum is never traded for a smoother flight.** Whatever the view said, the line is never under the ground below the aircraft + minimum + 10 m, and never under the look-ahead's climb line (below), which looks further ahead for ground the aircraft has to climb hard for.
- **Minimum per waypoint.** Each waypoint can carry its own minimum height above ground (50 – 120 m) for its leg; without one the leg uses `terrain_nav_min_agl`.
- **The look-ahead follows the route.** In a mission the forward scan runs to the waypoint and then round the turn toward the next one. When the next leg has a terrain option too, it goes on along that leg, using that leg's minimum there. When it has none, the scan ends where the aircraft is on course for it (the turn guard, below). The aircraft starts climbing before the waypoint for higher ground or a higher minimum after the turn, and does not react to ground straight ahead that the route turns away from. The next waypoint is the one the mission flies to: a JUMP is followed to its target. Past a waypoint the mission does not fly on from as a leg (RTH, the end of the mission) the scan stops at the waypoint.
- **The look-ahead scans the turns as a band.** A turn is scanned where the aircraft may fly it, not as one line. The scan starts along the course over ground and turns toward the waypoint along several paths at once: the tightest at the bank the navigation controller plans its turns for (`nav_fw_bank_angle`, capped by `max_angle_inclination_rll` and lowered by the controller's energy guard when a climb stalls in the turn), the widest at half that bank. The ground between those paths is sampled too, and the highest ground of the whole band counts. An aircraft that banks less than planned — climbing at full throttle, or in wind — flies a wider turn over other ground; the band covers it.
- **The turn at a waypoint is scanned before the aircraft gets there.** With `nav_fw_wp_turn_mode = COORD_FLYBY` the firmware switches to the next waypoint up to `nav_fw_wp_turn_max_lead_time` before reaching the current one, and the aircraft may then take a few hundred metres to come round, over ground neither leg crosses. The scan leaves for the next waypoint at that same point and turns as the aircraft will, so the climb for ground under the turn starts on the leg before it. Each path counts on half the usual climb while it turns, because a banked aircraft climbs worse.
- **The scan draws the turn each `nav_fw_wp_turn_mode` flies.** COORD_FLYBY: the corner cut. DIRECT and COORD_FLYOVER: over the waypoint, then round outside the corner. COORD_FLYINTO: the swing to the far side of the leg before the waypoint, then the arc across it. An arc the turn coordinator is already flying is followed the way it goes round, the long way if that is the arc's. With `nav_fw_wp_tracking_accuracy` on, the paths head back onto the next leg's line as the tracker does, not straight for the next waypoint.
- **The turn guard: a terrain leg protects its own turn.** The aircraft turns onto the next leg over ground that neither leg crosses: past the waypoint on the outside of the corner, inside it, or beside the leg before it. When the next leg has no terrain option, the terrain leg's floor (its minimum + 10 m, also after a Follow leg) stays after the firmware has switched to the next waypoint, until the aircraft is on course for it: the course over ground within 15° of the bearing to the waypoint for 2 s, no turn arc in progress and, with `nav_fw_wp_tracking_accuracy` on, also within 15° of the leg's course. It ends in any case after the time of a full circle at half the planning bank (20 – 90 s). After that the leg is a stock leg and flies its stock line, through ground if it is drawn that way. Without `nav_fw_wp_tracking_accuracy` the aircraft heads for the waypoint, not for the leg's line, so after a turn that went wide it can still be a few hundred metres beside the line when the guard ends, over ground the leg does not cross; it comes down to the stock line there. The guard only ever adds height: where the stock line is above the floor it changes nothing, except that while it is on the target comes down no faster than a terrain leg's does (half of `nav_fw_auto_climb_rate`), so over ground that falls away faster it trails the stock line by a few metres for a few seconds. A descent on the stock leg therefore starts when the turn is over, not at the waypoint, wherever the ground under the turn would put it under the minimum. Without terrain data the guard does not act: the turn is flown as stock.
- **What the band costs.** A turning scan reads up to 64 more points than a straight one, nearest first; the far end of a wide band waits for a later scan. The band never makes a scan touch more terrain blocks than the block cache allows for one scan, so it adds SD card reads only where the wider turn really crosses into another block (720 m × 840 m each). A mission scan is too long for one cycle of the control loop (up to 685 µs measured on an H7, in a 500 µs loop), so the firmware runs it in pieces: a scan starts every 100 ms from where the aircraft is then, a small share of it runs in each cycle, and its answer is used when the last piece is done, about 10 ms later. `debug_mode = TERRAIN_COST` shows the time and the number of points.
- **Stored altitudes stay ordinary altitudes.** Stock firmware, or this firmware without usable terrain data, flies the stock line to the stored altitude. Plan with that in mind: between two waypoints that line can pass through ground the terrain option would have cleared.
- **No terrain data at the start of a leg** (`terrain_enabled` OFF, no card, no tile): the leg flies stock and the OSD says *TERRAIN NOT READY*.
- **Terrain data lost during a leg:** the altitude target never drops below the last terrain target (it still follows a higher stock line), the OSD says *TERRAIN LOST - ALT FROZEN*, and the leg resumes after a few seconds of good data.
- **Follow and the altitude reference.** With a sea-level altitude the held height is altitude minus map terrain at the waypoint, wherever the aircraft took off. With an altitude relative to home it also depends on the terrain height of the actual take-off point, so it shifts if you take off somewhere other than planned.
- **The altitude target leads the aircraft.** The altitude controller commands a climb rate in proportion to the altitude error (`nav_fw_alt_control_response`: 0.4 m/s per metre of error at the default of 40). An aircraft on a rising target would therefore trail it by (climb rate ÷ response × 100) metres: 25 m at 10 m/s. A terrain leg hands the controller its line raised by that lag: the lag of the rate the line itself rises at, smoothed over a second so that a change of slope reaches the controller as a ramp, and at most that of the full `nav_fw_auto_climb_rate`. A line that comes down has no lead. The look-ahead of a mission leg plans with the ground speed smoothed over about 10 s: an aircraft slows when it pitches up, and with the speed of the moment its own climb would take the climb asked for away again. The look-ahead's climb line also aims at minimum + 10 m, not at the bare minimum. In a log the commanded target (debug slot 3) reads above the line (debug slot 4, as a height above the ground under the aircraft) while the line rises. An aircraft that cannot climb as fast as the ground rises still sags below the minimum, and *TERRAIN! PULL UP!* appears. The cruise hold has no lead: it trails a rising target as before.
- **A terrain leg descends at half of `nav_fw_auto_climb_rate` at most** (2.5 m/s at the default of 500). The look-ahead plans every climb from level flight, and an aircraft diving at the controller's full rate meets the next rise later than planned. Past a ridge the aircraft comes back down to its height more slowly than the ground falls away, and the last metres down are eased. The line does not come down while a point in its view has no terrain data.
- **From one terrain leg to the next the line goes on.** A new leg needs about 0.2 s for its own look-ahead scan, and a Follow leg for the terrain height at its waypoint. Until it has them, the line does not come down. Before the waypoint the line is already aimed at the next leg's minimum + 10 m past it, so a higher minimum on the next leg is climbed to on the way there.
- **With `terrain_nav_lookahead = 0`** there is no view: the line is the ground under the aircraft + the height the leg holds, and the aircraft follows the ground as closely as its lag allows.
- The OSD messages and `debug_mode = TERRAIN_NAV` work as in cruise. Debug slot 0 adds: bit 3 mission leg with a terrain option, bits 4–5 its option (1 floor, 2 follow, 3 the turn guard on a leg without one), bit 6 follow height known, bit 7 terrain at the waypoint known. On a mission leg debug slot 4 is the height of the line above the ground under the aircraft.
- Fixed wing only. LAND, RTH and JUMP items never carry a terrain option.

MSP2 command `0x2F02` (`MSP2_INAV_TERRAIN_HEIGHT`) returns the terrain height the SD card map has at up to eight positions, so a planner can check that the card holds the tiles a mission needs. It is refused while armed.

## Watching it in a log

`set debug_mode = TERRAIN_NAV` publishes the hold's state on the eight debug values (blackbox log and the Configurator's debug view). Otherwise the mode is dormant — nothing is read or computed for it.

| Slot | Value |
|---|---|
| 0 | Flags: bit 0 terrain data healthy, bit 1 AGL valid, bit 2 hold engaged; bits 8–9 / 10–11 / 12–13 / 14–15 the estimator's position / velocity / heading / altitude status (0 none, 1 usable, 2 trusted) |
| 1 | Height above ground, cm (0 when not valid) |
| 2 | Estimated altitude, cm |
| 3 | Commanded altitude target, cm |
| 4 | Held AGL, cm (0 unless engaged) |
| 5 | Hold status: 0 inactive, 1 waiting for usable terrain data, 2 active, 3 frozen |
| 6 | Warning: 0 none, 1 *TERRAIN NOT READY*, 2 *TERRAIN LOST - ALT FROZEN*, 3 *TERRAIN VS MAX ALT*, 4 *TERRAIN! PULL UP!*, 5 *TERRAIN LOOKAHEAD OFF*, 6 *TERRAIN AUTO CLIMB TO MIN*, 7 *TERRAIN AHEAD!*, 8 *TERRAIN! TURN AWAY!* |
| 7 | Navigation state × 1000 + a running counter |

## Safety & limitations

- **Fixed-wing only.** It never runs on multirotors, rovers, or boats, and it works **only inside 3D Cruise** (mandatory at this stage).
- **RTH does not use it, and missions only where a waypoint asks for it** (see *Waypoint missions*). The mode **steps aside cleanly** (hands the aircraft back to stock behaviour) during launch, autoland, emergency landing, RTH, mission legs without a terrain option, and on **GPS loss**. Stock RTH is terrain-blind: it does not follow the ground — **so keep the aircraft in line of sight:** holding a low AGL down the far side of a hill puts terrain between you and the aircraft, the radio and video links drop, and a failsafe RTH whose return altitude is below the ridge heads home *into* the hill. For terrain flights set `nav_rth_alt_mode = MAX`: the aircraft returns at the highest altitude of the whole flight, clearing anything it flew over on the way out, with your held height as margin (it cannot know about high ground you never overflew — on unfamiliar terrain keep extra margin). Or simply: don't fly where you lose line of sight.
- The **floor is deliberately high** (default 60 m, minimum 50 m). Terrain data and GPS each carry a few metres of error, so keep clearance in hand — don't set the floor low.
- **Not a rangefinder replacement.** The value is the ground elevation at your GPS position from the map — it does **not** see trees, buildings, wires, or the actual nearest object below you.
- **Never stacks with the rangefinder** — it refuses to engage while SURFACE mode is active.
- If terrain data is lost while engaged, the target **freezes at the last valid altitude** (*TERRAIN LOST - ALT FROZEN*) — it never descends on dead data.
