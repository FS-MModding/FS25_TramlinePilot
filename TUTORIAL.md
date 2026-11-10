# Tramline Pilot: how to use it

## Getting started

- You need Farming Simulator 25 in single player and a sprayer or fertiliser spreader. AutoDrive is only needed for refills.
- **Install:** download `FS25_TramlinePilot.zip` from [Releases](https://github.com/FS-MModding/FS25_TramlinePilot/releases). Put it, still zipped, in `Documents/My Games/FarmingSimulator2025/mods`, then tick Tramline Pilot in the mod list when you load your savegame.

1. Stand on the field and press Right Ctrl+M.
2. Set Tramline spacing to your boom or spreading width.
3. Pick a Direction and the number of Headland passes.
4. Press Enter to generate the tramlines.
5. Fill your sprayer or spreader and drive onto the amber headland tramline.
6. When the green Start here flag shows, press Alt+T.
7. Press Alt+T or Alt+Q to stop.

## Setting up a field

- **Direction:** longest edge, vehicle heading, A-B points (Alt+1, Alt+2), an angle, or as stored.
- **Tramline spacing:** your boom or spreading width (6-60 m).
- **Headland passes:** 0-4 tramlines round the field edge.
- **Worker turns:** along the headland tramline, or loops where they fit.
- **Turn radius:** your machine's turning circle (4-15 m).
- **Headland corners:** round, or back into corners (self-propelled sprayers, mounted sprayers and spinners).
- **Spray headland:** first, last or skip.
- **Work speed / Turn speed:** for spraying or spreading, and for turns.
- **Refill with AutoDrive:** on or off.
- **Create tramlines in crop:** no, wheel tracks or a full lane.
- **Wheel track width:** the distance between your sprayer's wheel centres (1.50-4.00 m). The width of the machine you are sitting in is marked "your vehicle".
- **Show lines:** off, this field or all fields.
- **Precision Farming tramlines:** on, off on tramline fields, or off everywhere.
- **Buttons:** Enter generates, Space opens Saved wheelings, Clear field deletes the layout, Esc goes back.

## Seeing your tramlines

- Press Alt+4 to show or hide the lines.
- White lines are the main tramlines; amber lines are the headland tramlines.
- A green line shows where the worker sprays or spreads next; blue shows where it drives with the boom off.
- A green flag marks where the course starts and a chequered flag where it finishes.
- Last sprayed here (Last spread here for a spreader) marks where an unfinished job stopped.
- Field gate marks where AutoDrive takes over for a refill.

## Spraying or spreading with the AI worker

1. Drive onto a tramline: the headland tramline for a new job, or a main tramline to start from there.
2. Press Alt+T. The worker sprays or spreads the field and never leaves the tramlines. It stops at the end of the course.
3. Press Alt+T or Alt+Q to stop. Alt+Q also works from outside the vehicle.
4. To carry on, press Alt+T again:
   - On the spot where it stopped, it carries on.
   - Anywhere else, choose Drive back there, Carry on from here or New job.

- With Back into corners, the sprayer or spinner reverses into sharp headland corners so none are missed (not trailed machines).
- An unfinished job is kept when you save, so you can carry on another day.

## Refilling with AutoDrive

**Set it up once per field:**

1. In AutoDrive, put a marker at the field edge: the field gate.
2. Set that marker as the sprayer's AutoDrive target.
3. Set your refill point as the sprayer's 2nd target.
4. In the menu, set Refill with AutoDrive to On.

**When the tank or hopper runs empty:**

1. The worker folds up (if it has to) and drives on the tramlines to the gate.
2. AutoDrive takes it to refill and brings it back.
3. The worker drives back to where it ran out and carries on.

- If AutoDrive is stopped on the way, start AutoDrive again or press Alt+T on the field.

## Saved wheelings

1. Every layout is saved when you press Generate.
2. Next season, open the menu on the field and press Space (Saved wheelings).
3. Pick the one you want, then press Generate.

- Saved wheelings work in every savegame on the same map.

## Tramlines in the crop and Precision Farming

- **Create tramlines in crop:** choose Wheel tracks or Full lane and set Wheel track width to match your sprayer's wheels (a full lane is 1 m wider), then press Generate. The crop is removed along the tramlines straight away.
- **Precision Farming tramlines:** choose Off on tramline fields or Off everywhere, so Precision Farming's seeders don't make their own tramlines.

## Keys

| Key | Action |
| --- | --- |
| Right Ctrl+M | Setup menu |
| Alt+T | Start or stop the worker |
| Alt+Q | Stop the worker |
| Alt+4 | Show or hide the lines |
| Alt+1 / Alt+2 | Set points A and B |

**Handy console commands:**
- `tlNewJob` forgets an unfinished job.
- `tlRefill` refills now.
- `tlPaint` creates the tramlines in the crop. Add a width, e.g. `tlPaint tracks 2.25`.
- `tlWheelings`, `tlLoad` and `tlSave` handle saved wheelings.
- `tlReport` adds a bug-report snapshot to the mod's log.

## If something goes wrong

- **"Drive onto the headland tramline":** you are not on a tramline; drive to the amber marker.
- **It won't back into corners:** it needs a self-propelled sprayer or a mounted sprayer or spinner, a headland pass and Back into corners.
- **Refill doesn't start:** check the AutoDrive target is a marker at the field edge and the 2nd target is the refill point.
- **The unfinished job has gone:** it is dropped when you generate a different layout.
- **No lines:** press Alt+4.
- **It can't follow the corners:** set Turn radius to your machine's turning circle.
- **Gaps in the wheel tracks:** press Generate again with Create tramlines in crop set to Wheel tracks.

## Reporting a bug

- **The mod's own log:** Tramline Pilot writes `TramlinePilot_log.txt` to `Documents/My Games/FarmingSimulator2025/modSettings/FS25_TramlinePilot`. The session before is kept as `TramlinePilot_log_previous.txt`.
- **What's in it:** the mod's messages, the machine and settings of every job, and any error with what the worker was doing. Your Windows user name is left out.
- **When something goes wrong:** [open a bug report](https://github.com/FS-MModding/FS25_TramlinePilot/issues/new/choose) and drag that file into it, saying what happened. With the developer console on, type `tlReport` straight away to add a snapshot of the field, the job and the vehicle.
- **If the worker hits a script error:** it stops and brakes, and the error goes in the log.
