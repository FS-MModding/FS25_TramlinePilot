# Tramline Pilot

**Farming Simulator 25 mod by MModding**

Tramline Pilot plans the tramlines for every field, shows them in the field and gives you an AI worker that sprays or spreads on them, and only on them. It works with sprayers, fertiliser spinners and other spreaders.

## Download

Get **FS25_TramlinePilot.zip** from [Releases](https://github.com/FF-MModding/FS25_TramlinePilot/releases). Put it, still zipped, in `Documents/My Games/FarmingSimulator2025/mods`, then tick Tramline Pilot in the mod list when you load your savegame.

The green **Code** button downloads the source files, not the mod.

## What it does

- **Plan your tramlines:** stand on a field and press Right Ctrl+M. The field outline is read from the map, and a live map preview shows the layout as you change it.
  - Direction: along the longest edge, your vehicle heading, A-B points (Alt+1 / Alt+2), any angle, or as stored.
  - Spacing 6-60 m (your boom or spreading width) and 0-4 headland passes.
  - Turn radius, worker turns, headland corners, when to do the headland, work and turn speed.
- **Create tramlines in the crop:** cut the wheel tracks (1.50-4.00 m apart, to match your sprayer) or a full lane straight away. The width of the machine you sit in is marked in the list.
- **See them in the field:** Alt+4 shows the lines, with a flag where the course starts and a chequered flag where it finishes.
- **Let the worker spray or spread:** park on the headland tramline and press Alt+T.
  - It works the headland and then every tramline, and never leaves the tramlines.
  - It reverses into sharp headland corners where the machine can.
  - It stops at the end of the course.
- **Stop and carry on:** stop the worker any time. The job is kept with the savegame, and Alt+T drives back to where it stopped and carries on.
- **Refill with AutoDrive:** when the tank or hopper is empty, the worker drives on the tramlines to the field gate. AutoDrive takes it to refill and back, and it carries on where it ran out.
- **Saved wheelings:** every layout is saved, so you can lay the same tramlines for the next crop or in another savegame on the same map.
- **Precision Farming:** leave its own tramlines on, switch them off on your tramline fields, or switch them off everywhere.

## Keys

| Key | Action |
| --- | --- |
| Right Ctrl+M | Setup menu |
| Alt+T | Start / stop the worker |
| Alt+Q | Stop the worker |
| Alt+4 | Show / hide the tramlines |
| Alt+1 / Alt+2 | Set A and B points |

## Works with

- **AutoDrive** (optional): only needed for the refill trips. Set the field gate as the vehicle's AutoDrive target and the refill point as its second target.
- **Precision Farming** (optional).
- Single player.

## Reporting a bug

1. Tramline Pilot keeps its own log: `Documents/My Games/FarmingSimulator2025/modSettings/FS25_TramlinePilot/TramlinePilot_log.txt`. The session before is kept as `TramlinePilot_log_previous.txt`. The log leaves out your Windows user name.
2. If you have the developer console on, type `tlReport` straight after the problem. This adds a snapshot of the field, the job and the vehicle.
3. [Open a bug report](https://github.com/FF-MModding/FS25_TramlinePilot/issues/new/choose) and drag the log file into it. Screenshots help too.

## Changelog

See [CHANGELOG.md](CHANGELOG.md).
