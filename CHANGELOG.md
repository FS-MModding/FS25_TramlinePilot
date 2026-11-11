# Changelog

## 1.1.0.0 (11 November 2026)

### New

- **Headland direction:** a new row in the setup menu, under Headland passes (it needs at least one headland pass). Choose As the vehicle faces, Clockwise or Anticlockwise.
  - With Clockwise or Anticlockwise, the worker always goes round the headland that way.
  - Where it turns off the headland into the tramlines, and back onto the headland when they are done, there is one curve each (going that way round) instead of two. Where a curve doesn't fit, that part of the field keeps both and the menu tells you.
  - The tramlines are still done up and down, turning one way at one end and the other way at the other end.
  - With Back into corners, the worker backs into each corner from the same side, so each corner needs one corner tramline instead of two.
  - If you start facing the wrong way, the worker won't set off. The start marker shows "Turn round" and which way the field goes.
  - Saved wheelings remember the direction.
  - As the vehicle faces works as before.
- **Mod log for bug reports:** Tramline Pilot keeps its own log in Documents/My Games/FarmingSimulator2025/modSettings/FS25_TramlinePilot/TramlinePilot_log.txt. The session before is kept as TramlinePilot_log_previous.txt. The log has the mod's messages, the machine and settings of every job and any error, without your Windows user name.
- **tlReport:** a console command that adds a snapshot of the field, the job and the vehicle to the log. Use it straight after something goes wrong.
- **Bug reports and suggestions** now go to [github.com/FS-MModding/FS25_TramlinePilot/issues](https://github.com/FS-MModding/FS25_TramlinePilot/issues). The link is in the mod's description too.

### Fixes

- The worker now brakes to a stop at the end of the job. Before, it could drive off with cruise control still on.
- If the worker hits a script error, it now stops and brakes instead of driving on unsteered, and the error goes in the log.

## 1.0.0.0 (9 November 2026)

- First release: tramline layouts for every field, tramlines shown in the field and made in the crop, saved wheelings, and an AI worker that sprays or spreads only on the tramlines, with AutoDrive refills.
