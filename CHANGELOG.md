# Changelog

## 1.1.0.2 (14 November 2026)

### New

- **Joined fields:** fields you have joined by ploughing or cultivating the land between them now count as one field, the way the game's own workers see them.
  - Open the setup menu on any part of it. The outline goes round the whole joined field, with one headland round all of it.
  - The menu header shows which fields are joined, for example "Fields 9 + 18 (joined)".
  - The tramlines, the worker, the lines in the field and Precision Farming all treat it as one field.
  - This works on land your farm owns. Contract fields are planned as the map has them.
  - To plan them separately again, sell one of them or put grass back between them, then generate the layout again.

### Fixes

- The headland no longer crosses itself in an X or splits into two loops on fields with stepped edges. Some maps (maps made with maps4fs, for example) paint field ground in 2 m squares, so every slanting edge is a staircase. Field 9 on Needham Hall Farm was one of these. The headland is now always one ring, a steady distance in from the edge.
- On those fields the worker could ask you to turn round when you were already facing the right way. That came from the crossed headland and is fixed with it.

## 1.1.0.1 (12 November 2026)

### Fixes

- On fields made from two old fields, where a hedge or wall between them has been taken out, the headland no longer splits in two round the strip where it stood. The strip isn't field ground, so the outline used to go round it. Tramline Pilot now plans straight over a strip like that (up to about 8 m wide) when the map has it as one field and nothing stands in it. Where a hedge, wall, fence or trees are still there, it still goes round.

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

- Tramlines made in the crop no longer turn into another crop. On a map with extra crops they showed as the last crop in the list (onion, for example) on the crop map ([issue #1](https://github.com/FS-MModding/FS25_TramlinePilot/issues/1)).
- The worker now brakes to a stop at the end of the job. Before, it could drive off with cruise control still on.
- If the worker hits a script error, it now stops and brakes instead of driving on unsteered, and the error goes in the log.

## 1.0.0.0 (9 November 2026)

- First release: tramline layouts for every field, tramlines shown in the field and made in the crop, saved wheelings, and an AI worker that sprays or spreads only on the tramlines, with AutoDrive refills.
