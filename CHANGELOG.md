# Changelog

## 1.2.0.0 (15 November 2026)

### New

- **Soil strip at the ends, for crops grown to the end of the field (potatoes, onions):** a new Headland choice, Soil strip 6-30 m, for fields with no headland tramline.
  - The tramlines stop at a strip of bare soil that wide at the ends of the field. Along the sides the crop goes right to the edge.
  - At the end of each tramline the worker drives on until the boom has left the crop, folds it, turns round on the strip, then unfolds lined up with the next tramline and sprays back up it.
  - Where the strip is too narrow to turn round in one go, it does a three-point turn (forwards, back, forwards). Self-propelled sprayers and mounted sprayers can back up; trailed sprayers turn forwards only, so they need a wider strip.
  - The turns are worked out for the machine you sit in when you press Generate: its length, where its boom is, and whether it can back up. When the worker starts, they are worked out again for the machine it drives.
  - With Create tramlines in crop on, the crop on the strip is cleared too, so the worker turns on bare soil.
  - The menu tells you if any turns don't fit. Make the strip wider or the turn radius smaller.
  - The menu row is now called Headland: 0-4 passes, or a soil strip.
  - A soil strip field starts at the start of its first tramline (the green flag). Refill with AutoDrive needs a headland tramline, so it is off on these fields.

### Changes

- On a field without a headland tramline, a worker started while it stands in the first tramline, facing along it, now sets off from there instead of driving round to come in at its end.

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
