# House navigation experiments

Work in progress: no configuration has yet passed full-house exploration with
zero obstacle contacts. Fast early mapping and successful action counts are not
proof of completion. Generated logs live in `log/house-speed-tuning/` relative to
the workspace root. Small retained results are in `evidence/house-speed/`.

## Established failure mechanisms

* **FAIL — already-reached goal loop.** `direct-075-450` sent goals about 15 cm
  away with a 25 cm position tolerance. Nav2 immediately succeeded repeatedly,
  without moving or revealing the remaining house. The next profile uses 10 cm
  goal/planner tolerances and a 30 cm dispatch minimum. A successful action is
  not counted as evidence of new map coverage.
* **FAIL — room blocked by circular footprint.** In `consistent-075-450`, the
  saved map at simulation second 191 has a connected route at 18 cm clearance,
  but not at 20 cm. The 24 cm circular footprint closes that route. This finding
  concerns that SLAM snapshot, not the true architectural width of the doorway.
* **FAIL — collision with a low table base.** `lattice-consistent-075-450` used
  a measured polygon and an orientation-aware lattice planner, but Gazebo
  reported base/wheel contacts with `cafe_table::link::base`. The SDF base is
  0.56 by 0.56 by 0.04 m; the column is only 0.042 m wide. A horizontal laser
  above the base cannot directly measure its full outline. This profile is
  rejected. Collision checks must remain enabled; a clear laser scan does not
  prove body clearance.
* **Confirmed source issue — explorer coordinate frames.** The pinned explorer's
  `localCostmapCallback` passes the grid directly to the core. Goal cost checks
  call `worldToMapNoThrow` with map-frame points without a TF conversion. Nav2's
  local grid is in `odom`. As SLAM changes map-to-odom, the wrong cell is checked.
  The candidate workaround assigns `/global_costmap/costmap` to the explorer's
  `local_costmap_topic` as well as `costmap_topic`. Both then share `map`.
  Nav2's real local obstacle layer and controller remain active and unchanged.
* **Confirmed completion limitation.** The pinned explorer reports completion
  when no candidate survives its frontier search. It latches a finished flag
  and stops dispatching. This is not a geometric test that every room was mapped.
  Disabling the completion event alone would only hide the notification.

The polygon `[-0.21,-0.17], [0.08,-0.17], [0.08,0.17], [-0.21,0.17]`, with
1 cm padding, encloses the installed Waffle's box and wheel collision geometry
in `base_link`. It is specific to this simulation model. A 2-D point planner
plus that polygon was also rejected; narrow-space planning needs orientation
checks. The differential-drive lattice supplies those checks and in-place turns.

## Reproduce a diagnostic run

From the workspace root, with no other Gazebo house instance running:

```bash
pixi shell
source install/local_setup.bash
python src/midterm/tools/benchmark_house.py \
  --params PATH_TO_NAV_YAML \
  --frontier-params PATH_TO_FRONTIER_YAML \
  --slam-params PATH_TO_SLAM_YAML \
  --contacts --seconds 600 --output log/NEW_UNIQUE_DIRECTORY
```

The helper launches the five requested components in DDS domain 96 and cleans
up only its own processes. It records odometry, known map area, completion events,
stall snapshots, active controller settings, and Gazebo collision pairs. New runs
also save action feedback and a map snapshot every 30 wall seconds. Contact
monitoring starts at first odometry, before exploration motion. Ground contacts
by wheels/casters are expected; contacts with furniture or walls fail the test.
Known area includes occupied cells and is not a percentage of house completion.

## Validation status

| Check | Status |
| --- | --- |
| Diagnostic helper Python syntax and feedback-message import | PASS |
| Recorded aggressive-profile failure reproduced and retained | FAIL (profiles rejected) |
| Tighter tracking at 0.5 m/s | IN PROGRESS; no acceptance claim |
| Explorer frame workaround in a full run | NOT TESTED |
| Every room mapped and exits traversed without collision | NOT TESTED |
| Repeated clean-start successful exploration | NOT TESTED |
| Physical hardware at these speeds | NOT TESTED; simulation configuration |

The source configuration README is the operational entry point. Experimental
YAML in `log/` is not automatically loaded by the normal launch commands.
