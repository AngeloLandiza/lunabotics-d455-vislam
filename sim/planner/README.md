# Planner head-to-head: custom stack vs Nav2

The plan's rule (Step 2, "Custom planner versus Nav2"): keep Nav2 unless a custom stack beats **tuned** Nav2 on cycle
time and contacts, on the same arenas, missions and robot model. This folder is that test.

Labels: **Verified** (read from a source or measured), **Inferred** (follows from runs or code here), **Assumed** (a
chosen number with no source). Every number the bench uses is in `arena2d.PARAMS` with its label and source.

## The bench (`arena2d.py`)

- **Arena:** the BP-1 sandbox arenas (`../bp1_sandbox/arena.py`), so seed 3000 is the same layout in both. Obstacles
  are discs (boulders, craters), the berm is a box the robot must not enter, the walls are the 6.8 x 5.0 m arena.
- **Robot:** the team URDF body, 1.0 x 0.7 m about base_link (Verified), track 0.54 m (Verified), wheel speed
  0.75 m/s and acceleration 0.5 m/s^2 (Assumed).
- **Slip:** a skid-steer model where the wheels slip and turning needs more wheel travel than the track width says
  (ICR expansion). Three levels, all Assumed, as no BP-1 data exists for this robot:
  none (0, 0, 1.0, 0), nominal (slip 0.10 +- 0.05, ICR factor 1.5 +- 0.15), heavy (0.25 +- 0.08, 2.0 +- 0.25), redrawn
  every second.
- **Mission:** start in the start zone, then 3 cycles of dig point -> dump pose at the berm -> next dig point. A dump
  pose faces the berm with the front 5 cm from it. A leg passes if the robot ends within 10 cm and 10 deg and stands
  still; a dump counts as on target within 5 cm and 5 deg.
- **Scoring:** leg time from leg start to wheels stopped at the goal, planning time excluded and reported apart
  (this machine, not the Orin NX). Contact = any part of the footprint inside an obstacle, the berm box or a wall.
  Planners get the true pose; localization is tested elsewhere (`../bp1_sandbox/localize.py`).

## The planners

| Name | What it is | Where |
| --- | --- | --- |
| `gcs` | The planned custom stack: convex free-space regions (Drake IRIS), a shortest route through them (Drake GCS, straight pieces, Clarabel), corners rounded, then a minimum-time speed profile (TOPP-RA) in wheel coordinates | `custom_gcs.py` |
| `tangent` | A second custom candidate: the exact shortest route around the inflated obstacles (straight tangents and arcs, Dijkstra), with the body's footprint checked along every edge of the route and edges that touch dropped before searching again. Where that finds nothing, a fallback search over (x, y, heading) on a 5 cm, 5 deg grid with straight moves forwards or backwards and turns in place, shortened by straight cuts | `tangent_route.py`, `lattice_route.py` |
| Nav2 | Smac State Lattice (5 cm diff-drive primitives) + MPPI, default BT with replanning and recoveries; runs on ROS 2 Humble with an RViz window | `nav2/` |

Both custom planners hand their route to the same executor (`execute.py`): stop and turn at corners sharper than
45 deg, follow each piece on its TOPP-RA speed schedule with a Kanayama steering law. A route that ends with a straight
run along the goal heading (0.3 or 0.6 m) stops where the run begins, aims at the goal point and drives straight in, so
an error left by the curve becomes a small heading error rather than a sideways miss; the dump spot is only 5 cm from
the berm. At the goal the robot closes the last distance along its heading only, then turns in place. Every turn in
place is checked for the footprint it sweeps. Each tries obstacle inflations of 0.45, 0.40 and 0.37 m and takes the
first that gives a route.

`world_cache/` holds the 100 test arenas (seeds 3000-3099) so every machine runs the identical layouts.

Drake has no Linux ARM wheel on PyPI (checked 2026-10-12), so `gcs` as written does not run on the Orin NX without a
source build; `tangent` needs numpy, scipy and toppra, no Drake.

## Running

Custom planners (here or anywhere with `pip install drake toppra clarabel`):

    python3 compare.py results/compare_v1 --seeds 100 --workers 4     # tangent + gcs, 3 slip levels

Nav2 (ROS 2 Humble with nav2_bringup, nav2_smac_planner, nav2_mppi_controller):

    nav2/run_nav2.sh results/nav2/nav2_default.json --slip nominal --seeds 2          # smoke test, check cmd ~20 Hz
    nav2/run_nav2.sh results/nav2/nav2_default.json --slip none,nominal,heavy --seeds 100

Tuned Nav2: `nav2/tune.sh results/nav2_tune` runs a first set of variants (MPPI critic weights, sampling, costmap
decay, lattice turn penalty; `--set` overrides on the params file) on tuning arenas 1000+, kept apart from the test
arenas, then the best variant goes through `run_nav2.sh` on seeds 3000-3099 with the same `--set` flags.

`run_nav2.sh` fills in the lattice file, starts Nav2 with sim time and runs `nav2/sim_bridge.py`, which steps the
same robot model, publishes /clock, /odom, TF and /map, sends each leg as a NavigateToPose goal and scores it the same
way. Sim time stands still until Nav2's first plan for a leg, as planning is left out of the custom planners' times too.

Side by side:

    python3 compare.py results/merged --merge results/compare_v1/compare.json,results/nav2/nav2_default.json

## Results

Custom planners on the 100 test arenas (2026-10-12; `results/compare_v3` for `tangent`, `results/compare_v2` for
`gcs`; Inferred, sim):

| Slip | Planner | Missions complete | Legs with contact | Mission time median / 95th pct | Dumps within 5 cm, 5 deg | Plan time median / 95th pct |
| --- | --- | --- | --- | --- | --- | --- |
| none | tangent | 82 of 100 | 6 | 100.7 / 153.4 s | 100% | 0.35 / 4.99 s |
| nominal | tangent | 82 of 100 | 2 | 120.8 / 179.6 s | 100% | 0.34 / 4.81 s |
| heavy | tangent | 80 of 100 | 1 | 146.9 / 202.1 s | 100% | 0.33 / 4.75 s |
| none | gcs | 18 of 100 | 0 | - | 100% | 2.29 / 7.47 s |
| nominal | gcs | 19 of 100 | 0 | - | 100% | 2.17 / 6.98 s |
| heavy | gcs | 19 of 100 | 1 | - | 100% | 2.17 / 7.19 s |

Every incomplete mission stopped at its first dump leg, the crossing from the dig area to the berm. A configuration
space check (5 cm, 5 deg) finds no way through for the body in about a third of the arenas where the first run
failed, even at 2 cm clearance. The obstacle layout is Assumed, because Figure 9 is unreadable, so the arenas may be
denser than the real pit.

### Nav2 against the custom planner (decision, 2026-10-12)

Nav2 Humble (Smac State Lattice + MPPI) ran on the same bench on Angelo's laptop (`results/nav2*`; Inferred, sim):

| Run | Arenas | Missions complete | Legs with contact |
| --- | --- | --- | --- |
| Nav2, default settings | test 3000-3099, 3 slip levels | 3 of 300 (none 1, nominal 2, heavy 0) | 6 |
| Nav2, 12 tuned settings (`nav2/tune.sh`, two rounds) | tuning 1000-1019, nominal slip | best 1 of 20 (`combo`, 17 contact legs; `finalturn`, 1 contact leg) | 0 to 17 |
| `tangent` | tuning 1000-1019, nominal slip | 17 of 20 | 1 |
| `tangent` | test 3000-3099, 3 slip levels | 242 of 300 (82, 82, 80) | 9 |

**Decision: the custom stack (`tangent` routes + the shared executor) replaces Nav2 for driving between poses; Nav2
stays the fallback** (Angelo, 2026-10-12). Why Nav2 failed here, from its logs:

- At a goal MPPI turns in place at up to 1.85 rad/s and stops; the wheels' spin-down (0.5 m/s^2, Assumed) carries the
  body 10-50 deg past the target heading, it turns back, overshoots again, and the progress checker's spin recovery
  adds more turning until the 60 s leg timeout. Humble's MPPI has no acceleration limits in its motion model.
- A 0.5 rad/s final turn (`finalturn`) stops the overshoot, but near rocks the 0.70 m safety margin keeps the robot
  0.2-0.9 m short; with a 0.45 m margin (`finalcombo`) it gets within 0.27 m and still does not close the last
  0.2 m to the 8 cm goal tolerance; with the small margin and fast turns (`inflate`, `combo`) it hits rocks.
- Some dump poses (5 cm from the berm) get no Smac plan within 10 s.

Caveats: the arenas' obstacle layout is Assumed (Figure 9 unreadable) and may be denser than the real pit; the wheel
acceleration and slip are Assumed; Nav2 was tuned by parameters only, not by a custom behaviour tree or a rotation
controller. The custom executor's stop-then-turn steps are what Nav2 lacks here, so the gap is about the final
approach, not route finding.
