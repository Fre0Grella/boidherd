# Design log

Decisions taken during the project, newest at the bottom, so the report can
explain *why* things are the way they are. Each entry: what we chose, why,
and what we turned down. Citations refer to [references.md](references.md).

## 2026-10-14 · Research question

A flock of n drones spawns at random in sector A. Only j of them (the
leaders) know where sector C is; the others just follow Reynolds' rules.
Can the leaders bring the whole flock to C, through an obstacle course in
sector B? Experiments vary the map, n and j.

This is the informed-individuals question of Couzin et al. (2005): leaders
are not recognised by the others, they are ordinary boids with an extra pull
toward the goal. Their result to compare against: the larger the group, the
smaller the fraction of informed individuals needed for accurate guidance.

The prototype testbed (`rb_test`, e-puck robots with infra-red range &
bearing) is the predecessor of this repo.

## 2026-10-14 · Robot: Crazyflie drones instead of e-pucks

Reynolds' boids are point masses: each rule produces an *acceleration*, and
velocity and position follow. An e-puck cannot behave like that: its wheel
speeds set its velocity directly (no momentum) and it only moves where it
faces. The Bitcraze Crazyflie model shipped with Webots R2025a flies, so it
has real inertia.

Useful facts about the Webots model:
- `extensionSlot` to mount extra devices (our IR emitter/receiver);
- range sensors `range_front/left/back/right`, `inertial_unit`, `gps`, `gyro`;
- motors `m1_motor`..`m4_motor`;
- a Python cascaded controller, `pid_velocity_fixed_height_controller`
  (velocity -> desired tilt -> attitude -> motor mixing), in
  `projects/robots/bitcraze/crazyflie/controllers/crazyflie_py_wallfollowing/`.

Turned down: a spherical robot (Sphero BB-8, underactuated and hard to
steer, 0.25 m radius by default), a ballbot (balancing becomes the
project), and a "fake inertia" e-puck (kept only as a quick way to test the
rules).

## 2026-10-14 · Acceleration becomes tilt, not a velocity setpoint

If the flocking law sent velocity setpoints to the stock controller, the
drone would track them tightly and behave like a floating e-puck. Instead
the boid acceleration is turned straight into a desired tilt, skipping the
velocity loop (`pitch ~ a_x / g`, `roll ~ -a_y / g`, signs to be checked in
simulation). The drone's momentum is then integrated by the physics engine,
not by our code. Vertical acceleration (3D phase) goes to thrust, or, more
simply, to the altitude setpoint.

## 2026-10-14 · Alignment uses broadcast velocity, not heading

A quadrotor can fly sideways, so its yaw says nothing about where it is
going. Each drone broadcasts its world-frame velocity over IR (`id;vx;vy;vz`);
all drones measure yaw in the same world frame with their IMU, so the
vectors are comparable. Followers use GPS only to estimate their *own*
velocity; only leaders use position to navigate toward C.

## 2026-10-14 · 3D-ready from the start

All vectors are 3D. A single `VERTICAL_GAIN` parameter switches between
fixed-altitude flocking (0, comparable with the e-puck runs) and full 3D.
Relative neighbour positions from `getEmitterDirection()` are in the body
frame and must be rotated to the world frame with roll/pitch/yaw, since the
drone tilts whenever it accelerates.

For 3D: spawn at different heights (otherwise symmetry keeps the flock
flat), a floor/ceiling avoidance rule, lower weight or speed limit for
vertical motion, and ellipsoidal separation (bigger vertically) because
Webots does not simulate propeller downwash.

## 2026-10-14 · Obstacle avoidance: Reynolds' steer-to-avoid

Reynolds (1987) compares a repulsive force field with *steer-to-avoid* and
prefers the latter: the force field slows a boid flying head-on into a wall
instead of turning it, and barely acts on a boid flying parallel to it.
Steer-to-avoid looks ahead along the path and steers past the nearest edge
of the obstacle. On the Crazyflie, `range_front` is the look-ahead probe if
the drone yaws to face its velocity; extra DistanceSensors in
`extensionSlot` cover up/down for the 3D phase.

## 2026-10-14 · Phased plan

Each phase is a complete deliverable on its own.

1. Flat flocking: fixed altitude, no obstacles, no leaders. The flock forms
   using only IR range & bearing.
2. Flat + j leaders, empty map: the n x j experiment (core result).
3. Obstacles: pillars, then a wall with a gap; steer-to-avoid.
4. 3D manoeuvres: `VERTICAL_GAIN > 0`, obstacles to fly over.
5. Hybrid leaders: before setting off, each leader plans a path to C on
   the known map (deliberative layer), then pulls toward its next waypoint
   instead of straight at C; Reynolds' rules and steer-to-avoid stay the
   reactive layer. Followers don't change.

Metrics, mostly from the supervisor's ground truth: fraction of drones in
C, time to arrival, number of separate groups, minimum inter-drone
distance, polarization, crashes (a crash counts as a failure; no recovery).

## 2026-10-14 · Maps: TOML files loaded into dataclasses

Each map is a TOML file (read with the standard library's `tomllib`, so no
extra dependency inside Webots' Python). It is loaded into Python
dataclasses that define and validate the format; procedural obstacles (e.g.
a random pillar forest) are named generators implemented in Python.

A generator compiles (map, n, j, seed) into a self-contained `.wbt`:
- leaders get the goal in `Robot.customData` (the field documented for
  per-robot configuration);
- the supervisor gets the sectors in `controllerArgs`.

Controllers never parse map files; any generated world can be opened and
run by hand.

Precedents: ARGoS experiment files with `<distribute>` for random
placement (Pinciroli 2012); Scenic, a scenario language with a Webots
interface (Fremont 2019); Webots' own procedural PROTO nodes.

Turned down: maps as pure Python code (mixes data with code, harder to say
which map a result came from); pure data with no generators (cannot express
procedural layouts).

## 2026-10-14 · Map format: what a map contains

- A map describes geometry only. n, j, the seed, IR range and boid weights
  belong to the run, so one map serves the whole n x j grid.
- Only sectors A (spawn) and C (goal) are declared, as axis-aligned rectangles;
  "B" is simply the rest of the course.
- Procedural obstacles have a fixed layout per map (seed in the map file).
  The run seed changes only spawn positions and which drones are leaders, so
  a change in results can't come from a different obstacle layout.
- The arena is large and the spawn region relaxed, so crashes happen only
  when the behaviour is bad, not by accident at spawn.
- The floor is one rectangle covering the bounding box of the arena; the
  parts outside the walls are unreachable, so they don't need cutting.
- Walls have no height field: they are as high as the flight ceiling, so
  no IR ray between two drones can pass over them.
- The arena walls block IR, so a sharp turn cuts the line of sight between
  the front and the back of the flock. Expected effect to measure: does
  success drop at the turn, and does it depend on j?

## 2026-10-14 · Arena shape: union of rectangles

The free space of the arena is the union of a list of axis-aligned
rectangles, each given by two opposite corners; the walls follow the
outline of the union. An L-shaped hall is two overlapping rectangles:

```toml
[arena]
ceiling = 2.5
rects = [
  [[0, 0], [20, 4]],    # hall 1
  [[16, 0], [20, 16]],  # hall 2
]
```

The world generator computes the union and its outline with shapely (only
the generator depends on it; the `.wbt` contains plain wall boxes). It
rejects rectangles that don't connect (the union falls apart in pieces) and
A or C outside the arena. Whether A can hold n drones isn't checked on the
map, which doesn't know n; the spawn step simply fails if it can't place
them.

Turned down:
- a polygon outline (listing corners by hand is error-prone; rectangles are
  easier to read and to extend);
- a `corner_radius` that rounds corners by shrinking and growing the shape:
  it only rounds the inside of a turn, and rounding outer corners erases
  any hall narrower than twice the radius (tested: a 4 m hall vanishes at
  r = 2 m). A truly curved hall would need its own arc shape;
- non-rectangular shapes, for now.

## 2026-10-14 · Map version field and obstacle types

- The first key of every map is `version = 1`, bumped whenever the format
  changes, so old maps fail loudly instead of being misread.
- Two obstacle types:
  - **walls**: axis-aligned rectangles, given by two opposite corners like
    the arena;
  - **pillars**: cylinders, with no sharp edges for drones to clip.
- Pillars can be placed by hand or by a **pillar forest** generator: a
  region, a count, a radius range, a minimum free gap and a seed (fixed
  layout per map). It fails with a clear error if it can't fit them.
- Every obstacle takes an optional `height`: omitted means as tall as the
  ceiling (nothing passes over it), a value gives a low obstacle to fly
  over in phase 4. It is in the format from version 1, so phase 4 needs no
  format change. A wall written as a plain corner pair is full height; the
  table form `{ rect = [[x1, y1], [x2, y2]], height = 1.0 }` sets a height.
  Low obstacles also let IR pass over them.

## 2026-10-14 · World generation pipeline

Four pure steps, each testable without Webots:

1. `load_map()`: TOML -> `Map`, exactly what the file says (generators
   included); errors name the field, e.g. `obstacles.walls[1]: ...`.
2. `build_course()`: `Map` -> `Course`, only concrete walls and pillars
   (arena union and outline, forests grown with the map seed, cross-checks).
3. `place_swarm(n, j, seed)`: spawn poses in A and the choice of leaders,
   with the run seed.
4. `render_world()`: `Course` + swarm -> `.wbt` text.

The two seeds live in different steps, so the obstacle layout cannot change
between runs. The generator also draws a PNG preview of the map (arena,
obstacles, sectors, spawn points) to check a map without Webots and to use
in the report. Generated worlds are not kept in git: they are rebuilt from
map + seed.

## 2026-10-14 · Repo layout and command line

One job per folder; the repo root is also the Webots project (Webots
expects `worlds/` and `controllers/` there).

```
boidherd/
├── make_world.py     the one command to run
├── maps/             map files (.toml), written by hand
├── worlds/           generated worlds and previews (not in git)
├── controllers/      Webots entry points (drone, referee), no logic
├── src/              all the Python code: maps, worldgen, later boids, flight
├── tests/
└── docs/
```

`src/` holds the packages directly (`src/maps`, `src/worldgen`, ...); they are
installed in the project's virtual environment, so every script and test can
import them (see "Tooling" below).

```
uv run make_world.py l_hall                    # maps/l_hall.toml -> worlds/l_hall.wbt
uv run make_world.py l_hall -n 20 -j 2 -s 7    # drones, leaders, seed (defaults 10, 1, 0)
uv run make_world.py l_hall --preview          # also draw worlds/l_hall.png
```

The map is always validated; on an error nothing is written. Experiment
batches call the same Python functions directly, so the command stays small.

## 2026-10-14 · Map loader

`src/maps/model.py` holds frozen dataclasses that mirror the format;
`src/maps/load.py` turns a TOML file into them. The loader rejects unknown
keys at every level and suggests the closest one (`heigth` -> "did you mean
'height'?"), so a typo in an optional key can't silently change the map;
rejects booleans where numbers are expected (in Python `True` counts as an
integer); accepts rectangle corners in any order. Checks that need geometry
are left to the course builder. The map has no `name` field: the file name
is the map's name, so there is one source of truth.

## 2026-10-14 · Tooling

- **uv** manages Python 3.11 (`.python-version`), the virtual environment
  (`.venv`) and a lockfile (`uv.lock`) pinning every package version, so the
  environment can be rebuilt identically. One command: `uv sync`.
- The project is built with hatchling (setuptools left a `*.egg-info`
  folder in `src/`) and installed in editable mode, so `src/` packages are
  importable everywhere without path tricks.
- Editor settings (`.vscode/`) are personal and not in git; the README says
  which import paths the editor needs.
- **ruff** for linting and formatting, **pyright** in "standard" mode (the
  same as VS Code), **pytest** for tests.
- **CI** (GitHub Actions) runs the same four checks on every push and pull
  request, and fails if `uv.lock` is out of date.

## 2026-10-14 · How Webots starts Python controllers (tested)

Tested on this machine with Webots R2025a, with throwaway probe controllers:

- By default Webots runs `python -u <controller>.py` (Preferences > General
  > Python command = `python`), i.e. the system Python, which can't import
  our `src/` packages. Output is unbuffered (`-u`).
- Webots sets `PYTHONPATH` to its own `lib/controller/python`, plus
  `WEBOTS_HOME`, `WEBOTS_INSTANCE_PATH`, `WEBOTS_ROBOT_NAME`. The working
  directory is the controller's folder.
- A `runtime.ini` in the controller folder with `[python] COMMAND = <absolute
  path to .venv python>` works: the venv Python runs, imports our packages
  and the Webots `controller` module.
- **A relative path in `COMMAND` crashes all of Webots** (access violation,
  exit code -1073741819), not just the controller.
- `$(VAR)` references in `COMMAND` are expanded, but Webots provides no
  variable pointing at the project folder.
- Launching `webots.exe` from Git Bash segfaults; from PowerShell it works.
  A second instance needs `--port` (default 1234).

Decision: `make_world.py` writes a `runtime.ini` into every controller
folder, with the absolute path of the interpreter it runs in (the `.venv`
one, via `uv run`). The files are kept out of git, so no machine-specific
path is committed and a fresh clone gets correct files on its first world
build. Turned down: a committed `runtime.ini` reading an environment
variable (a manual step on every machine), and controllers on the system
Python with `src/` added by hand (loses the pinned package versions).

## 2026-10-14 · Course builder

`build_course()` (`src/maps/course.py`) turns a `Map` into a `Course`:
concrete boundary walls, map walls and pillars, plus the sectors.

- **Boundary walls.** The union of the arena rectangles only has horizontal
  and vertical edges, so each edge becomes one axis-aligned box, 0.1 m
  thick, just outside the free space. Boxes are stretched by the thickness
  at convex corners (to close them) but not at concave ones (where they
  would poke into the hall).
- **Forests** (`src/maps/forest.py`): rejection sampling with the forest's
  seed; each pillar keeps `min_gap` from other pillars, the map walls and
  the arena boundary, and stays out of sectors A and C. Forests are grown
  in file order, each avoiding the pillars already placed.
- **Checks that need geometry:** the rectangles form one piece; A and C are
  inside the arena and don't overlap; no hand-placed obstacle leaves the
  arena or overlaps A or C; and **C is reachable from A** through a passage
  at least 0.2 m wide (twice `DRONE_RADIUS`). Seen from above: the free
  space minus the obstacle footprints, shrunk by the drone's radius, is
  where a drone's centre can be; A and C must lie in the same piece of it.
  Until phase 4 every obstacle blocks, whatever its height, because the
  flock flies at a fixed altitude the map doesn't know. Tested on an L hall
  and on a ring track (an arena with a hole, which gets walls around the
  hole too).
- shapely is imported only by `course.py` and `forest.py`; `maps/__init__`
  doesn't export them, so importing `maps` doesn't pull in shapely.

The first example map asked for 15 pillars that couldn't fit with a 0.8 m
gap in a 4 m wide hall (the gap also applies to the side walls). Default
maps are now large (the example is a 40 x 8 m hall turning into a 30 x 8 m
one), so fitting is never the problem while the behaviour is being
developed; narrower maps come later, as experiments.

## 2026-10-14 · Spawning the swarm

Drones start **on the floor** of sector A and take off to their own
altitude, like a real Crazyflie; flocking starts once they hover. Spawning
in the air was turned down: with motors off at t = 0 every drone would
drop until its controller spins up.

`place_swarm(course, n, j, seed)` (`src/worldgen/spawn.py`) returns n
`Drone`s (id, floor position, yaw, takeoff altitude, leader flag), all
drawn from one generator seeded with the run seed, so course + n + j +
seed always give the same swarm.

- Floor positions: uniform in sector A, rejected if closer than `spacing`
  (default 1 m, horizontal) to another drone or closer than `clearance`
  (default 0.5 m) to the arena edge, a wall or a pillar.
- Takeoff altitude: uniform in A's `z` band, kept 0.3 m away from the floor
  and the ceiling. `z` in the map file:
  - omitted: floor to ceiling (for any sector; the loader fills it in);
  - `[min, max]`: a band;
  - a single number, A only: every drone takes off to that altitude, for
    the fixed-altitude phases. C needs a range: a goal one altitude thick
    makes no sense.
- Yaw: uniform; a quadrotor's heading doesn't constrain its motion.
- Leaders: j distinct ids drawn at random, not the first j, so leadership
  isn't tied to spawn order. `j = 0` is allowed (phase 1, no leaders).
- If A can't hold n drones, the error says how many fit. The example map's
  A is 11 x 6 m: random placement fits 51-58 drones at 1 m spacing.

## 2026-10-14 · Writing the world file

`render_world(course, swarm, title, ir_range=3.0)` (`src/worldgen/wbt.py`)
returns the `.wbt` text: one small function per kind of node.

- Obstacles and boundary walls are static `Solid`s (a `boundingObject` but
  no `physics`, so they collide but never move); full-height ones reach the
  ceiling. Colours: grey boundary, orange obstacles.
- Sectors A (green) and C (gold) are marked by a faint patch on the floor
  (85% transparent; brighter, glowing patches hid the drones standing on
  them) and an even fainter box over their altitude band (A: the takeoff
  band; C: floor to ceiling), drawn only when the band is at least 10 cm
  thick: a single altitude would make a thin slab that looks solid from a
  low angle. The patch sits 1 cm above the floor: at 2 mm it flickered from
  far away (z-fighting).
- Leaders carry a glowing red ball on top, to tell them apart while the
  simulation runs.
- The floor keeps Webots' own checkerboard but with `tileSize 4` (2 m
  squares): the default 0.5 m tiles flicker when the whole course is in
  view. No custom textures or other assets, by choice.
- The floor is exactly the arena's bounding rectangle. Webots starts the
  tile pattern at the floor's corner, so a floor reaching 10 cm under the
  walls left a cut-off strip of squares along every wall.
- All markers are visual only (no `boundingObject`): no mass, no
  collisions, invisible to IR and to the distance sensors.
- Each drone is a `Crazyflie` standing on the floor (`DEF DRONE_<id>`,
  name `drone_<id>`, controller `drone`). Its `customData` is compact JSON:
  `{"altitude": 1.2}` for followers, plus `"goal": [x_min, y_min, x_max,
  y_max]` for leaders. The IR emitter and receiver (infra-red, channel 1,
  range `ir_range`) go in `extensionSlot`.
- The `referee` (supervisor) gets `--sector-a`, `--sector-c` and
  `--ceiling` in `controllerArgs`.
- The viewpoint looks at the centre of the floor from the south-east,
  tilted down, far enough to fit the floor's diagonal.

Checked in Webots R2025a: the generated example world loads with no
warnings, the referee reads its arguments and every drone's `customData`
arrives intact (JSON quotes escaped correctly).

## 2026-10-14 · make_world.py

`make_world.py` (repo root) is three lines; the command lives in
`src/worldgen/cli.py`, where `generate()` runs the whole pipeline (load,
build, spawn, write the world, write `runtime.ini`, optional preview) so
experiment scripts can call it directly.

- The map argument is a name (`l_hall` -> `maps/l_hall.toml`) or a path;
  an unknown name lists the maps that exist.
- Map and spawn errors are printed as one line, without a traceback, and
  nothing is written. Exit codes: 0 done, 1 bad map or run settings, 2 no
  such map.
- `--preview` draws `worlds/<map>.png` with matplotlib: free space, walls,
  pillars (low ones lighter), sectors, followers and leaders, in metres.
  matplotlib is imported only when asked, since it is slow to load.
- `controllers/drone` and `controllers/referee` are placeholders that do
  nothing, so a generated world opens without errors before phase 1.

Checked end to end: `uv run make_world.py l_hall -n 20 -j 2 -s 7` and
opening the world in Webots started all 21 controllers with the `.venv`
Python (through the generated `runtime.ini`), with no errors or warnings.

## 2026-10-14 · Sectors use corners, like every other rectangle

Sectors were first written as ranges (`x = [x_min, x_max], y = [y_min,
y_max]`) while the arena and walls use two corners (`[[x1, y1], [x2,
y2]]`). Writing the first map by hand, the corner habit produced `x = [0,
0], y = [4, 30]` (meant as corners (0, 0) and (4, 30)), which the loader
read as a sector with no width. Two notations for one thing invite exactly
that, so sectors now use corners too:

```toml
A = { rect = [[1, 1], [12, 7]], z = [0.8, 1.5] }
C = { rect = [[32, 24], [40, 30]] }
```

The old `x`/`y` form gets an error pointing to `rect`. The map version
stays 1: no maps in the old form exist outside this repo.

## 2026-10-14 · Phase 1 plan

Agreed order: the flight layer first (one drone), then the whole swarm
taking off, then neighbour detection over IR with an ellipsoidal zone; the
Reynolds rules are written in parallel as pure code and attached to the
drones only once flight is reliable. Speed limit: a high default for now.
Yaw: held constant until phase 3. Settings: frozen dataclasses with
defaults, one file per layer (`flight/params.py`, later `boids/params.py`).
Shared helpers live in `src/util/` (`Vec3`).

## 2026-10-14 · Flight layer

`src/flight/` turns a world-frame acceleration into motor speeds:

- `control.py` (pure): `tilt_for()` rotates the acceleration into the body
  frame with the measured yaw and converts it to pitch (forward) and roll
  (sideways), a = g tan(tilt), capped at 20 degrees; `MotorMixer` is the
  altitude + attitude PID and motor mixing of Webots' Crazyflie example
  (Apache 2.0), without its velocity loop. Tilt signs are taken from the
  example, not guessed.
- `autopilot.py` (pure): GROUNDED -> CLIMBING -> HOVERING -> FOLLOWING.
- `crazyflie.py`: the only module importing Webots; velocity comes from
  `GPS.getSpeedVector()`.
- `params.py`: every number, in `FlightParams`.

Tested with `tools/flight_test.py`: one drone spawned at a -47 degree yaw
takes off, hovers, then follows scripted world accelerations (+x push,
coast, brake, +y push, brake, hold), at Webots' default 32 ms step.

| | first version | final |
|---|---|---|
| overshoot at the end of the climb | 67 cm | 2 mm |
| hover altitude error, mean / max | 121 / 302 mm | 5 / 23 mm |
| altitude error while manoeuvring | 314 mm | 7 mm |
| world velocity after each push/brake | within 0.06 m/s | within 0.06 m/s |

What it took:
- **Hover speed from the physics**: 4 k w^2 = m g gives 55.4 rad/s (proto:
  m = 0.05 kg, k = 4e-5); measured in steady flight: 55.37. The example's
  48 plus integral made the drone lag and overshoot.
- **The setpoint kept rising after the phase changed** (bug): it now rises
  until it reaches the target, slowing down over the last metre (1/s times
  the gap), which removed the overshoot.
- **Liftoff**: the drone leaves the floor only about 2 s after the motors
  start, even at a constant 70 rad/s (cause inside Webots not pinned down).
  A PID waiting on the floor winds up its integral, so on the floor the
  motors run at a fixed liftoff speed and the PIDs start fresh 5 cm up.
- **Tilt compensation**: the motors speed up by 1/sqrt(cos roll cos pitch).

Open: after braking the drone keeps ~0.05 m/s and drifts ~15 cm in 5 s;
open-loop acceleration has no position feedback, the flocking rules will
close the loop. Takeoff to 1.75 m takes ~10 s.

## 2026-10-14 · Swarm takeoff (phase 1, step 2)

- `controllers/drone/drone.py` now flies: lift off, climb to the altitude
  in its customData, hover. Flocking is not attached yet.
- `controllers/referee/referee.py` logs every drone's true position and
  velocity at every step to `logs/<world>.csv`, flushed as it goes. It
  never quits Webots; the tools stop the Webots they start.
- `tools/headless.py` runs a world without a window until a condition
  holds and stops only that Webots; shared by the test tools.
- `tools/takeoff_test.py` builds a world with the real controller, runs it
  and reports takeoff times, overshoot, the closest two drones ever got,
  sideways drift and crashes.

| run | hovering | overshoot | closest pair | drift | crashes | 1 s simulated in |
|---|---|---|---|---|---|---|
| square, 20 drones, 25 s | all, 6.0-6.2 s | 0 mm | 1.24 m | 0 | none | 0.4 s |
| l_hall, 40 drones, 30 s | all, 5.9-6.4 s | 2 mm | 1.00 m | 0 | none | 0.6 s |

The closest pair is the spawn spacing: the drones never get closer than
where they start. Headless Webots runs faster than real time with 40
Python controllers.

## 2026-10-14 · Neighbour sensing over IR (phase 1, step 3)

- `flight/radio.py`: each drone broadcasts `id;vx;vy;vz` every step, its
  world-frame velocity (for alignment). Stray messages are ignored.
- `flight/sensing.py`: for each packet, distance = 1/sqrt(signal strength)
  and the direction (body frame: the IR devices sit in `extensionSlot`,
  1.5 cm under the body origin, not rotated) is rotated into the world
  frame with R = Rz(yaw) Ry(pitch) Rx(roll). Own packets are dropped.
- `boids/zone.py`: one ellipsoidal distance with a vertical scale, for both
  the perception zone and (later) the separation zone. The IR range (3 m)
  caps the perception zone in any case.
- `tools/ir_test.py`: 20 drones in a test room (written to `logs/`, not
  `maps/`) hover, then sway together back and forth (no drift, spacing
  unchanged, every drone tilted), logging what they hear; the tool compares
  it with the referee's ground truth.

| | result |
|---|---|
| tilt while measuring | up to 6.2 deg |
| distance error | mean 1.4 cm, max 3.4 cm |
| direction error | mean 0.41 deg, max 1.7 deg |
| in-range pairs heard | 52922 / 52922 (100%) |
| heard although out of range | 78, all within ~2 cm of the 3 m limit |

Answered: **a drone's own body does not block its IR**, and in this test
no drone blocked the line of sight between two others either, even packed
at 1 m: Webots' IR is kinder than real IR here. Walls blocking IR is still
to check (phase 3). Without the body-to-world rotation the direction error
would be about the tilt itself (up to 6 deg); measured: 0.4 deg mean.

## 2026-10-15 · Phase 5: hybrid control for the leaders

Up to phase 4 a leader only knows a direction: an extra pull straight
toward C. That is purely reactive control, and it fails wherever the
straight line to C crosses a wall. In the L hall, a leader still in the
first hall is pulled into the outer wall of the corner; steer-to-avoid
only slides it along the wall, a local minimum like a potential field's.
The flock then gets stuck with it, or reaches C by chance.

Phase 5 gives the leaders a hybrid architecture (deliberative on top of
reactive, the hybrid control of the course's lecture on control
architectures):

- **Deliberative layer**, once, while the leader takes off: it plans a path
  from its position to C on the map of the course. The planning space is
  the one of the reachability check (free space minus the obstacles, shrunk
  by a clearance), so the planner starts from `_check_reachable`'s geometry.
- **Reactive layer**: Reynolds' rules and steer-to-avoid, unchanged; the
  goal pull now aims at the current waypoint instead of at C. The leader
  aims at the farthest waypoint still in line of sight, so being pushed off
  the path by the flock doesn't need replanning.
- **Followers are untouched**: they don't know a map exists, so the n x j
  question stays the same; only what a leader knows changes.

Experiment: reactive leaders (pull toward C) against hybrid leaders, with
the same maps, n, j and seeds. Hypotheses, fixed before running: on a
straight hall, no difference; on maps where the straight line to C hits a
wall (the L hall, a dead end facing C), hybrid leaders bring a larger
fraction of the flock into C, sooner. Secondary: does a planned path lower
the j needed?

Depends on phase 3 (obstacles, steer-to-avoid), not on phase 4: it can
come before 3D, planning at fixed altitude where every obstacle blocks.

Turned down: building the map online from the sensors (SLAM). The map is
known, and mapping would become the project. Also turned down: giving the
path to the followers too, since then there would be no leaders.

## 2026-10-20 · Reynolds rules on the drones: the ideal boid is not the robot

`boids/rules.py` holds the three rules as pure maths (plus drag and a speed
limit), one world-frame acceleration out, as in Reynolds (1987). Attached
to the drones as they were, they failed, and why is a result in itself:
rules that are stable for an ideal boid are not stable on a robot.

**Baseline** (pure rules, flight layer of 2026-10-14; `tools/flock_test.py`,
20 drones on `square`): calm at 16 s, then neighbouring pairs swung in and
out with a ~2 s period, wider each time; two drones touched at ~23 s and
the physics blew up (speeds over 100 m/s).

**Diagnosis.** The rules assume a boid accelerates the instant it is asked
to. A quadrotor must tilt first, and the tilt controller (gains from
Webots' example) was slow and underdamped: a tilt step overshot by ~45%
and peaked after ~0.8 s; on a 13 deg step the motors differed by only
0.4 rad/s, almost no torque. A stiff spring (separation grows as 1/d^2)
behind a lagging, resonant actuator oscillates. An offline model with a
second-order actuator (zeta ~0.25, omega ~4 rad/s) reproduced both the
failure and a failed patch: a look-ahead term in separation made it worse,
and that patch also put drone knowledge into the abstract rules. Reverted.

Literature, the same problem: Virágh et al. (2014) model a flying robot by
its imperfections (controller relaxation time, max acceleration, delay,
noise) and report delay-induced oscillations that cause collisions even
in a hovering flock; Forgoston & Schwartz (2008) show such instabilities
are a Hopf bifurcation. Plan agreed: A) make the drone follow commands
well; C) command a velocity instead of an acceleration; measure after
each; then decide B, Virágh's robot-aware rules.

### A: attitude in two nested loops

`flight/control.py` now holds attitude as the Crazyflie firmware does:
angle error -> wanted rotation rate, capped -> rate error (gyro) -> motor
differential. Yaw is held at the takeoff heading the same way (before, it
was only damped). Other changes:
- **Tilt limit 30 deg** (was 20): holding altitude then needs 1.15x the
  weight, 56% of a real Crazyflie's thrust. 20 deg is Bitcraze's firmware
  default in velocity mode; physics stops at 61 deg.
- **Tilt limited by magnitude**, not per axis, so the direction is kept.
- **Motors capped at 79.7 rad/s**: the proto's 600 rad/s gives ~117x the
  weight in thrust, a real Crazyflie 2.1 ~2.07x (60 g thrust, 29 g). The
  controllers can't use thrust the real drone doesn't have.

What it took (`tools/flight_test.py`, which now reports tilt steps,
including a 5 m/s^2 push and its 40 deg reversal):
- A single PD loop tuned fast was clean on steps but flipped the drone on
  the reversal (tilt overshot to 47 deg, yaw kicked 20 deg, motors pinned).
- Nested loops alone did not fix it either. Ruled out by experiment: the
  motor cap (lifting it changed nothing) and the 32 ms step (crashes at
  4 ms too). What breaks the drone is rotating fast: above ~2 rad/s the
  inner loop can't brake a rotation in time and yaw couples in. Capping
  the rate at 1.5 rad/s fixed it.

| tilt step | before | after |
|---|---|---|
| small (2 deg): 90% after, overshoot | 0.64 s, 40% | 0.19 s, 3% |
| hard (~21 deg) | never reached 90% within 1 s | 0.26 s, 4% |
| reversal (~40 deg swing) | flipped (fast single loop) | 0.48 s, 2% |
| yaw change, altitude error | - | 0.0 deg, 8 mm |

Checked at four spawn yaws (-47, 54, 95, 153 deg): no crash, yaw held.
Swarm takeoff unchanged (`takeoff_test` now uses a `hover` controller, so
it measures takeoff alone).

**The same rules, unchanged, on the new drone** (`flock_test`):

| run | speed at the end | closest pair | groups | crashes |
|---|---|---|---|---|
| baseline, 20 drones | blew up | 0.05 m (touched) | - | yes |
| A, 20 drones, seed 0 | 0.00 m/s | 0.64 m | 3 -> 4 | none |
| A, 40 drones, seed 3 | 0.01 m/s | 0.61 m | 1 -> 4 | none |

The flock now settles into a lattice (~1 m to the nearest neighbour)
instead of oscillating. It splits into groups, though, even 40 drones
that start as one: a matter of the rules (averaged cohesion, no leaders,
drag), not of stability; for B.

Tools: `--set section.name=value` overrides any setting for one run
(`util/overrides.py`, passed to the controllers through the environment).

### Time step: 16 ms

Webots' default 32 ms (our choice so far, not a Webots limit) is now 16 ms
(`worldgen/wbt.py`, `TIME_STEP`): sensors and IR update twice as often,
and the delay between a neighbour moving and a drone reacting halves. The
real Crazyflie runs its attitude loops at 500 Hz (2 ms; position at 100 Hz).

| 60 s simulated | 32 ms | 16 ms |
|---|---|---|
| 20 drones | 22 s | 39 s (1.8x) |
| 40 drones | 69 s | 143 s (2.1x) |

Same flock at both (closest pair 0.64 m / 0.61 m, no crash), same tilt
response (a few % more overshoot at 16 ms). The cost doubles with each
halving: 8 ms would be ~4x.

Hoped for, not obtained: a higher rotation cap. At 16 ms, as at 32 ms,
the drone flips above ~2 rad/s (2 rad/s itself overshoots 15-24%), so the
time step is not what limits rotation. Not the motors either (no
acceleration limit in the proto, 30 N m max torque). A suspect, the
proto's propeller torque constant (2.4e-6 against 4e-5 for thrust: a yaw
torque of 0.06 m per newton, about ten times a real Crazyflie's, figure
from memory, unchecked), was not confirmed: with yaw control off, weaker or
stronger, a 3 rad/s cap still crashes, already on the hard push. A lead:
the direction matters. Spawned at yaw 95 deg, the push is nearly pure roll
and 3 rad/s is clean (reversal 0.35 s, 7%); at yaw -47 deg it is diagonal
(roll and pitch both ~20 deg) and crashes. Two candidates: the rate loop
treats gyro body rates as Euler-angle rates, which at 20 deg of roll and
pitch leak ~12% into each other; or the X layout, where a diagonal
rotation is driven by one motor pair only. Cause open; the cap stays at
1.5 rad/s, clean in every direction at both steps.

### Drone markers

A 10 cm drone is hard to follow in the 3D view. Every drone now carries a
glowing 15 cm square plate just above it (cyan, red for leaders, replacing
the leaders' red ball) with a black tab on its front edge. It sits in the
extensionSlot, so it tilts and turns with the drone: roll and pitch can be
read off it, the tab tells which is which. Visual only (no
boundingObject): the same flock run gives the same numbers to the step.
A little over the drone's 10 cm, so the spacing between drones still looks
true (30 cm made neighbours look closer than they are). On by default;
`make_world.py --no-markers` leaves them out.

The floor keeps Webots' checkerboard picture but on a fully matte surface:
the stock `Parquetry` appearance adds a roughness map (glossy patches) and
a normal map, and the light glared off the floor at some camera angles.

### C and B: studied, deferred

A was enough: the pure rules are stable on the tuned drone, so the rules
stay as they are until a problem asks for more. What we would do then, so
it needn't be studied again:

**C, command a velocity instead of an acceleration.** The flight layer
would take a wanted velocity v_d and fly a = (v_d - v)/tau, capped at
a_max (Virágh's robot model; Reynolds 1999 steers the same way:
desired velocity minus current velocity). The design log's 2026-10-14
objection ("a floating e-puck") only holds for tau ~ 0. Written out for a
follower, v_d = mean(v_j) + p_coh * (offset to centre) + v_sep, which
expands to today's rules with alignment weight 1/tau and `drag` replaced
by braking when alone: on its own C re-parameterises, it fixes nothing
new. Its value is as the base for B (Virágh's terms are velocities) and
to describe the robot by two measurable numbers, tau and a_max.

**B, Virágh's rules adapted to a real robot** (desired velocities, summed,
|v_d| capped at v_max; d_ij from i to j, d = |d_ij|, d^ = d_ij / d):

    self-propulsion  v_spp = v_flock * v_i / |v_i|          (Virágh, Couzin)
    separation       v_sep = sum_{d<r_sep} p_sep (r_sep - d) (-d^)  linear, not 1/d
    alignment        v_ali = sum_j C_frict (v_j - v_i) / max(r_min, d)^2, capped
    cohesion         v_coh = p_coh * mean_j d_ij, capped  (Reynolds; Virágh has none)
    leaders only     v_spp = v_flock * unit(C - x_i)
    walls (phase 3)  steer-to-avoid or Virágh's virtual walls

What each answers: linear separation has no 1/d^2 stiffness and is robust
to IR distance noise; pairwise friction alignment damps two close drones
(averaged alignment barely does); per-term caps keep any command bounded.
Self-propelled followers need walls from the start (the square would be
flown into). Indicative values at our scale: r_sep ~1 m, p_sep ~1/s,
C_frict ~0.2 m^2, r_min ~0.5 m, v_flock ~0.5 m/s, tau ~0.5 s.

Revisit if: the flock oscillates again (leaders pulling, higher speeds,
obstacles); group splitting stops leaders from bringing the flock along
(phase 2); or IR noise at close range makes separation jumpy.

## 2026-10-20 · Phase 2: leaders

**Leaders** are ordinary boids with one extra term summed to the same
Reynolds rules, so they still feel the flock (a lagging flock holds them
back); followers can't tell them apart (Couzin 2005). The extra term is
Reynolds' (1999) seek + arrive: steer toward `goal_speed` (0.5 m/s) aimed
at the centre of C, the wanted speed shrinking within `arrive_radius`
(3 m) so the leader stops there. Turned down: a constant pull toward C,
which keeps accelerating the leader up to the speed limit and overshoots C
into the wall (no wall avoidance before phase 3). A leader computes its
offset to the goal from GPS and the goal in its customData; the rules only
get the offset. Leaders set off `goal_after` (10 s) after they start
hovering, so the flock forms first, as Couzin's groups start formed.

**Drag removed.** Phase 1's `drag` (-0.3 v, a virtual friction, not air
drag: Webots' Crazyflie has none, it coasts at 0.99 of its speed after 3 s)
would have made followers lag: with alignment 1/s, each neighbour hop
behind a leader flies at 1/(1 + 0.3) = 0.77 of the speed ahead of it, so
the flock stretches and tears. Without drag a leaderless flock keeps its
drift: wanted, a drifting flock is likelier to meet a leader than one that
stands still. Turned down: physical drag through Webots (`defaultDamping`),
a value to tune for no benefit to the research question.

**Map `open`**: one 40 x 20 m room, A 8 x 8 m (about 50 drones fit at 1 m
spacing, so the swarm starts as one group; `square`'s 4 x 30 m strip
started 20 drones as three), C 8 x 8 m with its centre 29 m away.
`flock_test` now takes `-j` and reports how many drones are in C over time
and when 90% are.

| 20 drones, `open` | groups | cruise | 90% in C | closest pair | crashes |
|---|---|---|---|---|---|
| j = 0, 60 s | 1 | (drift, see below) | never | 0.47 m | none |
| j = 2, 120 s | 1 throughout | 0.50-0.56 m/s, polarization 1.00 | 78.6 s (all 20 by 86 s) | 0.47 m | none |

Leaders set off at ~16 s, so the 29 m took ~63 s. After arriving, the
flock sloshes a little (0.05-0.15 m/s, no drag to stop the followers) but
all 20 stay in C.

**Why a leaderless flock speeds up (found).** Without leaders the flock
lines up by itself (polarization 1.00) and accelerates steadily, ~0.006
m/s^2 (0.04 to 0.26 m/s in 50 s). Ruled out one by one: a flight-layer bias
(a lone drone asked for zero drifts by exactly 0.000 m/s, at four yaws), a
force on a moving drone (the flight test's coast holds 1.0005 m/s), a
response that depends on direction (every drone at the same yaw: the same
drift). Logging every drone's command settled it:

| mean over 20 drones, 30-50 s (mm/s^2) | x | y |
|---|---|---|
| separation | 0.00 | 0.00 |
| alignment | ~+0.5 | ~-0.3 |
| cohesion | -7 to -10 | +7 to +8 |
| realised (35-55 s) | -5.9 | +5.8 |

The rules ask for it; the drone just flies it. Separation is reciprocal
(A pushes off B as hard as B off A), so it sums to zero. Reynolds'
cohesion is not: each drone steers toward the *mean* of *its own*
neighbours, and an edge drone with 5 and a middle one with 15 average over
different sets, so a pair's pulls are not equal and opposite and the flock
keeps a net force. With no drag, the flock settles into a shape whose
imbalance propels it, in a direction set by chance. Drag used to hide it.

Decision: **keep Reynolds' averaged cohesion** and accept a self-propelled
flock as a property of the model. Turned down: reciprocal cohesion, a sum
of k * offset over the neighbours, as Olfati-Saber (2006) builds flocking
from pairwise symmetric terms precisely to conserve the flock's momentum.
Consequences to keep in mind: a leaderless flock accelerates toward the
1 m/s limit in a random direction (and into a wall: no avoidance before
phase 3), and in leader runs the same push competes with the leaders, so
the n x j results include it.

### First n x j sweep (short)

`tools/sweep.py` runs every (n, j, seed) headless on `open` and appends a
row per run to `logs/sweep_results.csv` (resumable). A short sweep, n 10
and 20, j 0 1 2 4 8, 150 s (90% of the drones in C, s; "-" never):

| j | n = 10, seeds 0 / 1 / 2 | n = 20, seeds 0 / 1 |
|---|---|---|
| 0 | - / - / - | - / - |
| 1 | 83 / - (5 left) / 77 (1 left) | 89 / 101 |
| 2 | 74 / - (5 left) / 74 (1 left) | 79 / 81 |
| 4 | 71 / 73 (1 left) / 71 (1 left) | 75 / 80 |
| 8 | 69 / 72 (1 left) / 69 (1 left) | 73 / 76 |

No crashes; closest pair 0.42 m; leaderless flocks reached a wall (0.04
m, no avoidance yet). At n = 20 one leader (5%) brings the whole flock;
more leaders only shorten the trip, down to the leaders' own pace (~74 s:
set off at ~16 s, 29 m at 0.5 m/s). Every n = 10 failure is the spawn:
A is a fixed 8 x 8 m, so 10 drones start sparse, in 3 groups (seed 1) or
2 (seed 2), and a group without a leader never hears where C is. The
sweep mixes swarm size with start density.

Left for a later test: spawn at constant density so every swarm starts
connected, and weaker leaders (e.g. `goal_weight` 0.3) to find the j at
which leading starts to fail (Couzin's accuracy curve).

## 2026-10-26 · Phase 3: from Reynolds to Vásárhelyi

Obstacle avoidance raised a question about the whole model: Reynolds'
boids were made for animation (perfect perception, instant response), so
is forcing them onto a robot the wrong model?

**What the literature says.** Not wrong for flocking: Reynolds is the
usual starting point for real swarms. Hauert et al. (2011) flew Reynolds
flocking with a migration point on 10 fixed-wing robots ("Reynolds
flocking in reality") and note that rotorcraft "can be approximated as
omnidirectional motion": the quadrotor is the easiest robot for it.
Virágh et al. (2014) start from Reynolds' three principles for any robot
that moves "in arbitrary directions independently of its orientation...
(including zero velocity hovering)" and show they work "only if some
specific extra aspects of the robots are taken into account" (inertia,
delay, noise, range): our phase-1 experience. Olfati-Saber (2006) shows
the plain rules "generically" fragment without a navigational term: our
n = 10 sweep. Where the doubt holds is obstacles: Soria et al. (2021)
argue that potential-field swarm models (Reynolds' family) do not
guarantee fast, safe flight in clutter. And no flocking model solves
seeing obstacles: Reynolds assumed perfect sight, Vásárhelyi et al.
(2018) flew 30 drones among obstacles that were virtual GPS walls.

**Sensing obstacles, turned down.** The Crazyflie's Multi-ranger (4
single rays, 2 m; the real VL53L1x sees a ~27 deg cone) leaves blind
spots, needs the drone to yaw to face its velocity for a look-ahead, and
sees flockmates as obstacles. Removing them with the IR neighbours (two
sensors cancelling each other) is not robust. A map built in flight
(occupancy grid from rangers and GPS: walls stay, neighbours move on) is
the realistic route and a project of its own: future work.

**Decision.**

- **Model: Vásárhelyi et al. (2018), plus Reynolds' cohesion and the
  leaders' goal term.** A model designed for real drones, still in
  Reynolds' family (repulsion, alignment, attraction), so Couzin's
  question stands. It has no cohesion (its small arena keeps the flock
  together); our open room needs it and Couzin's leaders pull through it,
  so Reynolds' averaged cohesion stays.
- **Obstacles: map + GPS.** Every drone gets the course geometry in its
  `customData` (controllers still never read map files) and finds the
  nearest point of each obstacle and of the arena boundary with shapely.
  Neighbours come only from IR, obstacles only from the map: no sensor
  corrects another. An obstacle missing from the map is invisible; the
  setting is a known environment with unknown neighbours.
- **Self-propulsion on, off inside C.** Followers do not know where C is
  but recognise it when inside (a ground marker, simulated with GPS
  against C's rectangle): goal recognition, not goal knowledge, so they
  stay uninformed in Couzin's sense. A drone in C drops its cruise term;
  alignment slows the others to it. A leaderless flock now wanders and
  may reach C by chance: j = 0 becomes a random-search baseline.
- **Locomotion: velocity in front of the cascade** (option C above):
  a = (v_d - v) / tau, capped at a_max. tau and a_max describe the robot.

**The model** (from the authors' simulator, csviragh/robotsim,
`algo_spp_evol.c`; d_ij from i to j, d = |d_ij|). The result is a wanted
velocity v_d, capped at v_max:

    self-propulsion  v_flock * unit(v_i), or 0 inside C
    repulsion        sum_{d<r0} p_rep (r0 - d) (-unit(d_ij)), capped
    friction         sum_j c_frict (|v_ij| - D_j) unit(v_ij)  if |v_ij| > D_j
                     v_ij = v_j - v_i, D_j = max(v_frict, D(d - r0_frict))
    walls/obstacles  the same against a shill per obstacle: at the
                     obstacle's nearest point, flying away from it at
                     v_shill; allowed difference D(d - r0_shill)
    cohesion         Reynolds: p_coh * mean_j d_ij (ours)
    goal (leaders)   Reynolds' seek + arrive toward C (ours)

D(x) is a braking curve: the largest velocity difference from which the
drone can still stop within x at deceleration a: 0 for x <= 0, p x near
the wall, sqrt(2 a x - a^2/p^2) farther out (v^2 = 2 a x). A drone
brakes early and smoothly instead of being shoved at the last moment,
and flying alongside a wall is undisturbed: what neither steer-to-avoid
(acts only ahead) nor a force field (too weak far, too stiff near) gives.

**To watch.** Their arena code handles squares and circles only (concave
polygons left as a TODO); the nearest-point shill should work in our
L-hall's inner corner, to check first. Their parameters were evolved for
4-8 m/s and ~10 m spacing; ours (0.5 m/s, ~1 m) are to be rescaled and
tuned, first on point masses, then in Webots. The phase-2 sweep is redone
with the new model, a Reynolds-vs-Vásárhelyi comparison for the report.

**Order of work.** 1. Velocity layer in the flight code, with a flight
test. 2. The model as pure rules with tests, against open space (no
obstacles): the flock and the leaders as in phase 2. 3. Course geometry
in `customData`, shills for the arena walls. 4. Obstacles: pillars, then
the wall with a gap. 5. Sweep again.

### Step 1: the velocity layer

`flight.acceleration_for(wanted, velocity, tau, a_max)` turns a wanted
velocity into the acceleration the autopilot already flies:
(v_d - v) / tau, horizontal, capped at a_max (2.5 m/s^2). The autopilot
keeps its acceleration input; the controller puts this step in front.
`tools/flight_test.py` now scripts velocities (+x 0.5 m/s, stop, +y 1
m/s, a 2 m/s reversal, stop) and compares each step with a point mass
of the same tau and a_max (seed 3, yaw -47 deg, 16 ms):

| tau | 63% of a 0.5 / 1 m/s step (ideal) | 2 m/s reversal, 63% / 90% (ideal) | overshoot |
|---|---|---|---|
| 0.5 s | 0.51 / 0.51 s (0.50) | 0.66 / 1.14 s (0.56 / 1.22) | 0% |
| 0.3 s | 0.35 / 0.40 s (0.30 / 0.32) | 0.64 / 0.90 s (0.51 / 0.90) | 0% |
| 0.2 s | 0.29 / 0.40 s (0.19 / 0.27) | 0.64 / 0.86 s (0.51 / 0.78) | 15% / 5% |
| 0.1 s | 0.27 / 0.40 s (0.13 / 0.26) | 0.64 / 0.85 s (0.51 / 0.74) | 33% / 14% |

Below ~0.3 s the 63% time stops shrinking (0.27 s for a small step):
that is the attitude loops' own speed, the drone's tau_CTRL in Virágh's
terms, and asking for more only adds overshoot. Chosen: tau = 0.3 s,
the fastest with no overshoot, about 0.05-0.1 s behind the ideal curve.
No drift in the hold, altitude within 5 mm, no yaw change.

### Step 2: the model as rules, open room

`boids.flock` now returns a wanted velocity: repulsion + friction +
Reynolds cohesion, plus either the leaders' seek or, when the controller
allows it, self-propulsion; capped at `max_speed`. The parameters start
from robotsim's defaults scaled to our speed and spacing (params.py).

**The start sequence (mission clock).** Takeoff; until `go_time` (16 s,
all hover by ~6 s) the flock forms with no cruise and no goal; from then
leaders seek the centre of C and every drone may cruise, except inside C.
A shared start time tells followers nothing about where C is.
Self-propulsion follows the drone's own heading, which is noise at hover,
so it is off below `wake_speed` (0.1 m/s) and full from twice that: at
GO the leaders move first, their neighbours get dragged above the
threshold and cruise the same way. A leader's direction mixes its heading
with the goal (Couzin's weight, `goal_weight` = 1); within
`arrive_radius` it slows to stop at the centre.

**Without cruise a drone stops by itself.** The wanted velocity has no
"keep going" term: no neighbours and no cruise means v_d = 0, braked in
~tau. Cruising is what keeps a flock moving; inside C it stops.

**Friction gain: robotsim's 1 makes the drones jiggle.** A point-mass
check (first-order tau = 0.3 s, a_max, IR one step late, even with 0.2 s
of extra attitude lag) flew cleanly; Webots did not: cruise 0.3-0.4 m/s,
polarization 0.8-0.9, 90% in C at 127 s, and in C the drones kept moving
at ~0.2 m/s. A resting flock (`go_time` = 1000: no cruise, no leaders)
jiggled the same way (0.2 m/s, random headings, centre still), so it is
neither cruise nor leaders. Friction sums over every neighbour within
2 m, ~10 of them: at gain 1 a drone asks to remove ~10x its velocity
difference, so the velocity loop closes in tau / 11 ~ 0.03 s, ten times
faster than the drone can tilt (~0.27 s, step 1); the hover noise that
point masses lack excites it (Virágh's delay instability, through the
gain). At 0.2 the resting flock settles to 0.01 m/s by 36 s.

| 20 drones, j = 2, `open`, 150 s | cruise | polarization | in C | 90% in C | closest |
|---|---|---|---|---|---|
| point masses, friction 1 | 0.49 m/s | 1.00 | all, still | 95 s | 0.35 m |
| Webots, friction 1 | 0.3-0.4 m/s | 0.8-0.9 | all, moving 0.2 m/s | 127 s | 0.24 m |
| Webots, friction 0.2 | 0.50 m/s | 1.00 | all, still (0.02 m/s) | 106 s | 0.32 m |

**The tail: a stop wave.** With friction 0.2 the front reached C at ~66 s
and 14 of 20 were in by 76 s, but the last ones only by 136 s. The front
stops in C (no cruise there); friction slows the drones behind it below
`wake_speed`; their cruise switched off while still outside C, and only
cohesion pulled them in, over about a minute. Chosen: **once woken, a
drone keeps cruising until it is in C** (`boids.Cruise`), along its last
heading taken above `wake_speed`, so a drone squeezed to a stop behind
the crowd still pushes the way it was going. Turned down: accepting the
slow tail (arrival times would measure it, not the leaders), and a lower
`wake_speed` (moves the threshold, keeps the stall). Result: 90% in C at
91.8 s (was 106), all 20 by 116 s (was 136), still one group, closest
pair 0.31 m. The last two still trail by ~20 s.

**Where the flock stops.** Leaders seek C's centre (and stopped 0.6 m
short of it), but the flock arrives behind them and followers stopped the
moment they entered C: the whole flock piled up between the entry edge
(x 30.5) and the leaders (33.4), the far half of C empty, the tail on the
border. Chosen: **a drone keeps cruising for `settle_time` after it
recognises C**, then stops; it still needs nothing but recognition, and
works for any flock size. Turned down: leaders aiming past the centre (a
stopping point before C's far edge), which needs a depth tuned to the
flock's length.

| 20 drones, j = 2, `open` | 90% in C | all in C | flock at the end (C: x 30-38) |
|---|---|---|---|
| stop on entering C (cruise latch) | 91.8 s | 116 s | x 30.5-33.4 |
| settle_time 3 s | 70.0 s | by 76 s | x 30.9-33.8 |
| settle_time 5 s (chosen) | 69.9 s | by 76 s | x 31.5-34.3 |

The tail is gone: 4 drones in C at 66 s, all 20 ten seconds later. Leaders
set off at 16 s, so the 29 m took ~54 s at 0.50 m/s (Reynolds, phase 2:
79 s to 90%, from a 16 s start too). The flock still sits in C's near
half, held by its leaders at the centre; 5 s keeps it 1.5 m off the edge.

### Step 3: walls from the map

**The rule.** Vásárhelyi's shill agents, as robotsim's
`Shill_Obstacle_LinSqrt`: at the nearest point of a wall sits a virtual
drone flying away from the wall at `wall_speed`; my velocity may differ
from its velocity by at most the braking curve of my distance to the
wall, and the excess is corrected (gain 1, no separate weight). It is the
friction rule again, against the wall: a drone may fly at a wall only as
fast as it can still stop before it. Defaults scaled from robotsim as the
rest (V_Shill 6 m/s → 0.75, Slope_Shill 0.4 → 0.5 1/s, Acc_Shill 2.5 →
0.4 m/s², R_0_Shill 0).

What it means in numbers, at 0.5 m/s (distances stay the same at any
speed, see "Speed" below):

- A drone cruising at a wall at 0.5 m/s starts braking 2.75 m from it,
  where the braking curve reaches the 1.25 m/s between it and the shill.
- A drone at rest is pushed until it is 1.5 m from the wall, where the
  curve reaches 0.75 m/s.
- Pressed by its cruise (0.5 m/s), it holds at 0.5 m.

**Which walls, and the concave arena.** robotsim handles a square or
circular arena, and leaves the general polygon as a TODO: "with concave
polygons it is not trivial". We don't need it. The course already builds
the arena outline as axis-aligned boxes (`maps.course._boundary_walls`),
and each box is convex. So each box is an obstacle with its own shill,
like robotsim's obstacles. Its nearest point is a clamp of my position
into the box (`boids/obstacles.py`), so the controller needs no shapely.

The one oddity is at a concave corner of the arena (the L hall's inner
corner), where two boxes meet: a drone right at the corner feels both,
so it gets twice the push there. That's harmless, so it's left as is.

The drones get the boxes in `customData` ("walls"), with GPS for their
own position: option 1 of the sensing decision, map + GPS. Only the
arena outline goes in for now. Map walls and pillars come in step 4.

**Results.**

- **`open`, 20 drones, j = 2:** unchanged. 90% in C at 69.9 s, closest
  pair 0.33 m. The walls are never closer than A's own 1 m.
- **The bare L hall (`maps/l_turn.toml`, new):** the `l_hall` map without
  its forest and gap wall, which the drones don't know about yet. With
  20 drones and j = 2 the flock turns the 90° corner as one group,
  polarization 1.00 throughout. 90% in C at 118 s, all 20 by 126 s,
  closest pair 0.30 m. No drone ever came closer to a wall than 1.01 m,
  and that was at spawn. No crashes. Point masses predicted 124 s.
- **Leaderless (j = 0, `open`, seeds 1–3):** no crash, never closer than
  1.04 m to a wall.

**Leaderless flocks don't pick a random direction any more.** A's back
edge is 1 m from the room's wall, so before the start the wall already
pushes the nearest drones off it, to 1.5 m. At the start the flock wakes
up already moving that way, which is toward C. All three seeds ended in
C (seed 3 split in two at wake-up, 18/20 in C).

So the j = 0 baseline of a sweep now measures "away from the back wall",
not luck. That could be fixed by spawning A farther from the walls. Not
changed: leaderless runs are the control case, not the mission.

**Speed: 1 m/s, and one number for it.** At 0.5 m/s the flock looked
slow, so the cruise speed went up. Raising `v_flock` alone is unsafe. A
drone pressed at a wall by its cruise is held back by at most the shill's
`wall_speed`, so once v_flock passes `wall_speed` the drone hits the wall.
Every speed-dependent setting has to move with v_flock: speeds and rates
(1/s) in proportion, decelerations with its square, times inversely.
Distances stay put.

So the settings are now written in units of v_flock. `wall_speed = 1.5`
means 1.5 × v_flock, and `settle_time` became `settle_distance` (2.5 m
into C). The rules multiply each term back out, so v_flock is the one
knob. robotsim's defaults now map to ours with the cruise speed ÷ 4
instead of ÷ 8; distances are still ÷ 10.

In the maths the model is then **exactly speed-free**: at k times v_flock,
with every velocity k times as large, every term is k times as large. The
same flock flies the same paths, faster. `test_the_rules_are_speed_free`
checks this for both branches of the braking curve. A mutation check
confirmed it fails when any single factor of v_flock is left out.

The robot is not speed-free. `velocity_tau`, `max_acceleration` and the
attitude loops are fixed, so a faster flock reacts later relative to its
speed. Above ~1.25 m/s the braking curves would ask for more than the
2.5 m/s² the flight layer allows.

| `l_turn`, 20 drones, j = 2 | 90% in C | closest pair | closest to a wall |
|---|---|---|---|
| 0.5 m/s (altitudes 0.8-1.5 m) | 118 s | 0.30 m | 1.01 m |
| 1.0 m/s, settle 5 s, all doubled by hand (altitudes 0.8-1.5) | 70.4 s | 0.22 m | 0.70 m |
| 1.0 m/s defaults, settle 2.5 m (one altitude, 1.5 m) | 71.7 s | 0.18 m | 0.73 m |

On point masses `v_flock = 0.5` reproduces the old run to the
millisecond, so the rewrite changed nothing at 0.5 m/s.

At 1 m/s the closest pair is the price of speed. In the last run it
happened at 18.2 s, while the flock was starting from rest: 8 cm between
bodies, no contact. With one altitude, height no longer separates the
drones. No crashes.

### Step 4: obstacles, and three changes to the model

The mechanics were small. Map walls are boxes like the boundary, and
pillars are disks (nearest point: one step along the radius). Both go
into `customData` with the boundary. Then the full `l_hall` (a forest of
15 thin pillars, the turn, a wall with a 2 m gap) broke the model three
times. Each break was diagnosed on point masses (the scratch simulator)
before anything changed in `src`.

**1. The forest bounced the flock back.** A 10 cm pillar got the same
shill as a wall: a push zone of 1.5 m around it, braking from about
2.75 m. The forest's gaps (1-2 m) all lie inside some push zone, so its
front edge looks like a solid wall. And a shill pushes a drone away at
1.5 × v_flock, faster than the drone cruises, so a drone meeting it
head-on is reversed, not stopped; the cruise latch then carried it back
at full speed. The followers ping-ponged between the forest and the back
wall. The 2 leaders threaded the forest alone and reached C.

Two things that didn't help:

- Leader pull (`goal_weight`) only moved the failure. At 3 the leaders
  split off sooner; at 0.5 or 0.25 the group stayed together but dragged
  the leaders back with it.
- Softer shills (`wall_speed` 1.1) let the flock through the forest, but
  it then bounced off the wall with the gap.

Bouncing is how Vásárhelyi's flocks stay inside an arena with no goal.
For a flock that has to get somewhere it is the wrong reflex; Soria et
al. (2019, 2021) measured the same weakness of reactive models in
clutter.

**Fix 1: the stopping limit replaces the shill.** This is the same
braking curve, used as a limit instead of a push. After everything is
summed, the wanted velocity's component toward each wall is cut down to
D(distance − `stop_wall`). It only takes speed away, so nothing bounces.
It bounds the sum, so no cruise can push a drone into a wall.

Tried on point masses: the flock crossed the forest as one group in
about 20 s, at every leader pull. Then a new failure appeared, hidden
until then by the bouncing: **jams**. When the front of the flock
stopped (at the gap, in a corner), the drones behind kept cruising into
it at v_flock. Repulsion is at most v_flock, reached only at contact, so
a cruising drone can't be held off a stopped one. They converged to
0.00 m (point masses pass through each other). Bending the cruise
heading along the walls made it worse: it slid drones into the corners
where the gap wall meets the side walls.

**Fix 2: the limit applies to neighbours too, relative to their
velocity.** I may close on a neighbour only as fast as its own speed
away from me plus D(distance − `stop_drone`). A neighbour moving ahead at
my speed doesn't hold me back; a stopped one does.

Walls win where the two limits disagree. A drone pinned between a wall
and a neighbour closing in otherwise went through the wall (seen on
point masses). So the walls get the last rounds of the projection.
Each limit is a half-plane of allowed velocities, and a few rounds of
projecting onto them in turn find a velocity inside all of them
(`rules.stopping_limit`).

Even so, 20 drones still jammed at the 2 m gap. With a 0.3 m margin per
side that is a 1.4 m lane. **The gap is now 4 m** (agreed: the map, not
the model). On point masses: 20/20 in C, 90% at 122 s, closest pair
0.21 m, no contact.

Fixes 1 and 2 in Webots: 17/20 in C by 86 s, no crashes, never closer
than 0.30 m to a wall (the margin, held on the real dynamics). The three
that never arrived exposed the third problem.

**3. Private headings.** Each drone's cruise heading was a memory of its
own velocity (the latch from step 2):

- Drones 5 and 17 slid along the gap wall into its inner corner while
  the flock went through the gap. Their heading still pointed into the
  corner, so they hovered there forever, out of the flock's perception.
- Drone 2, **a leader**, went up and down hall 1 alone. Its direction
  was unit(own heading + goal_weight × direction to C). From hall 1, C
  lies mostly behind the hall's top wall: from (31, 7.7) the goal
  direction is about (0.25, 0.97), and the limit takes the 0.97 away.
  Once a small overshoot had flipped the leader's velocity, its own
  heading (−1, 0) outweighed the 0.25 left, so it flew away from C,
  which kept its heading pointing away. A loop that feeds itself.

Marco's observation from watching a run: the flock moved "like a liquid
flowing round the boxes", and the leaders hardly pulled.

Both have one cause. Couzin's result, a small informed minority steering
a large group, relies on followers having no direction of their own
beyond aligning with their neighbours. Our followers each carried their
own momentum, and 18 private memories outvoted 2 leaders. And Couzin's
informed individual mixes the goal with the social direction (its
neighbours'), not with its own momentum. I had misread that.

**Fix 3: the heading is alignment.**

- A follower's heading is the mean direction of the neighbours it sees
  moving (above `wake_speed`), its own included: Couzin's zone of
  orientation. It keeps the last heading only when alone.
- A leader's social direction is the same mean, without itself and
  without memory. Alone, a leader just seeks C (sliding along walls) and
  can't run away from it.

The flock now wakes from its leaders outward: before any neighbour moves,
a follower has no heading. The latch's job, from step 2 (keep pushing
when slowed down by the crowd), is now done by the crowd's own
direction.

| Webots, 20 drones | groups | 90% in C | closest pair | wall | crashes |
|---|---|---|---|---|---|
| `l_hall` j = 2, seed 0 | 1 throughout, polarization 1.00 | 78.1 s (all by 86 s) | 0.27 m | 0.30 m | none |
| `l_hall` j = 1, seed 0 | | 122.0 s | 0.23 m | 0.30 m | none |
| `l_hall` j = 2, seed 1 | 2 from spawn | never (11/20) | 0.28 m | 0.30 m | none |
| `open` j = 2, seed 0 | | 42.7 s (was 69.9) | 0.41 m | 1.01 m | none |

Seed 1 fails from the spawn. The swarm takes off as two groups (2 groups
at 6.4 s, centres 6.6 m apart), and both leaders are in one of them. The
leaderless 9 follow late, miss the turn and cruise up and down hall 1.
`l_hall`'s A is 11 × 6 m with altitudes 0.8-1.5 m, too sparse for 20
drones at a 2.5 m perception radius. This is the constant-density spawn
deferred after the first sweep; the L hall turns it from a nuisance into
a failure.

`open` is faster because the flock now wakes along the leaders'
direction instead of each drone waiting for its own push.

### The spawn: constant density, one group

Drones used to be drawn anywhere in A, so the starting density depended
on n and on A's size. The first sweep's n = 10 failures came from that,
and so did `l_hall` seed 1 above. Now `place_swarm`:

- draws them in a patch around A's centre with `area_per_drone` = 2 m²
  each (about 1.2 m between neighbours). The patch is a disk, grown
  until enough of it lies in A's free space, so a narrow A just makes it
  longer. Only an A too small for n falls back to all of A, as before.
- redraws (same seed stream, so still deterministic) any swarm that
  falls apart into groups, two drones counting as linked under 2 m. With
  `l_hall`'s altitude band (0.7 m, counted twice), 2 m stays under the
  2.5 m perception radius.

Mean nearest-neighbour distance at spawn: 1.1-1.25 m on `open`,
`l_hall` and `l_turn` for 10, 20 and 40 drones. `open` at n = 40 is the
exception: its A (64 m²) is too small for 80 m², so it falls back and
is slightly denser.

**The start, woken by noise.** On `l_hall` seed 2 the flock set off away
from C within half a second of the start, too fast for 2 leaders to have
any effect. The denser spawn is still contracting toward flight spacing
at 16 s: every drone drifts at about 0.2 m/s (median 0.17, max 0.25).
That is above the old `wake_speed` (0.1 m/s), so all 20 woke at once in
random directions, and alignment made their average the flock's
direction. The leaders turned it round only after a detour into A's back
corner (90% in C at 133 s).

`wake_speed` is now in units of v_flock like every other speed, set
above a forming flock's drift and well below cruise. Leaders pass it
within half a second, so their neighbours still wake from them. The
runs below used 0.4; Marco chose **0.3 × v_flock**, between that and
the old 0.1, not yet run headless.

| `l_hall`, 20 drones, j = 2 | old spawn | new spawn, wake 0.1 | new spawn, wake 0.4 × v_flock |
|---|---|---|---|
| seed 0 | 78.1 s | 92.7 s | never (17/20) |
| seed 1 | never (11/20, split at spawn) | 74.6 s | 114.6 s |
| seed 2 | | 133.0 s (detour) | 74.5 s |

Every run with the new spawn took off as one group, and none crashed.

Seed 0's failure with wake 0.4 is a different one. Its tail (3 drones)
stopped under the gap wall's left half while the flock went through.
They then aligned with the wrong neighbours: two ended at the bottom of
hall 2, one above the gap wall's right half. A follower that loses the
flock can't find C, by design. Three seeds per setting can't separate
the wake settings. That's what the sweep (step 5) is for.

### Step 5: the n x j sweep on `l_hall`

The setup:

- n = 10, 20, 30; j = 1, 2, 4; seeds 0-2; up to 200 s simulated.
- 1 m/s, `wake_speed` 0.3 × v_flock, the constant-density spawn, one
  altitude (1.5 m).
- 25 of the 27 runs fit the hour. n = 30, seed 2, j = 2 and 4 are
  missing.
- No j = 0 runs: a leaderless flock can't find C on the L hall.

Time until 90% of the drones are in C, s (seeds 0 / 1 / 2). Where a run
didn't get everyone in, the number left outside is in brackets:

| j | n = 10 | n = 20 | n = 30 |
|---|---|---|---|
| 1 | 74 / never (2) / 76 | 76 (1) / 76 (2) / 75 (2) | 78 (1) / 115 / 95 (2) |
| 2 | 69 / 71 / 69 | 116 / 119 / 117 | 81 / never (15) / - |
| 4 | 67 / 67 / 66 | 72 / 72 / 72 | 112 / 84 / - |

Every drone got into C in 3 of 9 runs with j = 1, in 7 of 8 with j = 2,
and in 8 of 8 with j = 4. No crashes.

- **One leader brings the flock, but not all of it.** With j = 1, 1-2
  drones end in a group with no leader in 6 of 9 runs. They're the tail
  losing the flock at the gap or the corner, as in the step-4 seed 0.
  With one leader the flock's front is the only informed part, so the
  rear has nothing to correct it.
- **More leaders, sooner and safer, but not monotonic.** j = 4 brought
  everyone in, every time, in 66-72 s at n ≤ 20: about the leaders' own
  pace. That's 16 s at the start, then about 50 s for roughly 50 m
  at 1 m/s.
- **n = 20, j = 2 is slow in all three seeds** (116-119 s against 72 s
  for j = 4). That's too regular for chance; something holds the flock
  for ~45 s, worth a look.
- **n = 30, j = 2, seed 1 split in half** (15 drones in 3 leaderless
  groups).
- **Spacing is too tight.** The closest pair reached 0.10-0.12 m
  (centres) in 6 runs, all at n ≥ 20. With a 0.1 m drone that is
  touching distance, though nothing crashed. So `stop_drone` (0.3 m)
  isn't held on the real dynamics in a crowd: the limit is one-sided
  per drone, and the flight layer lags it by ~tau.
- **Walls:** 0.30 m (the margin) in every run but two (0.24 m, n = 20,
  j = 2). This time it was measured to pillars and map walls too
  (`flock_test.solid`).

## 2026-10-27 · Comparing models: Reynolds, Couzin, hybrid

**Why.** The slow n = 20, j = 2 runs turned out to be a tail of 5 drones
cut off at the gap. The tail took a fixed lap around the bottom of hall 2
before finding the gap again (about 40 s; 90% in C waits for it). The
cause: a follower copies its neighbours' direction, not their path. "Up
and to the left" is right for drones already through the gap, but slides
those still below the wall away from it. Cohesion, an additive push, lost
to the cruise.

Looking at what Couzin's model really says, Marco asked whether to go
fully Couzin, then whether to compare models outright. The comparison is
the report's question in numbers: which flocking model suits a real
drone?

**A fair comparison: swap only the social rule.** `boids.model` picks
one; everything else is shared and identical:

- the mission clock and settling in C (`boids/mission.py`);
- the stopping limit (`boids/limit.py`);
- the flight layer, perception (2.5 m), v_flock, the spawn;
- which drones lead and where C is.

| model | what decides | leaders |
|---|---|---|
| `reynolds` (phase 2) | separation + alignment + cohesion, an acceleration a; asks for v + a·τ, which the velocity layer turns back into exactly a | seek v_flock at the goal, then arrive |
| `couzin` (Couzin et al. 2005) | a direction kept as state: repulsion first and alone (< 0.6 m); else Σ unit(offset) + Σ unit(v_j) + own direction, equal weights; flown at v_flock | + 0.5 × the goal's direction (Couzin's ω) |
| `hybrid` (phase 3) | Vásárhelyi friction + repulsion + cohesion + a cruise along the neighbours' mean direction | Couzin-style mix, goal weight 1 |

What was tuned:

- `hybrid`: tuned on the L hall over phase 3.
- `couzin`: Couzin's structure as written. α = 0.6 m, ρ = 2.5 m (our
  perception), ω = 0.5. Nothing tuned.
- `reynolds`: phase 2's weights, tuned in the open room, rescaled from
  0.5 m/s to units of v_flock.

Two deliberate departures from the papers:

- A Couzin follower has no direction until a neighbour moves. His agents
  start with random directions, which would scatter the swarm at the
  start.
- Couzin's turning-rate limit and noise are left to the real drone (τ,
  sensors).

**First runs** (20 drones, 2 leaders, Webots):

| | `l_turn` (no obstacles) | `l_hall` seed 0 |
|---|---|---|
| hybrid | 90% at 71.7 s, polarization 1.00 | 116.4 s, 5-drone tail lapped hall 2, closest 0.11 m |
| couzin | 122.4 s, one group, polarization 0.8-0.9 | **124.7 s, all 20 together, closest 0.28 m** |
| reynolds | never in 150 s | never in 200 s (0/20), closest 0.26 m |

- **Couzin is slower but more cohesive and safer.** The flock averages
  about 0.7 m/s against the hybrid's 1.0 and is less aligned. But it lost
  no tail at the gap, and its closest pair stayed at 0.28 m, the stopping
  margin, against the hybrid's 0.11.
- **Reynolds' leaders get stuck at the inner corner.** Seen from hall 1,
  C lies mostly behind the hall's top wall. The straight-line seek (about
  (0.23, 0.97) from the corner) slides a leader along that wall at only
  0.2 m/s. At (31.4, 7.7) the box of hall 2's wall blocks even that, so
  the leaders sat there from about 70 s to 110 s: the classic local
  minimum of goal seeking. The hybrid and Couzin get past it because the
  flock's own momentum (the cruise, or the direction memory) carries the
  leaders round. Reynolds has none: nothing drives a drone but its
  neighbours and the goal.

### The `l_hall` sweep, three models

The same setup as phase 3's step 5: n = 10, 20, 30; j = 1, 2, 4; seeds
0-2; 200 s; one hour per model (`logs/sweep_l_hall_{model}.csv`). The
hybrid's runs are step 5's own. The budget left out n = 30, seed 2 for
j = 2 and 4 (hybrid), j = 4 (Couzin), and j = 2 and 4 (Reynolds).

Drones in C at the end (seeds 0 / 1 / 2):

| model | j | n = 10 | n = 20 | n = 30 |
|---|---|---|---|---|
| hybrid | 1 | 10 / 8 / 10 | 19 / 18 / 18 | 29 / 30 / 28 |
| hybrid | 2 | 10 / 10 / 10 | 20 / 20 / 20 | 30 / 15 / - |
| hybrid | 4 | 10 / 10 / 10 | 20 / 20 / 20 | 30 / 30 / - |
| couzin | 1 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| couzin | 2 | 9 / 10 / 0 | 20 / 0 / 0 | 0 / 24 / 0 |
| couzin | 4 | 10 / 10 / 10 | 19 / 19 / 19 | 23 / 17 / - |
| reynolds | 1 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| reynolds | 2 | 0 / 0 / 9 | 0 / 0 / 0 | 0 / 0 / - |
| reynolds | 4 | 0 / 8 / 0 | 18 / 19 / 18 | 0 / 2 / - |

Over all n:

| model | j | all in C | 90% in C | median time to 90% | closest pair, min / median |
|---|---|---|---|---|---|
| hybrid | 1 | 3 / 9 | 8 / 9 | 76 s | 0.10 / 0.24 m |
| hybrid | 2 | 7 / 8 | 7 / 8 | 81 s | 0.10 / 0.11 m |
| hybrid | 4 | 8 / 8 | 8 / 8 | 72 s | 0.11 / 0.24 m |
| couzin | 1 | 0 / 9 | 0 / 9 | - | 0.12 / 0.29 m |
| couzin | 2 | 3 / 9 | 3 / 9 | 144 s | 0.14 / 0.28 m |
| couzin | 4 | 6 / 8 | 6 / 8 | 120 s | 0.16 / 0.30 m |
| reynolds | 1 | 0 / 9 | 0 / 9 | - | 0.26 / 0.48 m |
| reynolds | 2 | 0 / 8 | 1 / 8 | 128 s | 0.26 / 0.47 m |
| reynolds | 4 | 0 / 8 | 3 / 8 | 137 s | 0.26 / 0.44 m |

"All in C" counts a run once every drone has been in C at the same
time, even if one later left. No crashes in any model.

- **The hybrid is the only model that reliably brings the flock.**
  With 2 or 4 leaders it got everyone in 15 of 16 runs, in 70-80 s,
  about the leaders' own pace. It pays in spacing: its closest pairs
  are the tightest (median 0.11 m with j = 2). It was also tuned on this
  very map, which the other two weren't. That's a confound, so this
  sweep can't say the hybrid's *rules* are better, only that the tuned
  hybrid is.
- **Couzin needs many leaders.** With j = 1, no drone reached C in any
  run, *not even the leader*: with ω = 0.5 the flock outvotes it. That
  matches Couzin et al. (2005), where accuracy grows with the informed
  fraction and a weak ω needs more informed individuals. j = 2 is a coin
  toss (3 of 9). j = 4 works up to n = 20 and degrades at n = 30
  (23 and 17 of 30, one run split). Where it works it is slower
  (120-145 s) but keeps its spacing (median 0.29 m, close to the 0.3 m
  stopping margin).
- **Couzin, n = 20, j = 4 ends with 19 in all three seeds.** Every
  drone was in C at some point in each run, then one was outside at
  200 s. Settled Couzin drones hold still, so it must have left before
  settling: a follower flies at full v_flock until it settles, unlike a
  leader, so it may overshoot C or be pushed out. Too regular for
  chance; not yet looked at in Webots.
- **Reynolds never brought the whole flock.** The inner-corner local
  minimum of the first runs holds across the sweep. With j = 4 and
  n = 20 most drones got in (18-19 of 20), but the flock split in two
  every time. Its spacing is the widest (0.44-0.48 m median), because a
  stuck flock doesn't squeeze through anything.
- **The ceiling hides the failures.** Every failed run sits at the
  200 s cap, so "time to 90%" says nothing about them. The drones in C
  at the end is the fairer primary measure. Three seeds per cell is a
  pilot, not a test: the course's method (hypotheses fixed beforehand,
  many replicas, Wilcoxon tests) needs 20-30 seeds on fewer cells.

## 2026-10-27 · Phase 5: the plan, closed

The 2026-10-15 entry set the idea: leaders plan a path on the map
(deliberative), the flocking rules stay reactive, and followers stay
untouched. Before implementing, Marco settled the open choices.

**Who plans.** Each leader, for itself, from its own position, once at
the start (the map is static). Where an obstacle splits the way, a leader
on the right takes the right passage and one on the left takes the left:
the natural choice for each. The flock may then split around the obstacle.
That's fine for the mission if each part keeps a leader; the sweep will
show how often one doesn't. Turned down: planning in the world generator,
which would leave the robot only following a list, no longer
deliberating.

**What it plans around.** The arena and the walls, including the gap
wall. Any obstacle whose footprint is smaller than 1 m across (longest
side of a box, diameter of a pillar) is ignored and left to the drones'
own avoidance, the stopping limit. So pillars, at most 0.3 m wide, go; the
gap wall's 2 m halves stay. The arena outline is always kept, whatever
the size of its pieces.

**Clearance, and so the algorithm.** No single clearance works:

- too large, and narrow passages vanish from the map, though drones fly
  through spaces narrower than the flock, as in the forest;
- too small, and the path hugs every corner and drags the flock into it.

So the clearance is soft: **A\* on a grid with a clearance cost**, the
cost-map approach of mobile robotics.

- Cells are 0.25 m.
- A cell closer than 0.3 m to a wall (the drone's stop margin) is
  forbidden.
- Closer than 2 m it costs more, rising toward the wall. In an 8 m hall
  the centre line is free; a 4 m gap is crossed through its middle; a
  narrower passage is still allowed, at a price.
- Leaders don't know n, so the clearance is one fixed value.

Then the path is pruned to its corners. A shortcut between two waypoints
is kept only if it never comes closer to a wall than the planned path
between them already did. That way pruning can't undo the clearance by
cutting a corner.

Turned down:

- A visibility graph: exact and short, but needs the one clearance we
  can't choose.
- Voronoi or medial-axis roadmaps: maximal clearance everywhere, but
  long detours in open rooms.
- Sampling planners (RRT, PRM): built for high-dimensional spaces, and
  random.

**Following the path: pure pursuit.** The leader aims at the point 2 m
ahead along its path, from wherever the flock has pushed it, so it never
replans. The models already slow a leader down by the length of its
`to_goal`. So `to_goal` becomes the direction to that point, times the
distance left along the path. A leader still stops at C, and no model
changes.

**The experiment** (hypotheses from 2026-10-15, fixed before running):
reactive against planning leaders, same n, j and seeds.

- `l_hall`: the current sweeps (all three models) are the reactive
  baseline.
- A new map whose straight line to C runs into a dead end, where
  reactive leaders should clearly fail.
- Plan on and off, swept once the code is in.

### Wired in, first runs

`plan.planning` switches it (`--plan 1`), off by default so every earlier
run stays the baseline. The world now gives the arena outline
(`boundary`) apart from the map's `walls`: the plan needs both, the
stopping limit gets them joined as before. A leader plans once, at go
time, from where it is; with no passage wide enough, it aims straight at
C as before. Yes/no flags now take 1/0, true/false or on/off:
`bool("0")` is True, so `--plan 0` would have switched it on.

The new map is `maps/dead_end.toml`: an open room with a cup between A
and C, open toward A, so the straight line to C ends in its bottom.

20 drones, 2 leaders, hybrid, seed 0:

| | straight at C | planned |
|---|---|---|
| `dead_end` | 0/20 in C: the flock presses into the cup and stays (its centre moved 5 m in 150 s) | 90% at 118 s, 19/20 |
| `l_hall` | 116 s (the sweep's run: the tail laps hall 2) | **76 s**, 20/20, one group |

- The dead end traps reactive leaders, as expected, and planning ones get
  round it.
- On `l_hall` the plan also fixed the slow tail: the path rounds the
  corner wide and crosses the gap in its middle, so the tail isn't cut
  off by the gap's wall. One seed; the sweep will tell.
- On `dead_end` the flock split into up to 3 groups from ~56 s, when the
  leaders turn round the cup's arm. One drone ended outside C. One run:
  noted, not chased yet.
- Pairs still touch (0.10 m), plan or not: the crowd spacing problem of
  step 5.

The plan on/off sweeps were started, then stopped after 7 runs: the
models get tuned first (next section), so the sweeps run once, on the
tuned models.

## 2026-10-27 · Tuning Couzin and Reynolds: the protocol

**Why.** The three-model sweep compared a hybrid tuned over a whole phase
on `l_hall` with two models taken as published. "The hybrid wins" then
mostly measures the tuning. Marco proposed tuning the other two.

**How, fixed before any tuning run:**

- **Same budget.** Each model gets 15 runs. All 12 combinations of 2
  settings run on seed 100, then the best 3 run again on seed 101, and
  the winner is the best of those 3 over both seeds. (First planned as
  12 × 2 seeds = 24 runs, cut for time after 5 runs, before any ranking.)
  The hybrid's budget is already spent: its phase-3 settings are its
  tuned version.
- **Same setup.** `l_hall`, 20 drones, 2 leaders, plan off (as the
  hybrid was tuned), 200 s, seeds 100 and 101. The sweeps use seeds
  from 0, so tuning and evaluation never share a seed.
- **Same score.** Mean fraction of drones in C at the end first; then
  the median time to 90% in C. The tie-break looks at spacing only by
  eye: a setting that wins by crowding to touching distance gets
  flagged.
- **The grids**, two settings each, chosen from how each model failed:

| model | setting | values | why |
|---|---|---|---|
| couzin | `couzin_weight` (ω) | 0.5, 1, 2, 3 | with j = 1 the flock outvoted the leader |
| couzin | `couzin_repulsion` (α) | 0.4, 0.6, 0.8 m | spacing against cohesion at the gap |
| reynolds | `reynolds_goal` | 1, 2, 4, 8 | the leaders' pull against the corner and the flock |
| reynolds | `reynolds_alignment` | 1, 2, 4 | how much a follower copies the leaders' velocity |

- **Then evaluate on what nobody tuned on.** All three models were tuned
  on `l_hall`, so `l_hall` results say how well each fits its tuning
  map. The comparison that counts runs on `dead_end` and `l_turn` (and
  possibly a new map), with the sweep seeds, plan on and off: model ×
  plan.

**Hypotheses:**

1. Couzin has a best ω in the middle. Too low, the flock outvotes the
   leaders (the j = 1 failure); too high, the leaders leave it behind
   (Couzin et al. 2005).
2. No Reynolds setting brings the whole flock round `l_hall` with the
   plan off. The inner corner is a local minimum of straight-line goal
   seeking, which weights can soften but not remove. With the plan on,
   it should.
3. Tuned, Couzin closes most of the gap to the hybrid on reaching C, and
   keeps its wider spacing.

`tools/tune.py` runs a grid and ranks it
(`logs/tune_<map>_<model>.csv`).

### Tuning results

Couzin, seed 100 (drones in C at 200 s, time to 90%):

| ω \ α | 0.4 m | 0.6 m | 0.8 m |
|---|---|---|---|
| 0.5 | 18/20, 116 s | 0/20 | 0/20 |
| 1 | 20/20, 184 s | 2/20 | 0/20 |
| 2 | 20/20, 74 s | 0/20 | 0/20 |
| 3 | 0/20, closest pair 0.02 m | 0/20 | 0/20 |

The best 3 on seed 101: ω = 1 got 20/20 at 83 s, ω = 2 got 18/20 (90% at
97 s), and ω = 0.5 got 0/20.

- **Winner: ω = 1, α = 0.4 m**, all 20 in on both seeds. Flagged: its
  closest pair was 0.10 m in both runs, touching distance, the hybrid's
  own crowd problem. ω = 2 kept 0.13-0.16 m and was faster but lost 2
  drones on seed 101. With two seeds the two can't be told apart.
- **α decides everything.** Only 0.4 m works: with a wider zone of
  repulsion, which overrides everything else, drones spend too long only
  avoiding each other in the narrow spots (the gap, the corner).
- **Hypothesis 1 holds:** ω has a best value in the middle. At ω = 3 the
  leaders push through the flock, and the closest pair reached 0.02 m.

Reynolds, seed 100: 0/20 for 11 of the 12 combinations; goal 4 with
alignment 4 got 1/20, and 1/20 again on seed 101. With eleven tied at
zero there was no "best 3" to confirm, so Reynolds got 13 runs, not 15.

- **Winner: goal 4, alignment 4**, though it hardly works.
- **Hypothesis 2 holds** so far: no weights bring Reynolds round the
  corner with the plan off. It stays at the inner corner, as before.

Both winners are now the defaults (`boids/params.py`, with the untuned
values in the comments). The hybrid's are unchanged.

### The wall was a brake: Couzin retuned with the plan on too

The first evaluation runs (seed 0, 20 drones, 2 leaders) had Couzin
fail *with* the plan: 2/20 on `dead_end` and 3/20 on `l_turn`, where two
drones collided (0.03 m). Without the plan it got 18/20 and 20/20.

Replaying `l_turn` (the run is deterministic) showed why. At go time the
two leaders were already 2.4-3.3 m from the flock's centre, at the edge
of perception. They flew the path at a full 1.00 m/s while the flock
averaged 0.85-0.9: with a hard 0.4 m repulsion zone the followers
zig-zag. By 24 s the leaders were 6-7 m ahead, out of sight for good.
The leaderless flock kept its heading to the end of hall 1 and stayed
there; the collision was inside it.

Aiming straight at C, the leaders press into the hall's wall, and the
stopping limit slows them to the flock's pace. That wall was a brake,
and ω was tuned with it. The plan keeps the leaders off the walls, and
the brake goes. The hybrid's and Reynolds' leaders also feel the flock
(cohesion); a Couzin leader only feels the neighbours it still sees.

**The mistake was the protocol's, not the model's:** ω was tuned with
the plan off only, so it fits leaders the walls hold back. Marco's call:
retune ω so that one value works with the plan off and on. The other
options were to report Couzin as it stands, or to give its leaders a
rule to wait for the flock, which is no longer Couzin's model.

The retune, fixed before running:

- ω = 0.25, 0.5, 0.75, 1 (the current value as the reference); α stays
  0.4 m, the only value that worked.
- Each ω runs with the plan off and on (`tune.py --across
  plan.planning=0,1`), on `l_hall` with seed 100. The best 2 run again
  on seed 101. 12 runs.
- The score is the fraction of drones in C, averaged over plan off and
  on, then time to 90%.
- Hypothesis: a weaker pull keeps the leaders inside the flock with the
  plan on, at some cost in arrival with the plan off.

The Couzin seed-0 evaluation runs were made with the old ω, so they're
set aside (`logs/stale/`) and run again with the new one.

**Result: ω stays 1.** Drones in C at 200 s, plan off / on:

| ω | seed 100 | seed 101 |
|---|---|---|
| 0.25 | 0/20 · 20/20 | - |
| 0.5 | 18/20 · 20/20 | 0/20 · 20/20 |
| 0.75 | 20/20 · 0/20 | - |
| **1** | **20/20 · 20/20** | **20/20 · 20/20** |

- **The hypothesis is rejected.** A weaker pull didn't keep the leaders
  in the flock: ω = 1 got all 20 in on all 4 runs; the others failed on
  some. With no trend in ω (0.75 fails where 0.5 and 1 work), each run
  is all or nothing.
- **The failure doesn't happen on the tuning map.** On `l_hall`, ω = 1
  works with the plan on (20/20 at 78-89 s). It failed on `l_turn` and
  `dead_end`, held out from tuning on purpose. So no `l_hall` tuning can
  fix it. Losing the flock depends on where the leaders stand at go
  time and on the followers' slower pace, not on how hard they pull.
- **So it's a property of the model, not a tuning error.** A Couzin
  leader gets no feedback on whether it's followed. Off the walls,
  nothing holds it to the flock's pace, and once out of sight it's
  gone for good. The evaluation's 10 seeds measure how often.

**Decision (Marco): keep Couzin as published.** A rule for its leaders
to wait for the flock would fix this, but it's no longer Couzin's model.
The failure is reported as a finding, measured over the seeds. A waiting
variant could be a fourth model later.

With ω unchanged, the seed-0 runs set aside are valid again (Webots runs
are deterministic), so they're back in `logs/`. Seeds 1-9 run with the
hybrid and Reynolds first, Couzin last.

## 2026-10-28 · The evaluation: held-out maps, plan off and on

The comparison the tuning protocol pointed to: the three tuned models
on the two maps none of them was tuned on, `dead_end` and `l_turn`, with
the plan off and on. 20 drones, 2 leaders, 200 s, seeds 0-9: 120 runs,
`logs/sweep_<map>_<model>[_plan].csv`. The same seed puts the drones in
the same starting spots for every model and plan setting, so the
comparisons pair the runs by seed (Wilcoxon signed-rank on the drones in
C at the end; with 10 pairs the smallest possible p is 0.002).

Drones in C at 200 s, per seed 0-9, and how many runs reached 90%:

| map | model | plan off | 90% | plan on | 90% |
|---|---|---|---|---|---|
| `dead_end` | hybrid | 0 0 18 0 10 0 0 0 0 0 | 1/10 | 19 19 20 20 19 19 19 18 0 0 | 8/10 |
| | couzin | 18 18 18 0 0✗ 18 20 18 18 0✗ | 7/10 | 2 20 20 20 1 19 20 17 1✗ 20 | 6/10 |
| | reynolds | all 0 | 0/10 | 20 17 20 20 19 2 19 2 20 20 | 8/10 |
| `l_turn` | hybrid | all 20 | 10/10 | all 20 | 10/10 |
| | couzin | 20 18 20 20 20 20 20 20 20 20 | 10/10 | 3✗ 20 20 20 20 20 20 20 20 20 | 9/10 |
| | reynolds | 0 (one run: 1) | 0/10 | all 20 | 10/10 |

✗: a run with a crash (two drones collided).

Median time to 90% in C, over the runs that got there, plan off / on:
`dead_end` hybrid 148 s (one run) / 113 s, Couzin 97 / 58 s, Reynolds -
/ 56 s; `l_turn` hybrid 68 / 66 s, Couzin 75 / 75 s, Reynolds - / 69 s.

**Plan on against off**, per model (seeds better / worse / tied, p):

| | `dead_end` | `l_turn` |
|---|---|---|
| hybrid | 8 / 0 / 2, p = 0.008 | identical (all 20/20); 90% 2 s sooner on all 10, p = 0.002 |
| couzin | 6 / 3 / 1, p = 0.40 | 1 / 1 / 8, p = 1 |
| reynolds | 10 / 0 / 0, p = 0.002 | 10 / 0 / 0, p = 0.002 |

**Findings:**

- **The plan is what gets the hybrid and Reynolds through `dead_end`.**
  Aiming straight at C, they push into the cup and stay there: one run
  in 20 reached 90%. With the plan, 16 in 20. The three significant
  differences survive a Holm correction over the six plan comparisons.
- **Hypothesis 2 holds:** with the plan, Reynolds goes round the corner
  every time on `l_turn` (0/10 → 10/10). The corner trap is the
  straight-line goal, not the rules.
- **Couzin neither gains nor loses from the plan overall**, but it
  changes how it fails. Without the plan it fails 3 times in 10 on
  `dead_end`; with it, 4 runs in 20 end with only the leaders and one or
  two followers in C (seeds 0, 4, 8 on `dead_end`, seed 0 on `l_turn`):
  the leaders lost the flock, as the replay showed. When the flock does
  follow, the plan makes it fast (58 s against 97 s on `dead_end`).
- **With the plan on, the three models can't be told apart** on reaching
  C (all paired p > 0.8). Each fails on a few seeds, each its own way:
  the hybrid's whole flock stays behind on seeds 8 and 9 (0/20),
  Reynolds loses most of it on seeds 5 and 7 (2/20), Couzin's leaders
  leave it on three seeds.
- **Plan off, Couzin is the best on `dead_end`** (7/10 against the
  hybrid's 1/10, p = 0.03; against Reynolds' 0/10, p = 0.016), though
  neither survives a correction over all the model comparisons. Why is
  a guess so far (its leaders slide along the cup's walls where the
  others stop); it wants a replay before it goes in the report.
- **The hybrid is slow on `dead_end` with the plan.** Two runs reach
  90% at 47 s, six at 108-118 s, as fast as Couzin and Reynolds (54-63
  s) only on the two quick seeds. Something holds most of its flock back
  for about a minute: a split at the cup is the guess, to replay.
- **Hypothesis 3 holds for reaching C, not for spacing.** Tuned Couzin
  matches or beats the hybrid on reaching C. But it isn't wider: its
  closest pairs are 0.10-0.22 m (median), and it is the only model that
  crashed, in 4 runs of 40. Reynolds keeps 0.42-0.51 m and never
  crashed; the hybrid reaches touching distance (0.10 m) but didn't
  crash.
- **Ceiling on `l_turn`.** Hybrid and Reynolds with the plan put all 20
  in C every time; only time to 90% separates them there. `l_turn` is
  too easy to rank models on reaching C, `dead_end` is the map that
  discriminates.

Still to look at: replays of the hybrid's slow and failed `dead_end`
runs with the plan (seeds 8, 9 and one slow one), Couzin's wall-sliding
on `dead_end` without the plan, and the crowding to 0.10 m.

## 2026-10-28 · Leaders plan again; runs end at a time limit

### The replays: the leaders stuck in the cup

The hybrid on `dead_end` with the plan, replayed (the runs repeat
exactly):

- **Seed 8 (0/20).** At go time the followers line up with the leaders
  and the whole flock heads +x, into the cup's mouth, dragging both
  leaders in. From 40 s to the end both leaders sit at x = 21.5, the
  stopping margin in front of the cup's back wall, with the flock
  around them.
- **Seed 0 (19/20, 90% at 118 s).** One leader is stuck the same way
  to the end. The other gets back out of the cup and round, but the
  flock has lost it by then. The followers carry on west to the room's
  wall, then down and along the bottom wall into C, which they reach
  by chance: the room is closed and C is at its far end.

**The cause is the path following, not the plan.** Pure pursuit aims at
the point 2 m along the path in a straight line. Pushed into the cup,
the leader finds that point beyond the cup's wall, aims through it, and
the stopping limit holds it there: the local minimum the plan was meant
to remove, back on a small scale. Phase 5 assumed the leader "never
replans" because pursuit brings it back to the path from wherever the
flock pushes it. That holds in open space, not behind a wall.

Options (Marco chose 1):

1. **Plan again when a wall cuts the leader off.** Monitoring the
   plan's execution and replanning when it fails is the missing part of
   a hybrid architecture, not a patch on the reactive layer.
2. Keep planning once and report it as a limitation. The n × j sweep on
   `dead_end` would then partly measure this flaw instead of
   leadership.

### Planning again

- **When.** Whenever an obstacle the plan goes round (the arena outline,
  walls and pillars at least 1 m across) stands between the leader and
  its lookahead point. The test is exact (segment against box or disk),
  so following the path normally never triggers it.
- **How often.** At most once a second (`plan.replan_every`), which
  bounds the cost of a leader stuck where the view keeps flickering.
- **From where.** From the leader's position, with the same A* and
  clearance. The grid is built once per leader (`planner.Planner`), so
  each new plan costs one search. If no path is found, the old one
  stays.
- **The code.** `planner.Route` holds the plan and the pursuit and
  decides when to plan again; the controller only asks it for
  `to_goal`. The `planner` tests cover a cup like `dead_end`'s: on the
  path, one plan; pushed into the cup, a new plan heading back out of
  its mouth first.

Replayed with replanning:

| seed | plan once | plan again |
|---|---|---|
| 8 | 0/20; both leaders stuck in the cup | 20/20, 90% at 120 s; leaders in C at ~80 and ~100 s |
| 0 | 19/20, 90% at 118 s; one leader stuck | 20/20, 90% at 110 s; both leaders in C by ~100 s |

The leaders now get out, but the flock still doesn't follow them. When
they turn back out of the cup, the followers keep their heading west,
out of the leaders' sight, and reach C later by wandering. That's no
longer a planner flaw: it's the leadership question itself, what two
unrecognised leaders can do against eighteen followers' momentum.

### A time limit, and a control with no leaders

**The problem.** A closed room lets a flock that lost its leaders reach
C by chance, given time. At 200 s, those runs count as successes: false
positives for leadership. Marco's proposal: end each run soon after the
time a guided flock needs.

**The rule:** the limit is the go time plus twice the time the leaders
would need alone along their planned path (A's centre to C's centre, at
v_flock):

| map | planned path | leaders alone (go at 16 s) | limit |
|---|---|---|---|
| `dead_end` | 33.4 m | 49 s | **83 s** |
| `l_turn` | 47.9 m | 64 s | **112 s** |

- Twice, not "+10 s": larger flocks take longer to pass a gap (on
  `l_hall`, n = 30 took 10-40 s more than n = 20), so a tight margin
  would count guided large flocks as failures and bias the n × j result
  against large n.
- It separates the evaluation's runs: on `dead_end` guided arrivals took
  47-63 s, the wanderers 108-148 s.
- A tighter cutoff can still be applied afterwards from each run's time
  to 90%; a looser one couldn't.

**The measure becomes "90% in C within the limit"**, a success or a
failure per run, with the time to 90% second. Runs end at the limit, so
sweeps get faster too. A flock with no leaders (j = 0) runs at the same
limit as a control: it measures how often wandering alone succeeds.

**Too hard a limit? Checked, kept.** With replanning, the hybrid's flock
reached 90% on `dead_end` within the limit in only 2 of 10 runs, which
raised the question. But at the limit only the leaders (1-2 drones) were
in C, not a flock half in, and in both replays the followers' centre was
15-23 m from the nearest leader at 80 s, far outside the 2.5 m perception: lost,
not slow. A looser limit (3×) as a check was considered and dropped.

The 200 s evaluation's plan-off runs are unaffected by replanning and
are rescored from their time to 90%. The plan-on runs are redone with
replanning and the limits; the plan-once results are kept in
`logs/stale/plan-once/`.

### The evaluation again: replanning, success within the limit

Same runs as the 2026-10-28 evaluation (20 drones, 2 leaders, seeds
0-9), plan-on runs redone, all scored as success = 90% in C within the
limit. This replaces that section's findings where they differ. Paired
by seed, exact McNemar test (binomial on the seeds where the two
differ; with 10 seeds the smallest possible p is 0.002).

Runs reaching 90% in C within the limit (+ per seed, 0-9):

| | `dead_end` (83 s), plan off | plan on | `l_turn` (112 s), plan off | plan on |
|---|---|---|---|---|
| hybrid | 0/10 | 2/10 `..++......` | 10/10 | 10/10 |
| couzin | 3/10 | 6/10 `.+++.++..+` | 9/10 | 9/10 |
| reynolds | 0/10 | 8/10 `+++++.+.++` | 0/10 | 10/10 |

Median time to 90% where reached: `dead_end` plan on hybrid 47 s,
Couzin 57 s, Reynolds 56 s; `l_turn` 66-75 s for every model and
setting that got there.

- **Replanning changed only the failures.** Every run that succeeded
  with one plan succeeded with replanning at the same time to the
  millisecond: replanning never triggered when the flock followed.
- **With the plan, the order on `dead_end` reverses.** Reynolds, which
  never got round anything without the plan, brings the flock round the
  cup in 8 of 10 runs; Couzin in 6; the hybrid, the best model on its
  tuning map, in 2. Hybrid against Reynolds: 6 seeds to Reynolds, none
  the other way, p = 0.03 (uncorrected). In the hybrid's 8 failures
  only the leaders (1-2 drones) are in C at the limit.
- **Why, a hypothesis for the n × j sweep and the replays:** the
  hybrid's followers keep a cruise along their neighbours' mean
  direction, damped by friction, so the flock has momentum, and two
  leaders turning back out of the cup can't turn it. Reynolds'
  followers have no cruise of their own: tuned alignment and cohesion
  make them copy the leaders' turn.
- **The plan matters most for Reynolds** (`dead_end` 0 → 8/10, p = 0.008;
  `l_turn` 0 → 10/10, p = 0.002). For the hybrid and Couzin the
  differences are 2 and 4 seeds, not significant with 10.
- **Plan off, rescored**: Couzin's 7/10 on `dead_end` at 200 s is 3/10
  within the limit. Its late arrivals were mostly the slow drift the
  limit is meant to exclude.
- **`l_turn` is at the ceiling** for the hybrid and Couzin either way and
  for Reynolds with the plan: it doesn't separate them.
- **Safety unchanged:** Couzin is the only model with crashes (1 run of
  10 per map with the plan); Reynolds keeps the widest spacing (closest
  pair 0.44 m or more), the hybrid and Couzin reach touching distance.
  Plan-off crash counts come from 200 s runs, so they include crashes
  after the limit.

The final sweep's question follows: does the hybrid's flock follow with
more leaders, and how does each model's success grow with j at each n?

## Open questions

- Do followers recognise leaders (a flag in the IR packet) or not (Couzin)?
  Default: not; marked leaders could be a variant.
- Do drones yaw to face their velocity? Not needed since obstacles come
  from the map (phase 3); yaw stays constant.
- `DRONE_RADIUS = 0.1` makes the reachability check permissive: a 0.2 m
  passage lets one drone through, not a flock. Raise it?
- IR range: 3 m is a placeholder, to tune in phase 1 against the flock's
  spacing. (A drone's own body does not block its IR: checked, see above.)
- Phase 4: which obstacles can be flown over in the reachability check
  (today every obstacle blocks).
