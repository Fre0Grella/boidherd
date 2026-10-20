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

## Open questions

- Do followers recognise leaders (a flag in the IR packet) or not (Couzin)?
  Default: not; marked leaders could be a variant.
- Do drones yaw to face their velocity (needed for steer-to-avoid with the
  front sensor)? Deferred to phase 3; yaw is held constant until then.
- `DRONE_RADIUS = 0.1` makes the reachability check permissive: a 0.2 m
  passage lets one drone through, not a flock. Raise it?
- IR range: 3 m is a placeholder, to tune in phase 1 against the flock's
  spacing. (A drone's own body does not block its IR: checked, see above.)
- Phase 4: which obstacles can be flown over in the reachability check
  (today every obstacle blocks).
- Phase 5, planner: A* on a grid (simple, any shape) or shortest paths on a
  visibility graph of the shrunk free space (exact, few waypoints; shapely
  is already a dependency)?
- Phase 5, where the map comes from: the generator writes the course into
  the leaders' `customData` and the leader plans (controllers still never
  parse map files), or the generator plans and writes only the waypoints
  (simpler, but then the robot no longer deliberates). Leaning: the first.
- Phase 5, clearance: a path hugging the inner corner drags the flock into
  it, and the flock is wider when n is larger. Plan with a clearance of
  about the flock's radius, not `DRONE_RADIUS`?
