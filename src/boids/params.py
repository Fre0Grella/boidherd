"""Flocking settings: every number the rules use, in one place.

Change a default here, or build `BoidParams(...)` with other values for a
run; nothing else in the boids code hard-codes these.

The flock's speed is one number, `v_flock`; every other speed-dependent
setting is given in units of it ("x v_flock"), so changing it alone makes
the same flock faster or slower: speeds and rates (1/s) scale with v_flock,
decelerations with v_flock^2, distances not at all. The rules are then
exactly speed-free: at k times the speed every wanted velocity is k times
as large (a test checks it). The robot isn't: tau and max_acceleration
don't scale, so a faster flock tilts later and packs tighter (design log,
step 3). Above ~1.25 m/s the braking curves ask for more deceleration than
the flight layer's max_acceleration (2.5 m/s^2).

Vásárhelyi's terms start from the defaults of the authors' simulator
(csviragh/robotsim, `algo_spp_evol.c`: 4 m/s cruise, 10 m spacing),
scaled to ours (1 m/s, 1 m): distances by 1/10, everything else in units of
the cruise speed as above. Tuned from there (design log).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class BoidParams:
    # who counts: neighbours within this (ellipsoidal) radius; the IR range (3 m) caps it anyway
    perception_radius: float = 2.5  # m
    vertical_scale: float = 2.0  # a neighbour above or below counts as this x closer (zone.py)
    # self-propulsion: cruise at v_flock along my heading (cruise.py)
    v_flock: float = 1.0  # m/s: THE speed knob, everything marked "x v_flock" follows it
    wake_speed: float = 0.1  # m/s: a drone wakes, and a heading counts, from this speed
    settle_distance: float = 2.5  # m a drone keeps cruising into C after it recognises it
    # repulsion: linear, from repulsion_radius in
    repulsion_radius: float = 1.0  # m (robotsim R_0)
    repulsion_gain: float = 1.0  # 1/s x v_flock: push per metre inside the radius
    # friction (velocity alignment): neighbours may differ in velocity by the braking curve
    # of their distance, never less than friction_slack; the excess is corrected
    # robotsim's C_Frict is 1, but friction sums over ~10 neighbours: at 1 a drone corrects a
    # velocity difference ~10x faster than it can tilt, and a resting flock jiggles at 0.2 m/s
    # in Webots; at 0.2 it settles (design log, step 2)
    friction_gain: float = 0.2
    friction_slack: float = 0.12  # x v_flock (robotsim V_Frict)
    friction_radius: float = 2.0  # m: closer than this only the slack is allowed
    friction_slope: float = 1.0  # 1/s x v_flock, linear part of the braking curve
    friction_braking: float = 1.6  # m/s^2 x v_flock^2, constant-deceleration part
    # walls (Vásárhelyi's shill agents): at the nearest point of every wall a virtual drone
    # flies away from it at wall_speed; mine may differ from its velocity only by the braking
    # curve of my distance to the wall, the excess is corrected (gain 1, as in robotsim)
    wall_speed: float = 1.5  # x v_flock (robotsim V_Shill); above 1 or walls lose
    wall_radius: float = 0.0  # m: the braking curve starts this far from the wall (R_0_Shill)
    wall_slope: float = 1.0  # 1/s x v_flock, linear part of the curve (Slope_Shill)
    wall_braking: float = 1.6  # m/s^2 x v_flock^2, constant-deceleration part (Acc_Shill)
    # cohesion (Reynolds, ours: Vásárhelyi has none): toward the neighbours' centre
    cohesion: float = 0.4  # 1/s x v_flock: per metre to the centre
    # leaders only (Couzin's informed individuals): cruise toward the goal, mixed with
    # their own heading with weight goal_weight, slowing down within arrive_radius
    goal_weight: float = 1.0
    arrive_radius: float = 3.0  # m
    # the mission clock: before go_time the flock forms (no cruise, no goal); from then
    # leaders head for the goal and everyone may cruise (design log, the start sequence)
    go_time: float = 16.0  # s of simulated time (all hover by ~6 s)
    # limits
    max_speed: float = 2.0  # x v_flock, the wanted velocity is capped here
    # 0: fixed altitude (z of the result dropped); 1: full 3D (phase 4)
    vertical_gain: float = 0.0
