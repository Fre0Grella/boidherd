"""Flocking settings: every number the rules use, in one place.

Change a default here, or build `BoidParams(...)` with other values for a
run; nothing else in the boids code hard-codes these.

Vásárhelyi's terms start from the defaults of the authors' simulator
(csviragh/robotsim, `algo_spp_evol.c`: 4 m/s cruise, 10 m spacing),
scaled to ours (0.5 m/s, 1 m): speeds by 1/8, distances by 1/10, rates
(1/s) kept as the ratio speed / distance, accelerations as speed^2 /
distance. Tuned from there (design log).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class BoidParams:
    # who counts: neighbours within this (ellipsoidal) radius; the IR range (3 m) caps it anyway
    perception_radius: float = 2.5  # m
    vertical_scale: float = 2.0  # a neighbour above or below counts as this x closer (zone.py)
    # self-propulsion: cruise at v_flock the way I am already going
    v_flock: float = 0.5  # m/s
    wake_speed: float = 0.1  # m/s: below it no cruise (the heading is noise), full at 2x
    # repulsion: linear, from repulsion_radius in
    repulsion_radius: float = 1.0  # m (robotsim R_0)
    repulsion_gain: float = 0.5  # 1/s: m/s of push per metre inside the radius
    # friction (velocity alignment): neighbours may differ in velocity by the braking curve
    # of their distance, never less than friction_slack; the excess is corrected
    # robotsim's C_Frict is 1, but friction sums over ~10 neighbours: at 1 a drone corrects a
    # velocity difference ~10x faster than it can tilt, and a resting flock jiggles at 0.2 m/s
    # in Webots; at 0.2 it settles (design log, step 2)
    friction_gain: float = 0.2
    friction_slack: float = 0.06  # m/s (robotsim V_Frict)
    friction_radius: float = 2.0  # m: closer than this only the slack is allowed
    friction_slope: float = 0.5  # 1/s, linear part of the braking curve
    friction_braking: float = 0.4  # m/s^2, constant-deceleration part
    # cohesion (Reynolds, ours: Vásárhelyi has none): toward the neighbours' centre
    cohesion: float = 0.2  # 1/s: m/s per metre to the centre
    # leaders only (Couzin's informed individuals): cruise toward the goal, mixed with
    # their own heading with weight goal_weight, slowing down within arrive_radius
    goal_weight: float = 1.0
    arrive_radius: float = 3.0  # m
    # the mission clock: before go_time the flock forms (no cruise, no goal); from then
    # leaders head for the goal and everyone may cruise (design log, the start sequence)
    go_time: float = 16.0  # s of simulated time (all hover by ~6 s)
    # limits
    max_speed: float = 1.0  # m/s, the wanted velocity is capped here
    # 0: fixed altitude (z of the result dropped); 1: full 3D (phase 4)
    vertical_gain: float = 0.0
