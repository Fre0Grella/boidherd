"""Write a course and a swarm as a Webots world (.wbt) file.

One small function per kind of node, joined by `render_world()`. The world
is self-contained: each drone finds its takeoff altitude (and, for leaders,
the goal) in its `customData`, and the referee finds the sectors in its
`controllerArgs`, so no controller ever reads a map file.
"""

import json
import math

from maps.course import Course
from maps.model import Pillar, Rect, Sector, Wall

from .spawn import Drone

PROTOS = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects"
EXTERNPROTOS = (
    f"{PROTOS}/objects/backgrounds/protos/TexturedBackground.proto",
    f"{PROTOS}/objects/backgrounds/protos/TexturedBackgroundLight.proto",
    f"{PROTOS}/objects/floors/protos/Floor.proto",
    f"{PROTOS}/robots/bitcraze/crazyflie/protos/Crazyflie.proto",
)
CRAZYFLIE_REST_Z = 0.015  # the Crazyflie proto's height when standing on the floor
IR_CHANNEL = 1

FLOOR_TILE = 4.0  # metres; Webots' default 0.5 flickers when zoomed out
GREY = (0.55, 0.57, 0.6)  # arena boundary, darker than the floor
ORANGE = (0.9, 0.45, 0.1)  # obstacles
GREEN = (0.1, 0.7, 0.2)  # sector A
GOLD = (1.0, 0.8, 0.0)  # sector C


def render_world(
    course: Course,
    swarm: tuple[Drone, ...],
    title: str,
    ir_range: float = 3.0,
    controller: str = "drone",
    time_step: int = 32,
) -> str:
    """The whole .wbt file.

    `ir_range`: how far the infra-red packets carry, metres.
    `controller`: the drones' controller (another one only for tests and tools).
    `time_step`: Webots' basic time step, milliseconds.
    """
    nodes = [
        _header(),
        _world_info(title, time_step),
        _viewpoint(course.floor),
        "TexturedBackground {\n}\nTexturedBackgroundLight {\n}\n",
        _floor(course.floor),
        _sector_marker(course.sector_a, "sector A", GREEN),
        _sector_marker(course.sector_c, "sector C", GOLD),
        *(_wall(w, f"boundary {i}", course.ceiling, GREY) for i, w in enumerate(course.boundary)),
        *(_wall(w, f"wall {i}", course.ceiling, ORANGE) for i, w in enumerate(course.walls)),
        *(_pillar(p, f"pillar {i}", course.ceiling) for i, p in enumerate(course.pillars)),
        _referee(course),
        *(_drone(d, course.sector_c, ir_range, controller) for d in swarm),
    ]
    return "\n".join(nodes)


# --- scene ------------------------------------------------------------------


def _header() -> str:
    lines = ["#VRML_SIM R2025a utf8", ""]
    lines += [f'EXTERNPROTO "{url}"' for url in EXTERNPROTOS]
    return "\n".join(lines) + "\n"


def _world_info(title: str, time_step: int) -> str:
    return f"WorldInfo {{\n  title {_string(title)}\n  basicTimeStep {time_step}\n}}\n"


def _viewpoint(floor: Rect, heading: float = 3 * math.pi / 4, pitch: float = 0.8) -> str:
    """An overview of the whole floor, looking down at its centre.

    Webots cameras look along their x axis with z up; the orientation is a
    turn of `heading` about z (3/4 pi: towards the north-west, so seen from
    the south-east corner) after tilting down by `pitch` about y.
    """
    cx, cy = floor.center
    distance = 1.1 * math.hypot(floor.width, floor.depth)  # fits the diagonal in 45°
    forward = (
        math.cos(pitch) * math.cos(heading),
        math.cos(pitch) * math.sin(heading),
        -math.sin(pitch),
    )
    x, y, z = (c - distance * f for c, f in zip((cx, cy, 0.0), forward, strict=True))
    axis, angle = _axis_angle(heading, pitch)
    return (
        "Viewpoint {\n"
        f"  orientation {' '.join(_num(a) for a in axis)} {_num(angle)}\n"
        f"  position {_num(x)} {_num(y)} {_num(z)}\n"
        "}\n"
    )


def _axis_angle(yaw: float, pitch: float) -> tuple[tuple[float, float, float], float]:
    """Rotation `yaw` about z composed with `pitch` about y, as an axis and an angle."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    w, x, y, z = cy * cp, -sy * sp, cy * sp, sy * cp  # q_z(yaw) * q_y(pitch)
    angle = 2 * math.acos(min(w, 1.0))
    s = math.sqrt(max(1 - w * w, 0.0))
    if s < 1e-9:  # no rotation: any axis will do
        return (0.0, 0.0, 1.0), 0.0
    return (x / s, y / s, z / s), angle


def _floor(floor: Rect) -> str:
    """Webots' own checkerboard, with tiles large enough not to flicker from far away."""
    cx, cy = floor.center
    return (
        "Floor {\n"
        f"  translation {_num(cx)} {_num(cy)} 0\n"
        f"  size {_num(floor.width)} {_num(floor.depth)}\n"
        f"  tileSize {_num(FLOOR_TILE)} {_num(FLOOR_TILE)}\n"
        "}\n"
    )


def _sector_marker(sector: Sector, name: str, color: tuple[float, float, float]) -> str:
    """A glowing patch on the floor and a see-through box over the sector's altitude band.

    Visual only: no boundingObject, so nothing collides with it and it blocks
    neither IR nor the distance sensors.
    """
    cx, cy = sector.area.center
    w, d = _num(sector.area.width), _num(sector.area.depth)
    z_min, z_max = sector.z
    return (
        f"DEF {name.upper().replace(' ', '_')} Pose {{\n"
        f"  translation {_num(cx)} {_num(cy)} 0\n"
        "  children [\n"
        "    Pose {\n"
        "      translation 0 0 0.01\n"  # high enough to avoid z-fighting from far away
        "      children [\n"
        "        Shape {\n"
        f"          appearance {_appearance(color, transparency=0.25, glow=0.6)}\n"
        f"          geometry Plane {{ size {w} {d} }}\n"
        "        }\n"
        "      ]\n"
        "    }\n"
        "    Pose {\n"
        f"      translation 0 0 {_num((z_min + z_max) / 2)}\n"
        "      children [\n"
        "        Shape {\n"
        f"          appearance {_appearance(color, transparency=0.85, glow=0.3)}\n"
        f"          geometry Box {{ size {w} {d} {_num(max(z_max - z_min, 0.02))} }}\n"
        "          castShadows FALSE\n"
        "        }\n"
        "      ]\n"
        "    }\n"
        "  ]\n"
        "}\n"
    )


def _wall(wall: Wall, name: str, ceiling: float, color: tuple[float, float, float]) -> str:
    height = ceiling if wall.height is None else wall.height
    cx, cy = wall.rect.center
    size = f"{_num(wall.rect.width)} {_num(wall.rect.depth)} {_num(height)}"
    return _solid(name, (cx, cy, height / 2), f"Box {{ size {size} }}", color)


def _pillar(pillar: Pillar, name: str, ceiling: float) -> str:
    height = ceiling if pillar.height is None else pillar.height
    geometry = f"Cylinder {{ height {_num(height)} radius {_num(pillar.radius)} }}"
    return _solid(name, (*pillar.center, height / 2), geometry, ORANGE)


def _solid(
    name: str,
    position: tuple[float, float, float],
    geometry: str,
    color: tuple[float, float, float],
) -> str:
    """A static obstacle: it collides (boundingObject) but has no physics, so it never moves."""
    x, y, z = position
    return (
        "Solid {\n"
        f"  translation {_num(x)} {_num(y)} {_num(z)}\n"
        "  children [\n"
        "    Shape {\n"
        f"      appearance {_appearance(color)}\n"
        f"      geometry {geometry}\n"
        "    }\n"
        "  ]\n"
        f"  name {_string(name)}\n"
        f"  boundingObject {geometry}\n"
        "}\n"
    )


# --- robots -----------------------------------------------------------------


def _referee(course: Course) -> str:
    """The supervisor: knows the sectors, watches the drones, judges the run."""
    args = [
        "--sector-a", _rect_arg(course.sector_a.area),
        "--sector-c", _rect_arg(course.sector_c.area),
        "--ceiling", _num(course.ceiling),
    ]  # fmt: skip
    return (
        "Robot {\n"
        '  name "referee"\n'
        '  controller "referee"\n'
        f"  controllerArgs [ {' '.join(_string(a) for a in args)} ]\n"
        "  supervisor TRUE\n"
        "}\n"
    )


def _drone(drone: Drone, goal: Sector, ir_range: float, controller: str) -> str:
    x, y = drone.xy
    data: dict[str, object] = {"altitude": round(drone.altitude, 4)}
    if drone.leader:
        r = goal.area
        data["goal"] = [r.x_min, r.y_min, r.x_max, r.y_max]
    return (
        f"DEF DRONE_{drone.id} Crazyflie {{\n"
        f"  translation {_num(x)} {_num(y)} {CRAZYFLIE_REST_Z}\n"
        f"  rotation 0 0 1 {_num(drone.yaw)}\n"
        f'  name "drone_{drone.id}"\n'
        f"  controller {_string(controller)}\n"
        f"  customData {_string(json.dumps(data, separators=(',', ':')))}\n"
        "  extensionSlot [\n"
        "    Emitter {\n"
        '      name "ir emitter"\n'
        '      type "infra-red"\n'
        f"      range {_num(ir_range)}\n"
        f"      channel {IR_CHANNEL}\n"
        "    }\n"
        "    Receiver {\n"
        '      name "ir receiver"\n'
        '      type "infra-red"\n'
        f"      channel {IR_CHANNEL}\n"
        "    }\n"
        f"{LEADER_MARKER if drone.leader else ''}"
        "  ]\n"
        "}\n"
    )


# A glowing red ball on top of each leader, to tell them apart in the simulation.
# Visual only: no boundingObject, so no mass, and it blocks neither IR nor sensors.
LEADER_MARKER = (
    "    Pose {\n"
    "      translation 0 0 0.05\n"
    "      children [\n"
    "        Shape {\n"
    "          appearance PBRAppearance { baseColor 1 0.1 0.1 roughness 1 metalness 0"
    " emissiveColor 1 0.1 0.1 emissiveIntensity 2 }\n"
    "          geometry Sphere { radius 0.04 }\n"
    "          castShadows FALSE\n"
    "        }\n"
    "      ]\n"
    "    }\n"
)


# --- values -----------------------------------------------------------------


def _appearance(
    color: tuple[float, float, float], transparency: float = 0.0, glow: float = 0.0
) -> str:
    """`glow` makes the colour emissive, so it stays bright whatever the lighting."""
    rgb = " ".join(_num(c) for c in color)
    extra = f" transparency {_num(transparency)}" if transparency else ""
    if glow:
        extra += f" emissiveColor {rgb} emissiveIntensity {_num(glow)}"
    return f"PBRAppearance {{ baseColor {rgb} roughness 1 metalness 0{extra} }}"


def _rect_arg(r: Rect) -> str:
    return " ".join(_num(v) for v in (r.x_min, r.y_min, r.x_max, r.y_max))


def _num(value: float) -> str:
    """Up to 4 decimals (0.1 mm), without trailing zeros: 2.0 -> 2, 1.2345678 -> 1.2346."""
    text = f"{value:.4f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def _string(text: str) -> str:
    """A VRML string literal: quotes and backslashes escaped."""
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'
