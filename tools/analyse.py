"""The report's numbers and figures, from the sweeps' logs (design log, 2026-10-28 and -29).

    uv run tools/analyse.py evaluation    # held-out maps, n 20, j 2: model x plan
    uv run tools/analyse.py sweep         # dead_end, plan on: n x j per model
    uv run tools/analyse.py split         # do two leaders plan round the same arm?
    uv run tools/analyse.py trajectory logs/replays/dead_end_hybrid_plan_n20_j2_seed4.csv \
        --map dead_end -n 20 -j 2 --seed 4 --title "hybrid"
    uv run tools/analyse.py maps          # the three experiment maps, with a spawn
    uv run tools/analyse.py all           # all but trajectory

A run succeeds when 90% of its drones are in C within its map's limit: the
go time plus twice what the leaders alone need along their planned path,
A's centre to C's centre at v_flock, in whole seconds. Runs of the same
setting on the same seed share a world, so comparisons pair them by seed:
exact McNemar test on success (a binomial test on the seeds where the two
differ), Wilcoxon signed-rank on the closest two drones came. p values are
uncorrected; with 10 seeds the smallest McNemar p is 0.002. Tables print
here; figures go to report/figures as PDF.
"""

import argparse
import csv
import itertools
import math
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # files only, no window
matplotlib.rcParams["figure.constrained_layout.use"] = True  # room for suptitles and labels

import matplotlib.pyplot as plt  # noqa: E402 - after choosing the backend
from scipy.stats import binomtest, fisher_exact, wilcoxon  # noqa: E402
from sweep import ROOT  # noqa: E402

from boids import BoidParams  # noqa: E402
from maps import load_map  # noqa: E402
from maps.course import Course, build_course  # noqa: E402
from maps.model import Rect  # noqa: E402
from planner import Planner, PlanParams  # noqa: E402
from worldgen.preview import FOLLOWER, LEADER, draw_course  # noqa: E402
from worldgen.spawn import place_swarm  # noqa: E402

Row = dict[str, str]
Runs = dict[tuple[int, int, int], Row]  # (n, j, seed) -> the sweep's row

MODELS = ["hybrid", "couzin", "reynolds"]
HELD_OUT = ["dead_end", "l_turn"]  # no model was tuned on them
SEEDS = range(10)
FIGURES = ROOT / "report" / "figures"
COLOURS = {"hybrid": "#2b5fad", "couzin": "#d9a800", "reynolds": "#2e9e44"}


def main() -> int:
    parser = argparse.ArgumentParser(description="The report's analysis.", allow_abbrev=False)
    parser.add_argument(
        "what", choices=["evaluation", "sweep", "split", "trajectory", "maps", "all"]
    )
    parser.add_argument("log", nargs="?", type=Path, help="trajectory: a referee log")
    parser.add_argument("--map", default="dead_end", help="trajectory: the run's map")
    parser.add_argument("-n", type=int, default=20)
    parser.add_argument("-j", type=int, default=2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--title", default="")
    parser.add_argument("--out", type=Path, default=FIGURES)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.what in ("evaluation", "all"):
        evaluation(args.out)
    if args.what in ("sweep", "all"):
        sweep(args.out)
    if args.what in ("split", "all"):
        split()
    if args.what in ("maps", "all"):
        maps(args.out)
    if args.what == "trajectory":
        if args.log is None:
            parser.error("trajectory needs a referee log")
        trajectory(args.log, args.map, args.n, args.j, args.seed, args.title, args.out)
    return 0


# --- the measure -------------------------------------------------------------------


def course(map_name: str) -> Course:
    return build_course(load_map(ROOT / "maps" / f"{map_name}.toml"))


def planner(c: Course) -> Planner:
    def box(r: Rect) -> tuple[float, float, float, float]:
        return (r.x_min, r.y_min, r.x_max, r.y_max)

    return Planner(
        [box(w.rect) for w in c.boundary],
        [box(w.rect) for w in c.walls],
        [(*p.center, p.radius) for p in c.pillars],
        PlanParams(),
    )


def centre(r: Rect) -> tuple[float, float]:
    return ((r.x_min + r.x_max) / 2, (r.y_min + r.y_max) / 2)


def limit(map_name: str) -> float:
    """The go time plus twice the leaders' own time along their plan, whole seconds."""
    c = course(map_name)
    path = planner(c).plan(centre(c.sector_a.area), centre(c.sector_c.area))
    assert path is not None, f"{map_name}: no path from A to C"
    length = sum(math.dist(a, b) for a, b in itertools.pairwise(path))
    rules = BoidParams()
    return math.ceil(rules.go_time + 2 * length / rules.v_flock)


def load(map_name: str, model: str, plan: bool) -> Runs:
    file = ROOT / "logs" / f"sweep_{map_name}_{model}{'_plan' if plan else ''}.csv"
    with open(file, newline="") as f:
        return {(int(r["n"]), int(r["j"]), int(r["seed"])): r for r in csv.DictReader(f)}


def ok(r: Row, by: float) -> bool:
    return bool(r["arrived"]) and float(r["arrived"]) <= by


def cell(runs: Runs, n: int, j: int) -> list[Row]:
    return [runs[n, j, s] for s in SEEDS if (n, j, s) in runs]


def mcnemar(a: list[Row], b: list[Row], by: float) -> tuple[int, int, float]:
    """Seeds only b got there, seeds only a did, and the exact McNemar p."""
    to_b = sum(ok(y, by) and not ok(x, by) for x, y in zip(a, b, strict=True))
    to_a = sum(ok(x, by) and not ok(y, by) for x, y in zip(a, b, strict=True))
    return to_b, to_a, float(binomtest(to_b, to_b + to_a).pvalue) if to_a + to_b else 1.0


def closest_p(a: list[Row], b: list[Row]) -> float:
    d = [float(y["closest"]) - float(x["closest"]) for x, y in zip(a, b, strict=True)]
    return 1.0 if not any(d) else float(wilcoxon(d).pvalue)  # pyright: ignore[reportAttributeAccessIssue]


def marks(rows: list[Row], by: float) -> str:
    return "".join("+" if ok(r, by) else "." for r in rows)


def median(xs: list[float]) -> str:
    return f"{statistics.median(xs):.0f}" if xs else "-"


# --- held-out evaluation -------------------------------------------------------------


def evaluation(out: Path) -> None:
    """n 20, j 2, seeds 0-9 on the maps nobody tuned on: every model, plan off and on."""
    print("=== evaluation: n 20, j 2, seeds 0-9 (+ = 90% in C within the limit)")
    data = {
        (m, mo, p): cell(load(m, mo, p), 20, 2)
        for m in HELD_OUT
        for mo in MODELS
        for p in (False, True)
    }
    limits = {m: limit(m) for m in HELD_OUT}
    for m in HELD_OUT:
        print(
            f"\n{m}, limit {limits[m]:.0f} s: success, median time to 90%, closest pair "
            "median / min, crashes"
        )
        for mo, p in itertools.product(MODELS, (False, True)):
            rows = data[m, mo, p]
            times = [float(r["arrived"]) for r in rows if ok(r, limits[m])]
            closest = [float(r["closest"]) for r in rows]
            crashes = sum(bool(r["crashed"]) for r in rows)
            print(
                f"  {mo:8s} plan {'on ' if p else 'off'}: {len(times):2d}/{len(rows)} "
                f"{marks(rows, limits[m])}  t90 {median(times):>3} s  closest "
                f"{statistics.median(closest):.2f} / {min(closest):.2f} m  crashes {crashes}"
            )
    print("\nplan on against off (seeds to on, to off, McNemar p; closest pair Wilcoxon p)")
    for m, mo in itertools.product(HELD_OUT, MODELS):
        a, b = data[m, mo, False], data[m, mo, True]
        to_b, to_a, p = mcnemar(a, b, limits[m])
        print(f"  {m:8s} {mo:8s}: {to_b} / {to_a}, p = {p:.3g}; closest p = {closest_p(a, b):.3g}")
    for plan in (False, True):
        print(f"\nmodels, plan {'on' if plan else 'off'} (seeds to the 2nd, to the 1st, p)")
        for m, (x, y) in itertools.product(HELD_OUT, itertools.combinations(MODELS, 2)):
            to_b, to_a, p = mcnemar(data[m, x, plan], data[m, y, plan], limits[m])
            print(f"  {m:8s} {x} vs {y}: {to_b} / {to_a}, p = {p:.3g}")

    fig, axes = plt.subplots(1, 2, figsize=(9, 3.4), sharey=True)
    for ax, m in zip(axes, HELD_OUT, strict=True):
        boxes = [
            [float(r["closest"]) for r in data[m, mo, p]]
            for mo, p in itertools.product(MODELS, (False, True))
        ]
        _boxes(
            ax,
            boxes,
            [
                f"{mo}\n{'plan' if p else 'no plan'}"
                for mo, p in itertools.product(MODELS, (False, True))
            ],
            [COLOURS[mo] for mo in MODELS for _ in (0, 1)],
        )
        ax.axhline(BoidParams().stop_drone, color="grey", linestyle=":", linewidth=1)
        ax.set_title(f"{m}, n = 20, j = 2")
    axes[0].set_ylabel("closest two drones came (m)")
    _save(fig, out / "evaluation_closest.pdf")


# --- the n x j sweep ---------------------------------------------------------------


NS = [10, 20, 30]
JS = [0, 1, 2, 4]


def sweep(out: Path) -> None:
    """dead_end, plan on: success by drones and leaders, per model, and what it costs."""
    by = limit("dead_end")
    data = {mo: load("dead_end", mo, True) for mo in MODELS}
    print(f"\n=== sweep: dead_end, plan on, limit {by:.0f} s, success out of 10 seeds")
    for mo in MODELS:
        print(f"  {mo}")
        for n in NS:
            print(
                f"    n {n}: "
                + "  ".join(f"j {j} {sum(ok(r, by) for r in cell(data[mo], n, j)):2d}" for j in JS)
            )

    print("\npooled over n (30 runs paired by n and seed): leaders within a model")
    pooled = {(mo, j): [r for n in NS for r in cell(data[mo], n, j)] for mo in MODELS for j in JS}
    for mo in MODELS:
        print(
            f"  {mo}: " + ", ".join(f"j {j} {sum(ok(r, by) for r in pooled[mo, j])}/30" for j in JS)
        )
        for x, y in [(1, 2), (2, 4), (1, 4)]:
            to_b, to_a, p = mcnemar(pooled[mo, x], pooled[mo, y], by)
            print(f"    j {x} vs {y}: {to_b} to j {y}, {to_a} to j {x}, p = {p:.3g}")
    print("\npooled over n: models at each j (seeds to the 2nd, to the 1st, p)")
    for j, (x, y) in itertools.product(JS[1:], itertools.combinations(MODELS, 2)):
        to_b, to_a, p = mcnemar(pooled[x, j], pooled[y, j], by)
        print(f"  j {j} {x} vs {y}: {to_b} / {to_a}, p = {p:.3g}")
    print("\nfailures: median drones in C at the limit (the leaders alone, or a near miss?)")
    for mo in MODELS:
        failed = {
            (n, j): [float(r["in_c"]) for r in cell(data[mo], n, j) if not ok(r, by)]
            for n in NS
            for j in JS[1:]
        }
        print(f"  {mo}: " + "  ".join(f"n{n} j{j} {median(v)}" for (n, j), v in failed.items()))
    print(
        "\ncrashes: "
        + ", ".join(
            f"{mo} {sum(bool(r['crashed']) for r in data[mo].values() if int(r['seed']) in SEEDS)}"
            for mo in MODELS
        )
    )

    fig, axes = plt.subplots(1, 3, figsize=(10, 3.2), sharey=True)
    for ax, mo in zip(axes, MODELS, strict=True):
        grid = [[sum(ok(r, by) for r in cell(data[mo], n, j)) for j in JS] for n in NS]
        ax.imshow(grid, cmap="Greens", vmin=0, vmax=10, origin="lower")
        for row, col in itertools.product(range(len(NS)), range(len(JS))):
            k = grid[row][col]
            ax.text(col, row, f"{k}", ha="center", va="center", color="white" if k > 6 else "black")
        ax.set_xticks(range(len(JS)), [str(j) for j in JS])
        ax.set_yticks(range(len(NS)), [str(n) for n in NS])
        ax.set_xlabel("leaders j")
        ax.set_title(mo)
    axes[0].set_ylabel("drones n")
    fig.suptitle(f"dead_end, plan on: runs with 90% in C within {by:.0f} s, of 10")
    _save(fig, out / "sweep_success.pdf")

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.4))
    positions = [(mo, j) for mo in MODELS for j in JS[1:]]
    _boxes(
        axes[0],
        [[float(r["arrived"]) for r in pooled[mo, j] if ok(r, by)] for mo, j in positions],
        [f"j {j}" for _, j in positions],
        [COLOURS[mo] for mo, _ in positions],
    )
    axes[0].set_ylabel("time to 90% in C (s), successes only")
    _boxes(
        axes[1],
        [[float(r["closest"]) for r in pooled[mo, j]] for mo, j in positions],
        [f"j {j}" for _, j in positions],
        [COLOURS[mo] for mo, _ in positions],
    )
    axes[1].axhline(BoidParams().stop_drone, color="grey", linestyle=":", linewidth=1)
    axes[1].set_ylabel("closest two drones came (m)")
    for ax in axes:
        for k, mo in enumerate(MODELS):
            ax.text(2 + 3 * k, 1.02, mo, transform=ax.get_xaxis_transform(), ha="center")
    _save(fig, out / "sweep_cost.pdf")


# --- two leaders, two arms -----------------------------------------------------------


def split() -> None:
    """Which way round does each leader plan? The flock fails when two disagree (2026-10-29).

    A leader plans at go time from where it hovers, within a metre of its spawn,
    and A* is deterministic, so its plan from the spawn says which way it goes:
    the side of the straight line from A to C its path bulges to.
    """
    by = limit("dead_end")
    c = course("dead_end")
    p = planner(c)
    goal = centre(c.sector_c.area)
    data = {mo: load("dead_end", mo, True) for mo in MODELS}
    print("\n=== split: dead_end, plan on; leaders on the minority side of the line A-C")
    tally: dict[tuple[str, int, int], list[bool]] = {}
    for n, j, seed in itertools.product(NS, [2, 4], SEEDS):
        sides = [_side(p, d.xy, goal) for d in place_swarm(c, n=n, j=j, seed=seed) if d.leader]
        minority = min(sides.count(1), sides.count(-1))
        for mo in MODELS:
            tally.setdefault((mo, j, minority), []).append(ok(data[mo][n, j, seed], by))
    for mo, j in itertools.product(MODELS, [2, 4]):
        cells = {k: tally[mo, j, k] for k in range(j // 2 + 1) if (mo, j, k) in tally}
        line = ", ".join(f"{k} apart {sum(v)}/{len(v)}" for k, v in cells.items())
        if j == 2:
            together, apart = cells[0], cells[1]
            table = [
                [sum(together), len(together) - sum(together)],
                [sum(apart), len(apart) - sum(apart)],
            ]
            line += f"; Fisher exact p = {fisher_exact(table).pvalue:.3g}"  # pyright: ignore[reportAttributeAccessIssue]
        print(f"  {mo:8s} j {j}: {line}")


def _side(p: Planner, start: tuple[float, float], goal: tuple[float, float]) -> int:
    path = p.plan(start, goal)
    assert path is not None
    (ax, ay), (bx, by) = start, goal

    def offset(q: tuple[float, float]) -> float:  # signed, left of A -> C positive
        return ((bx - ax) * (q[1] - ay) - (by - ay) * (q[0] - ax)) / math.dist(start, goal)

    return 1 if offset(max(path, key=lambda q: abs(offset(q)))) > 0 else -1


# --- the maps --------------------------------------------------------------------------


def maps(out: Path) -> None:
    """The tuning map and the two held-out ones, each with a 20-drone spawn (seed 0) and
    the path planned from A's centre to C's (the one the time limit is measured on)."""
    names = ["l_hall", "l_turn", "dead_end"]
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.2), width_ratios=[1, 1, 1.6])
    for ax, name in zip(axes, names, strict=True):
        c = course(name)
        draw_course(ax, c)
        path = planner(c).plan(centre(c.sector_a.area), centre(c.sector_c.area))
        assert path is not None
        ax.plot([q[0] for q in path], [q[1] for q in path], color="black", linewidth=1,
                linestyle="--", label="planned path")  # fmt: skip
        swarm = place_swarm(c, n=20, j=2, seed=0)
        for leader, colour, marker, size in ((False, FOLLOWER, "o", 6), (True, LEADER, "*", 40)):
            xy = [d.xy for d in swarm if d.leader == leader]
            ax.scatter([p[0] for p in xy], [p[1] for p in xy], s=size, c=colour, marker=marker)
        ax.set_title(name)
    axes[0].legend(loc="upper left", fontsize="x-small", framealpha=0.9)
    _save(fig, out / "maps.pdf")


# --- one run from above --------------------------------------------------------------


def trajectory(log: Path, map_name: str, n: int, j: int, seed: int, title: str, out: Path) -> None:
    """A referee log drawn on its map: each leader's track and the followers' centre."""
    c = course(map_name)
    leaders = {d.id for d in place_swarm(c, n=n, j=j, seed=seed) if d.leader}
    tracks: dict[int, list[tuple[float, float, float]]] = {}
    with open(log, newline="") as f:
        for r in csv.DictReader(f):
            tracks.setdefault(int(r["id"]), []).append(
                (float(r["t"]), float(r["x"]), float(r["y"]))
            )
    by = limit(map_name)
    followers = [t for i, t in tracks.items() if i not in leaders]
    steps = min(len(t) for t in followers)
    flock = [
        (
            followers[0][k][0],
            statistics.mean(t[k][1] for t in followers),
            statistics.mean(t[k][2] for t in followers),
        )
        for k in range(steps)
    ]

    fig = plt.figure(figsize=(8, 8 * c.floor.depth / c.floor.width + 0.8))
    ax = fig.add_subplot()
    draw_course(ax, c)
    ax.plot(
        [q[1] for q in flock if q[0] <= by],
        [q[2] for q in flock if q[0] <= by],
        color=FOLLOWER,
        linewidth=2,
        label="followers' centre",
    )
    for k, i in enumerate(sorted(leaders)):
        track = [q for q in tracks[i] if q[0] <= by]
        ax.plot(
            [q[1] for q in track],
            [q[2] for q in track],
            color=LEADER,
            linewidth=1.2,
            linestyle="--",
            label="leaders" if k == 0 else None,
        )
    every = 10.0  # s between time marks on the followers' centre
    marked, last = 0, (math.inf, math.inf)
    for q in flock:
        if q[0] <= by and q[0] // every > marked:
            marked = int(q[0] // every)
            if math.dist(last, q[1:]) < 1.0:  # a flock at rest: one mark is enough
                continue
            last = q[1:]
            ax.annotate(
                f"{q[0]:.0f}",
                (q[1], q[2]),
                fontsize=7,
                color=FOLLOWER,
                xytext=(3, 3),
                textcoords="offset points",
            )
            ax.plot(q[1], q[2], "o", color=FOLLOWER, markersize=3)
    ax.set_title(f"{title} {map_name}, n = {n}, j = {j}, seed {seed}, to {by:.0f} s".strip())
    ax.legend(loc="lower right", fontsize="small", framealpha=0.9)
    _save(fig, out / f"trajectory_{log.stem}.pdf")


# --- figures -----------------------------------------------------------------------


def _boxes(ax, data: list[list[float]], labels: list[str], colours: list[str]) -> None:
    """Boxplots with every run drawn on top: with 10 to 30 runs, the points matter."""
    shown = [d if d else [math.nan] for d in data]
    parts = ax.boxplot(shown, widths=0.6, patch_artist=True, showfliers=False)
    for box, colour in zip(parts["boxes"], colours, strict=True):
        box.set(facecolor=colour, alpha=0.35)
    for k, d in enumerate(data, 1):
        ax.scatter([k] * len(d), d, s=8, color="black", alpha=0.6, zorder=3)
    ax.set_xticks(range(1, len(labels) + 1), labels, fontsize=7)
    ax.grid(True, axis="y", linewidth=0.3, alpha=0.5)


def _save(fig, file: Path) -> None:
    fig.savefig(file)
    plt.close(fig)
    print(f"-> {file.relative_to(ROOT)}")


if __name__ == "__main__":
    raise SystemExit(main())
