# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Ship lane switching optimization for marine seismic exploration (towed-streamer operations). Two coupled problems:

1. **Local turn geometry** — compute feasible turning paths between parallel vertical survey lines under minimum turning radius R, mandatory Run-out (`L_out`), and Run-in (`L_in`) constraints.
2. **Global route planning** — optimize visit order of many survey lines via spatial partitioning, same-direction block constraints, interleaving, and SA + DP optimization.

## Commands

```bash
# Install dependencies (only matplotlib; rest is stdlib)
pip install -r requirements.txt

# Run the demo (generates lines, runs planner, prints results, produces route map + bar chart)
python main.py
```

No formal test suite exists. Validation is done by running `main.py` and inspecting output plots.

## Architecture

Module dependency graph (strictly layered, no circular imports):

```
common.py              Foundation: Point, State, VerticalLine, EPS, sampling/plotting helpers
  ├── survey.py        Normalize arbitrary parallel lines to vertical-line coordinate system
  ├── dubins_csc.py    W > 2R solver: Dubins CSC (LSL/RSR)
  ├── semicircle.py    W = 2R solver: semicircle turn
  └── three_arc.py     W < 2R solver: fixed symmetric three-arc (LRL/RLR)
        └── planner.py Global orchestrator: spatial partitioning → block refinement →
                       work-unit candidates → interleaving → SA (unit order) + DP (candidate chain) → route assembly
              └── main.py  Entry point, visualization, random-route comparison
```

### Solver pattern

Each geometry module follows the same interface pattern:
1. Low-level solver (e.g. `shortest_dubins_csc`, `solve_w_eq_2r_semicircle`, `solve_w_lt_2r_three_arc`)
2. `solve_fixed_head_tail_*` wrapper — builds EOL/SOL geometric points from `L_out`/`L_in`, then calls the low-level solver
3. `print_*_result` and `plot_*_turn_solution` for output

### Planner pipeline (planner.py)

The planner is **not** "block-internal TSP". It follows this pipeline:

1. **Pre-compute** all O(n²) pairwise turn distances into a global cache keyed by `(i, j, from_side)`
2. **Spatial partitioning** — expert-rule blocks based on max allowable span: `x_max = R(1 + cos(alpha))`
3. **Block refinement** — DP-based splitting so paired blocks have |size_a - size_b| ≤ 1
4. **Work-unit construction** — pair adjacent refined blocks; generate top-k candidate routes per unit via interleaving DP
5. **Global SA** — optimize unit ordering; neighbor operations: swap/reverse/move
6. **DP per fixed order** — choose best candidate per unit (with connector caching)
7. **Assemble** final `PlannerResult`

Key caches: `_build_global_distance_cache` (pairwise distances), connector cache during SA `(prev_unit_id, prev_cand_idx, curr_unit_id, curr_cand_idx)`.

## Geometry Conventions (non-negotiable)

- **AB side** = lower endpoint (`y_start`); **CD side** = upper endpoint (`y_end`)
- Enter from AB → travel upward (+Y); enter from CD → travel downward (−Y)
- **EOL geometric point** = EOL data point + L_out along current heading (turn geometry start)
- **SOL geometric point** = SOL data point − L_in against target heading (turn geometry end)
- Turn geometry is solved between **geometric points**, not data points
- Current connection model: same-side only (AB→AB, CD→CD)
- Same-direction blocks are realized via **interleaving** between paired blocks, not contiguous traversal

## Three Turn Scenarios

| Condition | Model | Path structure |
|-----------|-------|---------------|
| W > 2R | Dubins CSC (LSL vs RSR, pick shortest feasible) | Run out → arc → tangent → arc → Run in |
| W = 2R | Semicircle turn | Run out → pre-straight → semicircle → post-straight → Run in |
| W < 2R | Fixed symmetric three-arc (LRL or RLR) | Run out → pre-straight → three-arc → post-straight → Run in |

Do not collapse these into one generic solver; each has distinct geometric construction.

## Key API

```python
from planner import plan_route, random_route_distance

result = plan_route(
    lines,                    # List[VerticalLine]
    R=2800.0,                 # minimum turning radius
    L_out=3000.0,             # Run-out distance
    L_in=6200.0,              # Run-in distance
    speed=5.0,                # ship speed
    max_heading_angle_deg=0.0,
    block_top_k=12,           # candidates per work unit
    global_sa_restarts=20,
    global_sa_steps=2500,
    debug=False,              # print block/candidate/order info
)
# result: PlannerResult(route, sides, total_distance, turn_details)
```

## When Modifying Code

**Local geometry solvers** — preserve heading continuity, tangent consistency, correct L_out/L_in usage, and W=2R boundary logic. Do not redefine EOL/SOL geometric point semantics.

**planner.py** — preserve: same-direction inside refined blocks, interleaving between paired blocks, same-side connection, SA+DP structure. Do not revert to "block-internal TSP" modeling.

**Visualization** — Run-out (dim gray) and Run-in (dark orange) are semantic overlays, not decorations. Mind `zorder` when adjusting plot layers. Color convention: blue=upward, red=downward, green=turn path.

**survey.py** — handles rotation/translation to normalize arbitrary parallel lines into the vertical-line coordinate system used by the solvers. Changes here affect all downstream modules.

## Documentation

- `AGENTS.md` — additional agent-specific guidance (modification rules, common mistakes)
- `readme.md` — bilingual project documentation with parameter tables and usage examples
- When docs and code disagree, trust the code and update docs

## Naming Conventions

Preserve existing terminology: `AB`/`CD` (sides), `EOL`/`SOL` (line endpoints), `UP`/`DOWN` (directions), `L`/`R` (turn directions), `Run out`/`Run in` (mandatory approach/departure). Keep comments bilingual (Chinese + English) where practical.
