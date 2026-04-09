from dataclasses import dataclass
from math import cos, sin, pi, degrees
from typing import Optional, List, Tuple

import matplotlib.pyplot as plt

from common import (
    Point, State, VerticalLine,
    EPS,
    distance, normalize, point_is_ahead_along_heading,
    build_eol_geom, build_sol_geom_fixed_head,
    sample_arc, sample_line, draw_heading_arrow
)


# =========================
# W = 2R 场景数据结构
# =========================

@dataclass
class SemicircleTurnPath:
    valid: bool = False
    path_type: str = "SEMICIRCLE_W_EQ_2R"

    start_point: Optional[Point] = None
    end_point: Optional[Point] = None

    semicircle_start: Optional[Point] = None
    semicircle_end: Optional[Point] = None
    center: Optional[Point] = None

    pre_straight_length: float = 0.0
    arc_length: float = 0.0
    post_straight_length: float = 0.0

    total_length: float = 0.0
    total_time: float = 0.0

    turn_direction: str = ""


@dataclass
class SemicircleTurnCandidate:
    eol_data_y: float
    sol_data_y: float
    eol_data_point: Point
    sol_data_point: Point
    eol_geom: State
    sol_geom: State
    semi: SemicircleTurnPath
    total_cost: float


# =========================
# W = 2R 半圆
# =========================

def solve_w_eq_2r_semicircle(
    start_state: State,
    end_state: State,
    R: float,
    speed: float,
    tol: float = 1e-6,
    debug: bool = False
) -> SemicircleTurnPath:
    x1, y1 = start_state.p.x, start_state.p.y
    x2, y2 = end_state.p.x, end_state.p.y

    W = abs(x2 - x1)
    if abs(W - 2.0 * R) > tol:
        return SemicircleTurnPath(valid=False)

    h1x, h1y = cos(start_state.heading), sin(start_state.heading)
    h2x, h2y = cos(end_state.heading), sin(end_state.heading)
    heading_dot = h1x * h2x + h1y * h2y
    if heading_dot > -0.99:
        return SemicircleTurnPath(valid=False)

    candidate_y_stars = [y1, y2]
    feasible_paths: List[SemicircleTurnPath] = []

    for y_star in candidate_y_stars:
        semi_start = Point(x1, y_star)
        semi_end = Point(x2, y_star)
        center = Point((x1 + x2) / 2.0, y_star)

        if not point_is_ahead_along_heading(start_state.p, start_state.heading, semi_start, tol=tol):
            continue

        if not point_is_ahead_along_heading(semi_end, end_state.heading, end_state.p, tol=tol):
            continue

        pre_straight = distance(start_state.p, semi_start)
        post_straight = distance(semi_end, end_state.p)
        arc_length = pi * R
        total_length = pre_straight + arc_length + post_straight
        total_time = total_length / speed

        rx = semi_start.x - center.x
        ry = semi_start.y - center.y

        ltx, lty = -ry, rx
        ln = normalize(ltx, lty)

        rtx, rty = ry, -rx
        rn = normalize(rtx, rty)

        dotL = ln[0] * h1x + ln[1] * h1y if ln is not None else -1e9
        dotR = rn[0] * h1x + rn[1] * h1y if rn is not None else -1e9
        turn_direction = 'L' if dotL >= dotR else 'R'

        feasible_paths.append(
            SemicircleTurnPath(
                valid=True,
                path_type="SEMICIRCLE_W_EQ_2R",
                start_point=start_state.p,
                end_point=end_state.p,
                semicircle_start=semi_start,
                semicircle_end=semi_end,
                center=center,
                pre_straight_length=pre_straight,
                arc_length=arc_length,
                post_straight_length=post_straight,
                total_length=total_length,
                total_time=total_time,
                turn_direction=turn_direction
            )
        )

    if not feasible_paths:
        return SemicircleTurnPath(valid=False)

    return min(feasible_paths, key=lambda p: p.total_length)


def solve_fixed_head_tail_w_eq_2r_semicircle_turn(
    line1: VerticalLine,
    line2: VerticalLine,
    L_out: float,
    L_in: float,
    R: float,
    speed: float,
    tol: float = 1e-6,
    debug: bool = False
) -> Optional[SemicircleTurnCandidate]:
    W = abs(line2.x - line1.x)
    if abs(W - 2.0 * R) > tol:
        return None

    eol_data_y = line1.y_end
    sol_data_y = line2.y_start

    eol_geom, eol_data_point = build_eol_geom(line1, eol_data_y, L_out)
    sol_geom, sol_data_point = build_sol_geom_fixed_head(line2, L_in)

    semi = solve_w_eq_2r_semicircle(
        start_state=eol_geom,
        end_state=sol_geom,
        R=R,
        speed=speed,
        tol=tol,
        debug=debug
    )
    if not semi.valid:
        return None

    return SemicircleTurnCandidate(
        eol_data_y=eol_data_y,
        sol_data_y=sol_data_y,
        eol_data_point=eol_data_point,
        sol_data_point=sol_data_point,
        eol_geom=eol_geom,
        sol_geom=sol_geom,
        semi=semi,
        total_cost=semi.total_time
    )


# =========================
# 打印结果 (W = 2R)
# =========================

def print_semicircle_result(best: Optional[SemicircleTurnCandidate]):
    if best is None:
        print("未找到可行的 W = 2R 半圆换线路径。")
        return

    s = best.semi
    print("====== W = 2R 半圆换线最优结果 ======")
    print(f"EOL 数据点: ({best.eol_data_point.x:.3f}, {best.eol_data_point.y:.3f}) [固定在线尾]")
    print(f"SOL 数据点: ({best.sol_data_point.x:.3f}, {best.sol_data_point.y:.3f}) [固定在线头]")
    print(f"EOL 几何点: ({best.eol_geom.p.x:.3f}, {best.eol_geom.p.y:.3f})  heading={degrees(best.eol_geom.heading):.2f} deg")
    print(f"SOL 几何点: ({best.sol_geom.p.x:.3f}, {best.sol_geom.p.y:.3f})  heading={degrees(best.sol_geom.heading):.2f} deg")
    print(f"路径类型  : {s.path_type}")
    print(f"转向方向  : {s.turn_direction}")
    print(f"半圆起点  : ({s.semicircle_start.x:.3f}, {s.semicircle_start.y:.3f})")
    print(f"半圆终点  : ({s.semicircle_end.x:.3f}, {s.semicircle_end.y:.3f})")
    print(f"圆心      : ({s.center.x:.3f}, {s.center.y:.3f})")
    print(f"前直线长度: {s.pre_straight_length:.3f}")
    print(f"半圆长度  : {s.arc_length:.3f}")
    print(f"后直线长度: {s.post_straight_length:.3f}")
    print(f"总长度    : {s.total_length:.3f}")
    print(f"总时间    : {s.total_time:.3f}")
    print("====================================")


# =========================
# 可视化 (W = 2R)
# =========================

def plot_semicircle_turn_solution(
    line1: VerticalLine,
    line2: VerticalLine,
    best: Optional[SemicircleTurnCandidate],
    R: float,
    figsize=(10, 12),
    dpi=200,
    save_path: Optional[str] = None
):
    if best is None:
        print("没有可视化结果：best is None")
        return

    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

    ax.plot([line1.x, line1.x], [line1.y_start, line1.y_end], color="tab:blue", linewidth=3.0, label="Line 1")
    ax.plot([line2.x, line2.x], [line2.y_start, line2.y_end], color="tab:green", linewidth=3.0, label="Line 2")

    mid1 = 0.5 * (line1.y_start + line1.y_end)
    ax.arrow(line1.x, mid1, 0, 250 * line1.direction, head_width=45, head_length=90,
             fc="tab:blue", ec="tab:blue", length_includes_head=True)
    mid2 = 0.5 * (line2.y_start + line2.y_end)
    ax.arrow(line2.x, mid2, 0, 250 * line2.direction, head_width=45, head_length=90,
             fc="tab:green", ec="tab:green", length_includes_head=True)

    eol_data = best.eol_data_point
    sol_data = best.sol_data_point
    eol_geom = best.eol_geom
    sol_geom = best.sol_geom
    s = best.semi

    ax.scatter([eol_data.x], [eol_data.y], color="blue", s=80, zorder=5)
    ax.scatter([sol_data.x], [sol_data.y], color="green", s=80, zorder=5)
    ax.scatter([eol_geom.p.x], [eol_geom.p.y], color="navy", s=90, marker="s", zorder=6)
    ax.scatter([sol_geom.p.x], [sol_geom.p.y], color="darkgreen", s=90, marker="s", zorder=6)

    ax.text(eol_data.x + 30, eol_data.y, "EOL data", color="blue", fontsize=11)
    ax.text(sol_data.x + 30, sol_data.y, "SOL data", color="green", fontsize=11)
    ax.text(eol_geom.p.x + 30, eol_geom.p.y, "EOL geom", color="navy", fontsize=11)
    ax.text(sol_geom.p.x + 30, sol_geom.p.y, "SOL geom", color="darkgreen", fontsize=11)

    draw_heading_arrow(ax, eol_geom, length=140.0, color="navy", label="hdg1")
    draw_heading_arrow(ax, sol_geom, length=140.0, color="darkgreen", label="hdg2")

    ax.scatter([s.semicircle_start.x], [s.semicircle_start.y], color="purple", s=80, zorder=7)
    ax.scatter([s.semicircle_end.x], [s.semicircle_end.y], color="purple", s=80, zorder=7)
    ax.scatter([s.center.x], [s.center.y], color="red", s=80, marker="x", zorder=7)

    ax.text(s.semicircle_start.x + 25, s.semicircle_start.y, "Semi start", color="purple", fontsize=11)
    ax.text(s.semicircle_end.x + 25, s.semicircle_end.y, "Semi end", color="purple", fontsize=11)
    ax.text(s.center.x + 25, s.center.y, "C", color="red", fontsize=11)

    ax.plot([eol_data.x, eol_geom.p.x], [eol_data.y, eol_geom.p.y], color="blue", linestyle=":", linewidth=2.5, label="Run out")
    ax.plot([sol_geom.p.x, sol_data.x], [sol_geom.p.y, sol_data.y], color="green", linestyle=":", linewidth=2.5, label="Run in")

    if s.pre_straight_length > EPS:
        px, py = sample_line(s.start_point, s.semicircle_start, n=50)
        ax.plot(px, py, color="black", linewidth=3.0, linestyle="--", label="Pre-straight")

    arc_x, arc_y = sample_arc(s.center, R, s.semicircle_start, s.semicircle_end, s.turn_direction, n=240)
    ax.plot(arc_x, arc_y, color="orange", linewidth=3.5, label="Semicircle")

    if s.post_straight_length > EPS:
        qx, qy = sample_line(s.semicircle_end, s.end_point, n=50)
        ax.plot(qx, qy, color="gray", linewidth=3.0, linestyle="--", label="Post-straight")

    ax.add_patch(plt.Circle((s.center.x, s.center.y), R, color="red", fill=False, linestyle=":", alpha=0.5, linewidth=1.5))

    ax.set_title(f"W=2R Semicircle Turn  |  Total Length={s.total_length:.2f}  |  Total Time={s.total_time:.2f}", fontsize=14)
    ax.set_xlabel("X", fontsize=12)
    ax.set_ylabel("Y", fontsize=12)
    ax.legend(loc="best", fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_aspect("equal", adjustable="box")

    xs = [line1.x, line2.x, eol_data.x, sol_data.x, eol_geom.p.x, sol_geom.p.x, s.semicircle_start.x, s.semicircle_end.x, s.center.x]
    ys = [line1.y_start, line1.y_end, line2.y_start, line2.y_end, eol_data.y, sol_data.y, eol_geom.p.y, sol_geom.p.y, s.semicircle_start.y, s.semicircle_end.y, s.center.y]

    margin_x = max(120.0, 0.18 * (max(xs) - min(xs) + 1.0))
    margin_y = max(120.0, 0.18 * (max(ys) - min(ys) + 1.0))
    ax.set_xlim(min(xs) - margin_x, max(xs) + margin_x)
    ax.set_ylim(min(ys) - margin_y, max(ys) + margin_y)

    plt.tight_layout()
    if save_path is not None:
        plt.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"图片已保存到: {save_path}")
    plt.show()
