from dataclasses import dataclass
from math import sqrt, atan2, cos, sin, acos, pi, degrees
from typing import Optional, List, Tuple

import matplotlib.pyplot as plt

from common import (
    Point, State, VerticalLine,
    EPS, DOT_TOL,
    distance, mod2pi, normalize,
    compute_arc_angle, turning_circle_center, point_is_ahead_along_heading,
    build_eol_geom, build_sol_geom_fixed_head,
    sample_arc, sample_line, draw_heading_arrow
)


# =========================
# W > 2R 场景数据结构
# =========================

@dataclass
class TangentSolution:
    tangent_point1: Point
    tangent_point2: Point


@dataclass
class DubinsPath:
    path_type: str = ""
    valid: bool = False
    arc1_angle: float = 0.0
    arc1_length: float = 0.0
    straight_length: float = 0.0
    arc2_angle: float = 0.0
    arc2_length: float = 0.0
    total_length: float = 0.0
    total_time: float = 0.0
    center1: Optional[Point] = None
    center2: Optional[Point] = None
    tangent_point1: Optional[Point] = None
    tangent_point2: Optional[Point] = None


@dataclass
class TurnCandidate:
    eol_data_y: float
    sol_data_y: float
    eol_data_point: Point
    sol_data_point: Point
    eol_geom: State
    sol_geom: State
    dubins: DubinsPath
    total_cost: float


# =========================
# CSC 切向一致性检查
# =========================

def tangent_direction_on_circle(center: Point, point: Point, turn: str) -> Tuple[float, float]:
    rx = point.x - center.x
    ry = point.y - center.y
    r = normalize(rx, ry)
    if r is None:
        return 0.0, 0.0
    rx, ry = r

    if turn == 'L':
        tx, ty = -ry, rx
    else:
        tx, ty = ry, -rx

    t = normalize(tx, ty)
    return t if t is not None else (0.0, 0.0)


def tangent_direction_is_consistent(
    c1: Point, t1: Point,
    c2: Point, t2: Point,
    turn1: str, turn2: str,
    debug: bool = False
) -> bool:
    s = normalize(t2.x - t1.x, t2.y - t1.y)
    if s is None:
        return False
    sx, sy = s

    v1x, v1y = tangent_direction_on_circle(c1, t1, turn1)
    v2x, v2y = tangent_direction_on_circle(c2, t2, turn2)

    dot1 = v1x * sx + v1y * sy
    dot2 = v2x * sx + v2y * sy

    if debug:
        print(f"      tangent consistency: dot1={dot1:.6f}, dot2={dot2:.6f}")

    return dot1 > DOT_TOL and dot2 > DOT_TOL


# =========================
# CSC 公切线求解
# =========================

def solve_outer_tangent(c1: Point, c2: Point, R: float, turn: str, debug: bool = False) -> Optional[TangentSolution]:
    dx = c2.x - c1.x
    dy = c2.y - c1.y
    D = sqrt(dx * dx + dy * dy)
    if D < EPS:
        return None

    ux = dx / D
    uy = dy / D
    perp = (-uy, ux)

    feasible = []

    for sign in (+1.0, -1.0):
        t1 = Point(c1.x + sign * R * perp[0], c1.y + sign * R * perp[1])
        t2 = Point(c2.x + sign * R * perp[0], c2.y + sign * R * perp[1])

        ok = tangent_direction_is_consistent(c1, t1, c2, t2, turn, turn, debug=False)
        if debug:
            print(f"      outer tangent sign={sign:+.0f}, ok={ok}, "
                  f"T1=({t1.x:.2f},{t1.y:.2f}), T2=({t2.x:.2f},{t2.y:.2f})")

        if ok:
            feasible.append((t1, t2))

    if not feasible:
        return None

    t1, t2 = min(feasible, key=lambda pair: distance(pair[0], pair[1]))
    return TangentSolution(tangent_point1=t1, tangent_point2=t2)


def construct_inner_tangent_points(
    c1: Point, c2: Point, R: float,
    theta: float, turn1: str, turn2: str
) -> Optional[Tuple[Point, Point]]:
    dx, dy = cos(theta), sin(theta)

    left_n = (-dy, dx)
    right_n = (dy, -dx)

    if turn1 == 'L':
        r1 = right_n
    else:
        r1 = left_n

    if turn2 == 'L':
        r2 = right_n
    else:
        r2 = left_n

    t1 = Point(c1.x + R * r1[0], c1.y + R * r1[1])
    t2 = Point(c2.x + R * r2[0], c2.y + R * r2[1])

    s = normalize(t2.x - t1.x, t2.y - t1.y)
    if s is None:
        return None
    sx, sy = s

    if sx * dx + sy * dy < DOT_TOL:
        return None

    return t1, t2


def solve_inner_tangent(c1: Point, c2: Point, R: float, turn1: str, turn2: str, debug: bool = False) -> Optional[TangentSolution]:
    dx = c2.x - c1.x
    dy = c2.y - c1.y
    D = sqrt(dx * dx + dy * dy)

    if D < 2 * R - EPS:
        if debug:
            print(f"      inner tangent impossible: D={D:.3f} < 2R={2*R:.3f}")
        return None

    phi = atan2(dy, dx)
    alpha = acos(max(-1.0, min(1.0, 2 * R / D)))

    candidate_dirs = [phi + alpha, phi - alpha]
    feasible = []

    for theta in candidate_dirs:
        pair = construct_inner_tangent_points(c1, c2, R, theta, turn1, turn2)
        if pair is None:
            if debug:
                print(f"      inner theta={degrees(theta):.2f} deg -> construct failed")
            continue

        t1, t2 = pair
        ok = tangent_direction_is_consistent(c1, t1, c2, t2, turn1, turn2, debug=False)

        if debug:
            print(f"      inner theta={degrees(theta):.2f} deg, ok={ok}, "
                  f"T1=({t1.x:.2f},{t1.y:.2f}), T2=({t2.x:.2f},{t2.y:.2f})")

        if ok:
            feasible.append((t1, t2))

    if not feasible:
        return None

    t1, t2 = min(feasible, key=lambda pair: distance(pair[0], pair[1]))
    return TangentSolution(tangent_point1=t1, tangent_point2=t2)


# =========================
# Dubins CSC
# =========================

def solve_one_csc_path(
    start_state: State,
    end_state: State,
    R: float,
    path_type: str,
    speed: float,
    debug: bool = False
) -> DubinsPath:
    turn1 = path_type[0]
    turn2 = path_type[2]

    c1 = turning_circle_center(start_state, R, turn1)
    c2 = turning_circle_center(end_state, R, turn2)

    if debug:
        print(f"    solve {path_type}:")
        print(f"      start=({start_state.p.x:.2f},{start_state.p.y:.2f}), hdg={degrees(start_state.heading):.2f}")
        print(f"      end  =({end_state.p.x:.2f},{end_state.p.y:.2f}), hdg={degrees(end_state.heading):.2f}")
        print(f"      c1=({c1.x:.2f},{c1.y:.2f}), c2=({c2.x:.2f},{c2.y:.2f})")

    if turn1 == turn2:
        tangent_solution = solve_outer_tangent(c1, c2, R, turn1, debug=debug)
    else:
        tangent_solution = solve_inner_tangent(c1, c2, R, turn1, turn2, debug=debug)

    if tangent_solution is None:
        if debug:
            print("      tangent solution = None")
        return DubinsPath(valid=False)

    t1 = tangent_solution.tangent_point1
    t2 = tangent_solution.tangent_point2

    if not tangent_direction_is_consistent(c1, t1, c2, t2, turn1, turn2, debug=debug):
        if debug:
            print("      final tangent consistency failed")
        return DubinsPath(valid=False)

    arc1_angle = compute_arc_angle(c1, start_state.p, t1, turn1)
    arc2_angle = compute_arc_angle(c2, t2, end_state.p, turn2)

    arc1_length = R * arc1_angle
    straight_length = distance(t1, t2)
    arc2_length = R * arc2_angle

    total_length = arc1_length + straight_length + arc2_length
    total_time = total_length / speed

    if debug:
        print(f"      arc1={degrees(arc1_angle):.3f} deg, straight={straight_length:.3f}, "
              f"arc2={degrees(arc2_angle):.3f} deg, total={total_length:.3f}")

    return DubinsPath(
        path_type=path_type,
        valid=True,
        arc1_angle=arc1_angle,
        arc1_length=arc1_length,
        straight_length=straight_length,
        arc2_angle=arc2_angle,
        arc2_length=arc2_length,
        total_length=total_length,
        total_time=total_time,
        center1=c1,
        center2=c2,
        tangent_point1=t1,
        tangent_point2=t2
    )


def shortest_dubins_csc(
    start_state: State,
    end_state: State,
    R: float,
    speed: float,
    debug: bool = False
) -> DubinsPath:
    candidates = []

    for path_type in ["LSL", "RSR"]:
        p = solve_one_csc_path(start_state, end_state, R, path_type, speed, debug=debug)
        if p.valid:
            candidates.append(p)

    if not candidates:
        return DubinsPath(valid=False)

    return min(candidates, key=lambda item: item.total_length)


# =========================
# 固定线尾/线头求解器 (W > 2R)
# =========================

def solve_fixed_head_tail_turn_between_vertical_lines(
    line1: VerticalLine,
    line2: VerticalLine,
    L_out: float,
    L_in: float,
    R: float,
    speed: float,
    debug: bool = False
) -> Optional[TurnCandidate]:
    eol_data_y = line1.y_end
    sol_data_y = line2.y_start

    eol_geom, eol_data_point = build_eol_geom(line1, eol_data_y, L_out)
    sol_geom, sol_data_point = build_sol_geom_fixed_head(line2, L_in)

    dubins = shortest_dubins_csc(eol_geom, sol_geom, R, speed, debug=debug)
    if not dubins.valid:
        return None

    return TurnCandidate(
        eol_data_y=eol_data_y,
        sol_data_y=sol_data_y,
        eol_data_point=eol_data_point,
        sol_data_point=sol_data_point,
        eol_geom=eol_geom,
        sol_geom=sol_geom,
        dubins=dubins,
        total_cost=dubins.total_time
    )


# =========================
# 打印结果 (W > 2R)
# =========================

def print_result(best: Optional[TurnCandidate]):
    if best is None:
        print("未找到可行换线路径。")
        return

    d = best.dubins
    print("========== 最优换线结果 ==========")
    print(f"EOL 数据点: ({best.eol_data_point.x:.3f}, {best.eol_data_point.y:.3f}) [固定在线尾]")
    print(f"SOL 数据点: ({best.sol_data_point.x:.3f}, {best.sol_data_point.y:.3f}) [固定在线头]")
    print(f"EOL 几何点: ({best.eol_geom.p.x:.3f}, {best.eol_geom.p.y:.3f})  heading={degrees(best.eol_geom.heading):.2f} deg")
    print(f"SOL 几何点: ({best.sol_geom.p.x:.3f}, {best.sol_geom.p.y:.3f})  heading={degrees(best.sol_geom.heading):.2f} deg")
    print(f"路径类型: {d.path_type}")
    print(f"第一圆弧角度: {degrees(d.arc1_angle):.3f} deg")
    print(f"第一圆弧长度: {d.arc1_length:.3f}")
    print(f"直线段长度 : {d.straight_length:.3f}")
    print(f"第二圆弧角度: {degrees(d.arc2_angle):.3f} deg")
    print(f"第二圆弧长度: {d.arc2_length:.3f}")
    print(f"总长度     : {d.total_length:.3f}")
    print(f"总时间     : {d.total_time:.3f}")
    print(f"圆心1      : ({d.center1.x:.3f}, {d.center1.y:.3f})")
    print(f"圆心2      : ({d.center2.x:.3f}, {d.center2.y:.3f})")
    print(f"切点1      : ({d.tangent_point1.x:.3f}, {d.tangent_point1.y:.3f})")
    print(f"切点2      : ({d.tangent_point2.x:.3f}, {d.tangent_point2.y:.3f})")
    print("==================================")


# =========================
# 可视化 (W > 2R)
# =========================

def plot_turn_solution(
    line1: VerticalLine,
    line2: VerticalLine,
    best: Optional[TurnCandidate],
    R: float,
    show_circles: bool = True,
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
    d = best.dubins

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

    ax.scatter([d.center1.x], [d.center1.y], color="red", s=70, marker="x", zorder=6)
    ax.scatter([d.center2.x], [d.center2.y], color="red", s=70, marker="x", zorder=6)
    ax.scatter([d.tangent_point1.x], [d.tangent_point1.y], color="purple", s=70, zorder=6)
    ax.scatter([d.tangent_point2.x], [d.tangent_point2.y], color="purple", s=70, zorder=6)

    ax.text(d.center1.x + 25, d.center1.y, "C1", color="red", fontsize=11)
    ax.text(d.center2.x + 25, d.center2.y, "C2", color="red", fontsize=11)
    ax.text(d.tangent_point1.x + 25, d.tangent_point1.y, "T1", color="purple", fontsize=11)
    ax.text(d.tangent_point2.x + 25, d.tangent_point2.y, "T2", color="purple", fontsize=11)

    turn1 = d.path_type[0]
    turn2 = d.path_type[2]

    arc1_x, arc1_y = sample_arc(d.center1, R, eol_geom.p, d.tangent_point1, turn1, n=240)
    line_x, line_y = sample_line(d.tangent_point1, d.tangent_point2, n=120)
    arc2_x, arc2_y = sample_arc(d.center2, R, d.tangent_point2, sol_geom.p, turn2, n=240)

    ax.plot(arc1_x, arc1_y, color="orange", linewidth=3.5, label="Arc 1")
    ax.plot(line_x, line_y, color="black", linewidth=3.0, linestyle="--", label="Straight")
    ax.plot(arc2_x, arc2_y, color="magenta", linewidth=3.5, label="Arc 2")

    if show_circles:
        ax.add_patch(plt.Circle((d.center1.x, d.center1.y), R, color="red", fill=False, linestyle=":", alpha=0.5, linewidth=1.5))
        ax.add_patch(plt.Circle((d.center2.x, d.center2.y), R, color="red", fill=False, linestyle=":", alpha=0.5, linewidth=1.5))

    ax.plot([eol_data.x, eol_geom.p.x], [eol_data.y, eol_geom.p.y], color="blue", linestyle=":", linewidth=2.5, label="Run out")
    ax.plot([sol_geom.p.x, sol_data.x], [sol_geom.p.y, sol_data.y], color="green", linestyle=":", linewidth=2.5, label="Run in")

    ax.set_title(f"Best Turn Solution  |  Type={d.path_type}  |  Total Length={d.total_length:.2f}  |  Total Time={d.total_time:.2f}", fontsize=14)
    ax.set_xlabel("X", fontsize=12)
    ax.set_ylabel("Y", fontsize=12)
    ax.legend(loc="best", fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_aspect("equal", adjustable="box")

    xs = [line1.x, line2.x, eol_data.x, sol_data.x, eol_geom.p.x, sol_geom.p.x, d.center1.x, d.center2.x, d.tangent_point1.x, d.tangent_point2.x]
    ys = [line1.y_start, line1.y_end, line2.y_start, line2.y_end, eol_data.y, sol_data.y, eol_geom.p.y, sol_geom.p.y, d.center1.y, d.center2.y, d.tangent_point1.y, d.tangent_point2.y]
    margin_x = max(120.0, 0.18 * (max(xs) - min(xs) + 1.0))
    margin_y = max(120.0, 0.18 * (max(ys) - min(ys) + 1.0))
    ax.set_xlim(min(xs) - margin_x, max(xs) + margin_x)
    ax.set_ylim(min(ys) - margin_y, max(ys) + margin_y)

    plt.tight_layout()
    if save_path is not None:
        plt.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"图片已保存到: {save_path}")
    plt.show()
