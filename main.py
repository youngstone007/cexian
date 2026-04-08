from dataclasses import dataclass
from math import sqrt, atan2, cos, sin, acos, pi, degrees
from typing import Optional, List, Tuple

import matplotlib.pyplot as plt


# =========================
# 基础数据结构
# =========================

@dataclass
class Point:
    x: float
    y: float


@dataclass
class State:
    p: Point
    heading: float  # 弧度


@dataclass
class VerticalLine:
    """
    南北向测线（平行于Y轴）

    约定：
    - y_start: 采集起点
    - y_end  : 采集终点
    - direction = +1 表示沿 +Y 方向采集
    - direction = -1 表示沿 -Y 方向采集

    要求：
    - 若 direction = +1, 则通常 y_end > y_start
    - 若 direction = -1, 则通常 y_end < y_start
    """
    x: float
    y_start: float
    y_end: float
    direction: int  # +1 or -1

    @property
    def length(self) -> float:
        return abs(self.y_end - self.y_start)

    @property
    def heading(self) -> float:
        return pi / 2 if self.direction > 0 else -pi / 2


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
# W = 2R 独立半圆分支数据结构
# =========================

@dataclass
class SemicircleTurnPath:
    valid: bool = False
    path_type: str = "SEMICIRCLE_W_EQ_2R"

    start_point: Optional[Point] = None      # EOL_geom
    end_point: Optional[Point] = None        # SOL_geom

    semicircle_start: Optional[Point] = None
    semicircle_end: Optional[Point] = None
    center: Optional[Point] = None

    pre_straight_length: float = 0.0
    arc_length: float = 0.0
    post_straight_length: float = 0.0

    total_length: float = 0.0
    total_time: float = 0.0

    turn_direction: str = ""   # 'L' or 'R'


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
# 全局参数
# =========================

EPS = 1e-9
DOT_TOL = 0.995


# =========================
# 基础几何函数
# =========================

def distance(p1: Point, p2: Point) -> float:
    return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def mod2pi(a: float) -> float:
    while a < 0:
        a += 2 * pi
    while a >= 2 * pi:
        a -= 2 * pi
    return a


def left_normal(psi: float) -> Tuple[float, float]:
    return -sin(psi), cos(psi)


def right_normal(psi: float) -> Tuple[float, float]:
    return sin(psi), -cos(psi)


def polar_angle(center: Point, point: Point) -> float:
    return atan2(point.y - center.y, point.x - center.x)


def compute_arc_angle(center: Point, point_from: Point, point_to: Point, turn: str) -> float:
    a1 = polar_angle(center, point_from)
    a2 = polar_angle(center, point_to)

    if turn == 'L':
        return mod2pi(a2 - a1)
    else:
        return mod2pi(a1 - a2)


def turning_circle_center(state: State, R: float, turn: str) -> Point:
    if turn == 'L':
        nx, ny = left_normal(state.heading)
    else:
        nx, ny = right_normal(state.heading)

    return Point(
        x=state.p.x + R * nx,
        y=state.p.y + R * ny
    )


def normalize(vx: float, vy: float) -> Optional[Tuple[float, float]]:
    n = sqrt(vx * vx + vy * vy)
    if n < EPS:
        return None
    return vx / n, vy / n


def point_is_ahead_along_heading(p_from: Point, heading: float, p_to: Point, tol: float = 1e-9) -> bool:
    """
    判断 p_to 是否位于 p_from 沿 heading 的前向半直线上。
    """
    hx = cos(heading)
    hy = sin(heading)
    vx = p_to.x - p_from.x
    vy = p_to.y - p_from.y
    return hx * vx + hy * vy >= -tol


# =========================
# 切向一致性检查
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
# 公切线求解
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
            if debug:
                print(f"    [VALID] {path_type}: total_length={p.total_length:.3f}")
            candidates.append(p)
        else:
            if debug:
                print(f"    [INVALID] {path_type}")

    if not candidates:
        return DubinsPath(valid=False)

    best = min(candidates, key=lambda item: item.total_length)
    if debug:
        print(f"    [BEST] {best.path_type}, total_length={best.total_length:.3f}")
    return best


# =========================
# 测线点构造
# =========================

def point_on_vertical_line(line: VerticalLine, y: float) -> Point:
    return Point(line.x, y)


def build_eol_geom(line1: VerticalLine, eol_data_y: float, L_out: float) -> Tuple[State, Point]:
    """
    EOL数据点 = 第一条测线上的离线数据点
    EOL几何点 = EOL数据点沿测线方向继续走 L_out
             = Run out 终点
             = Dubins 转弯起点
    """
    data_pt = point_on_vertical_line(line1, eol_data_y)
    geom_pt = Point(line1.x, eol_data_y + line1.direction * L_out)
    return State(p=geom_pt, heading=line1.heading), data_pt


def build_sol_geom_fixed_head(line2: VerticalLine, L_in: float) -> Tuple[State, Point]:
    """
    SOL固定在线头（采集起点）
    SOL几何点 = 在正式采集前，沿反方向回退 L_in
             = Run in 起点
             = Dubins 转弯终点
    """
    sol_data_y = line2.y_start
    data_pt = point_on_vertical_line(line2, sol_data_y)
    geom_pt = Point(line2.x, sol_data_y - line2.direction * L_in)
    return State(p=geom_pt, heading=line2.heading), data_pt


# =========================
# 固定线尾/线头求解器
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
    """
    固定：
    - 第一条线必须完整跑到线尾 EOL_data = line1.y_end
    - 第二条线从线头开始上线 SOL_data = line2.y_start

    即：
    - EOL 不搜索
    - SOL 不搜索
    """
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
# 独立处理 W = 2R 的换线情形
# =========================

def solve_w_eq_2r_semicircle(
    start_state: State,
    end_state: State,
    R: float,
    speed: float,
    tol: float = 1e-6,
    debug: bool = False
) -> SemicircleTurnPath:
    """
    独立处理 W = 2R 的临界换线情形。

    路径形式：
        直线 + 半圆 + 直线
    其中前后直线某一段可能为 0。

    这里的 start_state / end_state 分别就是：
    - EOL_geom = Run out 终点
    - SOL_geom = Run in 起点
    """

    x1, y1 = start_state.p.x, start_state.p.y
    x2, y2 = end_state.p.x, end_state.p.y

    W = abs(x2 - x1)
    if abs(W - 2.0 * R) > tol:
        if debug:
            print(f"[W=2R SEMI] invalid: W={W:.6f}, 2R={2.0 * R:.6f}")
        return SemicircleTurnPath(valid=False)

    h1x, h1y = cos(start_state.heading), sin(start_state.heading)
    h2x, h2y = cos(end_state.heading), sin(end_state.heading)
    heading_dot = h1x * h2x + h1y * h2y
    if heading_dot > -0.99:
        if debug:
            print(f"[W=2R SEMI] invalid: headings are not opposite enough, dot={heading_dot:.6f}")
        return SemicircleTurnPath(valid=False)

    candidate_y_stars = [y1, y2]
    feasible_paths: List[SemicircleTurnPath] = []

    for y_star in candidate_y_stars:
        semi_start = Point(x1, y_star)
        semi_end = Point(x2, y_star)
        center = Point((x1 + x2) / 2.0, y_star)

        if not point_is_ahead_along_heading(start_state.p, start_state.heading, semi_start, tol=tol):
            if debug:
                print(f"[W=2R SEMI] reject y*={y_star:.3f}: pre-straight not along start heading")
            continue

        if not point_is_ahead_along_heading(semi_end, end_state.heading, end_state.p, tol=tol):
            if debug:
                print(f"[W=2R SEMI] reject y*={y_star:.3f}: post-straight not along end heading")
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

        if ln is not None:
            dotL = ln[0] * h1x + ln[1] * h1y
        else:
            dotL = -1e9

        if rn is not None:
            dotR = rn[0] * h1x + rn[1] * h1y
        else:
            dotR = -1e9

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

        if debug:
            print(f"[W=2R SEMI] feasible y*={y_star:.3f}")
            print(f"    semi_start=({semi_start.x:.3f}, {semi_start.y:.3f})")
            print(f"    semi_end  =({semi_end.x:.3f}, {semi_end.y:.3f})")
            print(f"    center    =({center.x:.3f}, {center.y:.3f})")
            print(f"    pre={pre_straight:.3f}, arc={arc_length:.3f}, post={post_straight:.3f}, total={total_length:.3f}")
            print(f"    turn={turn_direction}")

    if not feasible_paths:
        if debug:
            print("[W=2R SEMI] no feasible path")
        return SemicircleTurnPath(valid=False)

    best = min(feasible_paths, key=lambda p: p.total_length)
    if debug:
        print(f"[W=2R SEMI] best total_length={best.total_length:.3f}, turn={best.turn_direction}")

    return best


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
    """
    W = 2R 的独立半圆逻辑，且：
    - EOL 固定在线尾
    - SOL 固定在线头
    """
    W = abs(line2.x - line1.x)
    if abs(W - 2.0 * R) > tol:
        if debug:
            print(f"[FIXED SEMI] skip: W={W:.6f}, 2R={2.0 * R:.6f}")
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
# 打印结果
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
    print(f"圆心1       : ({d.center1.x:.3f}, {d.center1.y:.3f})")
    print(f"圆心2       : ({d.center2.x:.3f}, {d.center2.y:.3f})")
    print(f"切点1       : ({d.tangent_point1.x:.3f}, {d.tangent_point1.y:.3f})")
    print(f"切点2       : ({d.tangent_point2.x:.3f}, {d.tangent_point2.y:.3f})")
    print("==================================")


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
# 可视化
# =========================

def sample_arc(center: Point, radius: float, p_start: Point, p_end: Point, turn: str, n: int = 100):
    a1 = polar_angle(center, p_start)
    a2 = polar_angle(center, p_end)

    if turn == 'L':
        delta = mod2pi(a2 - a1)
        angles = [a1 + delta * i / (n - 1) for i in range(n)]
    else:
        delta = mod2pi(a1 - a2)
        angles = [a1 - delta * i / (n - 1) for i in range(n)]

    xs = [center.x + radius * cos(a) for a in angles]
    ys = [center.y + radius * sin(a) for a in angles]
    return xs, ys


def sample_line(p1: Point, p2: Point, n: int = 50):
    xs = [p1.x + (p2.x - p1.x) * i / (n - 1) for i in range(n)]
    ys = [p1.y + (p2.y - p1.y) * i / (n - 1) for i in range(n)]
    return xs, ys


def draw_heading_arrow(ax, state: State, length: float = 150.0, color: str = "black", label: Optional[str] = None):
    dx = length * cos(state.heading)
    dy = length * sin(state.heading)
    ax.arrow(
        state.p.x, state.p.y, dx, dy,
        head_width=0.12 * length,
        head_length=0.18 * length,
        fc=color, ec=color, length_includes_head=True, alpha=0.8
    )
    if label:
        ax.text(state.p.x + dx, state.p.y + dy, label, color=color, fontsize=10)


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

    ax.plot([line1.x, line1.x], [line1.y_start, line1.y_end],
            color="tab:blue", linewidth=3.0, label="Line 1")
    ax.plot([line2.x, line2.x], [line2.y_start, line2.y_end],
            color="tab:green", linewidth=3.0, label="Line 2")

    mid1 = 0.5 * (line1.y_start + line1.y_end)
    ax.arrow(line1.x, mid1, 0, 250 * line1.direction,
             head_width=45, head_length=90, fc="tab:blue", ec="tab:blue",
             length_includes_head=True)

    mid2 = 0.5 * (line2.y_start + line2.y_end)
    ax.arrow(line2.x, mid2, 0, 250 * line2.direction,
             head_width=45, head_length=90, fc="tab:green", ec="tab:green",
             length_includes_head=True)

    eol_data = best.eol_data_point
    sol_data = best.sol_data_point
    eol_geom = best.eol_geom
    sol_geom = best.sol_geom
    d = best.dubins

    ax.scatter([eol_data.x], [eol_data.y], color="blue", s=80, zorder=5)
    ax.text(eol_data.x + 30, eol_data.y, "EOL data", color="blue", fontsize=11)

    ax.scatter([sol_data.x], [sol_data.y], color="green", s=80, zorder=5)
    ax.text(sol_data.x + 30, sol_data.y, "SOL data", color="green", fontsize=11)

    ax.scatter([eol_geom.p.x], [eol_geom.p.y], color="navy", s=90, marker="s", zorder=6)
    ax.text(eol_geom.p.x + 30, eol_geom.p.y, "EOL geom", color="navy", fontsize=11)

    ax.scatter([sol_geom.p.x], [sol_geom.p.y], color="darkgreen", s=90, marker="s", zorder=6)
    ax.text(sol_geom.p.x + 30, sol_geom.p.y, "SOL geom", color="darkgreen", fontsize=11)

    draw_heading_arrow(ax, eol_geom, length=140.0, color="navy", label="hdg1")
    draw_heading_arrow(ax, sol_geom, length=140.0, color="darkgreen", label="hdg2")

    ax.scatter([d.center1.x], [d.center1.y], color="red", s=70, marker="x", zorder=6)
    ax.text(d.center1.x + 25, d.center1.y, "C1", color="red", fontsize=11)

    ax.scatter([d.center2.x], [d.center2.y], color="red", s=70, marker="x", zorder=6)
    ax.text(d.center2.x + 25, d.center2.y, "C2", color="red", fontsize=11)

    ax.scatter([d.tangent_point1.x], [d.tangent_point1.y], color="purple", s=70, zorder=6)
    ax.text(d.tangent_point1.x + 25, d.tangent_point1.y, "T1", color="purple", fontsize=11)

    ax.scatter([d.tangent_point2.x], [d.tangent_point2.y], color="purple", s=70, zorder=6)
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
        circle1 = plt.Circle((d.center1.x, d.center1.y), R, color="red", fill=False, linestyle=":", alpha=0.5, linewidth=1.5)
        circle2 = plt.Circle((d.center2.x, d.center2.y), R, color="red", fill=False, linestyle=":", alpha=0.5, linewidth=1.5)
        ax.add_patch(circle1)
        ax.add_patch(circle2)

    ax.plot([eol_data.x, eol_geom.p.x], [eol_data.y, eol_geom.p.y],
            color="blue", linestyle=":", linewidth=2.5, label="Run out")
    ax.plot([sol_geom.p.x, sol_data.x], [sol_geom.p.y, sol_data.y],
            color="green", linestyle=":", linewidth=2.5, label="Run in")

    title = (
        f"Best Turn Solution  |  Type={d.path_type}  |  "
        f"Total Length={d.total_length:.2f}  |  Total Time={d.total_time:.2f}"
    )
    ax.set_title(title, fontsize=14)
    ax.set_xlabel("X", fontsize=12)
    ax.set_ylabel("Y", fontsize=12)
    ax.legend(loc="best", fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_aspect("equal", adjustable="box")

    xs = [
        line1.x, line2.x,
        eol_data.x, sol_data.x,
        eol_geom.p.x, sol_geom.p.x,
        d.center1.x, d.center2.x,
        d.tangent_point1.x, d.tangent_point2.x
    ]
    ys = [
        line1.y_start, line1.y_end,
        line2.y_start, line2.y_end,
        eol_data.y, sol_data.y,
        eol_geom.p.y, sol_geom.p.y,
        d.center1.y, d.center2.y,
        d.tangent_point1.y, d.tangent_point2.y
    ]
    margin_x = max(120.0, 0.18 * (max(xs) - min(xs) + 1.0))
    margin_y = max(120.0, 0.18 * (max(ys) - min(ys) + 1.0))

    ax.set_xlim(min(xs) - margin_x, max(xs) + margin_x)
    ax.set_ylim(min(ys) - margin_y, max(ys) + margin_y)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"图片已保存到: {save_path}")

    plt.show()


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

    ax.plot([line1.x, line1.x], [line1.y_start, line1.y_end],
            color="tab:blue", linewidth=3.0, label="Line 1")
    ax.plot([line2.x, line2.x], [line2.y_start, line2.y_end],
            color="tab:green", linewidth=3.0, label="Line 2")

    mid1 = 0.5 * (line1.y_start + line1.y_end)
    ax.arrow(line1.x, mid1, 0, 250 * line1.direction,
             head_width=45, head_length=90, fc="tab:blue", ec="tab:blue",
             length_includes_head=True)

    mid2 = 0.5 * (line2.y_start + line2.y_end)
    ax.arrow(line2.x, mid2, 0, 250 * line2.direction,
             head_width=45, head_length=90, fc="tab:green", ec="tab:green",
             length_includes_head=True)

    eol_data = best.eol_data_point
    sol_data = best.sol_data_point
    eol_geom = best.eol_geom
    sol_geom = best.sol_geom
    s = best.semi

    ax.scatter([eol_data.x], [eol_data.y], color="blue", s=80, zorder=5)
    ax.text(eol_data.x + 30, eol_data.y, "EOL data", color="blue", fontsize=11)

    ax.scatter([sol_data.x], [sol_data.y], color="green", s=80, zorder=5)
    ax.text(sol_data.x + 30, sol_data.y, "SOL data", color="green", fontsize=11)

    ax.scatter([eol_geom.p.x], [eol_geom.p.y], color="navy", s=90, marker="s", zorder=6)
    ax.text(eol_geom.p.x + 30, eol_geom.p.y, "EOL geom", color="navy", fontsize=11)

    ax.scatter([sol_geom.p.x], [sol_geom.p.y], color="darkgreen", s=90, marker="s", zorder=6)
    ax.text(sol_geom.p.x + 30, sol_geom.p.y, "SOL geom", color="darkgreen", fontsize=11)

    draw_heading_arrow(ax, eol_geom, length=140.0, color="navy", label="hdg1")
    draw_heading_arrow(ax, sol_geom, length=140.0, color="darkgreen", label="hdg2")

    ax.scatter([s.semicircle_start.x], [s.semicircle_start.y], color="purple", s=80, zorder=7)
    ax.text(s.semicircle_start.x + 25, s.semicircle_start.y, "Semi start", color="purple", fontsize=11)

    ax.scatter([s.semicircle_end.x], [s.semicircle_end.y], color="purple", s=80, zorder=7)
    ax.text(s.semicircle_end.x + 25, s.semicircle_end.y, "Semi end", color="purple", fontsize=11)

    ax.scatter([s.center.x], [s.center.y], color="red", s=80, marker="x", zorder=7)
    ax.text(s.center.x + 25, s.center.y, "C", color="red", fontsize=11)

    ax.plot([eol_data.x, eol_geom.p.x], [eol_data.y, eol_geom.p.y],
            color="blue", linestyle=":", linewidth=2.5, label="Run out")
    ax.plot([sol_geom.p.x, sol_data.x], [sol_geom.p.y, sol_data.y],
            color="green", linestyle=":", linewidth=2.5, label="Run in")

    if s.pre_straight_length > EPS:
        px, py = sample_line(s.start_point, s.semicircle_start, n=50)
        ax.plot(px, py, color="black", linewidth=3.0, linestyle="--", label="Pre-straight")

    arc_x, arc_y = sample_arc(
        s.center, R, s.semicircle_start, s.semicircle_end, s.turn_direction, n=240
    )
    ax.plot(arc_x, arc_y, color="orange", linewidth=3.5, label="Semicircle")

    if s.post_straight_length > EPS:
        qx, qy = sample_line(s.semicircle_end, s.end_point, n=50)
        ax.plot(qx, qy, color="gray", linewidth=3.0, linestyle="--", label="Post-straight")

    circle = plt.Circle((s.center.x, s.center.y), R, color="red", fill=False, linestyle=":", alpha=0.5, linewidth=1.5)
    ax.add_patch(circle)

    title = (
        f"W=2R Semicircle Turn  |  "
        f"Total Length={s.total_length:.2f}  |  Total Time={s.total_time:.2f}"
    )
    ax.set_title(title, fontsize=14)
    ax.set_xlabel("X", fontsize=12)
    ax.set_ylabel("Y", fontsize=12)
    ax.legend(loc="best", fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_aspect("equal", adjustable="box")

    xs = [
        line1.x, line2.x,
        eol_data.x, sol_data.x,
        eol_geom.p.x, sol_geom.p.x,
        s.semicircle_start.x, s.semicircle_end.x, s.center.x
    ]
    ys = [
        line1.y_start, line1.y_end,
        line2.y_start, line2.y_end,
        eol_data.y, sol_data.y,
        eol_geom.p.y, sol_geom.p.y,
        s.semicircle_start.y, s.semicircle_end.y, s.center.y
    ]

    margin_x = max(120.0, 0.18 * (max(xs) - min(xs) + 1.0))
    margin_y = max(120.0, 0.18 * (max(ys) - min(ys) + 1.0))
    ax.set_xlim(min(xs) - margin_x, max(xs) + margin_x)
    ax.set_ylim(min(ys) - margin_y, max(ys) + margin_y)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"图片已保存到: {save_path}")

    plt.show()


# =========================
# 示例主程序
# =========================

if __name__ == "__main__":
    # 第一条线：从下往上采
    line1 = VerticalLine(
        x=0.0,
        y_start=0.0,
        y_end=4000.0,
        direction=+1
    )

    # 第二条线：从上往下采
    line2 = VerticalLine(
        x=1200.0,
        y_start=3000.0,
        y_end=500.0,
        direction=-1
    )

    L_out = 300.0
    L_in = 300.0
    R = 250.0
    speed = 5.0

    W = abs(line2.x - line1.x)

    print(f"当前测线间距 W = {W:.3f}, 2R = {2.0 * R:.3f}")

    if abs(W - 2.0 * R) < 1e-6:
        best_semi = solve_fixed_head_tail_w_eq_2r_semicircle_turn(
            line1=line1,
            line2=line2,
            L_out=L_out,
            L_in=L_in,
            R=R,
            speed=speed,
            debug=True
        )

        print_semicircle_result(best_semi)

        plot_semicircle_turn_solution(
            line1,
            line2,
            best_semi,
            R,
            figsize=(12, 14),
            dpi=300,
            save_path="turn_solution_w_eq_2r.png"
        )
    else:
        best = solve_fixed_head_tail_turn_between_vertical_lines(
            line1=line1,
            line2=line2,
            L_out=L_out,
            L_in=L_in,
            R=R,
            speed=speed,
            debug=True
        )

        print_result(best)

        plot_turn_solution(
            line1,
            line2,
            best,
            R,
            show_circles=True,
            figsize=(12, 14),
            dpi=300,
            save_path="turn_solution_csc.png"
        )