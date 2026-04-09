from dataclasses import dataclass
from math import sqrt, cos, sin, acos, pi, degrees
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
# W < 2R 场景数据结构
# =========================

@dataclass
class ThreeArcTurnPath:
    valid: bool = False
    path_type: str = "THREE_ARC_W_LT_2R"

    start_point: Optional[Point] = None
    end_point: Optional[Point] = None

    turn_start_point: Optional[Point] = None
    turn_end_point: Optional[Point] = None

    center1: Optional[Point] = None
    center2: Optional[Point] = None
    center3: Optional[Point] = None

    join12: Optional[Point] = None
    join23: Optional[Point] = None

    arc_a_angle: float = 0.0
    arc_b_angle: float = 0.0
    arc_c_angle: float = 0.0

    arc_a_length: float = 0.0
    arc_b_length: float = 0.0
    arc_c_length: float = 0.0

    pre_straight_length: float = 0.0
    post_straight_length: float = 0.0

    total_length: float = 0.0
    total_time: float = 0.0

    turn_sequence: str = ""


@dataclass
class ThreeArcTurnCandidate:
    eol_data_y: float
    sol_data_y: float
    eol_data_point: Point
    sol_data_point: Point
    eol_geom: State
    sol_geom: State
    three_arc: ThreeArcTurnPath
    total_cost: float


# =========================
# W < 2R 固定模板三圆弧
# =========================

def solve_w_lt_2r_three_arc(
    start_state: State,
    end_state: State,
    R: float,
    speed: float,
    tol: float = 1e-6,
    debug: bool = False
) -> ThreeArcTurnPath:
    """
    W < 2R 固定对称三圆弧模板

    工程规则：
    - 左线 -> 右线 : LRL
    - 右线 -> 左线 : RLR
    - 起转高度 y_turn = max(start.y, end.y)
    - 小弧 a / b 完全对称
    - 中间大圆圆心位于两测线中点的垂线方向
    - 模板角满足：
        cos(Q) = (W/2 + R) / (2R)
    """
    x1, y1 = start_state.p.x, start_state.p.y
    x2, y2 = end_state.p.x, end_state.p.y

    W = abs(x2 - x1)
    if W >= 2.0 * R - tol:
        if debug:
            print(f"[W<2R THREE-ARC] invalid: W={W:.6f}, 2R={2.0 * R:.6f}")
        return ThreeArcTurnPath(valid=False)

    h1x, h1y = cos(start_state.heading), sin(start_state.heading)
    h2x, h2y = cos(end_state.heading), sin(end_state.heading)
    heading_dot = h1x * h2x + h1y * h2y
    if heading_dot > -0.99:
        if debug:
            print(f"[W<2R THREE-ARC] invalid: headings are not opposite enough, dot={heading_dot:.6f}")
        return ThreeArcTurnPath(valid=False)

    y_turn = max(y1, y2)
    turn_start = Point(x1, y_turn)
    turn_end = Point(x2, y_turn)

    if not point_is_ahead_along_heading(start_state.p, start_state.heading, turn_start, tol=tol):
        if debug:
            print("[W<2R THREE-ARC] invalid: pre-straight not along start heading")
        return ThreeArcTurnPath(valid=False)

    if not point_is_ahead_along_heading(turn_end, end_state.heading, end_state.p, tol=tol):
        if debug:
            print("[W<2R THREE-ARC] invalid: post-straight not along end heading")
        return ThreeArcTurnPath(valid=False)

    pre_straight = distance(start_state.p, turn_start)
    post_straight = distance(turn_end, end_state.p)

    if x2 > x1:
        sequence = "LRL"
    else:
        sequence = "RLR"

    cosQ = (W / 2.0 + R) / (2.0 * R)
    cosQ = max(-1.0, min(1.0, cosQ))
    Q = acos(cosQ)

    h = sqrt(max(0.0, (2.0 * R) ** 2 - (R + W / 2.0) ** 2))
    xm = 0.5 * (x1 + x2)

    if x2 > x1:
        c1 = Point(x1 - R, y_turn)
        c3 = Point(x2 + R, y_turn)
        c2 = Point(xm, y_turn + h)
        turn1, turn2, turn3 = 'L', 'R', 'L'
    else:
        c1 = Point(x1 + R, y_turn)
        c3 = Point(x2 - R, y_turn)
        c2 = Point(xm, y_turn + h)
        turn1, turn2, turn3 = 'R', 'L', 'R'

    v12 = normalize(c2.x - c1.x, c2.y - c1.y)
    v32 = normalize(c2.x - c3.x, c2.y - c3.y)

    if v12 is None or v32 is None:
        if debug:
            print("[W<2R THREE-ARC] invalid: normalize failed")
        return ThreeArcTurnPath(valid=False)

    join12 = Point(c1.x + R * v12[0], c1.y + R * v12[1])
    join23 = Point(c3.x + R * v32[0], c3.y + R * v32[1])

    arc_a_angle = Q
    arc_b_angle = Q
    arc_c_angle = 2.0 * pi - 2.0 * Q

    arc_a_length = R * arc_a_angle
    arc_b_length = R * arc_b_angle
    arc_c_length = R * arc_c_angle

    total_length = pre_straight + arc_a_length + arc_c_length + arc_b_length + post_straight
    total_time = total_length / speed

    if debug:
        print(f"[W<2R THREE-ARC] feasible")
        print(f"    sequence={sequence}")
        print(f"    W={W:.3f}, R={R:.3f}")
        print(f"    y_turn={y_turn:.3f}")
        print(f"    Q={degrees(Q):.3f} deg")
        print(f"    c1=({c1.x:.3f}, {c1.y:.3f})")
        print(f"    c2=({c2.x:.3f}, {c2.y:.3f})")
        print(f"    c3=({c3.x:.3f}, {c3.y:.3f})")
        print(f"    join12=({join12.x:.3f}, {join12.y:.3f})")
        print(f"    join23=({join23.x:.3f}, {join23.y:.3f})")
        print(f"    pre={pre_straight:.3f}, post={post_straight:.3f}")
        print(f"    a={degrees(arc_a_angle):.3f} deg")
        print(f"    c={degrees(arc_c_angle):.3f} deg")
        print(f"    b={degrees(arc_b_angle):.3f} deg")
        print(f"    total={total_length:.3f}")

    return ThreeArcTurnPath(
        valid=True,
        path_type="THREE_ARC_W_LT_2R",
        start_point=start_state.p,
        end_point=end_state.p,
        turn_start_point=turn_start,
        turn_end_point=turn_end,
        center1=c1,
        center2=c2,
        center3=c3,
        join12=join12,
        join23=join23,
        arc_a_angle=arc_a_angle,
        arc_b_angle=arc_b_angle,
        arc_c_angle=arc_c_angle,
        arc_a_length=arc_a_length,
        arc_b_length=arc_b_length,
        arc_c_length=arc_c_length,
        pre_straight_length=pre_straight,
        post_straight_length=post_straight,
        total_length=total_length,
        total_time=total_time,
        turn_sequence=sequence
    )


def solve_fixed_head_tail_w_lt_2r_three_arc_turn(
    line1: VerticalLine,
    line2: VerticalLine,
    L_out: float,
    L_in: float,
    R: float,
    speed: float,
    tol: float = 1e-6,
    debug: bool = False
) -> Optional[ThreeArcTurnCandidate]:
    W = abs(line2.x - line1.x)
    if W >= 2.0 * R - tol:
        return None

    eol_data_y = line1.y_end
    sol_data_y = line2.y_start

    eol_geom, eol_data_point = build_eol_geom(line1, eol_data_y, L_out)
    sol_geom, sol_data_point = build_sol_geom_fixed_head(line2, L_in)

    three_arc = solve_w_lt_2r_three_arc(
        start_state=eol_geom,
        end_state=sol_geom,
        R=R,
        speed=speed,
        tol=tol,
        debug=debug
    )
    if not three_arc.valid:
        return None

    return ThreeArcTurnCandidate(
        eol_data_y=eol_data_y,
        sol_data_y=sol_data_y,
        eol_data_point=eol_data_point,
        sol_data_point=sol_data_point,
        eol_geom=eol_geom,
        sol_geom=sol_geom,
        three_arc=three_arc,
        total_cost=three_arc.total_time
    )


# =========================
# 打印结果 (W < 2R)
# =========================

def print_three_arc_result(best: Optional[ThreeArcTurnCandidate]):
    if best is None:
        print("未找到可行的 W < 2R 三圆弧换线路径。")
        return

    t = best.three_arc
    print("====== W < 2R 三圆弧换线结果 ======")
    print(f"EOL 数据点: ({best.eol_data_point.x:.3f}, {best.eol_data_point.y:.3f}) [固定在线尾]")
    print(f"SOL 数据点: ({best.sol_data_point.x:.3f}, {best.sol_data_point.y:.3f}) [固定在线头]")
    print(f"EOL 几何点: ({best.eol_geom.p.x:.3f}, {best.eol_geom.p.y:.3f})  heading={degrees(best.eol_geom.heading):.2f} deg")
    print(f"SOL 几何点: ({best.sol_geom.p.x:.3f}, {best.sol_geom.p.y:.3f})  heading={degrees(best.sol_geom.heading):.2f} deg")
    print(f"路径类型  : {t.path_type}")
    print(f"转弯序列  : {t.turn_sequence}")
    print(f"三圆弧起点: ({t.turn_start_point.x:.3f}, {t.turn_start_point.y:.3f})")
    print(f"三圆弧终点: ({t.turn_end_point.x:.3f}, {t.turn_end_point.y:.3f})")
    print(f"圆心1      : ({t.center1.x:.3f}, {t.center1.y:.3f})")
    print(f"圆心2      : ({t.center2.x:.3f}, {t.center2.y:.3f})")
    print(f"圆心3      : ({t.center3.x:.3f}, {t.center3.y:.3f})")
    print(f"连接点12   : ({t.join12.x:.3f}, {t.join12.y:.3f})")
    print(f"连接点23   : ({t.join23.x:.3f}, {t.join23.y:.3f})")
    print(f"a角        : {degrees(t.arc_a_angle):.3f} deg")
    print(f"c角        : {degrees(t.arc_c_angle):.3f} deg")
    print(f"b角        : {degrees(t.arc_b_angle):.3f} deg")
    print(f"前直线长度 : {t.pre_straight_length:.3f}")
    print(f"a弧长      : {t.arc_a_length:.3f}")
    print(f"c弧长      : {t.arc_c_length:.3f}")
    print(f"b弧长      : {t.arc_b_length:.3f}")
    print(f"后直线长度 : {t.post_straight_length:.3f}")
    print(f"总长度     : {t.total_length:.3f}")
    print(f"总时间     : {t.total_time:.3f}")
    print("==================================")


# =========================
# 可视化 (W < 2R)
# =========================

def plot_three_arc_turn_solution(
    line1: VerticalLine,
    line2: VerticalLine,
    best: Optional[ThreeArcTurnCandidate],
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
    t = best.three_arc

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

    ax.scatter([t.turn_start_point.x], [t.turn_start_point.y], color="purple", s=70, zorder=7)
    ax.scatter([t.turn_end_point.x], [t.turn_end_point.y], color="purple", s=70, zorder=7)
    ax.text(t.turn_start_point.x + 25, t.turn_start_point.y, "Turn start", color="purple", fontsize=11)
    ax.text(t.turn_end_point.x + 25, t.turn_end_point.y, "Turn end", color="purple", fontsize=11)

    ax.scatter([t.center1.x], [t.center1.y], color="red", s=70, marker="x", zorder=7)
    ax.scatter([t.center2.x], [t.center2.y], color="red", s=70, marker="x", zorder=7)
    ax.scatter([t.center3.x], [t.center3.y], color="red", s=70, marker="x", zorder=7)

    ax.text(t.center1.x + 25, t.center1.y, "C1", color="red", fontsize=11)
    ax.text(t.center2.x + 25, t.center2.y, "C2", color="red", fontsize=11)
    ax.text(t.center3.x + 25, t.center3.y, "C3", color="red", fontsize=11)

    ax.scatter([t.join12.x], [t.join12.y], color="black", s=60, zorder=7)
    ax.scatter([t.join23.x], [t.join23.y], color="black", s=60, zorder=7)
    ax.text(t.join12.x + 20, t.join12.y, "J12", color="black", fontsize=10)
    ax.text(t.join23.x + 20, t.join23.y, "J23", color="black", fontsize=10)

    ax.plot([eol_data.x, eol_geom.p.x], [eol_data.y, eol_geom.p.y], color="blue", linestyle=":", linewidth=2.5, label="Run out")
    ax.plot([sol_geom.p.x, sol_data.x], [sol_geom.p.y, sol_data.y], color="green", linestyle=":", linewidth=2.5, label="Run in")

    if t.pre_straight_length > EPS:
        px, py = sample_line(t.start_point, t.turn_start_point, n=50)
        ax.plot(px, py, color="black", linewidth=3.0, linestyle="--", label="Pre-straight")

    if t.turn_sequence == "LRL":
        turn1, turn2, turn3 = 'L', 'R', 'L'
    elif t.turn_sequence == "RLR":
        turn1, turn2, turn3 = 'R', 'L', 'R'
    else:
        raise ValueError(f"Unknown turn sequence: {t.turn_sequence}")

    arc1_x, arc1_y = sample_arc(t.center1, R, t.turn_start_point, t.join12, turn1, n=180)
    arc2_x, arc2_y = sample_arc(t.center2, R, t.join12, t.join23, turn2, n=260)
    arc3_x, arc3_y = sample_arc(t.center3, R, t.join23, t.turn_end_point, turn3, n=180)

    ax.plot(arc1_x, arc1_y, color="orange", linewidth=3.5, label="Arc a")
    ax.plot(arc2_x, arc2_y, color="magenta", linewidth=3.5, label="Arc c")
    ax.plot(arc3_x, arc3_y, color="brown", linewidth=3.5, label="Arc b")

    if t.post_straight_length > EPS:
        qx, qy = sample_line(t.turn_end_point, t.end_point, n=50)
        ax.plot(qx, qy, color="gray", linewidth=3.0, linestyle="--", label="Post-straight")

    ax.add_patch(plt.Circle((t.center1.x, t.center1.y), R, color="red", fill=False, linestyle=":", alpha=0.45, linewidth=1.5))
    ax.add_patch(plt.Circle((t.center2.x, t.center2.y), R, color="red", fill=False, linestyle=":", alpha=0.45, linewidth=1.5))
    ax.add_patch(plt.Circle((t.center3.x, t.center3.y), R, color="red", fill=False, linestyle=":", alpha=0.45, linewidth=1.5))

    ax.set_title(f"W<2R Three-Arc Turn  |  Type={t.turn_sequence}  |  Total Length={t.total_length:.2f}  |  Total Time={t.total_time:.2f}", fontsize=14)
    ax.set_xlabel("X", fontsize=12)
    ax.set_ylabel("Y", fontsize=12)
    ax.legend(loc="best", fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_aspect("equal", adjustable="box")

    xs = [
        line1.x, line2.x,
        eol_data.x, sol_data.x,
        eol_geom.p.x, sol_geom.p.x,
        t.turn_start_point.x, t.turn_end_point.x,
        t.center1.x, t.center2.x, t.center3.x,
        t.join12.x, t.join23.x
    ]
    ys = [
        line1.y_start, line1.y_end,
        line2.y_start, line2.y_end,
        eol_data.y, sol_data.y,
        eol_geom.p.y, sol_geom.p.y,
        t.turn_start_point.y, t.turn_end_point.y,
        t.center1.y, t.center2.y, t.center3.y,
        t.join12.y, t.join23.y
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
