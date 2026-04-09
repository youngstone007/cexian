from dataclasses import dataclass
from math import sqrt, atan2, cos, sin, acos, pi, degrees
from typing import Optional, List, Tuple


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
    - direction 可自动由 y_end - y_start 推断，无需手动指定
    """
    x: float
    y_start: float
    y_end: float
    direction: int = 0  # 0 表示自动推断，也可手动指定 +1 或 -1

    def __post_init__(self):
        if self.direction == 0:
            self.direction = 1 if self.y_end >= self.y_start else -1

    @property
    def length(self) -> float:
        return abs(self.y_end - self.y_start)

    @property
    def heading(self) -> float:
        return pi / 2 if self.direction > 0 else -pi / 2


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
    hx = cos(heading)
    hy = sin(heading)
    vx = p_to.x - p_from.x
    vy = p_to.y - p_from.y
    return hx * vx + hy * vy >= -tol


# =========================
# 测线点构造
# =========================

def point_on_vertical_line(line: VerticalLine, y: float) -> Point:
    return Point(line.x, y)


def build_eol_geom(line1: VerticalLine, eol_data_y: float, L_out: float) -> Tuple[State, Point]:
    data_pt = point_on_vertical_line(line1, eol_data_y)
    geom_pt = Point(line1.x, eol_data_y + line1.direction * L_out)
    return State(p=geom_pt, heading=line1.heading), data_pt


def build_sol_geom_fixed_head(line2: VerticalLine, L_in: float) -> Tuple[State, Point]:
    sol_data_y = line2.y_start
    data_pt = point_on_vertical_line(line2, sol_data_y)
    geom_pt = Point(line2.x, sol_data_y - line2.direction * L_in)
    return State(p=geom_pt, heading=line2.heading), data_pt


# =========================
# 可视化辅助函数
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
