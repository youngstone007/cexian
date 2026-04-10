from dataclasses import dataclass
from math import sqrt, atan2, cos, sin, pi, degrees
from typing import List, Tuple

from common import Point, VerticalLine, EPS


# =========================
# 通用测线数据结构
# =========================

@dataclass
class SurveyLine:
    """
    通用测线：两个端点定义，方向由 start→end 隐含

    - start: 采集起点
    - end:   采集终点
    - heading: start→end 的航向角（弧度），自动计算
    - length: 测线长度，自动计算
    """
    start: Point
    end: Point

    @property
    def heading(self) -> float:
        return atan2(self.end.y - self.start.y, self.end.x - self.start.x)

    @property
    def length(self) -> float:
        return sqrt((self.end.x - self.start.x) ** 2 + (self.end.y - self.start.y) ** 2)

    @property
    def direction_vector(self) -> Tuple[float, float]:
        dx = self.end.x - self.start.x
        dy = self.end.y - self.start.y
        n = sqrt(dx * dx + dy * dy)
        if n < EPS:
            return 0.0, 0.0
        return dx / n, dy / n


# =========================
# 坐标变换工具
# =========================

@dataclass
class Transform:
    """归一化变换参数，记录旋转和平移信息，可用于逆变换"""
    rotation_angle: float    # 旋转角（弧度）
    rotation_center: Point   # 旋转中心
    translation_dx: float    # X 平移量
    translation_dy: float    # Y 平移量


def rotate_point(p: Point, angle: float, center: Point) -> Point:
    """将点 p 绕 center 旋转 angle 弧度"""
    dx = p.x - center.x
    dy = p.y - center.y
    cos_a = cos(angle)
    sin_a = sin(angle)
    return Point(
        x=center.x + dx * cos_a - dy * sin_a,
        y=center.y + dx * sin_a + dy * cos_a
    )


def translate_point(p: Point, dx: float, dy: float) -> Point:
    """将点 p 平移 (dx, dy)"""
    return Point(x=p.x + dx, y=p.y + dy)


def denormalize_point(p: Point, transform: Transform) -> Point:
    """将归一化坐标下的点逆变换回原始坐标"""
    # 逆平移
    p1 = translate_point(p, -transform.translation_dx, -transform.translation_dy)
    # 逆旋转
    p2 = rotate_point(p1, -transform.rotation_angle, transform.rotation_center)
    return p2


# =========================
# 测线归一化
# =========================

def normalize_survey_lines(
    lines: List[SurveyLine],
    spacing_tol: float = 1e-3,
    parallel_tol: float = 1e-3,
    debug: bool = False
) -> Tuple[List[VerticalLine], Transform]:
    """
    将任意方向的平行测线归一化为 Y 轴对齐的 VerticalLine

    步骤：
    1. 从第一条测线计算公共航向角 θ
    2. 验证所有测线平行
    3. 计算旋转角 = π/2 - θ，旋转使测线对齐 Y 轴
    4. 按 x 排序，平移使最左侧线在 x=0
    5. 构造 VerticalLine 列表
    6. 验证相邻间距相等

    参数：
        lines: 原始测线列表
        spacing_tol: 间距验证容差
        parallel_tol: 平行验证容差（航向角差值，弧度）
        debug: 是否打印调试信息

    返回：
        (归一化后的 VerticalLine 列表, 变换参数)
    """
    if not lines:
        raise ValueError("测线列表不能为空")

    # --- 1. 计算公共航向角 ---
    ref_heading = lines[0].heading

    # --- 2. 验证所有测线平行 ---
    for i, line in enumerate(lines):
        angle_diff = abs(line.heading - ref_heading)
        # 处理角度环绕：两条平行线航向可能相差 π（反向平行）
        if angle_diff > pi:
            angle_diff = 2 * pi - angle_diff
        # 平行条件：航向差接近 0 或接近 π
        if angle_diff > parallel_tol and abs(angle_diff - pi) > parallel_tol:
            raise ValueError(
                f"测线 {i} 不平行于参考测线：航向差 = {degrees(angle_diff):.2f} deg"
            )

    # --- 3. 旋转使测线对齐 Y 轴 ---
    # 旋转角 = π/2 - θ，使测线方向映射到 +Y 方向
    rotation_angle = pi / 2 - ref_heading
    rotation_center = Point(0.0, 0.0)

    rotated_starts = [rotate_point(line.start, rotation_angle, rotation_center) for line in lines]
    rotated_ends = [rotate_point(line.end, rotation_angle, rotation_center) for line in lines]

    if debug:
        print(f"[归一化] 参考航向 = {degrees(ref_heading):.3f} deg")
        print(f"[归一化] 旋转角  = {degrees(rotation_angle):.3f} deg")

    # --- 4. 按 x 排序，平移使最左侧线在 x=0 ---
    # 旋转后每条测线的 x 坐标取端点的 x 均值作为测线位置
    line_data = []
    for i, line in enumerate(lines):
        rs = rotated_starts[i]
        re = rotated_ends[i]
        x_pos = (rs.x + re.x) / 2.0
        line_data.append((x_pos, rs, re, i))

    line_data.sort(key=lambda item: item[0])

    min_x = line_data[0][0]
    translation_dx = -min_x
    translation_dy = 0.0

    if debug:
        print(f"[归一化] 平移量  = ({translation_dx:.3f}, {translation_dy:.3f})")

    # --- 5. 构造 VerticalLine 列表 ---
    vertical_lines = []
    for x_pos, rs, re, orig_idx in line_data:
        # 平移
        ts = translate_point(rs, translation_dx, translation_dy)
        te = translate_point(re, translation_dx, translation_dy)

        vl = VerticalLine(
            x=round(ts.x, 10),  # 旋转+平移后 x 应相同，取 start.x
            y_start=ts.y,
            y_end=te.y,
        )
        vertical_lines.append(vl)

        if debug:
            print(f"[归一化] 测线 {orig_idx}: x={vl.x:.3f}, "
                  f"y_start={vl.y_start:.3f}, y_end={vl.y_end:.3f}, "
                  f"direction={vl.direction}")

    # --- 6. 验证相邻间距相等 ---
    if len(vertical_lines) >= 2:
        spacings = [
            vertical_lines[i + 1].x - vertical_lines[i].x
            for i in range(len(vertical_lines) - 1)
        ]
        for i, s in enumerate(spacings):
            if s <= 0:
                raise ValueError(f"测线 {i} 和 {i+1} 的间距非正: {s:.6f}")
            if abs(s - spacings[0]) > spacing_tol:
                raise ValueError(
                    f"测线间距不等：间距[0]={spacings[0]:.3f}, 间距[{i}]={s:.3f}"
                )

        if debug:
            print(f"[归一化] 测线间距 W = {spacings[0]:.3f}，共 {len(vertical_lines)} 条测线")

    transform = Transform(
        rotation_angle=rotation_angle,
        rotation_center=rotation_center,
        translation_dx=translation_dx,
        translation_dy=translation_dy,
    )

    return vertical_lines, transform
