import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Dict

from common import VerticalLine, EPS, sample_arc, sample_line
from dubins_csc import solve_fixed_head_tail_turn_between_vertical_lines
from semicircle import solve_fixed_head_tail_w_eq_2r_semicircle_turn
from three_arc import solve_fixed_head_tail_w_lt_2r_three_arc_turn


# =========================
# 规划结果数据结构
# =========================

@dataclass
class TurnDetail:
    """单段换线详情"""
    from_line: int          # 起始测线索引
    to_line: int            # 目标测线索引
    from_side: str          # 出发侧 ("AB" 或 "CD")
    distance: float         # 换线路径距离
    scenario: str           # 场景 ("W>2R", "W=2R", "W<2R")
    path_xs: List[float] = field(default_factory=list)  # 路径 X 坐标
    path_ys: List[float] = field(default_factory=list)  # 路径 Y 坐标


@dataclass
class PlannerResult:
    """规划结果"""
    route: List[int]                # 测线访问顺序（索引列表）
    sides: List[str]                # 每条线的进出侧 ("AB" 或 "CD")
    total_distance: float           # 总换线路径距离
    turn_details: List[TurnDetail]  # 每段换线的详细信息


# =========================
# 换线距离计算
# =========================

def _solve_turn(line1: VerticalLine, line2: VerticalLine,
                R: float, L_out: float, L_in: float, speed: float):
    """根据 W 与 2R 的关系调用对应的求解器"""
    W = abs(line2.x - line1.x)
    tol = 1e-6

    if W > 2.0 * R + tol:
        return solve_fixed_head_tail_turn_between_vertical_lines(
            line1, line2, L_out, L_in, R, speed), "W>2R"
    elif abs(W - 2.0 * R) <= tol:
        return solve_fixed_head_tail_w_eq_2r_semicircle_turn(
            line1, line2, L_out, L_in, R, speed, tol=tol), "W=2R"
    else:
        return solve_fixed_head_tail_w_lt_2r_three_arc_turn(
            line1, line2, L_out, L_in, R, speed, tol=tol), "W<2R"


def _extract_path_coords(result, scenario: str, R: float):
    """从求解器结果中提取路径坐标，返回 (path_xs, path_ys) 单条折线"""
    segments = []

    eol_data = result.eol_data_point
    sol_data = result.sol_data_point
    eol_geom = result.eol_geom
    sol_geom = result.sol_geom

    if scenario == "W>2R":
        d = result.dubins
        turn1 = d.path_type[0]
        turn2 = d.path_type[2]

        segments.append(sample_line(eol_data, eol_geom.p, n=10))
        segments.append(sample_arc(d.center1, R, eol_geom.p, d.tangent_point1, turn1, n=50))
        segments.append(sample_line(d.tangent_point1, d.tangent_point2, n=30))
        segments.append(sample_arc(d.center2, R, d.tangent_point2, sol_geom.p, turn2, n=50))
        segments.append(sample_line(sol_geom.p, sol_data, n=10))

    elif scenario == "W=2R":
        s = result.semi

        segments.append(sample_line(eol_data, eol_geom.p, n=10))
        if s.pre_straight_length > EPS:
            segments.append(sample_line(s.start_point, s.semicircle_start, n=20))
        segments.append(sample_arc(s.center, R, s.semicircle_start, s.semicircle_end,
                                   s.turn_direction, n=100))
        if s.post_straight_length > EPS:
            segments.append(sample_line(s.semicircle_end, s.end_point, n=20))
        segments.append(sample_line(sol_geom.p, sol_data, n=10))

    else:  # W<2R
        t = result.three_arc
        if t.turn_sequence == "LRL":
            turn1, turn2, turn3 = 'L', 'R', 'L'
        else:
            turn1, turn2, turn3 = 'R', 'L', 'R'

        segments.append(sample_line(eol_data, eol_geom.p, n=10))
        if t.pre_straight_length > EPS:
            segments.append(sample_line(t.start_point, t.turn_start_point, n=20))
        segments.append(sample_arc(t.center1, R, t.turn_start_point, t.join12, turn1, n=50))
        segments.append(sample_arc(t.center2, R, t.join12, t.join23, turn2, n=70))
        segments.append(sample_arc(t.center3, R, t.join23, t.turn_end_point, turn3, n=50))
        if t.post_straight_length > EPS:
            segments.append(sample_line(t.turn_end_point, t.end_point, n=20))
        segments.append(sample_line(sol_geom.p, sol_data, n=10))

    # 合并所有段为一条连续折线（去重连接点）
    all_xs = []
    all_ys = []
    for xs, ys in segments:
        if all_xs:
            all_xs.extend(xs[1:])
            all_ys.extend(ys[1:])
        else:
            all_xs.extend(xs)
            all_ys.extend(ys)

    return all_xs, all_ys


def compute_turn_with_path(line1: VerticalLine, line2: VerticalLine,
                           from_side: str, R: float, L_out: float,
                           L_in: float, speed: float):
    """计算换线距离、场景及路径坐标"""
    if from_side == "CD":
        tmp_line2 = VerticalLine(
            x=line2.x, y_start=line2.y_end, y_end=line2.y_start)
        solve_line1 = line1
        solve_line2 = tmp_line2
    else:  # AB
        tmp_line1 = VerticalLine(
            x=line1.x, y_start=line1.y_end, y_end=line1.y_start)
        solve_line1 = tmp_line1
        solve_line2 = line2

    result, scenario = _solve_turn(solve_line1, solve_line2, R, L_out, L_in, speed)

    if result is None:
        return float('inf'), scenario, [], []

    dist = result.total_cost
    path_xs, path_ys = _extract_path_coords(result, scenario, R)
    return dist, scenario, path_xs, path_ys


def compute_turn_distance(line1: VerticalLine, line2: VerticalLine,
                          from_side: str, R: float, L_out: float,
                          L_in: float, speed: float) -> float:
    """
    计算从 line1 的 from_side 侧下线到 line2 同侧上线的换线路径距离。

    from_side="CD": 从 line1 的 y_end 端下线，从 line2 的 y_end 端上线
    from_side="AB": 从 line1 的 y_start 端下线，从 line2 的 y_start 端上线

    同侧进出要求方向交替：line2 的行驶方向与 line1 相反。
    """
    if from_side == "CD":
        # line1 原样使用（EOL 在 y_end 端）
        # line2 需要方向反转（同侧进 → 方向交替）
        # 反转 line2：交换 y_start 和 y_end，direction 自动取反
        tmp_line2 = VerticalLine(
            x=line2.x,
            y_start=line2.y_end,
            y_end=line2.y_start,
        )
    else:  # AB 侧
        # line1 需要反转（从 y_start 端下线 = EOL 在 y_start 端，即方向反转）
        tmp_line1 = VerticalLine(
            x=line1.x,
            y_start=line1.y_end,
            y_end=line1.y_start,
        )
        # line2 从 y_start 端上线，方向与反转后的 line1 交替 → line2 保持原样
        line1 = tmp_line1
        tmp_line2 = line2

    result, _ = _solve_turn(line1, tmp_line2, R, L_out, L_in, speed)

    if result is None:
        return float('inf')

    if from_side == "CD":
        return result.total_cost if hasattr(result, 'total_cost') else result.dubins.total_length
    else:
        return result.total_cost if hasattr(result, 'total_cost') else result.dubins.total_length


def compute_turn_distance_with_detail(line1: VerticalLine, line2: VerticalLine,
                                       from_side: str, R: float, L_out: float,
                                       L_in: float, speed: float) -> tuple:
    """返回 (distance, scenario_str)"""
    if from_side == "CD":
        tmp_line2 = VerticalLine(
            x=line2.x,
            y_start=line2.y_end,
            y_end=line2.y_start,
        )
        solve_line1 = line1
        solve_line2 = tmp_line2
    else:  # AB
        tmp_line1 = VerticalLine(
            x=line1.x,
            y_start=line1.y_end,
            y_end=line1.y_start,
        )
        solve_line1 = tmp_line1
        solve_line2 = line2

    result, scenario = _solve_turn(solve_line1, solve_line2, R, L_out, L_in, speed)

    if result is None:
        return float('inf'), scenario

    dist = result.total_cost
    return dist, scenario


# =========================
# UCB 搜索策略
# =========================

def ucb_select(Q: List[List[float]], counts: List[List[int]],
               state: int, available: List[int], t: int,
               d: float = 1.414) -> int:
    """
    UCB 策略选择动作

    a = argmax(Q[s,a] + d * sqrt(ln(t) / N[s,a]))
    """
    if not available:
        raise ValueError("没有可选动作")

    if t <= 0:
        t = 1

    log_t = math.log(t)
    best_action = available[0]
    best_value = float('-inf')

    for a in available:
        n_sa = counts[state][a]
        if n_sa == 0:
            # 未被访问过的动作优先探索
            return a

        ucb_value = Q[state][a] + d * math.sqrt(log_t / n_sa)
        if ucb_value > best_value:
            best_value = ucb_value
            best_action = a

    return best_action


# =========================
# Q-Learning 航线规划器
# =========================

def plan_route(
    lines: List[VerticalLine],
    R: float,
    L_out: float,
    L_in: float,
    speed: float,
    episodes: int = 2000,
    alpha: float = 0.1,
    gamma: float = 0.9,
    ucb_d: float = 1.414,
    debug: bool = False
) -> PlannerResult:
    """
    基于强化 Q-Learning 的航线自动规划

    参数：
        lines: 已归一化的 VerticalLine 列表
        R: 最小转弯半径
        L_out: Run out 距离
        L_in: Run in 距离
        speed: 船舶速度
        episodes: 训练轮数
        alpha: 学习率
        gamma: 折扣因子
        ucb_d: UCB 置信区间参数
        debug: 是否打印调试信息

    返回：
        PlannerResult 规划结果
    """
    N = len(lines)
    if N < 2:
        raise ValueError("至少需要 2 条测线")

    # --- 初始化 Q 表和计数矩阵 ---
    # Q1: AB 侧（从 y_start 端下线）
    # Q2: CD 侧（从 y_end 端下线）
    Q1 = [[0.0] * N for _ in range(N)]
    Q2 = [[0.0] * N for _ in range(N)]
    C1 = [[0] * N for _ in range(N)]
    C2 = [[0] * N for _ in range(N)]

    # --- 预计算换线距离矩阵 ---
    dist_AB = [[0.0] * N for _ in range(N)]
    dist_CD = [[0.0] * N for _ in range(N)]

    for i in range(N):
        for j in range(N):
            if i == j:
                dist_AB[i][j] = float('inf')
                dist_CD[i][j] = float('inf')
            else:
                d_ab, _ = compute_turn_distance_with_detail(
                    lines[i], lines[j], "AB", R, L_out, L_in, speed)
                d_cd, _ = compute_turn_distance_with_detail(
                    lines[i], lines[j], "CD", R, L_out, L_in, speed)
                dist_AB[i][j] = d_ab
                dist_CD[i][j] = d_cd

    if debug:
        print(f"[Q-Learning] 预计算完成: {N} 条测线, {N*(N-1)*2} 条换线路径")

    # --- Q-Learning 训练 ---
    global_step = 0

    for ep in range(episodes):
        # 随机选择起始测线和起始侧
        s = random.randint(0, N - 1)
        side = random.choice(["AB", "CD"])
        visited = {s}

        while len(visited) < N:
            # 选择 Q 表
            Q = Q1 if side == "AB" else Q2
            C = C1 if side == "AB" else C2
            dist_mat = dist_AB if side == "AB" else dist_CD

            # 可选动作
            available = [j for j in range(N) if j not in visited]
            if not available:
                break

            global_step += 1

            # UCB 选择下一测线
            s_next = ucb_select(Q, C, s, available, global_step, ucb_d)

            # 获取奖励（负距离）
            dist = dist_mat[s][s_next]
            reward = -dist if dist != float('inf') else -1e10

            # 计算下一状态的最大 Q 值
            # 下一侧是翻转的
            Q_next = Q2 if side == "AB" else Q1
            future_available = [j for j in range(N) if j not in visited and j != s_next]
            if future_available:
                max_future_q = max(Q_next[s_next][j] for j in future_available)
            else:
                max_future_q = 0.0

            # 更新 Q 值
            Q[s][s_next] += alpha * (reward + gamma * max_future_q - Q[s][s_next])
            C[s][s_next] += 1

            # 状态转移
            s = s_next
            visited.add(s)
            side = "AB" if side == "CD" else "CD"  # 侧翻转

        if debug and (ep + 1) % 500 == 0:
            print(f"[Q-Learning] episode {ep + 1}/{episodes}")

    # --- 提取最优路线 ---
    best_route = None
    best_sides = None
    best_dist = float('inf')

    for start_s in range(N):
        for start_side in ["AB", "CD"]:
            route, sides = _extract_route(Q1, Q2, start_s, start_side, N, dist_AB, dist_CD)
            total = _compute_route_distance(route, sides, dist_AB, dist_CD)
            if total < best_dist:
                best_dist = total
                best_route = route
                best_sides = sides

    # --- 构建结果 ---
    turn_details = []
    for k in range(len(best_route) - 1):
        i = best_route[k]
        j = best_route[k + 1]
        side = best_sides[k]
        dist_mat = dist_AB if side == "AB" else dist_CD
        _, scenario, path_xs, path_ys = compute_turn_with_path(
            lines[i], lines[j], side, R, L_out, L_in, speed)
        turn_details.append(TurnDetail(
            from_line=i, to_line=j, from_side=side,
            distance=dist_mat[i][j], scenario=scenario,
            path_xs=path_xs, path_ys=path_ys
        ))

    if debug:
        print(f"[Q-Learning] 最优路线: {best_route}")
        print(f"[Q-Learning] 进出侧: {best_sides}")
        print(f"[Q-Learning] 总换线距离: {best_dist:.3f}")

    return PlannerResult(
        route=best_route,
        sides=best_sides,
        total_distance=best_dist,
        turn_details=turn_details,
    )


def _extract_route(Q1, Q2, start_s, start_side, N, dist_AB, dist_CD):
    """从 Q 表中贪心提取路线

    返回的 sides 列表长度为 N-1，sides[k] 表示第 k 段换线的 from_side
    （即从 route[k] 的哪个侧下线，也是 route[k+1] 的哪个侧上线）
    """
    route = [start_s]
    sides = []  # 不预存 start_side，每次迭代追加当前 turn 的 from_side
    visited = {start_s}
    s = start_s
    side = start_side

    while len(visited) < N:
        Q = Q1 if side == "AB" else Q2
        available = [j for j in range(N) if j not in visited]

        if not available:
            break

        # 贪心选 Q 值最大的
        best_next = max(available, key=lambda j: Q[s][j])
        route.append(best_next)
        sides.append(side)
        visited.add(best_next)
        s = best_next
        side = "AB" if side == "CD" else "CD"

    return route, sides


def _compute_route_distance(route, sides, dist_AB, dist_CD):
    """计算路线总距离"""
    total = 0.0
    for k in range(len(route) - 1):
        i = route[k]
        j = route[k + 1]
        side = sides[k]
        dist_mat = dist_AB if side == "AB" else dist_CD
        d = dist_mat[i][j]
        if d == float('inf'):
            return float('inf')
        total += d
    return total


# =========================
# 随机路线对比
# =========================

def random_route_distance(lines: List[VerticalLine], R: float, L_out: float,
                          L_in: float, speed: float, n_trials: int = 100) -> float:
    """生成随机路线的平均总距离，用于对比"""
    N = len(lines)

    # 预计算距离矩阵
    dist_AB = [[0.0] * N for _ in range(N)]
    dist_CD = [[0.0] * N for _ in range(N)]
    for i in range(N):
        for j in range(N):
            if i == j:
                dist_AB[i][j] = float('inf')
                dist_CD[i][j] = float('inf')
            else:
                d_ab, _ = compute_turn_distance_with_detail(
                    lines[i], lines[j], "AB", R, L_out, L_in, speed)
                d_cd, _ = compute_turn_distance_with_detail(
                    lines[i], lines[j], "CD", R, L_out, L_in, speed)
                dist_AB[i][j] = d_ab
                dist_CD[i][j] = d_cd

    total = 0.0
    for _ in range(n_trials):
        route = list(range(N))
        random.shuffle(route)
        start_side = random.choice(["AB", "CD"])
        sides = []
        for k in range(len(route) - 1):
            sides.append(start_side)
            start_side = "AB" if start_side == "CD" else "CD"
        dist = _compute_route_distance(route, sides, dist_AB, dist_CD)
        if dist != float('inf'):
            total += dist

    return total / n_trials
