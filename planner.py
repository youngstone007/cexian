import math
import random
import itertools
from functools import lru_cache
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple

from common import VerticalLine, EPS, sample_arc, sample_line
from dubins_csc import solve_fixed_head_tail_turn_between_vertical_lines
from semicircle import solve_fixed_head_tail_w_eq_2r_semicircle_turn
from three_arc import solve_fixed_head_tail_w_lt_2r_three_arc_turn


# ============================================================
# 数据结构
# ============================================================

@dataclass
class TurnDetail:
    """单段换线详情"""
    from_line: int
    to_line: int
    from_side: str          # "AB" / "CD"
    distance: float
    scenario: str           # "W>2R" / "W=2R" / "W<2R"
    path_xs: List[float] = field(default_factory=list)
    path_ys: List[float] = field(default_factory=list)


@dataclass
class PlannerResult:
    """整体规划结果"""
    route: List[int]
    sides: List[str]
    total_distance: float
    turn_details: List[TurnDetail]


@dataclass
class SurveyBlock:
    """
    空间块 / 细分块
    """
    block_id: int
    global_indices: List[int]   # 按 x 从小到大
    x_min: float
    x_max: float
    source_block_ids: List[int] = field(default_factory=list)

    @property
    def size(self) -> int:
        return len(self.global_indices)

    @property
    def width(self) -> float:
        return self.x_max - self.x_min


@dataclass
class WorkUnit:
    """
    工作单元：由两个相邻细分块组成
    """
    unit_id: int
    block_ids: List[int]


@dataclass
class UnitCandidate:
    """
    单个工作单元的一条候选路线
    """
    unit_id: int
    block_ids: List[int]

    route: List[int]
    sides: List[str]
    total_distance: float

    start_line: int
    end_line: int
    start_entry_side: str
    end_exit_side: str

    block_directions: Dict[int, str]   # {block_id: "UP"/"DOWN"}


# ============================================================
# 基础辅助
# ============================================================

def opposite_side(side: str) -> str:
    if side == "AB":
        return "CD"
    if side == "CD":
        return "AB"
    raise ValueError(f"Unknown side: {side}")


def opposite_direction(direction: str) -> str:
    if direction == "UP":
        return "DOWN"
    if direction == "DOWN":
        return "UP"
    raise ValueError(f"Unknown direction: {direction}")


def direction_to_entry_side(direction: str) -> str:
    if direction == "UP":
        return "AB"
    if direction == "DOWN":
        return "CD"
    raise ValueError(f"Unknown direction: {direction}")


def direction_to_exit_side(direction: str) -> str:
    if direction == "UP":
        return "CD"
    if direction == "DOWN":
        return "AB"
    raise ValueError(f"Unknown direction: {direction}")


def _build_sides_from_start_entry(route_len: int, start_entry_side: str) -> List[str]:
    """
    给定路线长度和第一条线入口侧，生成每段换线的 from_side。
    """
    if route_len <= 1:
        return []

    sides = []
    cur = opposite_side(start_entry_side)
    for _ in range(route_len - 1):
        sides.append(cur)
        cur = opposite_side(cur)
    return sides


# ============================================================
# 局部换线求解统一入口
# ============================================================

def _solve_turn(line1: VerticalLine, line2: VerticalLine,
                R: float, L_out: float, L_in: float, speed: float):
    W = abs(line2.x - line1.x)
    tol = 1e-6

    if W > 2.0 * R + tol:
        return solve_fixed_head_tail_turn_between_vertical_lines(
            line1, line2, L_out, L_in, R, speed
        ), "W>2R"
    elif abs(W - 2.0 * R) <= tol:
        return solve_fixed_head_tail_w_eq_2r_semicircle_turn(
            line1, line2, L_out, L_in, R, speed, tol=tol
        ), "W=2R"
    else:
        return solve_fixed_head_tail_w_lt_2r_three_arc_turn(
            line1, line2, L_out, L_in, R, speed, tol=tol
        ), "W<2R"


def _extract_total_length(result, scenario: str) -> float:
    if scenario == "W>2R":
        return result.dubins.total_length
    elif scenario == "W=2R":
        return result.semi.total_length
    else:
        return result.three_arc.total_length


def _extract_path_coords(result, scenario: str, R: float):
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
        segments.append(sample_arc(
            s.center, R, s.semicircle_start, s.semicircle_end, s.turn_direction, n=100
        ))
        if s.post_straight_length > EPS:
            segments.append(sample_line(s.semicircle_end, s.end_point, n=20))
        segments.append(sample_line(sol_geom.p, sol_data, n=10))

    else:
        t = result.three_arc
        if t.turn_sequence == "LRL":
            turn1, turn2, turn3 = "L", "R", "L"
        else:
            turn1, turn2, turn3 = "R", "L", "R"

        segments.append(sample_line(eol_data, eol_geom.p, n=10))
        if t.pre_straight_length > EPS:
            segments.append(sample_line(t.start_point, t.turn_start_point, n=20))
        segments.append(sample_arc(t.center1, R, t.turn_start_point, t.join12, turn1, n=50))
        segments.append(sample_arc(t.center2, R, t.join12, t.join23, turn2, n=70))
        segments.append(sample_arc(t.center3, R, t.join23, t.turn_end_point, turn3, n=50))
        if t.post_straight_length > EPS:
            segments.append(sample_line(t.turn_end_point, t.end_point, n=20))
        segments.append(sample_line(sol_geom.p, sol_data, n=10))

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
    """
    from_side="CD":
        从 line1 的 y_end 端下线，到 line2 的 y_end 端上线
    from_side="AB":
        从 line1 的 y_start 端下线，到 line2 的 y_start 端上线
    """
    if from_side == "CD":
        tmp_line2 = VerticalLine(x=line2.x, y_start=line2.y_end, y_end=line2.y_start)
        solve_line1 = line1
        solve_line2 = tmp_line2
    else:
        tmp_line1 = VerticalLine(x=line1.x, y_start=line1.y_end, y_end=line1.y_start)
        solve_line1 = tmp_line1
        solve_line2 = line2

    result, scenario = _solve_turn(solve_line1, solve_line2, R, L_out, L_in, speed)
    if result is None:
        return float("inf"), scenario, [], []

    dist = _extract_total_length(result, scenario)
    path_xs, path_ys = _extract_path_coords(result, scenario, R)
    return dist, scenario, path_xs, path_ys


def compute_turn_distance(line1: VerticalLine, line2: VerticalLine,
                          from_side: str, R: float, L_out: float,
                          L_in: float, speed: float) -> float:
    dist, _, _, _ = compute_turn_with_path(line1, line2, from_side, R, L_out, L_in, speed)
    return dist


def compute_turn_distance_with_detail(line1: VerticalLine, line2: VerticalLine,
                                      from_side: str, R: float, L_out: float,
                                      L_in: float, speed: float) -> Tuple[float, str]:
    if from_side == "CD":
        tmp_line2 = VerticalLine(x=line2.x, y_start=line2.y_end, y_end=line2.y_start)
        solve_line1 = line1
        solve_line2 = tmp_line2
    else:
        tmp_line1 = VerticalLine(x=line1.x, y_start=line1.y_end, y_end=line1.y_start)
        solve_line1 = tmp_line1
        solve_line2 = line2

    result, scenario = _solve_turn(solve_line1, solve_line2, R, L_out, L_in, speed)
    if result is None:
        return float("inf"), scenario
    return _extract_total_length(result, scenario), scenario


# ============================================================
# 全局距离缓存
# ============================================================

def _build_global_distance_cache(lines: List[VerticalLine],
                                 R: float,
                                 L_out: float,
                                 L_in: float,
                                 speed: float) -> Dict[Tuple[int, int, str], float]:
    n = len(lines)
    cache: Dict[Tuple[int, int, str], float] = {}

    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            d_ab, _ = compute_turn_distance_with_detail(
                lines[i], lines[j], "AB", R, L_out, L_in, speed
            )
            d_cd, _ = compute_turn_distance_with_detail(
                lines[i], lines[j], "CD", R, L_out, L_in, speed
            )
            cache[(i, j, "AB")] = d_ab
            cache[(i, j, "CD")] = d_cd

    return cache


def _turn_distance_cached(distance_cache: Dict[Tuple[int, int, str], float],
                          i: int,
                          j: int,
                          from_side: str) -> float:
    return distance_cache.get((i, j, from_side), float("inf"))


# ============================================================
# 专家经验法：工区分块
# ============================================================

def compute_block_max_span(R: float,
                           L_in: float,
                           max_heading_angle_deg: float = 30.0) -> float:
    """
    按论文图中的几何关系：
        x_max = R + R*cos(alpha)
    """
    alpha_deg = max(0.0, min(90.0, max_heading_angle_deg))
    alpha = math.radians(alpha_deg)
    x_max = R * (1.0 + math.cos(alpha))
    return max(0.0, x_max)


def partition_lines_by_expert_rule(lines: List[VerticalLine],
                                   R: float,
                                   L_in: float,
                                   max_heading_angle_deg: float = 30.0,
                                   tol: float = 1e-9,
                                   debug: bool = False) -> Tuple[List[SurveyBlock], float]:
    if not lines:
        return [], 0.0

    x_max_allow = compute_block_max_span(R, L_in, max_heading_angle_deg)
    sorted_indices = sorted(range(len(lines)), key=lambda i: lines[i].x)

    blocks: List[SurveyBlock] = []
    current = [sorted_indices[0]]
    current_x_min = lines[sorted_indices[0]].x
    block_id = 0

    for idx in sorted_indices[1:]:
        span = lines[idx].x - current_x_min
        if span <= x_max_allow + tol:
            current.append(idx)
        else:
            xs = [lines[g].x for g in current]
            blocks.append(SurveyBlock(
                block_id=block_id,
                global_indices=current[:],
                x_min=min(xs),
                x_max=max(xs),
                source_block_ids=[block_id],
            ))
            block_id += 1
            current = [idx]
            current_x_min = lines[idx].x

    xs = [lines[g].x for g in current]
    blocks.append(SurveyBlock(
        block_id=block_id,
        global_indices=current[:],
        x_min=min(xs),
        x_max=max(xs),
        source_block_ids=[block_id],
    ))

    if debug:
        print(f"[BLOCK] alpha={max_heading_angle_deg:.3f} deg")
        print(f"[BLOCK] x_max_allow={x_max_allow:.3f}")
        print(f"[BLOCK] num_blocks={len(blocks)}")
        for b in blocks:
            print(f"  raw spatial block {b.block_id}: size={b.size}, width={b.width:.3f}, lines={b.global_indices}")

    return blocks, x_max_allow


# ============================================================
# 可行性细分：把原始空间块切成可配对的细分块
# ============================================================

def _split_block_by_sizes(block: SurveyBlock, sizes: List[int], new_block_id_start: int,
                          lines: List[VerticalLine]) -> Tuple[List[SurveyBlock], int]:
    """
    将一个连续空间块按连续子段 sizes 切分。
    """
    res: List[SurveyBlock] = []
    pos = 0
    cur_id = new_block_id_start

    for sz in sizes:
        part = block.global_indices[pos:pos + sz]
        xs = [lines[g].x for g in part]
        res.append(SurveyBlock(
            block_id=cur_id,
            global_indices=part,
            x_min=min(xs),
            x_max=max(xs),
            source_block_ids=block.source_block_ids[:],
        ))
        cur_id += 1
        pos += sz

    if pos != len(block.global_indices):
        raise RuntimeError("Block split size mismatch")

    return res, cur_id


def _refine_blocks_for_pair_units(raw_blocks: List[SurveyBlock],
                                  lines: List[VerticalLine],
                                  debug: bool = False) -> List[SurveyBlock]:
    """
    核心修复：
    - 将原始空间块再细分成“可成对交错访问”的细分块
    - 最终要求细分块可以按顺序两两配对
    - 每一对块大小满足 |a-b| <= 1

    这样才能在“块内同向 + AB->AB / CD->CD”规则下可行。
    """
    if not raw_blocks:
        return []

    sizes = [b.size for b in raw_blocks]
    n = len(raw_blocks)

    @lru_cache(None)
    def dp(i: int, pending: int) -> Optional[Tuple[int, int]]:
        """
        pending = -1 表示当前没有等待配对的细分块
        pending >= 1 表示已有一个等待配对的细分块，其大小为 pending

        返回值：
        - (split_cnt, diff_cost) 的最优字典序
        - 若不可行则返回 None
        """
        if i == n:
            if pending == -1:
                return (0, 0)
            return None

        best: Optional[Tuple[int, int]] = None

        def better(a: Tuple[int, int], b: Optional[Tuple[int, int]]) -> bool:
            if b is None:
                return True
            return a < b

        s = sizes[i]

        # 方案1：当前 raw block 不切分
        pieces_options = [[s]]

        # 方案2：切成两段
        for a in range(1, s):
            b = s - a
            pieces_options.append([a, b])

        for pieces in pieces_options:
            split_cnt_here = 0 if len(pieces) == 1 else 1
            cur_pending = pending
            diff_cost_here = 0
            feasible = True

            for piece in pieces:
                if cur_pending == -1:
                    cur_pending = piece
                else:
                    if abs(cur_pending - piece) > 1:
                        feasible = False
                        break
                    diff_cost_here += abs(cur_pending - piece)
                    cur_pending = -1

            if not feasible:
                continue

            future = dp(i + 1, cur_pending)
            if future is None:
                continue

            cand = (future[0] + split_cnt_here, future[1] + diff_cost_here)
            if better(cand, best):
                best = cand

        return best

    best0 = dp(0, -1)
    if best0 is None:
        raise RuntimeError(
            "Raw spatial blocks cannot be refined into feasible same-direction pair units"
        )

    refined: List[SurveyBlock] = []
    pending = -1
    next_block_id = 0

    for i, block in enumerate(raw_blocks):
        s = sizes[i]
        chosen_pieces = None

        pieces_options = [[s]]
        for a in range(1, s):
            b = s - a
            pieces_options.append([a, b])

        for pieces in pieces_options:
            split_cnt_here = 0 if len(pieces) == 1 else 1
            cur_pending = pending
            diff_cost_here = 0
            feasible = True

            for piece in pieces:
                if cur_pending == -1:
                    cur_pending = piece
                else:
                    if abs(cur_pending - piece) > 1:
                        feasible = False
                        break
                    diff_cost_here += abs(cur_pending - piece)
                    cur_pending = -1

            if not feasible:
                continue

            future = dp(i + 1, cur_pending)
            if future is None:
                continue

            total = (future[0] + split_cnt_here, future[1] + diff_cost_here)
            if total == dp(i, pending):
                chosen_pieces = pieces
                pending = cur_pending
                break

        if chosen_pieces is None:
            raise RuntimeError("Refined block reconstruction failed")

        parts, next_block_id = _split_block_by_sizes(block, chosen_pieces, next_block_id, lines)
        refined.extend(parts)

    if pending != -1:
        raise RuntimeError("Refined block pairing reconstruction ended with unmatched block")

    if len(refined) % 2 != 0:
        raise RuntimeError("Refined block count should be even but is not")

    if debug:
        print(f"[REFINE] raw block sizes = {[b.size for b in raw_blocks]}")
        print(f"[REFINE] refined block sizes = {[b.size for b in refined]}")
        for b in refined:
            print(
                f"  refined block {b.block_id}: size={b.size}, width={b.width:.3f}, "
                f"src={b.source_block_ids}, lines={b.global_indices}"
            )

    return refined


# ============================================================
# 路线辅助
# ============================================================

def _build_turn_details_for_global_route(route_global: List[int],
                                         sides: List[str],
                                         lines: List[VerticalLine],
                                         R: float,
                                         L_out: float,
                                         L_in: float,
                                         speed: float) -> Tuple[List[TurnDetail], float]:
    turn_details: List[TurnDetail] = []
    total = 0.0

    for k in range(len(route_global) - 1):
        i = route_global[k]
        j = route_global[k + 1]
        from_side = sides[k]

        dist, scenario, path_xs, path_ys = compute_turn_with_path(
            lines[i], lines[j], from_side, R, L_out, L_in, speed
        )
        if not math.isfinite(dist):
            return [], float("inf")

        turn_details.append(TurnDetail(
            from_line=i,
            to_line=j,
            from_side=from_side,
            distance=dist,
            scenario=scenario,
            path_xs=path_xs,
            path_ys=path_ys,
        ))
        total += dist

    return turn_details, total


# ============================================================
# 单元内部：2块交错访问
# ============================================================

def _generate_block_internal_orders(block: SurveyBlock) -> List[List[int]]:
    asc = block.global_indices[:]
    desc = list(reversed(asc))
    if asc == desc:
        return [asc]
    return [asc, desc]


def _solve_pair_unit_interleaving(block_a: SurveyBlock,
                                  dir_a: str,
                                  order_a: List[int],
                                  block_b: SurveyBlock,
                                  dir_b: str,
                                  order_b: List[int],
                                  distance_cache: Dict[Tuple[int, int, str], float]
                                  ) -> Optional[Tuple[List[int], List[str], float]]:
    """
    两个块交错访问，且每个块内部方向一致。
    只有当 |len(a)-len(b)| <= 1 时才可能可行。
    """
    na = len(order_a)
    nb = len(order_b)

    if abs(na - nb) > 1:
        return None

    side_a = direction_to_exit_side(dir_a)
    side_b = direction_to_exit_side(dir_b)

    @lru_cache(None)
    def dp(i: int, j: int, last: int) -> float:
        """
        last:
        -1: 起点
         0: 上一步来自 A
         1: 上一步来自 B
        """
        if i == na and j == nb:
            return 0.0

        best = float("inf")

        if last in [-1, 1] and i < na:
            step = 0.0
            if last == 1:
                prev = order_b[j - 1]
                nxt = order_a[i]
                step = _turn_distance_cached(distance_cache, prev, nxt, side_b)
                if not math.isfinite(step):
                    step = float("inf")
            future = dp(i + 1, j, 0)
            if math.isfinite(step) and math.isfinite(future):
                best = min(best, step + future)

        if last in [-1, 0] and j < nb:
            step = 0.0
            if last == 0:
                prev = order_a[i - 1]
                nxt = order_b[j]
                step = _turn_distance_cached(distance_cache, prev, nxt, side_a)
                if not math.isfinite(step):
                    step = float("inf")
            future = dp(i, j + 1, 1)
            if math.isfinite(step) and math.isfinite(future):
                best = min(best, step + future)

        return best

    best_cost = dp(0, 0, -1)
    if not math.isfinite(best_cost):
        return None

    route: List[int] = []
    sides: List[str] = []

    i, j, last = 0, 0, -1
    while not (i == na and j == nb):
        cur = dp(i, j, last)

        moved = False

        if last in [-1, 1] and i < na:
            step = 0.0
            if last == 1:
                prev = order_b[j - 1]
                nxt = order_a[i]
                step = _turn_distance_cached(distance_cache, prev, nxt, side_b)
            future = dp(i + 1, j, 0)
            if math.isfinite(step) and math.isfinite(future) and abs(cur - (step + future)) < 1e-9:
                if last == 1:
                    sides.append(side_b)
                route.append(order_a[i])
                i += 1
                last = 0
                moved = True

        if moved:
            continue

        if last in [-1, 0] and j < nb:
            step = 0.0
            if last == 0:
                prev = order_a[i - 1]
                nxt = order_b[j]
                step = _turn_distance_cached(distance_cache, prev, nxt, side_a)
            future = dp(i, j + 1, 1)
            if math.isfinite(step) and math.isfinite(future) and abs(cur - (step + future)) < 1e-9:
                if last == 0:
                    sides.append(side_a)
                route.append(order_b[j])
                j += 1
                last = 1
                moved = True

        if not moved:
            return None

    return route, sides, best_cost


def _generate_pair_unit_candidates(block_a: SurveyBlock,
                                   block_b: SurveyBlock,
                                   distance_cache: Dict[Tuple[int, int, str], float],
                                   top_k: int = 12,
                                   debug: bool = False) -> List[UnitCandidate]:
    """
    两个细分块组成一个工作单元。
    """
    candidates: List[UnitCandidate] = []

    orders_a = _generate_block_internal_orders(block_a)
    orders_b = _generate_block_internal_orders(block_b)

    for first_dir in ["UP", "DOWN"]:
        dir_a = first_dir
        dir_b = opposite_direction(first_dir)

        for order_a in orders_a:
            for order_b in orders_b:
                solved = _solve_pair_unit_interleaving(
                    block_a, dir_a, order_a,
                    block_b, dir_b, order_b,
                    distance_cache
                )
                if solved is None:
                    continue

                route, sides, total_distance = solved
                if not route:
                    continue

                line_to_block = {}
                for g in order_a:
                    line_to_block[g] = block_a.block_id
                for g in order_b:
                    line_to_block[g] = block_b.block_id

                first_line = route[0]
                last_line = route[-1]
                first_block_id = line_to_block[first_line]
                last_block_id = line_to_block[last_line]

                block_directions = {
                    block_a.block_id: dir_a,
                    block_b.block_id: dir_b,
                }

                start_entry_side = direction_to_entry_side(block_directions[first_block_id])
                end_exit_side = direction_to_exit_side(block_directions[last_block_id])

                candidates.append(UnitCandidate(
                    unit_id=-1,
                    block_ids=[block_a.block_id, block_b.block_id],
                    route=route,
                    sides=sides,
                    total_distance=total_distance,
                    start_line=first_line,
                    end_line=last_line,
                    start_entry_side=start_entry_side,
                    end_exit_side=end_exit_side,
                    block_directions=block_directions,
                ))

    if not candidates:
        return []

    uniq: Dict[Tuple[Tuple[int, ...], str], UnitCandidate] = {}
    for cand in candidates:
        key = (tuple(cand.route), cand.start_entry_side)
        if key not in uniq or cand.total_distance < uniq[key].total_distance:
            uniq[key] = cand

    result = sorted(uniq.values(), key=lambda c: c.total_distance)
    result = result[:max(1, top_k)]

    if debug:
        print(f"[UNIT GEN] refined blocks={[block_a.block_id, block_b.block_id]}, candidates={len(result)}")
        for i, c in enumerate(result):
            print(
                f"  cand {i}: route={c.route}, start_entry={c.start_entry_side}, "
                f"end_exit={c.end_exit_side}, dirs={c.block_directions}, dist={c.total_distance:.3f}"
            )

    return result


# ============================================================
# 构建工作单元
# ============================================================

def _build_work_units(refined_blocks: List[SurveyBlock],
                      distance_cache: Dict[Tuple[int, int, str], float],
                      top_k: int = 12,
                      debug: bool = False) -> Tuple[List[WorkUnit], Dict[int, List[UnitCandidate]]]:
    if len(refined_blocks) % 2 != 0:
        raise RuntimeError("Refined block count must be even")

    units: List[WorkUnit] = []
    candidates_by_unit: Dict[int, List[UnitCandidate]] = {}

    unit_id = 0
    for i in range(0, len(refined_blocks), 2):
        a = refined_blocks[i]
        b = refined_blocks[i + 1]

        unit = WorkUnit(unit_id=unit_id, block_ids=[a.block_id, b.block_id])
        units.append(unit)

        cands = _generate_pair_unit_candidates(
            a, b,
            distance_cache,
            top_k=top_k,
            debug=debug
        )
        if not cands:
            raise RuntimeError(f"Failed to generate feasible candidates for refined block pair ({a.block_id}, {b.block_id})")

        fixed: List[UnitCandidate] = []
        for c in cands:
            fixed.append(UnitCandidate(
                unit_id=unit_id,
                block_ids=c.block_ids[:],
                route=c.route[:],
                sides=c.sides[:],
                total_distance=c.total_distance,
                start_line=c.start_line,
                end_line=c.end_line,
                start_entry_side=c.start_entry_side,
                end_exit_side=c.end_exit_side,
                block_directions=dict(c.block_directions),
            ))

        candidates_by_unit[unit_id] = fixed
        unit_id += 1

    if debug:
        print(f"[UNIT] num_units={len(units)}")
        for u in units:
            print(f"  unit {u.unit_id}: refined_blocks={u.block_ids}")

    return units, candidates_by_unit


# ============================================================
# 单元间拼接 / 全局优化
# ============================================================

def _build_connector(prev_cand: UnitCandidate,
                     next_cand: UnitCandidate,
                     lines: List[VerticalLine],
                     R: float,
                     L_out: float,
                     L_in: float,
                     speed: float) -> Optional[TurnDetail]:
    if prev_cand.end_exit_side != next_cand.start_entry_side:
        return None

    from_side = prev_cand.end_exit_side
    dist, scenario, path_xs, path_ys = compute_turn_with_path(
        lines[prev_cand.end_line],
        lines[next_cand.start_line],
        from_side,
        R, L_out, L_in, speed
    )
    if not math.isfinite(dist):
        return None

    return TurnDetail(
        from_line=prev_cand.end_line,
        to_line=next_cand.start_line,
        from_side=from_side,
        distance=dist,
        scenario=scenario,
        path_xs=path_xs,
        path_ys=path_ys,
    )


def _center_out_unit_order(num_units: int, start_unit: int) -> List[int]:
    order = [start_unit]
    left = start_unit - 1
    right = start_unit + 1
    toggle = True

    while left >= 0 or right < num_units:
        if toggle:
            if right < num_units:
                order.append(right)
                right += 1
            elif left >= 0:
                order.append(left)
                left -= 1
        else:
            if left >= 0:
                order.append(left)
                left -= 1
            elif right < num_units:
                order.append(right)
                right += 1
        toggle = not toggle

    return order


def _best_choices_for_unit_order(order: List[int],
                                 candidates_by_unit: Dict[int, List[UnitCandidate]],
                                 lines: List[VerticalLine],
                                 R: float,
                                 L_out: float,
                                 L_in: float,
                                 speed: float,
                                 connector_cache: Dict[Tuple[int, int, int, int], Optional[TurnDetail]]
                                 ) -> Tuple[Dict[int, int], float]:
    if not order:
        return {}, 0.0

    ordered_lists = [candidates_by_unit[u] for u in order]

    dp_rows: List[List[float]] = []
    parent: List[List[Optional[int]]] = []

    first_list = ordered_lists[0]
    dp_rows.append([cand.total_distance for cand in first_list])
    parent.append([None] * len(first_list))

    for pos in range(1, len(order)):
        prev_unit_id = order[pos - 1]
        curr_unit_id = order[pos]

        prev_list = ordered_lists[pos - 1]
        curr_list = ordered_lists[pos]

        cur_dp = [float("inf")] * len(curr_list)
        cur_parent = [None] * len(curr_list)

        for j, curr_cand in enumerate(curr_list):
            for i, prev_cand in enumerate(prev_list):
                if not math.isfinite(dp_rows[pos - 1][i]):
                    continue

                key = (prev_unit_id, i, curr_unit_id, j)
                if key not in connector_cache:
                    connector_cache[key] = _build_connector(
                        prev_cand, curr_cand,
                        lines, R, L_out, L_in, speed
                    )

                connector = connector_cache[key]
                if connector is None:
                    continue

                total = dp_rows[pos - 1][i] + connector.distance + curr_cand.total_distance
                if total < cur_dp[j]:
                    cur_dp[j] = total
                    cur_parent[j] = i

        dp_rows.append(cur_dp)
        parent.append(cur_parent)

    last_row = dp_rows[-1]
    finite_indices = [i for i, v in enumerate(last_row) if math.isfinite(v)]
    if not finite_indices:
        return {}, float("inf")

    best_last_idx = min(finite_indices, key=lambda i: last_row[i])
    best_cost = last_row[best_last_idx]

    chosen = [None] * len(order)
    chosen[-1] = best_last_idx
    for pos in range(len(order) - 1, 0, -1):
        pi = parent[pos][chosen[pos]]
        if pi is None:
            return {}, float("inf")
        chosen[pos - 1] = pi

    choices = {order[pos]: chosen[pos] for pos in range(len(order))}
    return choices, best_cost


def _neighbor_unit_order(order: List[int]) -> List[int]:
    new_order = order[:]
    if len(new_order) <= 1:
        return new_order

    move = random.choice(["swap", "reverse", "move"])

    if move == "swap":
        i, j = random.sample(range(len(new_order)), 2)
        new_order[i], new_order[j] = new_order[j], new_order[i]
    elif move == "reverse":
        i, j = sorted(random.sample(range(len(new_order)), 2))
        new_order[i:j + 1] = reversed(new_order[i:j + 1])
    else:
        i, j = random.sample(range(len(new_order)), 2)
        u = new_order.pop(i)
        new_order.insert(j, u)

    return new_order


def _anneal_global_units(units: List[WorkUnit],
                         candidates_by_unit: Dict[int, List[UnitCandidate]],
                         lines: List[VerticalLine],
                         R: float,
                         L_out: float,
                         L_in: float,
                         speed: float,
                         sa_restarts: int = 20,
                         sa_steps: int = 2500,
                         sa_init_temp: float = 3000.0,
                         sa_cooling: float = 0.996,
                         debug: bool = False) -> Tuple[List[int], Dict[int, int], float]:
    num_units = len(units)
    unit_ids = [u.unit_id for u in units]
    connector_cache: Dict[Tuple[int, int, int, int], Optional[TurnDetail]] = {}

    best_order = unit_ids[:]
    best_choices: Dict[int, int] = {}
    best_cost = float("inf")

    structured_orders = []
    structured_orders.append(unit_ids[:])
    structured_orders.append(list(reversed(unit_ids)))
    for s in range(num_units):
        structured_orders.append(_center_out_unit_order(num_units, s))

    unique_orders = []
    seen_orders = set()
    for od in structured_orders:
        key = tuple(od)
        if key not in seen_orders:
            seen_orders.add(key)
            unique_orders.append(od)

    all_initial_orders = unique_orders[:]
    for _ in range(sa_restarts):
        od = unit_ids[:]
        random.shuffle(od)
        all_initial_orders.append(od)

    for init_order in all_initial_orders:
        cur_order = init_order[:]
        cur_choices, cur_cost = _best_choices_for_unit_order(
            cur_order,
            candidates_by_unit,
            lines, R, L_out, L_in, speed,
            connector_cache
        )

        best_local_order = cur_order[:]
        best_local_choices = dict(cur_choices)
        best_local_cost = cur_cost

        T = sa_init_temp

        for _ in range(sa_steps):
            cand_order = _neighbor_unit_order(cur_order)
            cand_choices, cand_cost = _best_choices_for_unit_order(
                cand_order,
                candidates_by_unit,
                lines, R, L_out, L_in, speed,
                connector_cache
            )

            accept = False
            if cand_cost < cur_cost:
                accept = True
            else:
                if math.isfinite(cand_cost) and math.isfinite(cur_cost):
                    delta = cand_cost - cur_cost
                    if T > 1e-12 and random.random() < math.exp(-delta / T):
                        accept = True

            if accept:
                cur_order = cand_order
                cur_choices = cand_choices
                cur_cost = cand_cost

                if cur_cost < best_local_cost:
                    best_local_order = cur_order[:]
                    best_local_choices = dict(cur_choices)
                    best_local_cost = cur_cost

            T *= sa_cooling
            if T < 1e-10:
                T = 1e-10

        if best_local_cost < best_cost:
            best_cost = best_local_cost
            best_order = best_local_order[:]
            best_choices = dict(best_local_choices)

    if debug:
        print(f"[GLOBAL UNIT SA] best_cost={best_cost:.3f}")
        print(f"[GLOBAL UNIT SA] best_order={best_order}")
        print(f"[GLOBAL UNIT SA] best_choices={best_choices}")

    return best_order, best_choices, best_cost


# ============================================================
# 拼装最终结果
# ============================================================

def _assemble_final_result(order: List[int],
                           choices: Dict[int, int],
                           candidates_by_unit: Dict[int, List[UnitCandidate]],
                           lines: List[VerticalLine],
                           R: float,
                           L_out: float,
                           L_in: float,
                           speed: float) -> PlannerResult:
    final_route: List[int] = []
    final_sides: List[str] = []

    prev_cand: Optional[UnitCandidate] = None

    for pos, u in enumerate(order):
        cand = candidates_by_unit[u][choices[u]]

        if pos == 0:
            final_route.extend(cand.route)
            final_sides.extend(cand.sides)
            prev_cand = cand
            continue

        connector = _build_connector(prev_cand, cand, lines, R, L_out, L_in, speed)
        if connector is None:
            raise RuntimeError("Final assembly failed: infeasible connector")

        final_sides.append(connector.from_side)
        final_route.extend(cand.route)
        final_sides.extend(cand.sides)

        prev_cand = cand

    turn_details, total = _build_turn_details_for_global_route(
        final_route, final_sides, lines, R, L_out, L_in, speed
    )
    if not math.isfinite(total):
        raise RuntimeError("Final route reconstruction failed")

    return PlannerResult(
        route=final_route,
        sides=final_sides,
        total_distance=total,
        turn_details=turn_details,
    )


# ============================================================
# 主入口
# ============================================================

def plan_route(lines: List[VerticalLine],
               R: float,
               L_out: float,
               L_in: float,
               speed: float,
               max_heading_angle_deg: float = 0.0,
               block_top_k: int = 12,
               block_sa_restarts: int = 24,     # 兼容保留，当前版本未使用
               block_sa_steps: int = 1200,      # 兼容保留，当前版本未使用
               block_sa_init_temp: float = 1500.0,  # 兼容保留，当前版本未使用
               block_sa_cooling: float = 0.995,     # 兼容保留，当前版本未使用
               global_sa_restarts: int = 20,
               global_sa_steps: int = 2500,
               global_sa_init_temp: float = 3000.0,
               global_sa_cooling: float = 0.996,
               debug: bool = False) -> PlannerResult:
    """
    修正版论文风格规划器：

    1. 专家经验法得到原始空间块
    2. 自动细分为可配对的细分块
    3. 每两个细分块组成一个工作单元
    4. 单元内部：
       - 两块方向相反
       - 每块内部方向一致
       - 采用交错访问
    5. 单元间：
       - SA 搜单元顺序
       - DP 选最优候选链
    """
    n = len(lines)
    if n == 0:
        raise ValueError("lines is empty")
    if n == 1:
        return PlannerResult(route=[0], sides=[], total_distance=0.0, turn_details=[])

    if debug:
        print("[PLAN] build global distance cache ...")
    distance_cache = _build_global_distance_cache(lines, R, L_out, L_in, speed)

    raw_blocks, x_max_allow = partition_lines_by_expert_rule(
        lines, R, L_in,
        max_heading_angle_deg=max_heading_angle_deg,
        debug=debug
    )

    if debug:
        print(f"[PLAN] x_max_allow={x_max_allow:.3f}")

    refined_blocks = _refine_blocks_for_pair_units(
        raw_blocks, lines, debug=debug
    )

    units, candidates_by_unit = _build_work_units(
        refined_blocks,
        distance_cache,
        top_k=block_top_k,
        debug=debug
    )

    best_order, best_choices, best_cost = _anneal_global_units(
        units, candidates_by_unit,
        lines, R, L_out, L_in, speed,
        sa_restarts=global_sa_restarts,
        sa_steps=global_sa_steps,
        sa_init_temp=global_sa_init_temp,
        sa_cooling=global_sa_cooling,
        debug=debug
    )

    if not math.isfinite(best_cost):
        raise RuntimeError("Global optimization failed to find a feasible unit stitching solution")

    result = _assemble_final_result(
        best_order, best_choices,
        candidates_by_unit,
        lines, R, L_out, L_in, speed
    )

    if debug:
        print(f"[PLAN] final total_distance={result.total_distance:.3f}")
        print(f"[PLAN] final route={result.route}")
        print(f"[PLAN] final sides={result.sides}")
        print("[PLAN] chosen unit candidates:")
        for u in best_order:
            cand = candidates_by_unit[u][best_choices[u]]
            print(
                f"  unit {u}: blocks={cand.block_ids}, dirs={cand.block_directions}, "
                f"start_entry={cand.start_entry_side}, end_exit={cand.end_exit_side}, "
                f"dist={cand.total_distance:.3f}"
            )

    return result


# ============================================================
# 随机路线对比
# ============================================================

def _compute_route_distance(route: List[int],
                            sides: List[str],
                            lines: List[VerticalLine],
                            R: float,
                            L_out: float,
                            L_in: float,
                            speed: float) -> float:
    total = 0.0
    for k in range(len(route) - 1):
        d = compute_turn_distance(
            lines[route[k]], lines[route[k + 1]],
            sides[k], R, L_out, L_in, speed
        )
        if not math.isfinite(d):
            return float("inf")
        total += d
    return total


def random_route_distance(lines: List[VerticalLine],
                          R: float,
                          L_out: float,
                          L_in: float,
                          speed: float,
                          n_trials: int = 100) -> float:
    n = len(lines)
    if n <= 1:
        return 0.0

    total = 0.0
    valid_cnt = 0

    for _ in range(n_trials):
        route = list(range(n))
        random.shuffle(route)

        start_entry_side = random.choice(["AB", "CD"])
        sides = _build_sides_from_start_entry(len(route), start_entry_side)

        dist = _compute_route_distance(route, sides, lines, R, L_out, L_in, speed)
        if math.isfinite(dist):
            total += dist
            valid_cnt += 1

    if valid_cnt == 0:
        return float("inf")

    return total / valid_cnt