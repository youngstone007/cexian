use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;

use crate::common::*;
use crate::dubins_csc::{solve_fixed_head_tail_turn_between_vertical_lines, DubinsPath, TurnCandidate};
use crate::semicircle::{solve_fixed_head_tail_w_eq_2r_semicircle_turn, SemicircleTurnCandidate, SemicircleTurnPath};
use crate::three_arc::{solve_fixed_head_tail_w_lt_2r_three_arc_turn, ThreeArcTurnCandidate, ThreeArcTurnPath};

// ============================================================
// 数据结构
// ============================================================

/// 单段换线详情
#[derive(Debug, Clone)]
pub struct TurnDetail {
    pub from_line: usize,
    pub to_line: usize,
    pub from_side: String,      // "AB" / "CD"
    pub distance: f64,
    pub scenario: String,       // "W>2R" / "W=2R" / "W<2R"
    pub path_xs: Vec<f64>,
    pub path_ys: Vec<f64>,
}

/// 整体规划结果
#[derive(Debug, Clone)]
pub struct PlannerResult {
    pub route: Vec<usize>,
    pub sides: Vec<String>,
    pub total_distance: f64,
    pub turn_details: Vec<TurnDetail>,
}

/// 空间块 / 细分块
#[derive(Debug, Clone)]
pub struct SurveyBlock {
    pub block_id: usize,
    pub global_indices: Vec<usize>,   // 按 x 从小到大
    pub x_min: f64,
    pub x_max: f64,
    pub source_block_ids: Vec<usize>,
}

impl SurveyBlock {
    pub fn size(&self) -> usize {
        self.global_indices.len()
    }

    pub fn width(&self) -> f64 {
        self.x_max - self.x_min
    }
}

/// 工作单元：由两个相邻细分块组成
#[derive(Debug, Clone)]
pub struct WorkUnit {
    pub unit_id: usize,
    pub block_ids: Vec<usize>,
}

/// 单个工作单元的一条候选路线
#[derive(Debug, Clone)]
pub struct UnitCandidate {
    pub unit_id: usize,
    pub block_ids: Vec<usize>,
    pub route: Vec<usize>,
    pub sides: Vec<String>,
    pub total_distance: f64,
    pub start_line: usize,
    pub end_line: usize,
    pub start_entry_side: String,
    pub end_exit_side: String,
    pub block_directions: HashMap<usize, String>,   // {block_id: "UP"/"DOWN"}
}

/// 统一换线结果
#[derive(Debug, Clone)]
pub enum TurnResult {
    Dubins(TurnCandidate),
    Semicircle(SemicircleTurnCandidate),
    ThreeArc(ThreeArcTurnCandidate),
}

// ============================================================
// 基础辅助
// ============================================================

pub fn opposite_side(side: &str) -> String {
    match side {
        "AB" => "CD".to_string(),
        "CD" => "AB".to_string(),
        _ => panic!("Unknown side: {}", side),
    }
}

pub fn opposite_direction(direction: &str) -> String {
    match direction {
        "UP" => "DOWN".to_string(),
        "DOWN" => "UP".to_string(),
        _ => panic!("Unknown direction: {}", direction),
    }
}

pub fn direction_to_entry_side(direction: &str) -> String {
    match direction {
        "UP" => "AB".to_string(),
        "DOWN" => "CD".to_string(),
        _ => panic!("Unknown direction: {}", direction),
    }
}

pub fn direction_to_exit_side(direction: &str) -> String {
    match direction {
        "UP" => "CD".to_string(),
        "DOWN" => "AB".to_string(),
        _ => panic!("Unknown direction: {}", direction),
    }
}

pub fn build_sides_from_start_entry(route_len: usize, start_entry_side: &str) -> Vec<String> {
    if route_len <= 1 {
        return vec![];
    }

    let mut sides = Vec::with_capacity(route_len - 1);
    let mut cur = opposite_side(start_entry_side);
    for _ in 0..route_len - 1 {
        sides.push(cur.clone());
        cur = opposite_side(&cur);
    }
    sides
}

// ============================================================
// 局部换线求解统一入口
// ============================================================

pub fn solve_turn(
    line1: &VerticalLine,
    line2: &VerticalLine,
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> (Option<TurnResult>, String) {
    let w = (line2.x - line1.x).abs();
    let tol = 1e-6;

    if w > 2.0 * r + tol {
        let result = solve_fixed_head_tail_turn_between_vertical_lines(line1, line2, l_out, l_in, r, speed, false);
        (result.map(TurnResult::Dubins), "W>2R".to_string())
    } else if (w - 2.0 * r).abs() <= tol {
        let result = solve_fixed_head_tail_w_eq_2r_semicircle_turn(line1, line2, l_out, l_in, r, speed, tol, false);
        (result.map(TurnResult::Semicircle), "W=2R".to_string())
    } else {
        let result = solve_fixed_head_tail_w_lt_2r_three_arc_turn(line1, line2, l_out, l_in, r, speed, tol, false);
        (result.map(TurnResult::ThreeArc), "W<2R".to_string())
    }
}

pub fn extract_total_length(result: &TurnResult) -> f64 {
    match result {
        TurnResult::Dubins(tc) => tc.dubins.total_length,
        TurnResult::Semicircle(sc) => sc.semi.total_length,
        TurnResult::ThreeArc(tac) => tac.three_arc.total_length,
    }
}

pub fn extract_path_coords(result: &TurnResult, r: f64) -> (Vec<f64>, Vec<f64>) {
    let mut segments: Vec<(Vec<f64>, Vec<f64>)> = Vec::new();

    let (eol_data, sol_data, eol_geom, sol_geom) = match result {
        TurnResult::Dubins(tc) => (
            tc.eol_data_point,
            tc.sol_data_point,
            tc.eol_geom.clone(),
            tc.sol_geom.clone(),
        ),
        TurnResult::Semicircle(sc) => (
            sc.eol_data_point,
            sc.sol_data_point,
            sc.eol_geom.clone(),
            sc.sol_geom.clone(),
        ),
        TurnResult::ThreeArc(tac) => (
            tac.eol_data_point,
            tac.sol_data_point,
            tac.eol_geom.clone(),
            tac.sol_geom.clone(),
        ),
    };

    match result {
        TurnResult::Dubins(tc) => {
            let d = &tc.dubins;
            let turn1 = &d.path_type[0..1];
            let turn2 = &d.path_type[2..3];

            segments.push(sample_line_default(eol_data, eol_geom.p));
            if let (Some(c1), Some(t1), Some(t2), Some(c2)) =
                (d.center1, d.tangent_point1, d.tangent_point2, d.center2) {
                segments.push(sample_arc_default(c1, r, eol_geom.p, t1, turn1));
                segments.push(sample_line_default(t1, t2));
                segments.push(sample_arc_default(c2, r, t2, sol_geom.p, turn2));
            }
            segments.push(sample_line_default(sol_geom.p, sol_data));
        }
        TurnResult::Semicircle(sc) => {
            let s = &sc.semi;

            segments.push(sample_line_default(eol_data, eol_geom.p));
            if let (Some(start), Some(semi_start), Some(center), Some(semi_end), Some(end)) =
                (s.start_point, s.semicircle_start, s.center, s.semicircle_end, s.end_point) {
                if s.pre_straight_length > EPS {
                    segments.push(sample_line_default(start, semi_start));
                }
                segments.push(sample_arc_default(center, r, semi_start, semi_end, &s.turn_direction));
                if s.post_straight_length > EPS {
                    segments.push(sample_line_default(semi_end, end));
                }
            }
            segments.push(sample_line_default(sol_geom.p, sol_data));
        }
        TurnResult::ThreeArc(tac) => {
            let t = &tac.three_arc;
            let (turn1, turn2, turn3) = if t.turn_sequence == "LRL" {
                ("L", "R", "L")
            } else {
                ("R", "L", "R")
            };

            segments.push(sample_line_default(eol_data, eol_geom.p));
            if let (Some(start), Some(turn_start), Some(c1), Some(j12), Some(c2), Some(j23), Some(c3), Some(turn_end), Some(end)) =
                (t.start_point, t.turn_start_point, t.center1, t.join12, t.center2, t.join23, t.center3, t.turn_end_point, t.end_point) {
                if t.pre_straight_length > EPS {
                    segments.push(sample_line_default(start, turn_start));
                }
                segments.push(sample_arc_default(c1, r, turn_start, j12, turn1));
                segments.push(sample_arc_default(c2, r, j12, j23, turn2));
                segments.push(sample_arc_default(c3, r, j23, turn_end, turn3));
                if t.post_straight_length > EPS {
                    segments.push(sample_line_default(turn_end, end));
                }
            }
            segments.push(sample_line_default(sol_geom.p, sol_data));
        }
    }

    let mut all_xs = Vec::new();
    let mut all_ys = Vec::new();
    for (xs, ys) in segments {
        if !all_xs.is_empty() {
            all_xs.extend(xs.into_iter().skip(1));
            all_ys.extend(ys.into_iter().skip(1));
        } else {
            all_xs.extend(xs);
            all_ys.extend(ys);
        }
    }

    (all_xs, all_ys)
}

pub fn compute_turn_with_path(
    line1: &VerticalLine,
    line2: &VerticalLine,
    from_side: &str,
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> (f64, String, Vec<f64>, Vec<f64>) {
    let (solve_line1, solve_line2) = if from_side == "CD" {
        let tmp_line2 = VerticalLine::new(line2.x, line2.y_end, line2.y_start);
        (line1.clone(), tmp_line2)
    } else {
        let tmp_line1 = VerticalLine::new(line1.x, line1.y_end, line1.y_start);
        (tmp_line1, line2.clone())
    };

    let (result, scenario) = solve_turn(&solve_line1, &solve_line2, r, l_out, l_in, speed);

    match result {
        Some(turn_result) => {
            let dist = extract_total_length(&turn_result);
            let (path_xs, path_ys) = extract_path_coords(&turn_result, r);
            (dist, scenario, path_xs, path_ys)
        }
        None => (f64::INFINITY, scenario, vec![], vec![]),
    }
}

pub fn compute_turn_distance(
    line1: &VerticalLine,
    line2: &VerticalLine,
    from_side: &str,
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> f64 {
    let (dist, _, _, _) = compute_turn_with_path(line1, line2, from_side, r, l_out, l_in, speed);
    dist
}

pub fn compute_turn_distance_with_detail(
    line1: &VerticalLine,
    line2: &VerticalLine,
    from_side: &str,
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> (f64, String) {
    let (solve_line1, solve_line2) = if from_side == "CD" {
        let tmp_line2 = VerticalLine::new(line2.x, line2.y_end, line2.y_start);
        (line1.clone(), tmp_line2)
    } else {
        let tmp_line1 = VerticalLine::new(line1.x, line1.y_end, line1.y_start);
        (tmp_line1, line2.clone())
    };

    let (result, scenario) = solve_turn(&solve_line1, &solve_line2, r, l_out, l_in, speed);

    match result {
        Some(r) => (extract_total_length(&r), scenario),
        None => (f64::INFINITY, scenario),
    }
}

// ============================================================
// 全局距离缓存
// ============================================================

pub type DistanceCache = HashMap<(usize, usize, String), f64>;

pub fn build_global_distance_cache(
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> DistanceCache {
    let n = lines.len();
    let mut cache = DistanceCache::new();

    for i in 0..n {
        for j in 0..n {
            if i == j {
                continue;
            }
            let d_ab = compute_turn_distance_with_detail(&lines[i], &lines[j], "AB", r, l_out, l_in, speed).0;
            let d_cd = compute_turn_distance_with_detail(&lines[i], &lines[j], "CD", r, l_out, l_in, speed).0;
            cache.insert((i, j, "AB".to_string()), d_ab);
            cache.insert((i, j, "CD".to_string()), d_cd);
        }
    }

    cache
}

pub fn turn_distance_cached(cache: &DistanceCache, i: usize, j: usize, from_side: &str) -> f64 {
    *cache.get(&(i, j, from_side.to_string())).unwrap_or(&f64::INFINITY)
}

// ============================================================
// 专家经验法：工区分块
// ============================================================

pub fn compute_block_max_span(r: f64, _l_in: f64, max_heading_angle_deg: f64) -> f64 {
    let alpha_deg = max_heading_angle_deg.max(0.0).min(90.0);
    let alpha = alpha_deg * PI / 180.0;
    let x_max = r * (1.0 + alpha.cos());
    x_max.max(0.0)
}

pub fn partition_lines_by_expert_rule(
    lines: &[VerticalLine],
    r: f64,
    l_in: f64,
    max_heading_angle_deg: f64,
    tol: f64,
    debug: bool,
) -> (Vec<SurveyBlock>, f64) {
    if lines.is_empty() {
        return (vec![], 0.0);
    }

    let x_max_allow = compute_block_max_span(r, l_in, max_heading_angle_deg);

    let mut sorted_indices: Vec<usize> = (0..lines.len()).collect();
    sorted_indices.sort_by(|&a, &b| lines[a].x.partial_cmp(&lines[b].x).unwrap());

    let mut blocks: Vec<SurveyBlock> = Vec::new();
    let mut current = vec![sorted_indices[0]];
    let mut current_x_min = lines[sorted_indices[0]].x;
    let mut block_id = 0;

    for &idx in sorted_indices.iter().skip(1) {
        let span = lines[idx].x - current_x_min;
        if span <= x_max_allow + tol {
            current.push(idx);
        } else {
            let xs: Vec<f64> = current.iter().map(|&g| lines[g].x).collect();
            blocks.push(SurveyBlock {
                block_id,
                global_indices: current.clone(),
                x_min: xs.iter().cloned().fold(f64::INFINITY, f64::min),
                x_max: xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
                source_block_ids: vec![block_id],
            });
            block_id += 1;
            current = vec![idx];
            current_x_min = lines[idx].x;
        }
    }

    let xs: Vec<f64> = current.iter().map(|&g| lines[g].x).collect();
    blocks.push(SurveyBlock {
        block_id,
        global_indices: current,
        x_min: xs.iter().cloned().fold(f64::INFINITY, f64::min),
        x_max: xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
        source_block_ids: vec![block_id],
    });

    if debug {
        println!("[BLOCK] alpha={:.3} deg", max_heading_angle_deg);
        println!("[BLOCK] x_max_allow={:.3}", x_max_allow);
        println!("[BLOCK] num_blocks={}", blocks.len());
        for b in &blocks {
            println!(
                "  raw spatial block {}: size={}, width={:.3}, lines={:?}",
                b.block_id, b.size(), b.width(), b.global_indices
            );
        }
    }

    (blocks, x_max_allow)
}

// ============================================================
// 可行性细分：把原始空间块切成可配对的细分块
// ============================================================

fn split_block_by_sizes(
    block: &SurveyBlock,
    sizes: &[usize],
    new_block_id_start: usize,
    lines: &[VerticalLine],
) -> (Vec<SurveyBlock>, usize) {
    let mut res: Vec<SurveyBlock> = Vec::new();
    let mut pos = 0;
    let mut cur_id = new_block_id_start;

    for &sz in sizes {
        let part = block.global_indices[pos..pos + sz].to_vec();
        let xs: Vec<f64> = part.iter().map(|&g| lines[g].x).collect();
        res.push(SurveyBlock {
            block_id: cur_id,
            global_indices: part,
            x_min: xs.iter().cloned().fold(f64::INFINITY, f64::min),
            x_max: xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
            source_block_ids: block.source_block_ids.clone(),
        });
        cur_id += 1;
        pos += sz;
    }

    if pos != block.global_indices.len() {
        panic!("Block split size mismatch");
    }

    (res, cur_id)
}

/// DP 状态：(split_cnt, diff_cost) - 字典序最小
type DpState = Option<(i32, i32)>;

fn dp_refine(
    memo: &mut HashMap<(usize, i32), DpState>,
    sizes: &[usize],
    i: usize,
    pending: i32,
) -> DpState {
    if let Some(&cached) = memo.get(&(i, pending)) {
        return cached;
    }

    let n = sizes.len();
    if i == n {
        if pending == -1 {
            return Some((0, 0));
        }
        return None;
    }

    let mut best: DpState = None;

    let s = sizes[i];

    // 方案1：不切分
    let mut pieces_options: Vec<Vec<usize>> = vec![vec![s]];

    // 方案2：切成两段
    for a in 1..s {
        let b = s - a;
        pieces_options.push(vec![a, b]);
    }

    for pieces in pieces_options {
        let split_cnt_here = if pieces.len() == 1 { 0 } else { 1 };
        let mut cur_pending = pending;
        let mut diff_cost_here = 0;
        let mut feasible = true;

        for &piece in &pieces {
            if cur_pending == -1 {
                cur_pending = piece as i32;
            } else {
                if (cur_pending - piece as i32).abs() > 1 {
                    feasible = false;
                    break;
                }
                diff_cost_here += (cur_pending - piece as i32).abs();
                cur_pending = -1;
            }
        }

        if !feasible {
            continue;
        }

        let future = dp_refine(memo, sizes, i + 1, cur_pending);
        if let Some((f_split, f_diff)) = future {
            let cand = (f_split + split_cnt_here, f_diff + diff_cost_here);
            if best.is_none() || cand < best.unwrap() {
                best = Some(cand);
            }
        }
    }

    memo.insert((i, pending), best);
    best
}

pub fn refine_blocks_for_pair_units(
    raw_blocks: &[SurveyBlock],
    lines: &[VerticalLine],
    debug: bool,
) -> Vec<SurveyBlock> {
    if raw_blocks.is_empty() {
        return vec![];
    }

    let sizes: Vec<usize> = raw_blocks.iter().map(|b| b.size()).collect();
    let n = raw_blocks.len();

    let mut memo: HashMap<(usize, i32), DpState> = HashMap::new();
    let best0 = dp_refine(&mut memo, &sizes, 0, -1);

    if best0.is_none() {
        panic!("Raw spatial blocks cannot be refined into feasible same-direction pair units");
    }

    let mut refined: Vec<SurveyBlock> = Vec::new();
    let mut pending = -1;
    let mut next_block_id = 0;

    for (i, block) in raw_blocks.iter().enumerate() {
        let s = sizes[i];
        let mut chosen_pieces: Option<Vec<usize>> = None;

        let mut pieces_options: Vec<Vec<usize>> = vec![vec![s]];
        for a in 1..s {
            let b = s - a;
            pieces_options.push(vec![a, b]);
        }

        for pieces in pieces_options {
            let split_cnt_here = if pieces.len() == 1 { 0 } else { 1 };
            let mut cur_pending = pending;
            let mut diff_cost_here = 0;
            let mut feasible = true;

            for &piece in &pieces {
                if cur_pending == -1 {
                    cur_pending = piece as i32;
                } else {
                    if (cur_pending - piece as i32).abs() > 1 {
                        feasible = false;
                        break;
                    }
                    diff_cost_here += (cur_pending - piece as i32).abs();
                    cur_pending = -1;
                }
            }

            if !feasible {
                continue;
            }

            let future = dp_refine(&mut memo, &sizes, i + 1, cur_pending);
            if let Some((f_split, f_diff)) = future {
                let total = (f_split + split_cnt_here, f_diff + diff_cost_here);
                if total == best0.unwrap() {
                    chosen_pieces = Some(pieces);
                    pending = cur_pending;
                    break;
                }
            }
        }

        let chosen = chosen_pieces.expect("Refined block reconstruction failed");
        let (parts, new_id) = split_block_by_sizes(block, &chosen, next_block_id, lines);
        next_block_id = new_id;
        refined.extend(parts);
    }

    if pending != -1 {
        panic!("Refined block pairing reconstruction ended with unmatched block");
    }

    if refined.len() % 2 != 0 {
        panic!("Refined block count should be even but is not");
    }

    if debug {
        println!("[REFINE] raw block sizes = {:?}", sizes);
        println!("[REFINE] refined block sizes = {:?}", refined.iter().map(|b| b.size()).collect::<Vec<_>>());
        for b in &refined {
            println!(
                "  refined block {}: size={}, width={:.3}, src={:?}, lines={:?}",
                b.block_id, b.size(), b.width(), b.source_block_ids, b.global_indices
            );
        }
    }

    refined
}

// ============================================================
// 单元内部：2块交错访问
// ============================================================

fn generate_block_internal_orders(block: &SurveyBlock) -> Vec<Vec<usize>> {
    let asc = block.global_indices.clone();
    let mut desc = asc.clone();
    desc.reverse();
    if asc == desc {
        vec![asc]
    } else {
        vec![asc, desc]
    }
}

/// DP 状态：(i, j, last) -> 最小代价
/// last: -1=起点, 0=上一步来自A, 1=上一步来自B
type InterleaveCache = HashMap<(usize, usize, i32), f64>;

fn dp_interleave(
    cache: &mut InterleaveCache,
    order_a: &[usize],
    order_b: &[usize],
    na: usize,
    nb: usize,
    i: usize,
    j: usize,
    last: i32,
    side_a: &str,
    side_b: &str,
    distance_cache: &DistanceCache,
) -> f64 {
    if let Some(&cached) = cache.get(&(i, j, last)) {
        return cached;
    }

    if i == na && j == nb {
        cache.insert((i, j, last), 0.0);
        return 0.0;
    }

    let mut best = f64::INFINITY;

    // 从 A 取
    if last == -1 || last == 1 {
        if i < na {
            let step = if last == 1 {
                let prev = order_b[j - 1];
                let nxt = order_a[i];
                turn_distance_cached(distance_cache, prev, nxt, side_b)
            } else {
                0.0
            };
            let future = dp_interleave(cache, order_a, order_b, na, nb, i + 1, j, 0, side_a, side_b, distance_cache);
            if step.is_finite() && future.is_finite() {
                best = best.min(step + future);
            }
        }
    }

    // 从 B 取
    if last == -1 || last == 0 {
        if j < nb {
            let step = if last == 0 {
                let prev = order_a[i - 1];
                let nxt = order_b[j];
                turn_distance_cached(distance_cache, prev, nxt, side_a)
            } else {
                0.0
            };
            let future = dp_interleave(cache, order_a, order_b, na, nb, i, j + 1, 1, side_a, side_b, distance_cache);
            if step.is_finite() && future.is_finite() {
                best = best.min(step + future);
            }
        }
    }

    cache.insert((i, j, last), best);
    best
}

fn solve_pair_unit_interleaving(
    block_a: &SurveyBlock,
    dir_a: &str,
    order_a: &[usize],
    block_b: &SurveyBlock,
    dir_b: &str,
    order_b: &[usize],
    distance_cache: &DistanceCache,
) -> Option<(Vec<usize>, Vec<String>, f64)> {
    let na = order_a.len();
    let nb = order_b.len();

    if (na as i32 - nb as i32).abs() > 1 {
        return None;
    }

    let side_a = direction_to_exit_side(dir_a);
    let side_b = direction_to_exit_side(dir_b);

    let mut cache: InterleaveCache = HashMap::new();
    let best_cost = dp_interleave(
        &mut cache, order_a, order_b, na, nb, 0, 0, -1,
        &side_a, &side_b, distance_cache
    );

    if !best_cost.is_finite() {
        return None;
    }

    // 回溯构建路线
    let mut route: Vec<usize> = Vec::new();
    let mut sides: Vec<String> = Vec::new();
    let mut i = 0;
    let mut j = 0;
    let mut last = -1;

    while i != na || j != nb {
        let cur = cache.get(&(i, j, last)).copied().unwrap_or(f64::INFINITY);

        let mut moved = false;

        // 尝试从 A 取
        if (last == -1 || last == 1) && i < na {
            let step = if last == 1 {
                let prev = order_b[j - 1];
                let nxt = order_a[i];
                turn_distance_cached(distance_cache, prev, nxt, &side_b)
            } else {
                0.0
            };
            let future = cache.get(&(i + 1, j, 0)).copied().unwrap_or(f64::INFINITY);
            if step.is_finite() && future.is_finite() && (cur - (step + future)).abs() < 1e-9 {
                if last == 1 {
                    sides.push(side_b.clone());
                }
                route.push(order_a[i]);
                i += 1;
                last = 0;
                moved = true;
            }
        }

        if moved {
            continue;
        }

        // 尝试从 B 取
        if (last == -1 || last == 0) && j < nb {
            let step = if last == 0 {
                let prev = order_a[i - 1];
                let nxt = order_b[j];
                turn_distance_cached(distance_cache, prev, nxt, &side_a)
            } else {
                0.0
            };
            let future = cache.get(&(i, j + 1, 1)).copied().unwrap_or(f64::INFINITY);
            if step.is_finite() && future.is_finite() && (cur - (step + future)).abs() < 1e-9 {
                if last == 0 {
                    sides.push(side_a.clone());
                }
                route.push(order_b[j]);
                j += 1;
                last = 1;
                moved = true;
            }
        }

        if !moved {
            return None;
        }
    }

    Some((route, sides, best_cost))
}

fn generate_pair_unit_candidates(
    block_a: &SurveyBlock,
    block_b: &SurveyBlock,
    distance_cache: &DistanceCache,
    top_k: usize,
    debug: bool,
) -> Vec<UnitCandidate> {
    let mut candidates: Vec<UnitCandidate> = Vec::new();

    let orders_a = generate_block_internal_orders(block_a);
    let orders_b = generate_block_internal_orders(block_b);

    for first_dir in ["UP", "DOWN"] {
        let dir_a = first_dir;
        let dir_b = opposite_direction(first_dir);

        for order_a in &orders_a {
            for order_b in &orders_b {
                let solved = solve_pair_unit_interleaving(
                    block_a, dir_a, order_a,
                    block_b, &dir_b, order_b,
                    distance_cache
                );

                if let Some((route, sides, total_distance)) = solved {
                    if route.is_empty() {
                        continue;
                    }

                    let mut line_to_block: HashMap<usize, usize> = HashMap::new();
                    for &g in order_a {
                        line_to_block.insert(g, block_a.block_id);
                    }
                    for &g in order_b {
                        line_to_block.insert(g, block_b.block_id);
                    }

                    let first_line = route[0];
                    let last_line = *route.last().unwrap();
                    let first_block_id = line_to_block[&first_line];
                    let last_block_id = line_to_block[&last_line];

                    let mut block_directions = HashMap::new();
                    block_directions.insert(block_a.block_id, dir_a.to_string());
                    block_directions.insert(block_b.block_id, dir_b.to_string());

                    let start_entry_side = direction_to_entry_side(&block_directions[&first_block_id]);
                    let end_exit_side = direction_to_exit_side(&block_directions[&last_block_id]);

                    candidates.push(UnitCandidate {
                        unit_id: usize::MAX,
                        block_ids: vec![block_a.block_id, block_b.block_id],
                        route,
                        sides,
                        total_distance,
                        start_line: first_line,
                        end_line: last_line,
                        start_entry_side,
                        end_exit_side,
                        block_directions,
                    });
                }
            }
        }
    }

    if candidates.is_empty() {
        return vec![];
    }

    // 去重
    let mut uniq: HashMap<(Vec<usize>, String), UnitCandidate> = HashMap::new();
    for cand in candidates {
        let key = (cand.route.clone(), cand.start_entry_side.clone());
        if let Some(existing) = uniq.get_mut(&key) {
            if cand.total_distance < existing.total_distance {
                *existing = cand;
            }
        } else {
            uniq.insert(key, cand);
        }
    }

    let mut result: Vec<UnitCandidate> = uniq.into_values().collect();
    result.sort_by(|a, b| a.total_distance.partial_cmp(&b.total_distance).unwrap());
    result.truncate(top_k.max(1));

    if debug {
        println!("[UNIT GEN] refined blocks={:?}, candidates={}", [block_a.block_id, block_b.block_id], result.len());
        for (i, c) in result.iter().enumerate() {
            println!(
                "  cand {}: route={:?}, start_entry={}, end_exit={}, dirs={:?}, dist={:.3}",
                i, c.route, c.start_entry_side, c.end_exit_side, c.block_directions, c.total_distance
            );
        }
    }

    result
}

// ============================================================
// 构建工作单元
// ============================================================

pub fn build_work_units(
    refined_blocks: &[SurveyBlock],
    distance_cache: &DistanceCache,
    top_k: usize,
    debug: bool,
) -> (Vec<WorkUnit>, HashMap<usize, Vec<UnitCandidate>>) {
    if refined_blocks.len() % 2 != 0 {
        panic!("Refined block count must be even");
    }

    let mut units: Vec<WorkUnit> = Vec::new();
    let mut candidates_by_unit: HashMap<usize, Vec<UnitCandidate>> = HashMap::new();

    let mut unit_id = 0;
    for i in (0..refined_blocks.len()).step_by(2) {
        let a = &refined_blocks[i];
        let b = &refined_blocks[i + 1];

        let unit = WorkUnit {
            unit_id,
            block_ids: vec![a.block_id, b.block_id],
        };
        units.push(unit);

        let mut cands = generate_pair_unit_candidates(a, b, distance_cache, top_k, debug);
        if cands.is_empty() {
            panic!("Failed to generate feasible candidates for refined block pair ({}, {})", a.block_id, b.block_id);
        }

        for c in &mut cands {
            c.unit_id = unit_id;
        }

        candidates_by_unit.insert(unit_id, cands);
        unit_id += 1;
    }

    if debug {
        println!("[UNIT] num_units={}", units.len());
        for u in &units {
            println!("  unit {}: refined_blocks={:?}", u.unit_id, u.block_ids);
        }
    }

    (units, candidates_by_unit)
}

// ============================================================
// 单元间拼接 / 全局优化
// ============================================================

fn build_connector(
    prev_cand: &UnitCandidate,
    next_cand: &UnitCandidate,
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> Option<TurnDetail> {
    if prev_cand.end_exit_side != next_cand.start_entry_side {
        return None;
    }

    let from_side = &prev_cand.end_exit_side;
    let (dist, scenario, path_xs, path_ys) = compute_turn_with_path(
        &lines[prev_cand.end_line],
        &lines[next_cand.start_line],
        from_side,
        r, l_out, l_in, speed
    );

    if !dist.is_finite() {
        return None;
    }

    Some(TurnDetail {
        from_line: prev_cand.end_line,
        to_line: next_cand.start_line,
        from_side: from_side.clone(),
        distance: dist,
        scenario,
        path_xs,
        path_ys,
    })
}

fn center_out_unit_order(num_units: usize, start_unit: usize) -> Vec<usize> {
    let mut order = vec![start_unit];
    let mut left = if start_unit > 0 { Some(start_unit - 1) } else { None };
    let mut right = if start_unit + 1 < num_units { Some(start_unit + 1) } else { None };
    let mut toggle = true;

    while left.is_some() || right.is_some() {
        if toggle {
            if let Some(r) = right {
                order.push(r);
                right = if r + 1 < num_units { Some(r + 1) } else { None };
            } else if let Some(l) = left {
                order.push(l);
                left = if l > 0 { Some(l - 1) } else { None };
            }
        } else {
            if let Some(l) = left {
                order.push(l);
                left = if l > 0 { Some(l - 1) } else { None };
            } else if let Some(r) = right {
                order.push(r);
                right = if r + 1 < num_units { Some(r + 1) } else { None };
            }
        }
        toggle = !toggle;
    }

    order
}

type ConnectorCache = HashMap<(usize, usize, usize, usize), Option<TurnDetail>>;

fn best_choices_for_unit_order(
    order: &[usize],
    candidates_by_unit: &HashMap<usize, Vec<UnitCandidate>>,
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
    connector_cache: &mut ConnectorCache,
) -> (HashMap<usize, usize>, f64) {
    if order.is_empty() {
        return (HashMap::new(), 0.0);
    }

    let ordered_lists: Vec<&Vec<UnitCandidate>> = order.iter().map(|u| &candidates_by_unit[u]).collect();

    let mut dp_rows: Vec<Vec<f64>> = Vec::new();
    let mut parent: Vec<Vec<Option<usize>>> = Vec::new();

    let first_list = ordered_lists[0];
    dp_rows.push(first_list.iter().map(|cand| cand.total_distance).collect());
    parent.push(vec![None; first_list.len()]);

    for pos in 1..order.len() {
        let prev_unit_id = order[pos - 1];
        let curr_unit_id = order[pos];

        let prev_list = ordered_lists[pos - 1];
        let curr_list = ordered_lists[pos];

        let mut cur_dp = vec![f64::INFINITY; curr_list.len()];
        let mut cur_parent = vec![None; curr_list.len()];

        for (j, curr_cand) in curr_list.iter().enumerate() {
            for (i, prev_cand) in prev_list.iter().enumerate() {
                if !dp_rows[pos - 1][i].is_finite() {
                    continue;
                }

                let key = (prev_unit_id, i, curr_unit_id, j);
                if !connector_cache.contains_key(&key) {
                    connector_cache.insert(
                        key,
                        build_connector(prev_cand, curr_cand, lines, r, l_out, l_in, speed),
                    );
                }

                let connector = &connector_cache[&key];
                if let Some(conn) = connector {
                    let total = dp_rows[pos - 1][i] + conn.distance + curr_cand.total_distance;
                    if total < cur_dp[j] {
                        cur_dp[j] = total;
                        cur_parent[j] = Some(i);
                    }
                }
            }
        }

        dp_rows.push(cur_dp);
        parent.push(cur_parent);
    }

    let last_row = &dp_rows[dp_rows.len() - 1];
    let finite_indices: Vec<usize> = last_row.iter().enumerate()
        .filter(|(_, &v)| v.is_finite())
        .map(|(i, _)| i)
        .collect();

    if finite_indices.is_empty() {
        return (HashMap::new(), f64::INFINITY);
    }

    let best_last_idx = finite_indices.into_iter()
        .min_by(|&a, &b| last_row[a].partial_cmp(&last_row[b]).unwrap())
        .unwrap();
    let best_cost = last_row[best_last_idx];

    let mut chosen: Vec<Option<usize>> = vec![None; order.len()];
    chosen[order.len() - 1] = Some(best_last_idx);

    for pos in (1..order.len()).rev() {
        let pi = parent[pos][chosen[pos].unwrap()].unwrap();
        chosen[pos - 1] = Some(pi);
    }

    let choices: HashMap<usize, usize> = order.iter().enumerate()
        .filter_map(|(pos, &u)| chosen[pos].map(|c| (u, c)))
        .collect();

    (choices, best_cost)
}

fn neighbor_unit_order(order: &[usize]) -> Vec<usize> {
    use rand::Rng;
    let mut new_order = order.to_vec();
    if new_order.len() <= 1 {
        return new_order;
    }

    let mut rng = rand::thread_rng();
    let move_type = rng.gen_range(0..3);

    match move_type {
        0 => {
            // swap
            let i = rng.gen_range(0..new_order.len());
            let j = rng.gen_range(0..new_order.len());
            new_order.swap(i, j);
        }
        1 => {
            // reverse
            let i = rng.gen_range(0..new_order.len());
            let j = rng.gen_range(0..new_order.len());
            let (lo, hi) = if i < j { (i, j) } else { (j, i) };
            new_order[lo..=hi].reverse();
        }
        _ => {
            // move
            let i = rng.gen_range(0..new_order.len());
            let j = rng.gen_range(0..new_order.len());
            let u = new_order.remove(i);
            new_order.insert(j, u);
        }
    }

    new_order
}

pub fn anneal_global_units(
    units: &[WorkUnit],
    candidates_by_unit: &HashMap<usize, Vec<UnitCandidate>>,
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
    sa_restarts: usize,
    sa_steps: usize,
    sa_init_temp: f64,
    sa_cooling: f64,
    debug: bool,
) -> (Vec<usize>, HashMap<usize, usize>, f64) {
    use rand::seq::SliceRandom;

    let num_units = units.len();
    let unit_ids: Vec<usize> = units.iter().map(|u| u.unit_id).collect();
    let mut connector_cache: ConnectorCache = HashMap::new();

    let mut best_order = unit_ids.clone();
    let mut best_choices: HashMap<usize, usize> = HashMap::new();
    let mut best_cost = f64::INFINITY;

    // 结构化初始顺序
    let mut structured_orders: Vec<Vec<usize>> = Vec::new();
    structured_orders.push(unit_ids.clone());
    structured_orders.push(unit_ids.iter().rev().cloned().collect());
    for s in 0..num_units {
        structured_orders.push(center_out_unit_order(num_units, s));
    }

    // 去重
    let mut unique_orders: Vec<Vec<usize>> = Vec::new();
    let mut seen_orders: HashSet<Vec<usize>> = HashSet::new();
    for od in structured_orders {
        if !seen_orders.contains(&od) {
            seen_orders.insert(od.clone());
            unique_orders.push(od);
        }
    }

    let mut all_initial_orders = unique_orders;
    for _ in 0..sa_restarts {
        let mut od = unit_ids.clone();
        od.shuffle(&mut rand::thread_rng());
        all_initial_orders.push(od);
    }

    for init_order in all_initial_orders {
        let cur_order = init_order;
        let (cur_choices, cur_cost) = best_choices_for_unit_order(
            &cur_order,
            candidates_by_unit,
            lines, r, l_out, l_in, speed,
            &mut connector_cache
        );

        let mut best_local_order = cur_order.clone();
        let mut best_local_choices = cur_choices.clone();
        let mut best_local_cost = cur_cost;

        let mut t = sa_init_temp;

        for _ in 0..sa_steps {
            let cand_order = neighbor_unit_order(&cur_order);
            let (cand_choices, cand_cost) = best_choices_for_unit_order(
                &cand_order,
                candidates_by_unit,
                lines, r, l_out, l_in, speed,
                &mut connector_cache
            );

            let accept = if cand_cost < cur_cost {
                true
            } else if cand_cost.is_finite() && cur_cost.is_finite() {
                let delta = cand_cost - cur_cost;
                t > 1e-12 && rand::random::<f64>() < (-delta / t).exp()
            } else {
                false
            };

            if accept {
                let cur_order = cand_order;
                let cur_choices = cand_choices;
                let cur_cost = cand_cost;

                if cur_cost < best_local_cost {
                    best_local_order = cur_order;
                    best_local_choices = cur_choices;
                    best_local_cost = cur_cost;
                }
            }

            t *= sa_cooling;
            if t < 1e-10 {
                t = 1e-10;
            }
        }

        if best_local_cost < best_cost {
            best_cost = best_local_cost;
            best_order = best_local_order;
            best_choices = best_local_choices;
        }
    }

    if debug {
        println!("[GLOBAL UNIT SA] best_cost={:.3}", best_cost);
        println!("[GLOBAL UNIT SA] best_order={:?}", best_order);
        println!("[GLOBAL UNIT SA] best_choices={:?}", best_choices);
    }

    (best_order, best_choices, best_cost)
}

// ============================================================
// 拼装最终结果
// ============================================================

fn build_turn_details_for_global_route(
    route_global: &[usize],
    sides: &[String],
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> (Vec<TurnDetail>, f64) {
    let mut turn_details: Vec<TurnDetail> = Vec::new();
    let mut total = 0.0;

    for k in 0..route_global.len() - 1 {
        let i = route_global[k];
        let j = route_global[k + 1];
        let from_side = &sides[k];

        let (dist, scenario, path_xs, path_ys) = compute_turn_with_path(
            &lines[i], &lines[j], from_side, r, l_out, l_in, speed
        );

        if !dist.is_finite() {
            return (vec![], f64::INFINITY);
        }

        turn_details.push(TurnDetail {
            from_line: i,
            to_line: j,
            from_side: from_side.clone(),
            distance: dist,
            scenario,
            path_xs,
            path_ys,
        });
        total += dist;
    }

    (turn_details, total)
}

fn assemble_final_result(
    order: &[usize],
    choices: &HashMap<usize, usize>,
    candidates_by_unit: &HashMap<usize, Vec<UnitCandidate>>,
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> PlannerResult {
    let mut final_route: Vec<usize> = Vec::new();
    let mut final_sides: Vec<String> = Vec::new();

    let mut prev_cand: Option<&UnitCandidate> = None;

    for (pos, &u) in order.iter().enumerate() {
        let cand_idx = choices[&u];
        let cand = &candidates_by_unit[&u][cand_idx];

        if pos == 0 {
            final_route.extend(cand.route.iter().cloned());
            final_sides.extend(cand.sides.iter().cloned());
            prev_cand = Some(cand);
            continue;
        }

        let connector = build_connector(prev_cand.unwrap(), cand, lines, r, l_out, l_in, speed);
        if connector.is_none() {
            panic!("Final assembly failed: infeasible connector");
        }
        let conn = connector.unwrap();

        final_sides.push(conn.from_side);
        final_route.extend(cand.route.iter().cloned());
        final_sides.extend(cand.sides.iter().cloned());

        prev_cand = Some(cand);
    }

    let (turn_details, total) = build_turn_details_for_global_route(
        &final_route, &final_sides, lines, r, l_out, l_in, speed
    );

    if !total.is_finite() {
        panic!("Final route reconstruction failed");
    }

    PlannerResult {
        route: final_route,
        sides: final_sides,
        total_distance: total,
        turn_details,
    }
}

// ============================================================
// 主入口
// ============================================================

pub fn plan_route(
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
    max_heading_angle_deg: f64,
    block_top_k: usize,
    _block_sa_restarts: usize,
    _block_sa_steps: usize,
    _block_sa_init_temp: f64,
    _block_sa_cooling: f64,
    global_sa_restarts: usize,
    global_sa_steps: usize,
    global_sa_init_temp: f64,
    global_sa_cooling: f64,
    debug: bool,
) -> PlannerResult {
    let n = lines.len();
    if n == 0 {
        panic!("lines is empty");
    }
    if n == 1 {
        return PlannerResult {
            route: vec![0],
            sides: vec![],
            total_distance: 0.0,
            turn_details: vec![],
        };
    }

    if debug {
        println!("[PLAN] build global distance cache ...");
    }
    let distance_cache = build_global_distance_cache(lines, r, l_out, l_in, speed);

    let (raw_blocks, x_max_allow) = partition_lines_by_expert_rule(
        lines, r, l_in,
        max_heading_angle_deg,
        1e-9,
        debug
    );

    if debug {
        println!("[PLAN] x_max_allow={:.3}", x_max_allow);
    }

    let refined_blocks = refine_blocks_for_pair_units(&raw_blocks, lines, debug);

    let (units, candidates_by_unit) = build_work_units(
        &refined_blocks,
        &distance_cache,
        block_top_k,
        debug
    );

    let (best_order, best_choices, best_cost) = anneal_global_units(
        &units,
        &candidates_by_unit,
        lines, r, l_out, l_in, speed,
        global_sa_restarts,
        global_sa_steps,
        global_sa_init_temp,
        global_sa_cooling,
        debug
    );

    if !best_cost.is_finite() {
        panic!("Global optimization failed to find a feasible unit stitching solution");
    }

    let result = assemble_final_result(
        &best_order,
        &best_choices,
        &candidates_by_unit,
        lines, r, l_out, l_in, speed
    );

    if debug {
        println!("[PLAN] final total_distance={:.3}", result.total_distance);
        println!("[PLAN] final route={:?}", result.route);
        println!("[PLAN] final sides={:?}", result.sides);
        println!("[PLAN] chosen unit candidates:");
        for &u in &best_order {
            let cand_idx = best_choices[&u];
            let cand = &candidates_by_unit[&u][cand_idx];
            println!(
                "  unit {}: blocks={:?}, dirs={:?}, start_entry={}, end_exit={}, dist={:.3}",
                u, cand.block_ids, cand.block_directions, cand.start_entry_side, cand.end_exit_side, cand.total_distance
            );
        }
    }

    result
}

// ============================================================
// 随机路线对比
// ============================================================

fn compute_route_distance(
    route: &[usize],
    sides: &[String],
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
) -> f64 {
    let mut total = 0.0;
    for k in 0..route.len() - 1 {
        let d = compute_turn_distance(
            &lines[route[k]],
            &lines[route[k + 1]],
            &sides[k],
            r, l_out, l_in, speed
        );
        if !d.is_finite() {
            return f64::INFINITY;
        }
        total += d;
    }
    total
}

pub fn random_route_distance(
    lines: &[VerticalLine],
    r: f64,
    l_out: f64,
    l_in: f64,
    speed: f64,
    n_trials: usize,
) -> f64 {
    use rand::seq::SliceRandom;

    let n = lines.len();
    if n <= 1 {
        return 0.0;
    }

    let mut total = 0.0;
    let mut valid_cnt = 0;

    for _ in 0..n_trials {
        let mut route: Vec<usize> = (0..n).collect();
        route.shuffle(&mut rand::thread_rng());

        let start_entry_side = if rand::random() { "AB" } else { "CD" };
        let sides = build_sides_from_start_entry(route.len(), start_entry_side);

        let dist = compute_route_distance(&route, &sides, lines, r, l_out, l_in, speed);
        if dist.is_finite() {
            total += dist;
            valid_cnt += 1;
        }
    }

    if valid_cnt == 0 {
        return f64::INFINITY;
    }

    total / valid_cnt as f64
}
