use std::f64::consts::PI;

use crate::common::*;

// =========================
// W = 2R 场景数据结构
// =========================

#[derive(Debug, Clone)]
pub struct SemicircleTurnPath {
    pub valid: bool,
    pub path_type: String,

    pub start_point: Option<Point>,
    pub end_point: Option<Point>,

    pub semicircle_start: Option<Point>,
    pub semicircle_end: Option<Point>,
    pub center: Option<Point>,

    pub pre_straight_length: f64,
    pub arc_length: f64,
    pub post_straight_length: f64,

    pub total_length: f64,
    pub total_time: f64,

    pub turn_direction: String,
}

impl Default for SemicircleTurnPath {
    fn default() -> Self {
        SemicircleTurnPath {
            valid: false,
            path_type: "SEMICIRCLE_W_EQ_2R".to_string(),
            start_point: None,
            end_point: None,
            semicircle_start: None,
            semicircle_end: None,
            center: None,
            pre_straight_length: 0.0,
            arc_length: 0.0,
            post_straight_length: 0.0,
            total_length: 0.0,
            total_time: 0.0,
            turn_direction: String::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct SemicircleTurnCandidate {
    pub eol_data_y: f64,
    pub sol_data_y: f64,
    pub eol_data_point: Point,
    pub sol_data_point: Point,
    pub eol_geom: State,
    pub sol_geom: State,
    pub semi: SemicircleTurnPath,
    pub total_cost: f64,
}

// =========================
// W = 2R 半圆
// =========================

pub fn solve_w_eq_2r_semicircle(
    start_state: State,
    end_state: State,
    r: f64,
    speed: f64,
    tol: f64,
    debug: bool,
) -> SemicircleTurnPath {
    let x1 = start_state.p.x;
    let y1 = start_state.p.y;
    let x2 = end_state.p.x;
    let y2 = end_state.p.y;

    let w = (x2 - x1).abs();
    if (w - 2.0 * r).abs() > tol {
        return SemicircleTurnPath::default();
    }

    let h1x = start_state.heading.cos();
    let h1y = start_state.heading.sin();
    let h2x = end_state.heading.cos();
    let h2y = end_state.heading.sin();
    let heading_dot = h1x * h2x + h1y * h2y;
    if heading_dot > -0.99 {
        return SemicircleTurnPath::default();
    }

    let candidate_y_stars = [y1, y2];
    let mut feasible_paths: Vec<SemicircleTurnPath> = Vec::new();

    for y_star in candidate_y_stars {
        let semi_start = Point::new(x1, y_star);
        let semi_end = Point::new(x2, y_star);
        let center = Point::new((x1 + x2) / 2.0, y_star);

        if !point_is_ahead_along_heading(start_state.p, start_state.heading, semi_start, tol) {
            continue;
        }

        if !point_is_ahead_along_heading(semi_end, end_state.heading, end_state.p, tol) {
            continue;
        }

        let pre_straight = distance(start_state.p, semi_start);
        let post_straight = distance(semi_end, end_state.p);
        let arc_length = PI * r;
        let total_length = pre_straight + arc_length + post_straight;
        let total_time = total_length / speed;

        let rx = semi_start.x - center.x;
        let ry = semi_start.y - center.y;

        let (ltx, lty) = (-ry, rx);
        let ln = normalize(ltx, lty);

        let (rtx, rty) = (ry, -rx);
        let rn = normalize(rtx, rty);

        let dot_l = ln.map_or(-1e9, |(lx, ly)| lx * h1x + ly * h1y);
        let dot_r = rn.map_or(-1e9, |(rx_, ry_)| rx_ * h1x + ry_ * h1y);
        let turn_direction = if dot_l >= dot_r { "L" } else { "R" };

        feasible_paths.push(SemicircleTurnPath {
            valid: true,
            path_type: "SEMICIRCLE_W_EQ_2R".to_string(),
            start_point: Some(start_state.p),
            end_point: Some(end_state.p),
            semicircle_start: Some(semi_start),
            semicircle_end: Some(semi_end),
            center: Some(center),
            pre_straight_length: pre_straight,
            arc_length,
            post_straight_length: post_straight,
            total_length,
            total_time,
            turn_direction: turn_direction.to_string(),
        });
    }

    if feasible_paths.is_empty() {
        return SemicircleTurnPath::default();
    }

    feasible_paths
        .into_iter()
        .min_by(|a, b| a.total_length.partial_cmp(&b.total_length).unwrap())
        .unwrap()
}

// =========================
// 固定线尾/线头求解器 (W = 2R)
// =========================

pub fn solve_fixed_head_tail_w_eq_2r_semicircle_turn(
    line1: &VerticalLine,
    line2: &VerticalLine,
    l_out: f64,
    l_in: f64,
    r: f64,
    speed: f64,
    tol: f64,
    debug: bool,
) -> Option<SemicircleTurnCandidate> {
    let w = (line2.x - line1.x).abs();
    if (w - 2.0 * r).abs() > tol {
        return None;
    }

    let eol_data_y = line1.y_end;
    let sol_data_y = line2.y_start;

    let (eol_geom, eol_data_point) = build_eol_geom(line1, eol_data_y, l_out);
    let (sol_geom, sol_data_point) = build_sol_geom_fixed_head(line2, l_in);

    let semi = solve_w_eq_2r_semicircle(eol_geom, sol_geom, r, speed, tol, debug);
    if !semi.valid {
        return None;
    }

    Some(SemicircleTurnCandidate {
        eol_data_y,
        sol_data_y,
        eol_data_point,
        sol_data_point,
        eol_geom,
        sol_geom,
        total_cost: semi.total_time,
        semi,
    })
}

// =========================
// 打印结果 (W = 2R)
// =========================

pub fn print_semicircle_result(best: Option<&SemicircleTurnCandidate>) {
    if best.is_none() {
        println!("未找到可行的 W = 2R 半圆换线路径。");
        return;
    }

    let best = best.unwrap();
    let s = &best.semi;
    println!("====== W = 2R 半圆换线最优结果 ======");
    println!(
        "EOL 数据点: ({:.3}, {:.3}) [固定在线尾]",
        best.eol_data_point.x, best.eol_data_point.y
    );
    println!(
        "SOL 数据点: ({:.3}, {:.3}) [固定在线头]",
        best.sol_data_point.x, best.sol_data_point.y
    );
    println!(
        "EOL 几何点: ({:.3}, {:.3})  heading={:.2} deg",
        best.eol_geom.p.x, best.eol_geom.p.y, best.eol_geom.heading.to_degrees()
    );
    println!(
        "SOL 几何点: ({:.3}, {:.3})  heading={:.2} deg",
        best.sol_geom.p.x, best.sol_geom.p.y, best.sol_geom.heading.to_degrees()
    );
    println!("路径类型  : {}", s.path_type);
    println!("转向方向  : {}", s.turn_direction);
    if let Some(ss) = s.semicircle_start {
        println!("半圆起点  : ({:.3}, {:.3})", ss.x, ss.y);
    }
    if let Some(se) = s.semicircle_end {
        println!("半圆终点  : ({:.3}, {:.3})", se.x, se.y);
    }
    if let Some(c) = s.center {
        println!("圆心      : ({:.3}, {:.3})", c.x, c.y);
    }
    println!("前直线长度: {:.3}", s.pre_straight_length);
    println!("半圆长度  : {:.3}", s.arc_length);
    println!("后直线长度: {:.3}", s.post_straight_length);
    println!("总长度    : {:.3}", s.total_length);
    println!("总时间    : {:.3}", s.total_time);
    println!("====================================");
}
