use std::f64::consts::PI;

use crate::common::*;

// =========================
// W > 2R 场景数据结构
// =========================

#[derive(Debug, Clone)]
pub struct TangentSolution {
    pub tangent_point1: Point,
    pub tangent_point2: Point,
}

#[derive(Debug, Clone)]
pub struct DubinsPath {
    pub path_type: String,
    pub valid: bool,
    pub arc1_angle: f64,
    pub arc1_length: f64,
    pub straight_length: f64,
    pub arc2_angle: f64,
    pub arc2_length: f64,
    pub total_length: f64,
    pub total_time: f64,
    pub center1: Option<Point>,
    pub center2: Option<Point>,
    pub tangent_point1: Option<Point>,
    pub tangent_point2: Option<Point>,
}

impl Default for DubinsPath {
    fn default() -> Self {
        DubinsPath {
            path_type: String::new(),
            valid: false,
            arc1_angle: 0.0,
            arc1_length: 0.0,
            straight_length: 0.0,
            arc2_angle: 0.0,
            arc2_length: 0.0,
            total_length: 0.0,
            total_time: 0.0,
            center1: None,
            center2: None,
            tangent_point1: None,
            tangent_point2: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TurnCandidate {
    pub eol_data_y: f64,
    pub sol_data_y: f64,
    pub eol_data_point: Point,
    pub sol_data_point: Point,
    pub eol_geom: State,
    pub sol_geom: State,
    pub dubins: DubinsPath,
    pub total_cost: f64,
}

// =========================
// CSC 切向一致性检查
// =========================

pub fn tangent_direction_on_circle(center: Point, point: Point, turn: &str) -> (f64, f64) {
    let rx = point.x - center.x;
    let ry = point.y - center.y;
    let r = normalize(rx, ry);
    let (rx, ry) = match r {
        Some(v) => v,
        None => return (0.0, 0.0),
    };

    let (tx, ty) = if turn == "L" {
        (-ry, rx)
    } else {
        (ry, -rx)
    };

    match normalize(tx, ty) {
        Some(v) => v,
        None => (0.0, 0.0),
    }
}

pub fn tangent_direction_is_consistent(
    c1: Point,
    t1: Point,
    c2: Point,
    t2: Point,
    turn1: &str,
    turn2: &str,
    debug: bool,
) -> bool {
    let s = normalize(t2.x - t1.x, t2.y - t1.y);
    let (sx, sy) = match s {
        Some(v) => v,
        None => return false,
    };

    let (v1x, v1y) = tangent_direction_on_circle(c1, t1, turn1);
    let (v2x, v2y) = tangent_direction_on_circle(c2, t2, turn2);

    let dot1 = v1x * sx + v1y * sy;
    let dot2 = v2x * sx + v2y * sy;

    if debug {
        println!("      tangent consistency: dot1={:.6}, dot2={:.6}", dot1, dot2);
    }

    dot1 > DOT_TOL && dot2 > DOT_TOL
}

// =========================
// CSC 公切线求解
// =========================

pub fn solve_outer_tangent(
    c1: Point,
    c2: Point,
    r: f64,
    turn: &str,
    debug: bool,
) -> Option<TangentSolution> {
    let dx = c2.x - c1.x;
    let dy = c2.y - c1.y;
    let d = (dx * dx + dy * dy).sqrt();
    if d < EPS {
        return None;
    }

    let ux = dx / d;
    let uy = dy / d;
    let perp = (-uy, ux);

    let mut feasible: Vec<(Point, Point)> = Vec::new();

    for sign in [1.0_f64, -1.0_f64] {
        let t1 = Point::new(c1.x + sign * r * perp.0, c1.y + sign * r * perp.1);
        let t2 = Point::new(c2.x + sign * r * perp.0, c2.y + sign * r * perp.1);

        let ok = tangent_direction_is_consistent(c1, t1, c2, t2, turn, turn, false);
        if debug {
            println!(
                "      outer tangent sign={:+.0}, ok={}, T1=({:.2},{:.2}), T2=({:.2},{:.2})",
                sign, ok, t1.x, t1.y, t2.x, t2.y
            );
        }

        if ok {
            feasible.push((t1, t2));
        }
    }

    if feasible.is_empty() {
        return None;
    }

    let (t1, t2) = feasible
        .iter()
        .min_by(|a, b| {
            distance(a.0, a.1)
                .partial_cmp(&distance(b.0, b.1))
                .unwrap()
        })
        .unwrap();
    Some(TangentSolution {
        tangent_point1: *t1,
        tangent_point2: *t2,
    })
}

pub fn construct_inner_tangent_points(
    c1: Point,
    c2: Point,
    r: f64,
    theta: f64,
    turn1: &str,
    turn2: &str,
) -> Option<(Point, Point)> {
    let dx = theta.cos();
    let dy = theta.sin();

    let left_n = (-dy, dx);
    let right_n = (dy, -dx);

    let r1 = if turn1 == "L" { right_n } else { left_n };
    let r2 = if turn2 == "L" { right_n } else { left_n };

    let t1 = Point::new(c1.x + r * r1.0, c1.y + r * r1.1);
    let t2 = Point::new(c2.x + r * r2.0, c2.y + r * r2.1);

    let s = normalize(t2.x - t1.x, t2.y - t1.y);
    let (sx, sy) = match s {
        Some(v) => v,
        None => return None,
    };

    if sx * dx + sy * dy < DOT_TOL {
        return None;
    }

    Some((t1, t2))
}

pub fn solve_inner_tangent(
    c1: Point,
    c2: Point,
    r: f64,
    turn1: &str,
    turn2: &str,
    debug: bool,
) -> Option<TangentSolution> {
    let dx = c2.x - c1.x;
    let dy = c2.y - c1.y;
    let d = (dx * dx + dy * dy).sqrt();

    if d < 2.0 * r - EPS {
        if debug {
            println!(
                "      inner tangent impossible: D={:.3} < 2R={:.3}",
                d,
                2.0 * r
            );
        }
        return None;
    }

    let phi = dy.atan2(dx);
    let alpha = (2.0 * r / d).clamp(-1.0, 1.0).acos();

    let candidate_dirs = [phi + alpha, phi - alpha];
    let mut feasible: Vec<(Point, Point)> = Vec::new();

    for theta in candidate_dirs {
        let pair = construct_inner_tangent_points(c1, c2, r, theta, turn1, turn2);
        if pair.is_none() {
            if debug {
                println!(
                    "      inner theta={:.2} deg -> construct failed",
                    theta.to_degrees()
                );
            }
            continue;
        }

        let (t1, t2) = pair.unwrap();
        let ok = tangent_direction_is_consistent(c1, t1, c2, t2, turn1, turn2, false);

        if debug {
            println!(
                "      inner theta={:.2} deg, ok={}, T1=({:.2},{:.2}), T2=({:.2},{:.2})",
                theta.to_degrees(),
                ok,
                t1.x,
                t1.y,
                t2.x,
                t2.y
            );
        }

        if ok {
            feasible.push((t1, t2));
        }
    }

    if feasible.is_empty() {
        return None;
    }

    let (t1, t2) = feasible
        .iter()
        .min_by(|a, b| {
            distance(a.0, a.1)
                .partial_cmp(&distance(b.0, b.1))
                .unwrap()
        })
        .unwrap();
    Some(TangentSolution {
        tangent_point1: *t1,
        tangent_point2: *t2,
    })
}

// =========================
// Dubins CSC
// =========================

pub fn solve_one_csc_path(
    start_state: State,
    end_state: State,
    r: f64,
    path_type: &str,
    speed: f64,
    debug: bool,
) -> DubinsPath {
    let chars: Vec<char> = path_type.chars().collect();
    let turn1 = chars[0];
    let turn2 = chars[2];

    let turn1_str = &path_type[0..1];
    let turn2_str = &path_type[2..3];

    let c1 = turning_circle_center(start_state, r, turn1_str);
    let c2 = turning_circle_center(end_state, r, turn2_str);

    if debug {
        println!("    solve {}:", path_type);
        println!(
            "      start=({:.2},{:.2}), hdg={:.2}",
            start_state.p.x,
            start_state.p.y,
            start_state.heading.to_degrees()
        );
        println!(
            "      end  =({:.2},{:.2}), hdg={:.2}",
            end_state.p.x,
            end_state.p.y,
            end_state.heading.to_degrees()
        );
        println!("      c1=({:.2},{:.2}), c2=({:.2},{:.2})", c1.x, c1.y, c2.x, c2.y);
    }

    let tangent_solution = if turn1 == turn2 {
        solve_outer_tangent(c1, c2, r, turn1_str, debug)
    } else {
        solve_inner_tangent(c1, c2, r, turn1_str, turn2_str, debug)
    };

    let tangent_solution = match tangent_solution {
        Some(ts) => ts,
        None => {
            if debug {
                println!("      tangent solution = None");
            }
            return DubinsPath::default();
        }
    };

    let t1 = tangent_solution.tangent_point1;
    let t2 = tangent_solution.tangent_point2;

    if !tangent_direction_is_consistent(c1, t1, c2, t2, turn1_str, turn2_str, debug) {
        if debug {
            println!("      final tangent consistency failed");
        }
        return DubinsPath::default();
    }

    let arc1_angle = compute_arc_angle(c1, start_state.p, t1, turn1_str);
    let arc2_angle = compute_arc_angle(c2, t2, end_state.p, turn2_str);

    let arc1_length = r * arc1_angle;
    let straight_length = distance(t1, t2);
    let arc2_length = r * arc2_angle;

    let total_length = arc1_length + straight_length + arc2_length;
    let total_time = total_length / speed;

    if debug {
        println!(
            "      arc1={:.3} deg, straight={:.3}, arc2={:.3} deg, total={:.3}",
            arc1_angle.to_degrees(),
            straight_length,
            arc2_angle.to_degrees(),
            total_length
        );
    }

    DubinsPath {
        path_type: path_type.to_string(),
        valid: true,
        arc1_angle,
        arc1_length,
        straight_length,
        arc2_angle,
        arc2_length,
        total_length,
        total_time,
        center1: Some(c1),
        center2: Some(c2),
        tangent_point1: Some(t1),
        tangent_point2: Some(t2),
    }
}

pub fn shortest_dubins_csc(
    start_state: State,
    end_state: State,
    r: f64,
    speed: f64,
    debug: bool,
) -> DubinsPath {
    let mut candidates: Vec<DubinsPath> = Vec::new();

    for path_type in ["LSL", "RSR"] {
        let p = solve_one_csc_path(start_state, end_state, r, path_type, speed, debug);
        if p.valid {
            candidates.push(p);
        }
    }

    if candidates.is_empty() {
        return DubinsPath::default();
    }

    candidates
        .into_iter()
        .min_by(|a, b| a.total_length.partial_cmp(&b.total_length).unwrap())
        .unwrap()
}

// =========================
// 固定线尾/线头求解器 (W > 2R)
// =========================

pub fn solve_fixed_head_tail_turn_between_vertical_lines(
    line1: &VerticalLine,
    line2: &VerticalLine,
    l_out: f64,
    l_in: f64,
    r: f64,
    speed: f64,
    debug: bool,
) -> Option<TurnCandidate> {
    let eol_data_y = line1.y_end;
    let sol_data_y = line2.y_start;

    let (eol_geom, eol_data_point) = build_eol_geom(line1, eol_data_y, l_out);
    let (sol_geom, sol_data_point) = build_sol_geom_fixed_head(line2, l_in);

    let dubins = shortest_dubins_csc(eol_geom, sol_geom, r, speed, debug);
    if !dubins.valid {
        return None;
    }

    Some(TurnCandidate {
        eol_data_y,
        sol_data_y,
        eol_data_point,
        sol_data_point,
        eol_geom,
        sol_geom,
        total_cost: dubins.total_time,
        dubins,
    })
}

// =========================
// 打印结果 (W > 2R)
// =========================

pub fn print_result(best: Option<&TurnCandidate>) {
    if best.is_none() {
        println!("未找到可行换线路径。");
        return;
    }

    let best = best.unwrap();
    let d = &best.dubins;
    println!("========== 最优换线结果 ==========");
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
    println!("路径类型: {}", d.path_type);
    println!("第一圆弧角度: {:.3} deg", d.arc1_angle.to_degrees());
    println!("第一圆弧长度: {:.3}", d.arc1_length);
    println!("直线段长度 : {:.3}", d.straight_length);
    println!("第二圆弧角度: {:.3} deg", d.arc2_angle.to_degrees());
    println!("第二圆弧长度: {:.3}", d.arc2_length);
    println!("总长度     : {:.3}", d.total_length);
    println!("总时间     : {:.3}", d.total_time);
    if let Some(c1) = d.center1 {
        println!("圆心1      : ({:.3}, {:.3})", c1.x, c1.y);
    }
    if let Some(c2) = d.center2 {
        println!("圆心2      : ({:.3}, {:.3})", c2.x, c2.y);
    }
    if let Some(t1) = d.tangent_point1 {
        println!("切点1      : ({:.3}, {:.3})", t1.x, t1.y);
    }
    if let Some(t2) = d.tangent_point2 {
        println!("切点2      : ({:.3}, {:.3})", t2.x, t2.y);
    }
    println!("==================================");
}
