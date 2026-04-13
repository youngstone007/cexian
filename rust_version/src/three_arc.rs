use std::f64::consts::PI;

use crate::common::*;

// =========================
// W < 2R 场景数据结构
// =========================

#[derive(Debug, Clone)]
pub struct ThreeArcTurnPath {
    pub valid: bool,
    pub path_type: String,

    pub start_point: Option<Point>,
    pub end_point: Option<Point>,

    pub turn_start_point: Option<Point>,
    pub turn_end_point: Option<Point>,

    pub center1: Option<Point>,
    pub center2: Option<Point>,
    pub center3: Option<Point>,

    pub join12: Option<Point>,
    pub join23: Option<Point>,

    pub arc_a_angle: f64,
    pub arc_b_angle: f64,
    pub arc_c_angle: f64,

    pub arc_a_length: f64,
    pub arc_b_length: f64,
    pub arc_c_length: f64,

    pub pre_straight_length: f64,
    pub post_straight_length: f64,

    pub total_length: f64,
    pub total_time: f64,

    pub turn_sequence: String,
}

impl Default for ThreeArcTurnPath {
    fn default() -> Self {
        ThreeArcTurnPath {
            valid: false,
            path_type: "THREE_ARC_W_LT_2R".to_string(),
            start_point: None,
            end_point: None,
            turn_start_point: None,
            turn_end_point: None,
            center1: None,
            center2: None,
            center3: None,
            join12: None,
            join23: None,
            arc_a_angle: 0.0,
            arc_b_angle: 0.0,
            arc_c_angle: 0.0,
            arc_a_length: 0.0,
            arc_b_length: 0.0,
            arc_c_length: 0.0,
            pre_straight_length: 0.0,
            post_straight_length: 0.0,
            total_length: 0.0,
            total_time: 0.0,
            turn_sequence: String::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ThreeArcTurnCandidate {
    pub eol_data_y: f64,
    pub sol_data_y: f64,
    pub eol_data_point: Point,
    pub sol_data_point: Point,
    pub eol_geom: State,
    pub sol_geom: State,
    pub three_arc: ThreeArcTurnPath,
    pub total_cost: f64,
}

// =========================
// W < 2R 固定模板三圆弧
// =========================

/// W < 2R 固定对称三圆弧模板
///
/// 工程规则：
/// - 左线 -> 右线 : LRL
/// - 右线 -> 左线 : RLR
/// - 起转高度 y_turn = max(start.y, end.y)
/// - 小弧 a / b 完全对称
/// - 中间大圆圆心位于两测线中点的垂线方向
/// - 模板角满足：cos(Q) = (W/2 + R) / (2R)
pub fn solve_w_lt_2r_three_arc(
    start_state: State,
    end_state: State,
    r: f64,
    speed: f64,
    tol: f64,
    debug: bool,
) -> ThreeArcTurnPath {
    let x1 = start_state.p.x;
    let y1 = start_state.p.y;
    let x2 = end_state.p.x;
    let y2 = end_state.p.y;

    let w = (x2 - x1).abs();
    if w >= 2.0 * r - tol {
        if debug {
            println!("[W<2R THREE-ARC] invalid: W={:.6}, 2R={:.6}", w, 2.0 * r);
        }
        return ThreeArcTurnPath::default();
    }

    let h1x = start_state.heading.cos();
    let h1y = start_state.heading.sin();
    let h2x = end_state.heading.cos();
    let h2y = end_state.heading.sin();
    let heading_dot = h1x * h2x + h1y * h2y;
    if heading_dot > -0.99 {
        if debug {
            println!(
                "[W<2R THREE-ARC] invalid: headings are not opposite enough, dot={:.6}",
                heading_dot
            );
        }
        return ThreeArcTurnPath::default();
    }

    // 根据起止航向的Y分量决定转弯侧：
    // h1y > 0 (朝上) → 顶部转弯 → y_turn = max, C2 在上方
    // h1y < 0 (朝下) → 底部转弯 → y_turn = min, C2 在下方
    let (y_turn, turn_side) = if h1y > 0.0 {
        (y1.max(y2), 1_i32)
    } else {
        (y1.min(y2), -1_i32)
    };

    let turn_start = Point::new(x1, y_turn);
    let turn_end = Point::new(x2, y_turn);

    if !point_is_ahead_along_heading(start_state.p, start_state.heading, turn_start, tol) {
        if debug {
            println!("[W<2R THREE-ARC] invalid: pre-straight not along start heading");
        }
        return ThreeArcTurnPath::default();
    }

    if !point_is_ahead_along_heading(turn_end, end_state.heading, end_state.p, tol) {
        if debug {
            println!("[W<2R THREE-ARC] invalid: post-straight not along end heading");
        }
        return ThreeArcTurnPath::default();
    }

    let pre_straight = distance(start_state.p, turn_start);
    let post_straight = distance(turn_end, end_state.p);

    // 顶部转弯 (h1y>0): x2>x1 → LRL, x2<x1 → RLR
    // 底部转弯 (h1y<0): 镜像翻转序列, x2>x1 → RLR, x2<x1 → LRL
    let sequence = if turn_side > 0 {
        if x2 > x1 {
            "LRL"
        } else {
            "RLR"
        }
    } else {
        if x2 > x1 {
            "RLR"
        } else {
            "LRL"
        }
    };

    let cos_q = ((w / 2.0 + r) / (2.0 * r)).clamp(-1.0, 1.0);
    let q = cos_q.acos();

    let h = (0.0_f64).max((2.0 * r).powi(2) - (r + w / 2.0).powi(2)).sqrt();
    let xm = 0.5 * (x1 + x2);

    let (c1, c3, c2) = if x2 > x1 {
        (
            Point::new(x1 - r, y_turn),
            Point::new(x2 + r, y_turn),
            Point::new(xm, y_turn + turn_side as f64 * h),
        )
    } else {
        (
            Point::new(x1 + r, y_turn),
            Point::new(x2 - r, y_turn),
            Point::new(xm, y_turn + turn_side as f64 * h),
        )
    };

    let v12 = normalize(c2.x - c1.x, c2.y - c1.y);
    let v32 = normalize(c2.x - c3.x, c2.y - c3.y);

    let (v12, v32) = match (v12, v32) {
        (Some(a), Some(b)) => (a, b),
        _ => {
            if debug {
                println!("[W<2R THREE-ARC] invalid: normalize failed");
            }
            return ThreeArcTurnPath::default();
        }
    };

    let join12 = Point::new(c1.x + r * v12.0, c1.y + r * v12.1);
    let join23 = Point::new(c3.x + r * v32.0, c3.y + r * v32.1);

    let arc_a_angle = q;
    let arc_b_angle = q;
    let arc_c_angle = 2.0 * PI - 2.0 * q;

    let arc_a_length = r * arc_a_angle;
    let arc_b_length = r * arc_b_angle;
    let arc_c_length = r * arc_c_angle;

    let total_length = pre_straight + arc_a_length + arc_c_length + arc_b_length + post_straight;
    let total_time = total_length / speed;

    if debug {
        println!("[W<2R THREE-ARC] feasible");
        println!("    sequence={}", sequence);
        println!("    W={:.3}, R={:.3}", w, r);
        println!("    y_turn={:.3}", y_turn);
        println!("    Q={:.3} deg", q.to_degrees());
        println!("    c1=({:.3}, {:.3})", c1.x, c1.y);
        println!("    c2=({:.3}, {:.3})", c2.x, c2.y);
        println!("    c3=({:.3}, {:.3})", c3.x, c3.y);
        println!("    join12=({:.3}, {:.3})", join12.x, join12.y);
        println!("    join23=({:.3}, {:.3})", join23.x, join23.y);
        println!("    pre={:.3}, post={:.3}", pre_straight, post_straight);
        println!("    a={:.3} deg", arc_a_angle.to_degrees());
        println!("    c={:.3} deg", arc_c_angle.to_degrees());
        println!("    b={:.3} deg", arc_b_angle.to_degrees());
        println!("    total={:.3}", total_length);
    }

    ThreeArcTurnPath {
        valid: true,
        path_type: "THREE_ARC_W_LT_2R".to_string(),
        start_point: Some(start_state.p),
        end_point: Some(end_state.p),
        turn_start_point: Some(turn_start),
        turn_end_point: Some(turn_end),
        center1: Some(c1),
        center2: Some(c2),
        center3: Some(c3),
        join12: Some(join12),
        join23: Some(join23),
        arc_a_angle,
        arc_b_angle,
        arc_c_angle,
        arc_a_length,
        arc_b_length,
        arc_c_length,
        pre_straight_length: pre_straight,
        post_straight_length: post_straight,
        total_length,
        total_time,
        turn_sequence: sequence.to_string(),
    }
}

// =========================
// 固定线尾/线头求解器 (W < 2R)
// =========================

pub fn solve_fixed_head_tail_w_lt_2r_three_arc_turn(
    line1: &VerticalLine,
    line2: &VerticalLine,
    l_out: f64,
    l_in: f64,
    r: f64,
    speed: f64,
    tol: f64,
    debug: bool,
) -> Option<ThreeArcTurnCandidate> {
    let w = (line2.x - line1.x).abs();
    if w >= 2.0 * r - tol {
        return None;
    }

    let eol_data_y = line1.y_end;
    let sol_data_y = line2.y_start;

    let (eol_geom, eol_data_point) = build_eol_geom(line1, eol_data_y, l_out);
    let (sol_geom, sol_data_point) = build_sol_geom_fixed_head(line2, l_in);

    let three_arc = solve_w_lt_2r_three_arc(eol_geom, sol_geom, r, speed, tol, debug);
    if !three_arc.valid {
        return None;
    }

    Some(ThreeArcTurnCandidate {
        eol_data_y,
        sol_data_y,
        eol_data_point,
        sol_data_point,
        eol_geom,
        sol_geom,
        total_cost: three_arc.total_time,
        three_arc,
    })
}

// =========================
// 打印结果 (W < 2R)
// =========================

pub fn print_three_arc_result(best: Option<&ThreeArcTurnCandidate>) {
    if best.is_none() {
        println!("未找到可行的 W < 2R 三圆弧换线路径。");
        return;
    }

    let best = best.unwrap();
    let t = &best.three_arc;
    println!("====== W < 2R 三圆弧换线结果 ======");
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
    println!("路径类型  : {}", t.path_type);
    println!("转弯序列  : {}", t.turn_sequence);
    if let Some(tsp) = t.turn_start_point {
        println!("三圆弧起点: ({:.3}, {:.3})", tsp.x, tsp.y);
    }
    if let Some(tep) = t.turn_end_point {
        println!("三圆弧终点: ({:.3}, {:.3})", tep.x, tep.y);
    }
    if let Some(c1) = t.center1 {
        println!("圆心1      : ({:.3}, {:.3})", c1.x, c1.y);
    }
    if let Some(c2) = t.center2 {
        println!("圆心2      : ({:.3}, {:.3})", c2.x, c2.y);
    }
    if let Some(c3) = t.center3 {
        println!("圆心3      : ({:.3}, {:.3})", c3.x, c3.y);
    }
    if let Some(j12) = t.join12 {
        println!("连接点12   : ({:.3}, {:.3})", j12.x, j12.y);
    }
    if let Some(j23) = t.join23 {
        println!("连接点23   : ({:.3}, {:.3})", j23.x, j23.y);
    }
    println!("a角        : {:.3} deg", t.arc_a_angle.to_degrees());
    println!("c角        : {:.3} deg", t.arc_c_angle.to_degrees());
    println!("b角        : {:.3} deg", t.arc_b_angle.to_degrees());
    println!("前直线长度 : {:.3}", t.pre_straight_length);
    println!("a弧长      : {:.3}", t.arc_a_length);
    println!("c弧长      : {:.3}", t.arc_c_length);
    println!("b弧长      : {:.3}", t.arc_b_length);
    println!("后直线长度 : {:.3}", t.post_straight_length);
    println!("总长度     : {:.3}", t.total_length);
    println!("总时间     : {:.3}", t.total_time);
    println!("==================================");
}
