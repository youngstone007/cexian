use std::f64::consts::{FRAC_PI_2, PI};

// =========================
// 基础数据结构
// =========================

#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Point { x, y }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct State {
    pub p: Point,
    pub heading: f64, // 弧度
}

impl State {
    pub fn new(p: Point, heading: f64) -> Self {
        State { p, heading }
    }
}

/// 南北向测线（平行于Y轴）
///
/// 约定：
/// - y_start: 采集起点
/// - y_end  : 采集终点
/// - direction = +1 表示沿 +Y 方向采集
/// - direction = -1 表示沿 -Y 方向采集
/// - direction 可自动由 y_end - y_start 推断，无需手动指定
#[derive(Debug, Clone)]
pub struct VerticalLine {
    pub x: f64,
    pub y_start: f64,
    pub y_end: f64,
    pub direction: i32,
}

impl VerticalLine {
    pub fn new(x: f64, y_start: f64, y_end: f64) -> Self {
        let direction = if y_end >= y_start { 1 } else { -1 };
        VerticalLine {
            x,
            y_start,
            y_end,
            direction,
        }
    }

    pub fn with_direction(x: f64, y_start: f64, y_end: f64, direction: i32) -> Self {
        let direction = if direction == 0 {
            if y_end >= y_start { 1 } else { -1 }
        } else {
            direction
        };
        VerticalLine {
            x,
            y_start,
            y_end,
            direction,
        }
    }

    pub fn length(&self) -> f64 {
        (self.y_end - self.y_start).abs()
    }

    pub fn heading(&self) -> f64 {
        if self.direction > 0 {
            FRAC_PI_2
        } else {
            -FRAC_PI_2
        }
    }
}

// =========================
// 全局参数
// =========================

pub const EPS: f64 = 1e-9;
pub const DOT_TOL: f64 = 0.995;

// =========================
// 基础几何函数
// =========================

pub fn distance(p1: Point, p2: Point) -> f64 {
    ((p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2)).sqrt()
}

pub fn mod2pi(a: f64) -> f64 {
    let mut a = a;
    while a < 0.0 {
        a += 2.0 * PI;
    }
    while a >= 2.0 * PI {
        a -= 2.0 * PI;
    }
    a
}

pub fn left_normal(psi: f64) -> (f64, f64) {
    (-psi.sin(), psi.cos())
}

pub fn right_normal(psi: f64) -> (f64, f64) {
    (psi.sin(), -psi.cos())
}

pub fn polar_angle(center: Point, point: Point) -> f64 {
    (point.y - center.y).atan2(point.x - center.x)
}

pub fn compute_arc_angle(center: Point, point_from: Point, point_to: Point, turn: &str) -> f64 {
    let a1 = polar_angle(center, point_from);
    let a2 = polar_angle(center, point_to);

    if turn == "L" {
        mod2pi(a2 - a1)
    } else {
        mod2pi(a1 - a2)
    }
}

pub fn turning_circle_center(state: State, r: f64, turn: &str) -> Point {
    let (nx, ny) = if turn == "L" {
        left_normal(state.heading)
    } else {
        right_normal(state.heading)
    };

    Point::new(state.p.x + r * nx, state.p.y + r * ny)
}

pub fn normalize(vx: f64, vy: f64) -> Option<(f64, f64)> {
    let n = (vx * vx + vy * vy).sqrt();
    if n < EPS {
        return None;
    }
    Some((vx / n, vy / n))
}

pub fn point_is_ahead_along_heading(
    p_from: Point,
    heading: f64,
    p_to: Point,
    tol: f64,
) -> bool {
    let hx = heading.cos();
    let hy = heading.sin();
    let vx = p_to.x - p_from.x;
    let vy = p_to.y - p_from.y;
    hx * vx + hy * vy >= -tol
}

pub fn point_is_ahead_along_heading_default(p_from: Point, heading: f64, p_to: Point) -> bool {
    point_is_ahead_along_heading(p_from, heading, p_to, 1e-9)
}

// =========================
// 测线点构造
// =========================

pub fn point_on_vertical_line(line: &VerticalLine, y: f64) -> Point {
    Point::new(line.x, y)
}

pub fn build_eol_geom(line1: &VerticalLine, eol_data_y: f64, l_out: f64) -> (State, Point) {
    let data_pt = point_on_vertical_line(line1, eol_data_y);
    let geom_pt = Point::new(line1.x, eol_data_y + line1.direction as f64 * l_out);
    (State::new(geom_pt, line1.heading()), data_pt)
}

pub fn build_sol_geom_fixed_head(line2: &VerticalLine, l_in: f64) -> (State, Point) {
    let sol_data_y = line2.y_start;
    let data_pt = point_on_vertical_line(line2, sol_data_y);
    let geom_pt = Point::new(
        line2.x,
        sol_data_y - line2.direction as f64 * l_in,
    );
    (State::new(geom_pt, line2.heading()), data_pt)
}

// =========================
// 可视化辅助函数
// =========================

pub fn sample_arc(
    center: Point,
    radius: f64,
    p_start: Point,
    p_end: Point,
    turn: &str,
    n: usize,
) -> (Vec<f64>, Vec<f64>) {
    let a1 = polar_angle(center, p_start);
    let a2 = polar_angle(center, p_end);

    let angles: Vec<f64> = if turn == "L" {
        let delta = mod2pi(a2 - a1);
        (0..n)
            .map(|i| a1 + delta * i as f64 / (n - 1) as f64)
            .collect()
    } else {
        let delta = mod2pi(a1 - a2);
        (0..n)
            .map(|i| a1 - delta * i as f64 / (n - 1) as f64)
            .collect()
    };

    let xs: Vec<f64> = angles.iter().map(|&a| center.x + radius * a.cos()).collect();
    let ys: Vec<f64> = angles.iter().map(|&a| center.y + radius * a.sin()).collect();
    (xs, ys)
}

pub fn sample_arc_default(
    center: Point,
    radius: f64,
    p_start: Point,
    p_end: Point,
    turn: &str,
) -> (Vec<f64>, Vec<f64>) {
    sample_arc(center, radius, p_start, p_end, turn, 100)
}

pub fn sample_line(p1: Point, p2: Point, n: usize) -> (Vec<f64>, Vec<f64>) {
    let xs: Vec<f64> = (0..n)
        .map(|i| p1.x + (p2.x - p1.x) * i as f64 / (n - 1) as f64)
        .collect();
    let ys: Vec<f64> = (0..n)
        .map(|i| p1.y + (p2.y - p1.y) * i as f64 / (n - 1) as f64)
        .collect();
    (xs, ys)
}

pub fn sample_line_default(p1: Point, p2: Point) -> (Vec<f64>, Vec<f64>) {
    sample_line(p1, p2, 50)
}
