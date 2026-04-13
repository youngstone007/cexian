use std::f64::consts::FRAC_PI_2;

use crate::common::*;

// =========================
// 通用测线数据结构
// =========================

/// 通用测线：两个端点定义，方向由 start→end 隐含
///
/// - start: 采集起点
/// - end:   采集终点
/// - heading: start→end 的航向角（弧度），自动计算
/// - length: 测线长度，自动计算
#[derive(Debug, Clone)]
pub struct SurveyLine {
    pub start: Point,
    pub end: Point,
}

impl SurveyLine {
    pub fn new(start: Point, end: Point) -> Self {
        SurveyLine { start, end }
    }

    pub fn heading(&self) -> f64 {
        (self.end.y - self.start.y).atan2(self.end.x - self.start.x)
    }

    pub fn length(&self) -> f64 {
        ((self.end.x - self.start.x).powi(2) + (self.end.y - self.start.y).powi(2)).sqrt()
    }

    pub fn direction_vector(&self) -> (f64, f64) {
        let dx = self.end.x - self.start.x;
        let dy = self.end.y - self.start.y;
        let n = (dx * dx + dy * dy).sqrt();
        if n < EPS {
            return (0.0, 0.0);
        }
        (dx / n, dy / n)
    }
}

// =========================
// 坐标变换工具
// =========================

/// 归一化变换参数，记录旋转和平移信息，可用于逆变换
#[derive(Debug, Clone)]
pub struct Transform {
    pub rotation_angle: f64,    // 旋转角（弧度）
    pub rotation_center: Point, // 旋转中心
    pub translation_dx: f64,    // X 平移量
    pub translation_dy: f64,    // Y 平移量
}

/// 将点 p 绕 center 旋转 angle 弧度
pub fn rotate_point(p: Point, angle: f64, center: Point) -> Point {
    let dx = p.x - center.x;
    let dy = p.y - center.y;
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    Point::new(
        center.x + dx * cos_a - dy * sin_a,
        center.y + dx * sin_a + dy * cos_a,
    )
}

/// 将点 p 平移 (dx, dy)
pub fn translate_point(p: Point, dx: f64, dy: f64) -> Point {
    Point::new(p.x + dx, p.y + dy)
}

/// 将归一化坐标下的点逆变换回原始坐标
pub fn denormalize_point(p: Point, transform: &Transform) -> Point {
    // 逆平移
    let p1 = translate_point(p, -transform.translation_dx, -transform.translation_dy);
    // 逆旋转
    rotate_point(p1, -transform.rotation_angle, transform.rotation_center)
}

// =========================
// 测线归一化
// =========================

/// 将任意方向的平行测线归一化为 Y 轴对齐的 VerticalLine
///
/// 步骤：
/// 1. 从第一条测线计算公共航向角 θ
/// 2. 验证所有测线平行
/// 3. 计算旋转角 = π/2 - θ，旋转使测线对齐 Y 轴
/// 4. 按 x 排序，平移使最左侧线在 x=0
/// 5. 构造 VerticalLine 列表
/// 6. 验证相邻间距相等
pub fn normalize_survey_lines(
    lines: &[SurveyLine],
    spacing_tol: f64,
    parallel_tol: f64,
    debug: bool,
) -> Result<(Vec<VerticalLine>, Transform), String> {
    if lines.is_empty() {
        return Err("测线列表不能为空".to_string());
    }

    // --- 1. 计算公共航向角 ---
    let ref_heading = lines[0].heading();

    // --- 2. 验证所有测线平行 ---
    let pi = std::f64::consts::PI;
    for (i, line) in lines.iter().enumerate() {
        let mut angle_diff = (line.heading() - ref_heading).abs();
        // 处理角度环绕：两条平行线航向可能相差 π（反向平行）
        if angle_diff > pi {
            angle_diff = 2.0 * pi - angle_diff;
        }
        // 平行条件：航向差接近 0 或接近 π
        if angle_diff > parallel_tol && (angle_diff - pi).abs() > parallel_tol {
            return Err(format!(
                "测线 {} 不平行于参考测线：航向差 = {:.2} deg",
                i,
                angle_diff.to_degrees()
            ));
        }
    }

    // --- 3. 旋转使测线对齐 Y 轴 ---
    let rotation_angle = FRAC_PI_2 - ref_heading;
    let rotation_center = Point::new(0.0, 0.0);

    let rotated_starts: Vec<Point> = lines
        .iter()
        .map(|line| rotate_point(line.start, rotation_angle, rotation_center))
        .collect();
    let rotated_ends: Vec<Point> = lines
        .iter()
        .map(|line| rotate_point(line.end, rotation_angle, rotation_center))
        .collect();

    if debug {
        println!("[归一化] 参考航向 = {:.3} deg", ref_heading.to_degrees());
        println!("[归一化] 旋转角  = {:.3} deg", rotation_angle.to_degrees());
    }

    // --- 4. 按 x 排序，平移使最左侧线在 x=0 ---
    let mut line_data: Vec<(f64, Point, Point, usize)> = Vec::new();
    for (i, _line) in lines.iter().enumerate() {
        let rs = rotated_starts[i];
        let re = rotated_ends[i];
        let x_pos = (rs.x + re.x) / 2.0;
        line_data.push((x_pos, rs, re, i));
    }

    line_data.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let min_x = line_data[0].0;
    let translation_dx = -min_x;
    let translation_dy = 0.0;

    if debug {
        println!(
            "[归一化] 平移量  = ({:.3}, {:.3})",
            translation_dx, translation_dy
        );
    }

    // --- 5. 构造 VerticalLine 列表 ---
    let mut vertical_lines: Vec<VerticalLine> = Vec::new();
    for (_x_pos, rs, re, orig_idx) in &line_data {
        let ts = translate_point(*rs, translation_dx, translation_dy);
        let te = translate_point(*re, translation_dx, translation_dy);

        let vl = VerticalLine::new(
            round_to(ts.x, 10), // 旋转+平移后 x 应相同，取 start.x
            ts.y,
            te.y,
        );
        vertical_lines.push(vl.clone());

        if debug {
            println!(
                "[归一化] 测线 {}: x={:.3}, y_start={:.3}, y_end={:.3}, direction={}",
                orig_idx, vl.x, vl.y_start, vl.y_end, vl.direction
            );
        }
    }

    // --- 6. 验证相邻间距相等 ---
    if vertical_lines.len() >= 2 {
        let spacings: Vec<f64> = (0..vertical_lines.len() - 1)
            .map(|i| vertical_lines[i + 1].x - vertical_lines[i].x)
            .collect();
        for (i, s) in spacings.iter().enumerate() {
            if *s <= 0.0 {
                return Err(format!("测线 {} 和 {} 的间距非正: {:.6}", i, i + 1, s));
            }
            if (s - spacings[0]).abs() > spacing_tol {
                return Err(format!(
                    "测线间距不等：间距[0]={:.3}, 间距[{:.}]={:.3}",
                    spacings[0], i, s
                ));
            }
        }

        if debug {
            println!(
                "[归一化] 测线间距 W = {:.3}，共 {} 条测线",
                spacings[0],
                vertical_lines.len()
            );
        }
    }

    let transform = Transform {
        rotation_angle,
        rotation_center,
        translation_dx,
        translation_dy,
    };

    Ok((vertical_lines, transform))
}

fn round_to(x: f64, decimals: i32) -> f64 {
    let factor = 10.0_f64.powi(decimals);
    (x * factor).round() / factor
}
