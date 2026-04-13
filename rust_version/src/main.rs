use cexian::common::VerticalLine;
use cexian::planner::{plan_route, random_route_distance, PlannerResult};
use plotters::prelude::*;

// =========================
// 可视化函数
// =========================

fn plot_route(
    lines: &[VerticalLine],
    result: &PlannerResult,
    avg_random_dist: f64,
    l_out: f64,
    l_in: f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let route = &result.route;
    let sides = &result.sides;
    let n = lines.len();
    let show_labels = n <= 20;

    // 计算每条测线的入口侧（决定行驶方向）
    let mut line_entry_side: std::collections::HashMap<usize, String> = std::collections::HashMap::new();
    for (k, &idx) in route.iter().enumerate() {
        let entry_side = if k == 0 {
            if !sides.is_empty() {
                if sides[0] == "CD" { "AB".to_string() } else { "CD".to_string() }
            } else {
                "AB".to_string()
            }
        } else {
            sides[k - 1].clone()
        };
        line_entry_side.insert(idx, entry_side);
    }

    // ==================== 图1：航线图 ====================
    // 首先计算坐标范围 - 考虑实际的 Run-out 和 Run-in 位置
    let x_min = lines.iter().map(|l| l.x).fold(f64::INFINITY, f64::min);
    let x_max = lines.iter().map(|l| l.x).fold(f64::NEG_INFINITY, f64::max);

    // 根据实际入口侧计算 Y 轴范围
    let mut actual_y_min = f64::INFINITY;
    let mut actual_y_max = f64::NEG_INFINITY;

    for (i, line) in lines.iter().enumerate() {
        if let Some(entry_side) = line_entry_side.get(&i) {
            if entry_side == "AB" {
                // 从 AB 侧进入（向上行驶）
                // Run-in 在 y_start 下方
                actual_y_min = actual_y_min.min(line.y_start - l_in);
                // Run-out 在 y_end 上方
                actual_y_max = actual_y_max.max(line.y_end + l_out);
            } else {
                // 从 CD 侧进入（向下行驶）
                // Run-out 在 y_start 下方
                actual_y_min = actual_y_min.min(line.y_start - l_out);
                // Run-in 在 y_end 上方
                actual_y_max = actual_y_max.max(line.y_end + l_in);
            }
        } else {
            // 如果没有入口侧信息，也要考虑测线本身的范围
            actual_y_min = actual_y_min.min(line.y_start.min(line.y_end));
            actual_y_max = actual_y_max.max(line.y_start.max(line.y_end));
        }
    }

    // 添加足够的边距（上下各 15%）
    let y_margin = (actual_y_max - actual_y_min) * 0.15;
    let mut y_min_plot = actual_y_min - y_margin;
    let mut y_max_plot = actual_y_max + y_margin;

    // 还需要考虑换线路径的实际 Y 坐标范围（三圆弧转弯可能延伸更远）
    let mut path_y_min = f64::INFINITY;
    let mut path_y_max = f64::NEG_INFINITY;
    for td in &result.turn_details {
        for &y in &td.path_ys {
            path_y_min = path_y_min.min(y);
            path_y_max = path_y_max.max(y);
        }
    }

    // 扩展 Y 范围以包含所有路径坐标
    y_min_plot = y_min_plot.min(path_y_min - y_margin * 0.5);
    y_max_plot = y_max_plot.max(path_y_max + y_margin * 0.5);

    // 根据数据范围计算合适的图片尺寸，保持宽高比
    let x_range = x_max - x_min;
    let y_range_total = y_max_plot - y_min_plot;
    let aspect_ratio = y_range_total / x_range;

    // 设置基础宽度，高度根据宽高比计算
    let img_width = 2400u32;
    let img_height = (img_width as f64 * aspect_ratio * 1.2) as u32; // 额外 20% 给标题和边距
    let img_height = img_height.max(1600); // 最小高度

    let root = BitMapBackend::new("route_map.png", (img_width, img_height)).into_drawing_area();
    // 设置白色背景
    root.fill(&WHITE)?;
    let root = root.margin(80, 80, 100, 100);

    let mut chart = ChartBuilder::on(&root)
        .caption("SA + Expert Rule Optimized Route", ("sans-serif", 32).with_color(BLACK))
        .x_label_area_size(60)
        .y_label_area_size(80)
        .build_cartesian_2d(x_min - 5000.0..x_max + 5000.0, y_min_plot..y_max_plot)?;

    chart.configure_mesh()
        .x_labels(10)
        .y_labels(15)
        .x_label_formatter(&|v| format!("{:.0}", v))
        .y_label_formatter(&|v| format!("{:.0}", v))
        .axis_style(BLACK)
        .draw()?;

    // 颜色设置
    let runout_color = RGBColor(105, 105, 105);      // dimgray
    let runin_color = RGBColor(255, 140, 0);         // darkorange
    let turn_color = RGBColor(34, 139, 34);          // forestgreen
    let blue_color = RGBColor(0, 0, 255);
    let red_color = RGBColor(255, 0, 0);

    // --------------------------------------------------
    // 第一层：画测线 (先画图例项，再画其他)
    // --------------------------------------------------
    // 图例项 - Line UP
    chart.draw_series(LineSeries::new(
        vec![(f64::NEG_INFINITY, f64::NEG_INFINITY)],
        blue_color.stroke_width(3),
    ))?.label("Line UP (+Y)").legend(move |(x, y)| {
        PathElement::new(vec![(x, y), (x + 30, y)], blue_color.stroke_width(3))
    });

    // 图例项 - Line DOWN
    chart.draw_series(LineSeries::new(
        vec![(f64::NEG_INFINITY, f64::NEG_INFINITY)],
        red_color.stroke_width(3),
    ))?.label("Line DOWN (-Y)").legend(move |(x, y)| {
        PathElement::new(vec![(x, y), (x + 30, y)], red_color.stroke_width(3))
    });

    // 图例项 - Turn path
    chart.draw_series(LineSeries::new(
        vec![(f64::NEG_INFINITY, f64::NEG_INFINITY)],
        turn_color.stroke_width(2),
    ))?.label("Turn path").legend(move |(x, y)| {
        PathElement::new(vec![(x, y), (x + 30, y)], turn_color.stroke_width(2))
    });

    // 图例项 - Run-out
    chart.draw_series(LineSeries::new(
        vec![(f64::NEG_INFINITY, f64::NEG_INFINITY)],
        runout_color.stroke_width(3),
    ))?.label("Run-out").legend(move |(x, y)| {
        PathElement::new(vec![(x, y), (x + 30, y)], runout_color.stroke_width(3))
    });

    // 图例项 - Run-in
    chart.draw_series(LineSeries::new(
        vec![(f64::NEG_INFINITY, f64::NEG_INFINITY)],
        runin_color.stroke_width(3),
    ))?.label("Run-in").legend(move |(x, y)| {
        PathElement::new(vec![(x, y), (x + 30, y)], runin_color.stroke_width(3))
    });

    // 画实际测线
    for (i, line) in lines.iter().enumerate() {
        let color = if let Some(entry_side) = line_entry_side.get(&i) {
            if entry_side == "AB" { &blue_color } else { &red_color }
        } else {
            &RGBColor(128, 128, 128)
        };

        chart.draw_series(LineSeries::new(
            vec![(line.x, line.y_start), (line.x, line.y_end)],
            color.stroke_width(3),
        ))?;
    }

    // --------------------------------------------------
    // 第二层：画绿色换线路径
    // --------------------------------------------------
    for k in 0..route.len() - 1 {
        let td = &result.turn_details[k];

        if !td.path_xs.is_empty() {
            let points: Vec<(f64, f64)> = td.path_xs.iter().zip(td.path_ys.iter()).map(|(&x, &y)| (x, y)).collect();
            chart.draw_series(LineSeries::new(points, turn_color.stroke_width(2)))?;
        }
    }

    // --------------------------------------------------
    // 第三层：叠加画 RUN OUT
    // --------------------------------------------------
    if l_out > 0.0 {
        for (i, line) in lines.iter().enumerate() {
            if let Some(entry_side) = line_entry_side.get(&i) {
                let (runout_x0, runout_y0, runout_y1) = if entry_side == "AB" {
                    (line.x, line.y_end, line.y_end + l_out)
                } else {
                    (line.x, line.y_start, line.y_start - l_out)
                };

                chart.draw_series(LineSeries::new(
                    vec![(runout_x0, runout_y0), (runout_x0, runout_y1)],
                    runout_color.stroke_width(3),
                ))?;
            }
        }
    }

    // --------------------------------------------------
    // 第四层：叠加画 RUN IN
    // --------------------------------------------------
    if l_in > 0.0 {
        for (i, line) in lines.iter().enumerate() {
            if let Some(entry_side) = line_entry_side.get(&i) {
                let (runin_x0, runin_y0, runin_y1) = if entry_side == "AB" {
                    (line.x, line.y_start - l_in, line.y_start)
                } else {
                    (line.x, line.y_end + l_in, line.y_end)
                };

                chart.draw_series(LineSeries::new(
                    vec![(runin_x0, runin_y0), (runin_x0, runin_y1)],
                    runin_color.stroke_width(3),
                ))?;
            }
        }
    }

    // 标记起始点
    let start_line = &lines[route[0]];
    let (sx, sy) = if !sides.is_empty() && sides[0] == "CD" {
        (start_line.x, start_line.y_start)
    } else {
        (start_line.x, start_line.y_end)
    };
    chart.draw_series(PointSeries::of_element(
        vec![(sx, sy)],
        10,
        BLACK.filled(),
        &|coord, size, style| {
            EmptyElement::at(coord)
                + Circle::new((0, 0), size, style)
        },
    ))?.label("Start").legend(move |(x, y)| {
        Circle::new((x, y), 5, BLACK.filled())
    });

    // 图例 - 手动绘制在图表右上角
    chart.configure_series_labels()
        .background_style(WHITE.mix(0.9))
        .border_style(BLACK)
        .label_font(("sans-serif", 20).into_font().color(&BLACK))
        .position(SeriesLabelPosition::UpperRight)
        .draw()?;

    root.present()?;
    println!("航线图已保存到: route_map.png");

    // ==================== 图2：换线距离柱状图 ====================
    let root2 = BitMapBackend::new("route_bar_chart.png", (2800, 1600)).into_drawing_area();
    // 设置白色背景
    root2.fill(&WHITE)?;
    let root2 = root2.margin(80, 80, 80, 80);

    let distances: Vec<f64> = result.turn_details.iter().map(|td| td.distance).collect();
    let max_dist = distances.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let mut chart2 = ChartBuilder::on(&root2)
        .caption("Turn Distance per Segment", ("sans-serif", 32).with_color(BLACK))
        .x_label_area_size(60)
        .y_label_area_size(80)
        .build_cartesian_2d(0usize..distances.len() + 1, 0.0..max_dist * 1.2)?;

    chart2.configure_mesh()
        .x_labels(distances.len())
        .y_labels(10)
        .axis_style(BLACK)
        .draw()?;

    // 绘制柱状图
    for (i, &dist) in distances.iter().enumerate() {
        chart2.draw_series(std::iter::once(Rectangle::new(
            [(i, 0.0), (i + 1, dist)],
            turn_color.filled(),
        )))?;
    }

    // 平均线
    let avg_opt = result.total_distance / (n - 1) as f64;
    let avg_random = avg_random_dist / (n - 1) as f64;

    chart2.draw_series(LineSeries::new(
        (0..=distances.len() + 1).map(|x| (x, avg_random)),
        RED.stroke_width(2),
    ))?;

    chart2.draw_series(LineSeries::new(
        (0..=distances.len() + 1).map(|x| (x, avg_opt)),
        blue_color.stroke_width(2),
    ))?;

    root2.present()?;
    println!("柱状图已保存到: route_bar_chart.png");

    Ok(())
}

// =========================
// 示例主程序
// =========================

fn main() {
    let r = 2800.0;
    let l_out = 3000.0;
    let l_in = 6200.0;
    let speed = 5.0;
    let w = 450.0;  // 测线间距 W < 2R = 5600
    let max_heading_angle_deg = 0.0;

    let n1 = 30;
    let n2 = 20;
    let n3 = 30;

    let mut lines: Vec<VerticalLine> = Vec::new();

    // 第一组测线
    for i in 0..n1 {
        let x = i as f64 * w;
        let y_offset = 3000.0;
        let length = 10000.0;
        let y_start = y_offset;
        let y_end = y_offset + length;
        lines.push(VerticalLine::new(x, y_start, y_end));
    }

    // 第二组测线
    for i in 0..n2 {
        let x = (i + n1) as f64 * w;
        let y_offset = 1000.0;
        let length = 10000.0;
        let y_start = y_offset;
        let y_end = y_offset + length;
        lines.push(VerticalLine::new(x, y_start, y_end));
    }

    // 第三组测线
    for i in 0..n3 {
        let x = (i + n1 + n2) as f64 * w;
        let y_offset = 200.0;
        let length = 10000.0;
        let y_start = y_offset;
        let y_end = y_offset + length;
        lines.push(VerticalLine::new(x, y_start, y_end));
    }

    println!("测线数量: {}", n1 + n2 + n3);
    println!("测线间距 W = {}, 2R = {}", w, 2.0 * r);
    println!("场景: W < 2R (三圆弧)");
    println!();

    println!("测线信息:");
    for (i, line) in lines.iter().enumerate() {
        let dir_char = if line.direction > 0 { '↑' } else { '↓' };
        println!(
            "  线{:2}: x={:7.0}, y=[{:7.0}, {:7.0}], 长度={:6.0}, 方向={}",
            i, line.x, line.y_start, line.y_end, line.length(), dir_char
        );
    }
    println!();

    println!("{}", "=".repeat(60));
    println!("SA + Expert Rule 航线规划");
    println!("{}", "=".repeat(60));

    let result = plan_route(
        &lines,
        r,
        l_out,
        l_in,
        speed,
        max_heading_angle_deg,
        12,     // block_top_k
        20,     // block_sa_restarts (unused)
        1000,   // block_sa_steps (unused)
        1500.0, // block_sa_init_temp (unused)
        0.995,  // block_sa_cooling (unused)
        15,     // global_sa_restarts
        2000,   // global_sa_steps
        3000.0, // global_sa_init_temp
        0.996,  // global_sa_cooling
        true,   // debug
    );

    println!();
    println!("{}", "=".repeat(60));
    println!("规划结果");
    println!("{}", "=".repeat(60));
    println!("总换线距离: {:.3}", result.total_distance);
    println!("总换线时间: {:.3}", result.total_distance / speed);
    println!();

    println!("航线顺序:");
    for (k, &idx) in result.route.iter().enumerate() {
        let line = &lines[idx];
        let entry_side = if k == 0 {
            if !result.sides.is_empty() {
                if result.sides[0] == "CD" { "AB" } else { "CD" }.to_string()
            } else {
                "-".to_string()
            }
        } else {
            result.sides[k - 1].clone()
        };

        let dir_char = if entry_side == "AB" {
            '↑'
        } else if entry_side == "CD" {
            '↓'
        } else {
            '?'
        };

        let side_str = if entry_side == "AB" { "AB侧(y_start)进" } else { "CD侧(y_end)进" };
        println!(
            "  第{:2}条: 线{:2} [{}] {} (x={:.0}, y=[{:.0}, {:.0}])",
            k + 1, idx, side_str, dir_char, line.x, line.y_start, line.y_end
        );
    }

    println!();
    println!("换线详情:");
    for td in &result.turn_details {
        println!(
            "  线{:2} → 线{:2} | {}侧 | {:5} | 距离={:.3}",
            td.from_line, td.to_line, td.from_side, td.scenario, td.distance
        );
    }

    println!();
    println!("{}", "=".repeat(60));
    println!("与随机路线对比");
    println!("{}", "=".repeat(60));

    let avg_random = random_route_distance(&lines, r, l_out, l_in, speed, 500);
    let improvement = (avg_random - result.total_distance) / avg_random * 100.0;

    println!("随机路线平均距离: {:.3}", avg_random);
    println!("SA + Expert Rule 优化距离: {:.3}", result.total_distance);
    println!("优化率: {:.1}%", improvement);

    // ===== 可视化 =====
    if let Err(e) = plot_route(&lines, &result, avg_random, l_out, l_in) {
        eprintln!("可视化失败: {}", e);
    }
}
