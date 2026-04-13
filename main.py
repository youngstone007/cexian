import random
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from common import Point, VerticalLine
from planner import plan_route, random_route_distance


# =========================
# 可视化函数
# =========================

def plot_route(lines, result, avg_random_dist, L_out, L_in):
    """绘制规划路线可视化图（航线图 + 柱状图 分开保存）"""
    route = result.route
    sides = result.sides
    N = len(lines)
    show_labels = N <= 20  # 测线多于20条时不显示编号

    # 计算每条测线的入口侧（决定行驶方向）
    line_entry_side = {}
    for k, idx in enumerate(route):
        if k == 0:
            entry_side = ("AB" if sides[0] == "CD" else "CD") if len(sides) > 0 else "AB"
        else:
            entry_side = sides[k - 1]
        line_entry_side[idx] = entry_side

    # ==================== 图1：航线图 ====================
    fig1, ax = plt.subplots(figsize=(16, 20), dpi=200)
    ax.set_title("SA + Expert Rule Optimized Route", fontsize=16)

    # 颜色设置
    runout_color = "dimgray"
    runin_color = "darkorange"
    turn_color = "green"

    # --------------------------------------------------
    # 第一层：画测线
    # --------------------------------------------------
    for i, line in enumerate(lines):
        if i in line_entry_side:
            color = 'blue' if line_entry_side[i] == "AB" else 'red'
        else:
            color = 'gray'

        ax.plot(
            [line.x, line.x],
            [line.y_start, line.y_end],
            color=color,
            linewidth=2.5,
            alpha=0.75,
            zorder=2
        )

        if show_labels and i in line_entry_side:
            mid_y = (line.y_start + line.y_end) / 2
            dy = 200 if line_entry_side.get(i) == "AB" else -200
            ax.annotate(
                "",
                xy=(line.x, mid_y + dy),
                xytext=(line.x, mid_y),
                arrowprops=dict(arrowstyle="->", color=color, lw=1.5),
                zorder=3
            )
            ax.text(
                line.x, mid_y, str(i),
                ha='center', va='center',
                fontsize=8, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='gray', alpha=0.8),
                zorder=3
            )

    # --------------------------------------------------
    # 第二层：画绿色换线路径
    # --------------------------------------------------
    for k in range(len(route) - 1):
        td = result.turn_details[k]

        if td.path_xs:
            ax.plot(
                td.path_xs, td.path_ys,
                color=turn_color,
                linewidth=1.5,
                alpha=0.8,
                zorder=4
            )

            if show_labels:
                mid = len(td.path_xs) // 2
                mx, my = td.path_xs[mid], td.path_ys[mid]
                ax.text(
                    mx, my, str(k + 1),
                    fontsize=7, ha='center', va='center',
                    color=turn_color, fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.7),
                    zorder=5
                )
        else:
            li = lines[route[k]]
            lj = lines[route[k + 1]]
            side = sides[k]
            if side == "CD":
                p_out = (li.x, li.y_end)
                p_in = (lj.x, lj.y_end)
            else:
                p_out = (li.x, li.y_start)
                p_in = (lj.x, lj.y_start)
            ax.annotate(
                "",
                xy=p_in,
                xytext=p_out,
                arrowprops=dict(arrowstyle="->", color=turn_color, lw=1.8, alpha=0.7),
                zorder=4
            )

    # --------------------------------------------------
    # 第三层：叠加画 RUN OUT（粗实线，在线路绿色之上）
    # 箭头放在 RUN OUT 末端
    # --------------------------------------------------
    if L_out > 0:
        for i, line in enumerate(lines):
            if i not in line_entry_side:
                continue

            entry_side = line_entry_side[i]

            if entry_side == "AB":
                # 向上施工：从 y_end 继续向上
                runout_x0, runout_y0 = line.x, line.y_end
                runout_x1, runout_y1 = line.x, line.y_end + L_out

                # 箭头放末端
                arrow_x0, arrow_y0 = line.x, line.y_end + 0.75 * L_out
                arrow_x1, arrow_y1 = line.x, line.y_end + L_out
            else:
                # 向下施工：从 y_start 继续向下
                runout_x0, runout_y0 = line.x, line.y_start
                runout_x1, runout_y1 = line.x, line.y_start - L_out

                # 箭头放末端
                arrow_x0, arrow_y0 = line.x, line.y_start - 0.75 * L_out
                arrow_x1, arrow_y1 = line.x, line.y_start - L_out

            ax.plot(
                [runout_x0, runout_x1],
                [runout_y0, runout_y1],
                color=runout_color,
                linewidth=2.8,
                linestyle='-',
                alpha=0.95,
                solid_capstyle='round',
                zorder=6
            )

            ax.annotate(
                "",
                xy=(arrow_x1, arrow_y1),
                xytext=(arrow_x0, arrow_y0),
                arrowprops=dict(
                    arrowstyle="->",
                    color=runout_color,
                    lw=2.4,
                    alpha=0.98
                ),
                zorder=7
            )

    # --------------------------------------------------
    # 第四层：叠加画 RUN IN（粗实线，在线路绿色之上）
    # 箭头放在 RUN IN 开始端
    # --------------------------------------------------
    if L_in > 0:
        for i, line in enumerate(lines):
            if i not in line_entry_side:
                continue

            entry_side = line_entry_side[i]

            if entry_side == "AB":
                # 向上施工：从下方进入 y_start
                runin_x0, runin_y0 = line.x, line.y_start - L_in
                runin_x1, runin_y1 = line.x, line.y_start

                # 箭头放开始端，指向上线方向
                arrow_x0, arrow_y0 = line.x, line.y_start - L_in
                arrow_x1, arrow_y1 = line.x, line.y_start - 0.75 * L_in
            else:
                # 向下施工：从上方进入 y_end
                runin_x0, runin_y0 = line.x, line.y_end + L_in
                runin_x1, runin_y1 = line.x, line.y_end

                # 箭头放开始端，指向上线方向
                arrow_x0, arrow_y0 = line.x, line.y_end + L_in
                arrow_x1, arrow_y1 = line.x, line.y_end + 0.75 * L_in

            ax.plot(
                [runin_x0, runin_x1],
                [runin_y0, runin_y1],
                color=runin_color,
                linewidth=2.8,
                linestyle='-',
                alpha=0.95,
                solid_capstyle='round',
                zorder=8
            )

            ax.annotate(
                "",
                xy=(arrow_x1, arrow_y1),
                xytext=(arrow_x0, arrow_y0),
                arrowprops=dict(
                    arrowstyle="->",
                    color=runin_color,
                    lw=2.4,
                    alpha=0.98
                ),
                zorder=9
            )

    # 标记起始点
    start_line = lines[route[0]]
    if len(sides) > 0 and sides[0] == "CD":
        sx, sy = start_line.x, start_line.y_start
    else:
        sx, sy = start_line.x, start_line.y_end

    ax.scatter([sx], [sy], color='black', s=220, marker='*', zorder=10)

    legend_elements = [
        Line2D([0], [0], color='blue', linewidth=2.5, label='Line UP (+Y)'),
        Line2D([0], [0], color='red', linewidth=2.5, label='Line DOWN (-Y)'),
        Line2D([0], [0], color=runout_color, linewidth=2.8, linestyle='-', label='Run-out'),
        Line2D([0], [0], color=runin_color, linewidth=2.8, linestyle='-', label='Run-in'),
        Line2D([0], [0], color=turn_color, linewidth=1.5, label='Turn path'),
        Line2D([0], [0], marker='*', color='black', linestyle='None',
               markersize=10, label='Start'),
    ]
    ax.legend(handles=legend_elements, fontsize=11)
    ax.set_xlabel("X", fontsize=12)
    ax.set_ylabel("Y", fontsize=12)
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.set_aspect("equal", adjustable="box")

    fig1.tight_layout()
    fig1.savefig("route_map.png", dpi=200, bbox_inches="tight")
    print(f"航线图已保存到: route_map.png")

    # ==================== 图2：换线距离柱状图 ====================
    fig2, ax2 = plt.subplots(figsize=(14, 8), dpi=200)
    ax2.set_title("Turn Distance per Segment", fontsize=16)

    distances = [td.distance for td in result.turn_details]

    x_pos = range(len(distances))
    ax2.bar(x_pos, distances, color='green', alpha=0.8, edgecolor='gray')

    if show_labels:
        labels = [f"{td.from_line}->{td.to_line}" for td in result.turn_details]
        ax2.set_xticks(list(x_pos))
        ax2.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
    else:
        ax2.set_xticks(list(x_pos))
        ax2.set_xticklabels([str(k + 1) for k in x_pos], fontsize=7)

    ax2.set_xlabel("Turn #", fontsize=12)
    ax2.set_ylabel("Turn Distance", fontsize=12)
    ax2.axhline(
        y=avg_random_dist / (len(lines) - 1),
        color='red',
        linestyle='--',
        linewidth=1.5,
        label='Random avg per segment'
    )
    ax2.axhline(
        y=result.total_distance / (len(lines) - 1),
        color='blue',
        linestyle='--',
        linewidth=1.5,
        label='Optimized avg per segment'
    )

    textstr = (
        f"Optimized total: {result.total_distance:.0f}\n"
        f"Random avg total: {avg_random_dist:.0f}\n"
        f"Improvement: {(avg_random_dist - result.total_distance) / avg_random_dist * 100:.1f}%"
    )
    ax2.text(
        0.98, 0.98, textstr,
        transform=ax2.transAxes,
        fontsize=11,
        verticalalignment='top',
        horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8)
    )

    ax2.legend(fontsize=10)
    ax2.grid(True, linestyle="--", alpha=0.3, axis='y')

    fig2.tight_layout()
    fig2.savefig("route_bar_chart.png", dpi=200, bbox_inches="tight")
    print(f"柱状图已保存到: route_bar_chart.png")

    plt.show()


# =========================
# 示例主程序
# =========================

if __name__ == "__main__":
    R = 2800.0
    L_out = 3000.0
    L_in = 6200.0
    speed = 5.0
    W = 450.0  # 测线间距 W < 2R = 5600
    max_heading_angle_deg = 0.0

    N1 = 30
    random.seed(42)

    lines = []
    for i in range(N1):
        x = i * W
        length = random.uniform(10000, 10000)
        y_offset = random.uniform(3000, 3000)
        y_start = y_offset
        y_end = y_offset + length
        lines.append(VerticalLine(x=x, y_start=y_start, y_end=y_end))

    N2 = 20
    random.seed(42)

    for i in range(N2):
        x = (i + N1) * W
        length = random.uniform(10000, 10000)
        y_offset = random.uniform(1000, 1000)
        y_start = y_offset
        y_end = y_offset + length
        lines.append(VerticalLine(x=x, y_start=y_start, y_end=y_end))

    N3 = 30
    random.seed(42)

    for i in range(N3):
        x = (i + N1 + N2) * W
        length = random.uniform(10000, 10000)
        y_offset = random.uniform(200, 200)
        y_start = y_offset
        y_end = y_offset + length
        lines.append(VerticalLine(x=x, y_start=y_start, y_end=y_end))

    print(f"测线数量: {N1 + N2 + N3}")
    print(f"测线间距 W = {W}, 2R = {2 * R}")
    print(f"场景: W < 2R (三圆弧)")
    print()

    print("测线信息:")
    for i, line in enumerate(lines):
        print(
            f"  线{i:2d}: x={line.x:7.0f}, y=[{line.y_start:7.0f}, {line.y_end:7.0f}], "
            f"长度={line.length:6.0f}, 方向={'↑' if line.direction > 0 else '↓'}"
        )
    print()

    print("=" * 60)
    print("SA + Expert Rule 航线规划")
    print("=" * 60)

    result = plan_route(
        lines,
        R=R,
        L_out=L_out,
        L_in=L_in,
        speed=speed,
        max_heading_angle_deg=max_heading_angle_deg,
        block_top_k=12,
        block_sa_restarts=20,
        block_sa_steps=1000,
        global_sa_restarts=15,
        global_sa_steps=2000,
        debug=True,
    )

    print()
    print("=" * 60)
    print("规划结果")
    print("=" * 60)
    print(f"总换线距离: {result.total_distance:.3f}")
    print(f"总换线时间: {result.total_distance / speed:.3f}")
    print()

    print("航线顺序:")
    for k, idx in enumerate(result.route):
        line = lines[idx]
        if k == 0:
            if len(result.sides) > 0:
                entry_side = "AB" if result.sides[0] == "CD" else "CD"
            else:
                entry_side = "-"
        else:
            entry_side = result.sides[k - 1]

        if entry_side == "AB":
            dir_str = "↑"
        elif entry_side == "CD":
            dir_str = "↓"
        else:
            dir_str = "?"
        side_str = "AB侧(y_start)进" if entry_side == "AB" else "CD侧(y_end)进"
        print(
            f"  第{k + 1:2d}条: 线{idx:2d} [{side_str}] {dir_str} "
            f"(x={line.x:.0f}, y=[{line.y_start:.0f}, {line.y_end:.0f}])"
        )

    print()
    print("换线详情:")
    for k, td in enumerate(result.turn_details):
        print(
            f"  线{td.from_line:2d} → 线{td.to_line:2d} | "
            f"{td.from_side}侧 | {td.scenario:5s} | 距离={td.distance:.3f}"
        )

    print()
    print("=" * 60)
    print("与随机路线对比")
    print("=" * 60)
    avg_random = random_route_distance(lines, R, L_out, L_in, speed, n_trials=500)
    improvement = (avg_random - result.total_distance) / avg_random * 100
    print(f"随机路线平均距离: {avg_random:.3f}")
    print(f"SA + Expert Rule 优化距离: {result.total_distance:.3f}")
    print(f"优化率: {improvement:.1f}%")

    # ===== 可视化 =====
    plot_route(lines, result, avg_random, L_out, L_in)