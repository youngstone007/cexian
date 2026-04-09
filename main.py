from common import VerticalLine

from dubins_csc import (
    solve_fixed_head_tail_turn_between_vertical_lines,
    print_result,
    plot_turn_solution
)

from semicircle import (
    solve_fixed_head_tail_w_eq_2r_semicircle_turn,
    print_semicircle_result,
    plot_semicircle_turn_solution
)

from three_arc import (
    solve_fixed_head_tail_w_lt_2r_three_arc_turn,
    print_three_arc_result,
    plot_three_arc_turn_solution
)


# =========================
# 示例主程序
# =========================

if __name__ == "__main__":
    # 你当前测试例
	# 左上->右下
    line1 = VerticalLine(
        x=0.0,
        y_start=0.0,
        y_end=4000.0,
        direction=+1
    )

    line2 = VerticalLine(
        x=300.0,
        y_start=4500.0,
        y_end=900.0,
        direction=-1
    )
	
	# # 左下->右上
    # line1 = VerticalLine(
    #     x=0.0,
    #     y_start=4000.0,
    #     y_end=0.0,
    #     direction=-1
    # )

    # line2 = VerticalLine(
    #     x=300.0,
    #     y_start=900.0,
    #     y_end=4500.0,
    #     direction=+1
    # )
	
	
	# # 右上->左下
    # line2 = VerticalLine(
    #     x=0.0,
    #     y_start=4000.0,
    #     y_end=0.0,
    #     direction=-1
    # )

    # line1 = VerticalLine(
    #     x=300.0,
    #     y_start=900.0,
    #     y_end=4500.0,
    #     direction=+1
    # )
	
	# # 右下->左上
    # line2 = VerticalLine(
    #     x=0.0,
    #     y_start=0.0,
    #     y_end=4000.0,
    #     direction=+1
    # )

    # line1 = VerticalLine(
    #     x=300.0,
    #     y_start=4500.0,
    #     y_end=900.0,
    #     direction=-1
    # )

    L_out = 300.0
    L_in = 300.0
    R = 250.0
    speed = 5.0

    W = abs(line2.x - line1.x)
    print(f"当前测线间距 W = {W:.3f}, 2R = {2.0 * R:.3f}")

    if W > 2.0 * R + 1e-6:
        best = solve_fixed_head_tail_turn_between_vertical_lines(
            line1=line1,
            line2=line2,
            L_out=L_out,
            L_in=L_in,
            R=R,
            speed=speed,
            debug=True
        )
        print_result(best)
        plot_turn_solution(
            line1, line2, best, R,
            show_circles=True,
            figsize=(12, 14),
            dpi=300,
            save_path="turn_solution_csc.png"
        )

    elif abs(W - 2.0 * R) <= 1e-6:
        best_semi = solve_fixed_head_tail_w_eq_2r_semicircle_turn(
            line1=line1,
            line2=line2,
            L_out=L_out,
            L_in=L_in,
            R=R,
            speed=speed,
            debug=True
        )
        print_semicircle_result(best_semi)
        plot_semicircle_turn_solution(
            line1, line2, best_semi, R,
            figsize=(12, 14),
            dpi=300,
            save_path="turn_solution_w_eq_2r.png"
        )

    else:
        best_three = solve_fixed_head_tail_w_lt_2r_three_arc_turn(
            line1=line1,
            line2=line2,
            L_out=L_out,
            L_in=L_in,
            R=R,
            speed=speed,
            debug=True
        )
        print_three_arc_result(best_three)
        plot_three_arc_turn_solution(
            line1, line2, best_three, R,
            figsize=(12, 14),
            dpi=300,
            save_path="turn_solution_w_lt_2r_three_arc_v2.png"
        )
