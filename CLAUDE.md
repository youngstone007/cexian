# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

此文件为 Claude Code (claude.ai/code) 在本代码库中工作提供指导。

## Overview / 概述

This repository contains a geometric path planning algorithm for ship lane switching between parallel survey lines. The algorithm handles three scenarios based on the distance between lines (W) relative to the ship's minimum turning radius (R):

本代码库包含一个用于船舶在平行测线之间换线的几何路径规划算法。该算法根据测线间距(W)相对于船舶最小转弯半径(R)的关系处理三种场景：

1. **W > 2R**: Uses Dubins CSC (LSL/RSR) paths with external/internal tangents.
   **W > 2R**: 使用 Dubins CSC (LSL/RSR) 路径，采用外切/内切公切线。

2. **W = 2R**: Uses a semicircular turn with straight approach/departure segments.
   **W = 2R**: 使用半圆形转弯，带有直线接近/离开段。

3. **W < 2R**: Uses a fixed symmetric three‑arc template (LRL or RLR) with optional straight extensions.
   **W < 2R**: 使用固定的对称三圆弧模板 (LRL 或 RLR)，带有可选的直线延伸段。

The code is a single‑file Python implementation (`main.py`) with dataclasses for geometric entities, pure geometric functions, solvers for each scenario, and visualization using matplotlib.

代码是单文件的 Python 实现 (`main.py`)，包含用于几何实体的数据类、纯几何函数、每种场景的求解器，以及使用 matplotlib 的可视化功能。

## Common Development Tasks / 常用开发任务

### Running the Example / 运行示例

```bash
python main.py
```

This executes the demonstration at the bottom of `main.py`, which:

这将执行 `main.py` 底部的演示，该演示：

- Defines two vertical survey lines (`line1`, `line2`), run‑out/run‑in distances (`L_out`, `L_in`), turning radius `R`, and ship speed.
  定义两条垂直测线 (`line1`, `line2`)、驶出/驶入距离 (`L_out`, `L_in`)、转弯半径 `R` 和船舶速度。

- Determines which scenario applies based on `W = |line2.x - line1.x|`.
  根据 `W = |line2.x - line1.x|` 确定适用的场景。

- Calls the appropriate solver and prints the resulting path parameters.
  调用相应的求解器并打印结果路径参数。

- Generates a plot of the turn solution and saves it as a PNG file (e.g., `turn_solution_w_lt_2r_three_arc_v2.png`).
  生成转弯解决方案的图表并保存为 PNG 文件（例如 `turn_solution_w_lt_2r_three_arc_v2.png`）。

### Dependencies / 依赖项

The only external dependency is **matplotlib** for plotting. Install it with:

唯一的外部依赖是用于绘图的 **matplotlib**。使用以下命令安装：

```bash
pip install matplotlib
```

No other third‑party packages are required; the code uses only the Python standard library (`math`, `dataclasses`, `typing`).

不需要其他第三方包；代码仅使用 Python 标准库 (`math`, `dataclasses`, `typing`)。

### Testing / 测试

There are no formal unit tests in the repository. To verify correctness, you can:

代码库中没有正式的单元测试。要验证正确性，您可以：

1. Run the example and inspect the generated plot.
   运行示例并检查生成的图表。

2. Modify the parameters in the `if __name__ == "__main__"` block and observe how the algorithm adapts to different W/R ratios.
   修改 `if __name__ == "__main__"` 块中的参数，观察算法如何适应不同的 W/R 比值。

3. Add debug prints by passing `debug=True` to the solver functions (e.g., `solve_fixed_head_tail_turn_between_vertical_lines(..., debug=True)`).
   通过向求解器函数传递 `debug=True` 添加调试打印（例如 `solve_fixed_head_tail_turn_between_vertical_lines(..., debug=True)`）。

### Code Structure / 代码结构

- **Data classes** (`Point`, `State`, `VerticalLine`, `DubinsPath`, `SemicircleTurnPath`, `ThreeArcTurnPath`, and associated `*Candidate` classes) define the geometric state and solution outputs.
  **数据类** (`Point`, `State`, `VerticalLine`, `DubinsPath`, `SemicircleTurnPath`, `ThreeArcTurnPath` 以及相关的 `*Candidate` 类) 定义几何状态和解决方案输出。

- **Geometric utilities**: `distance`, `mod2pi`, `left_normal`, `right_normal`, `polar_angle`, `compute_arc_angle`, `turning_circle_center`, `normalize`, `point_is_ahead_along_heading`.
  **几何工具函数**: `distance`, `mod2pi`, `left_normal`, `right_normal`, `polar_angle`, `compute_arc_angle`, `turning_circle_center`, `normalize`, `point_is_ahead_along_heading`。

- **Tangent‑consistency checking**: `tangent_direction_on_circle`, `tangent_direction_is_consistent`.
  **切线一致性检查**: `tangent_direction_on_circle`, `tangent_direction_is_consistent`。

- **CSC tangent solvers**: `solve_outer_tangent`, `solve_inner_tangent`, `solve_one_csc_path`, `shortest_dubins_csc`.
  **CSC 切线求解器**: `solve_outer_tangent`, `solve_inner_tangent`, `solve_one_csc_path`, `shortest_dubins_csc`。

- **Line‑point construction**: `point_on_vertical_line`, `build_eol_geom`, `build_sol_geom_fixed_head`.
  **测线点构造**: `point_on_vertical_line`, `build_eol_geom`, `build_sol_geom_fixed_head`。

- **Scenario‑specific solvers** / **场景特定求解器**:
  - `solve_fixed_head_tail_turn_between_vertical_lines` – for W > 2R. / 用于 W > 2R。
  - `solve_w_eq_2r_semicircle` and `solve_fixed_head_tail_w_eq_2r_semicircle_turn` – for W = 2R. / 用于 W = 2R。
  - `solve_w_lt_2r_three_arc` and `solve_fixed_head_tail_w_lt_2r_three_arc_turn` – for W < 2R. / 用于 W < 2R。

- **Printing and visualization**: `print_result`, `print_semicircle_result`, `print_three_arc_result`, `plot_turn_solution`, `plot_semicircle_turn_solution`, `plot_three_arc_turn_solution`.
  **打印和可视化**: `print_result`, `print_semicircle_result`, `print_three_arc_result`, `plot_turn_solution`, `plot_semicircle_turn_solution`, `plot_three_arc_turn_solution`。

### Key Assumptions and Conventions / 关键假设和约定

- Survey lines are **vertical** (parallel to the Y‑axis) and defined by `x`, `y_start`, `y_end`, and a `direction` (+1 for +Y, -1 for -Y).
  测线是**垂直的**（平行于 Y 轴），由 `x`, `y_start`, `y_end` 和 `direction` 定义（+1 表示 +Y 方向，-1 表示 -Y 方向）。

- The ship exits the first line at its **end‑of‑line (EOL)** data point (`y_end`) and must travel a **run‑out distance** `L_out` before beginning the turn.
  船舶在第一条测线的**线尾 (EOL)** 数据点 (`y_end`) 退出，必须在开始转弯前行驶**驶出距离** `L_out`。

- The ship enters the second line at its **start‑of‑line (SOL)** data point (`y_start`) but must first travel a **run‑in distance** `L_in` along the line to straighten the tow cable.
  船舶在第二条测线的**线头 (SOL)** 数据点 (`y_start`) 进入，但必须首先沿测线行驶**驶入距离** `L_in` 以拉直拖缆。

- The **EOL geometric point** is `L_out` beyond the EOL data point along the line's heading; the **SOL geometric point** is `L_in` before the SOL data point along the line's heading.
  **EOL 几何点**是沿测线航向超出 EOL 数据点 `L_out` 的点；**SOL 几何点**是沿测线航向在 SOL 数据点之前 `L_in` 的点。

- The turn is planned between the two geometric points, with the ship's heading at each point equal to the line's heading.
  转弯计划在两个几何点之间进行，船舶在每个点的航向等于测线的航向。

- All angles are in radians, distances in consistent units (e.g., meters), speed in distance‑per‑time.
  所有角度单位为弧度，距离单位为一致的单位（例如米），速度单位为距离/时间。

### Extending the Code / 扩展代码

- To handle **non‑vertical lines**, modify `VerticalLine` and the point‑construction functions.
  要处理**非垂直测线**，请修改 `VerticalLine` 和点构造函数。

- To add **more Dubins path types** (e.g., CCC), extend `shortest_dubins_csc` with additional solvers.
  要添加**更多 Dubins 路径类型**（例如 CCC），请使用额外的求解器扩展 `shortest_dubins_csc`。

- To **change the optimization criterion** (currently shortest path length), adjust the `total_cost` field in the candidate classes.
  要**更改优化标准**（当前为最短路径长度），请调整候选类中的 `total_cost` 字段。

- To **integrate the solver into a larger system**, import the relevant functions and call them with your own line/parameter objects.
  要**将求解器集成到更大的系统中**，请导入相关函数并使用您自己的测线/参数对象调用它们。

## Notes for Maintainers / 维护者注意事项

- The code is written in English with Chinese comments; keep both for clarity.
  代码用英文编写，带有中文注释；为清晰起见，请保留两者。

- There are no configuration files, build scripts, or test suites. Consider adding a `requirements.txt` and a simple test module if the project grows.
  没有配置文件、构建脚本或测试套件。如果项目增长，请考虑添加 `requirements.txt` 和简单的测试模块。

- The visualization functions assume a Cartesian coordinate system with equal aspect ratio; they are meant for debugging and presentation, not for production use.
  可视化函数假设使用等比例的笛卡尔坐标系；它们用于调试和演示，不用于生产环境。

- All geometric tolerances are controlled by `EPS` and `DOT_TOL` global constants; adjust them if numerical issues arise.
  所有几何公差由全局常量 `EPS` 和 `DOT_TOL` 控制；如果出现数值问题，请调整它们。