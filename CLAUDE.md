下面是我按**你当前项目真实结构与当前实现逻辑**整理后的 `CLAUDE.md` 完整版本。  
你可以直接覆盖原文件。

```markdown
# CLAUDE.md

This file provides guidance to Claude Code (`claude.ai/code`) when working with code in this repository.

此文件为 Claude Code (`claude.ai/code`) 在本代码库中工作提供指导。

---

## Overview / 概述

This repository implements a geometric path planning and global route optimization system for ship lane switching between **parallel vertical survey lines**.

本代码库实现了一个面向**平行竖直测线**的船舶换线几何求解与全局路线优化系统。

The project contains two tightly coupled parts:

本项目包含两个紧密相关的部分：

1. **Local turn geometry solver**  
   Computes the feasible turn path between two survey lines while considering:
   - fixed minimum turning radius $R$
   - run-out distance `L_out`
   - run-in distance `L_in`
   - fixed heading at both ends

   **局部换线几何求解器**  
   在以下约束下，计算两条测线之间的可行换线路径：
   - 固定最小转弯半径 $R$
   - `Run out` 距离 `L_out`
   - `Run in` 距离 `L_in`
   - 起终点航向固定

2. **Global planner**  
   Optimizes the visiting order of many survey lines using:
   - expert-rule spatial partitioning
   - same-direction constraints inside spatial blocks
   - interleaving access between neighboring blocks
   - simulated annealing (SA) + dynamic programming (DP)

   **全局规划器**  
   对多条测线的访问顺序进行优化，结合：
   - 专家经验法空间分块
   - 区块内同向约束
   - 相邻区块交错访问
   - 模拟退火（SA）+ 动态规划（DP）

---

## Current Repository Structure / 当前代码结构

The repository is now a **multi-file Python project**, not a single-file implementation.

当前代码库已经是**多文件 Python 项目**，不再是单文件实现。

```text
project/
├── common.py         # 基础数据结构、几何常量、采样函数
├── dubins_csc.py     # W > 2R 场景：Dubins CSC (LSL/RSR)
├── semicircle.py     # W = 2R 场景：半圆换线
├── three_arc.py      # W < 2R 场景：固定对称三圆弧换线
├── planner.py        # 全局规划器：空间分块、候选生成、SA+DP
├── main.py           # 示例入口、打印结果、可视化绘图
├── README.md         # 项目说明文档（若存在）
└── CLAUDE.md         # 当前说明文件
```

---

## Core Problem / 核心问题

The project studies how a ship switches from one survey line to another adjacent or non-adjacent parallel survey line while keeping the towing system operationally valid.

本项目研究船舶如何从一条测线切换到另一条相邻或非相邻平行测线，并保证拖缆系统满足作业要求。

Key constraints:

关键约束包括：

- Survey lines are parallel and vertical.
  测线相互平行，且在当前实现中均为竖直线。

- The ship must continue along the current line direction for a `Run out` distance after finishing data acquisition.
  船舶完成当前测线采集后，必须继续沿当前测线方向航行一段 `Run out` 距离。

- Before entering the next line for acquisition, the ship must travel along that line direction for a `Run in` distance so the tow cable is straightened and aligned with the survey line.
  在进入目标测线正式采集前，船舶必须预留一段 `Run in` 距离，使拖缆被拉直并与目标测线平行。

- Turn geometry is planned between the **EOL geometric point** and the **SOL geometric point**.
  换线几何是在 **EOL 几何点** 与 **SOL 几何点** 之间求解的。

---

## Geometry Conventions / 几何约定

### AB / CD Side Convention / AB / CD 侧别约定

The current project uses **side-based line entry/exit modeling**.

当前项目使用**基于侧别的测线进出建模**。

For each vertical survey line:

对于每条竖直测线：

- `AB` side = lower endpoint = `y_start`
- `CD` side = upper endpoint = `y_end`

That means:

这意味着：

- Entering from `AB` means the ship travels upward along the line, i.e. along $+Y$.
  从 `AB` 侧进入表示船舶沿测线上行，即沿 $+Y$ 方向施工。

- Entering from `CD` means the ship travels downward along the line, i.e. along $-Y$.
  从 `CD` 侧进入表示船舶沿测线下行，即沿 $-Y$ 方向施工。

- A line entered from `AB` exits from `CD`.
  从 `AB` 侧进入的测线最终从 `CD` 侧离开。

- A line entered from `CD` exits from `AB`.
  从 `CD` 侧进入的测线最终从 `AB` 侧离开。

This convention is central to both the local turn solvers and the global planner.

这一约定同时是局部换线求解器与全局规划器的核心基础。

---

### EOL / SOL Definitions / EOL / SOL 定义

#### EOL data point / EOL 数据点

The End-Of-Line data point is the actual survey exit point on the current line.

EOL 数据点是当前测线上的实际采集结束点。

Its location depends on the current line direction:

其位置由当前测线施工方向决定：

- upward line: EOL is on `CD`
- downward line: EOL is on `AB`

#### EOL geometric point / EOL 几何点

The EOL geometric point is obtained by extending from the EOL data point along the current heading by `L_out`.

EOL 几何点是从 EOL 数据点沿当前航向继续前进 `L_out` 后得到的点。

This is the actual start of turn geometry.

它是换线几何的实际起点。

#### SOL data point / SOL 数据点

The Start-Of-Line data point is the actual acquisition start point on the target line.

SOL 数据点是目标测线上的实际采集起始点。

Its location depends on the target line direction:

其位置由目标测线施工方向决定：

- upward line: SOL is on `AB`
- downward line: SOL is on `CD`

#### SOL geometric point / SOL 几何点

The SOL geometric point is obtained by moving backward from the SOL data point, against the target line heading, by `L_in`.

SOL 几何点是从 SOL 数据点沿目标航向反方向回退 `L_in` 后得到的点。

This is the actual end of turn geometry.

它是换线几何的实际终点。

---

## Three Turn Scenarios / 三类换线场景

The local turn solver handles three scenarios depending on the relationship between line spacing $W$ and turning radius $R$.

局部换线求解器根据测线间距 $W$ 与转弯半径 $R$ 的关系处理三类场景。

### 1. $W > 2R$

Use Dubins CSC paths.

使用 Dubins CSC 路径。

Current implementation compares:

当前实现比较：

- `LSL`
- `RSR`

and chooses the shorter feasible one.

并在可行解中取更短者。

Path structure:

路径结构：

$$
\text{Run out} \to \text{arc} \to \text{tangent line} \to \text{arc} \to \text{Run in}
$$

---

### 2. $W = 2R$

Use a semicircle-based turn.

使用半圆换线模型。

The tangent line degenerates, so the ship turns through a semicircle, optionally preceded or followed by straight segments.

由于公切线退化，船舶通过半圆完成换线，必要时在前后补充直线段。

Path structure:

路径结构：

$$
\text{Run out} \to \text{pre-straight} \to \text{semicircle} \to \text{post-straight} \to \text{Run in}
$$

---

### 3. $W < 2R$

Use a fixed symmetric three-arc template.

使用固定对称三圆弧模板。

Current implementation uses:

当前实现使用：

- `LRL`
- `RLR`

depending on left-to-right or right-to-left switching.

根据左右换线方向选择 `LRL` 或 `RLR`。

Path structure:

路径结构：

$$
\text{Run out} \to \text{pre-straight} \to \text{three-arc template} \to \text{post-straight} \to \text{Run in}
$$

---

## Global Planning Model / 全局规划模型

The current planner is no longer a simple random or Q-learning route chooser.

当前规划器已经不再是简单的随机或 Q-learning 路线选择器。

It is a structured global planner based on:

它是一个结构化的全局规划器，核心包括：

1. **Expert-rule spatial partitioning**
   - Survey lines are partitioned into spatial blocks according to maximum allowable span.
   - 测线先根据最大允许横向跨度划分为空间块。

2. **Refined block pairing**
   - Raw spatial blocks may be subdivided so they can be paired feasibly.
   - 原始空间块会在必要时进一步细分，以保证可配对交错访问。

3. **Same-direction inside block**
   - All lines inside one refined block share one direction.
   - 每个细分块内部所有测线方向一致。

4. **Interleaving between paired blocks**
   - Neighboring paired blocks are accessed in an interleaved way.
   - 配对的相邻细分块之间采用交错访问。

5. **Global optimization over work units**
   - Work-unit order is optimized using SA.
   - For a fixed unit order, candidate chaining is solved by DP.
   - 工作单元顺序由 SA 优化；
   - 固定顺序下的候选拼接由 DP 精确求解。

This design is important: do **not** describe the current planner as “block-internal TSP only”.

这一点非常重要：**不要**再把当前规划器描述成“仅仅是块内 TSP”。

---

## Main Entry Points / 主要入口

### `main.py`

This is the example and visualization entry.

这是示例运行与可视化入口。

Typical usage:

典型运行方式：

```bash
python main.py
```

It typically:

它通常会：

- generate many parallel survey lines
- call `plan_route(...)`
- print the route and turn details
- compare with random routes
- plot the final route map and turn-distance bar chart

即：

- 生成多条平行测线
- 调用 `plan_route(...)`
- 打印全局航线与换线详情
- 与随机路线进行对比
- 输出航线图和换线距离柱状图

---

### `planner.plan_route(...)`

This is the main global planning API.

这是当前全局规划的主要接口。

Typical parameters:

常用参数包括：

- `lines`
- `R`
- `L_out`
- `L_in`
- `speed`
- `max_heading_angle_deg`
- `block_top_k`
- `global_sa_restarts`
- `global_sa_steps`
- `debug`

---

## Important Modules / 关键模块

### `common.py`

Contains fundamental geometric data structures and sampling helpers.

包含基础几何数据结构与路径采样辅助函数。

Typical contents:

典型内容包括：

- `Point`
- `VerticalLine`
- `EPS`
- `sample_arc(...)`
- `sample_line(...)`

---

### `dubins_csc.py`

Solver for the $W > 2R$ case.

用于 $W > 2R$ 场景的求解器。

Typical responsibility:

典型职责：

- fixed-head and fixed-tail Dubins CSC turn construction
- LSL / RSR candidate solving
- shortest feasible CSC selection

---

### `semicircle.py`

Solver for the $W = 2R$ case.

用于 $W = 2R$ 场景的半圆换线求解器。

---

### `three_arc.py`

Solver for the $W < 2R$ case.

用于 $W < 2R$ 场景的三圆弧换线求解器。

---

### `planner.py`

This is the most important orchestration module.

这是当前最核心的规划调度模块。

It includes:

它包括：

- turn distance abstraction
- expert-rule block partitioning
- refined block generation
- unit candidate generation
- interleaving construction
- work-unit connector building
- SA over unit order
- DP over candidate chain
- final route assembly

即：

- 换线距离统一封装
- 专家经验法分块
- 细分块生成
- 工作单元候选生成
- 区块交错访问构造
- 工作单元连接器构造
- 单元顺序 SA 优化
- 候选链 DP 求解
- 最终全局路线拼装

---

## Common Development Tasks / 常用开发任务

### Run the demo / 运行示例

```bash
python main.py
```

---

### Install dependencies / 安装依赖

The main external dependency is **matplotlib** for plotting.

当前主要外部依赖是用于绘图的 **matplotlib**。

```bash
pip install matplotlib
```

The rest of the code uses only the Python standard library.

其余部分主要依赖 Python 标准库。

---

### Debugging / 调试

Useful debugging methods:

常见调试方式包括：

1. Set `debug=True` in `plan_route(...)`.
   在 `plan_route(...)` 中设置 `debug=True`。

2. Inspect:
   - spatial blocks
   - refined block sizes
   - generated unit candidates
   - chosen global unit order

   查看：
   - 空间块划分
   - 细分块尺寸
   - 工作单元候选
   - 最终全局工作单元顺序

3. Check the plotted route map.
   查看输出的航线图。

---

## Testing / 测试

There is currently **no formal unit test suite**.

当前代码库**没有正式单元测试**。

To validate behavior:

可通过以下方式验证行为：

1. Run `main.py` and inspect the generated route map.
   运行 `main.py` 并检查生成的航线图。

2. Change:
   - line spacing $W$
   - turning radius $R$
   - `L_out`
   - `L_in`
   - block parameters

   修改：
   - 测线间距 $W$
   - 转弯半径 $R$
   - `L_out`
   - `L_in`
   - 分块参数

3. Verify the planner still produces:
   - feasible route
   - same-direction blocks
   - sensible interleaving
   - improved total turn distance

   验证规划器是否仍然产生：
   - 可行路线
   - 区块内同向
   - 合理交错访问
   - 更优总换线距离

---

## Important Assumptions / 重要假设

1. **Survey lines are vertical.**  
   Current line geometry assumes vertical lines only.

   **测线为竖直线。**  
   当前实现只考虑竖直测线。

2. **Turn radius is fixed.**  
   All turn construction is based on one fixed minimum turning radius $R$.

   **转弯半径固定。**  
   所有换线几何均基于固定最小转弯半径 $R$。

3. **Run-out and Run-in are mandatory.**  
   They are part of the operational path model, not optional visualization decorations.

   **Run out 与 Run in 是强制约束。**  
   它们是作业路径模型的一部分，不只是可视化装饰。

4. **Turn geometry is solved between geometric points, not data points.**  
   The actual local solver starts at EOL geometric point and ends at SOL geometric point.

   **换线几何是在几何点之间求解，不是在数据点之间直接求解。**

5. **Current side connection model is same-side only.**  
   The planner currently relies on:
   - `AB -> AB`
   - `CD -> CD`

   **当前连接模型仍然是同侧连接。**  
   当前规划器主要依赖：
   - `AB -> AB`
   - `CD -> CD`

6. **Global same-direction blocks are realized by interleaving**, not by forcing one spatial block to be traversed as a single contiguous subsequence.

   **全局“块内同向”是通过交错访问实现的**，而不是强制一个空间块在访问序列中连续跑完。

---

## When Modifying Code / 修改代码时的注意事项

### If working on local geometry solvers / 如果修改局部几何求解器

Be careful to preserve:

请务必保持：

- heading continuity
- tangent consistency
- correct use of `L_out` / `L_in`
- scenario boundary logic at $W = 2R$

即：

- 航向连续
- 切线一致性
- `L_out` / `L_in` 使用正确
- $W = 2R$ 边界逻辑正确

Do not casually change the geometric interpretation of EOL/SOL points.

不要随意改变 EOL / SOL 几何点的定义。

---

### If working on `planner.py` / 如果修改 `planner.py`

Remember that the current planner is based on:

请牢记当前规划器的核心是：

- spatial partitioning
- refined pairable blocks
- same-direction inside refined blocks
- interleaving between block pairs
- SA + DP at the global level

Do not revert the code to a “block internal TSP” interpretation unless explicitly intended.

除非明确需要，否则不要退回到“块内 TSP”式建模。

---

### If working on visualization / 如果修改可视化

Current visualization may contain:

当前可视化可能包含：

- survey lines
- turn path
- run-out overlays
- run-in overlays
- start marker
- bar chart of turn distance

When adjusting plotting order, pay attention to `zorder`.

修改绘图层级时，要特别注意 `zorder`。

---

## Extending the Project / 扩展项目

Possible future extensions:

未来可扩展方向包括：

- non-vertical lines
- obstacle-aware planning
- additional Dubins path families
- more exact global optimization
- formal test suite
- better separation of pure turn geometry vs run-out/run-in in visualization

即：

- 非竖直测线
- 障碍物约束
- 更多 Dubins 路径类型
- 更精确的全局优化
- 正式测试模块
- 在可视化中更彻底地区分纯转弯段与 Run-out / Run-in

---

## Notes for Maintainers / 维护者说明

- Keep comments bilingual where practical.
  尽量保留中英文双语注释。

- Prefer preserving current naming conventions:
  - `AB`, `CD`
  - `EOL`, `SOL`
  - `Run out`, `Run in`
  - `UP`, `DOWN`

  尽量保持当前命名约定：
  - `AB`, `CD`
  - `EOL`, `SOL`
  - `Run out`, `Run in`
  - `UP`, `DOWN`

- If repository structure changes again, update this file accordingly.
  若仓库结构后续再次变化，请同步更新本文件。

- If README and implementation disagree, prefer the **actual code behavior** and update documentation.
  如果 README 与实现不一致，应以**当前代码实际行为**为准，并及时更新文档。

---
```
