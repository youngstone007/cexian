下面给你一版适合当前项目的 `AGENTS.md`。  
它比 `CLAUDE.md` 更偏向**给代码代理/自动化助手的执行约束**，强调：

- 项目事实
- 不要误改的关键约定
- 修改代码时的优先级
- 哪些描述已经过时不能再沿用

你可以直接新建文件 `AGENTS.md` 并填入以下内容。

```markdown

```

# AGENTS.md

Guidance for coding agents working in this repository.

本文件用于指导各类代码代理在本仓库中进行分析、修改、调试与文档更新。

---

## 1. Repository Reality Check / 仓库现实情况

This is a **multi-file Python project**, not a single-file demo.

这是一个**多文件 Python 项目**，不是单文件演示脚本。

Current main modules:

当前主要模块包括：

- `common.py`
- `dubins_csc.py`
- `semicircle.py`
- `three_arc.py`
- `planner.py`
- `main.py`
- `README.md` (if present)
- `CLAUDE.md`
- `AGENTS.md`

Do not describe the repository as “a single-file implementation in `main.py`”.

不要再把该仓库描述为“`main.py` 单文件实现”。

---

## 2. Project Purpose / 项目目标

This repository solves **ship lane switching between parallel survey lines**.

本仓库解决的是**船舶在平行测线之间的换线问题**。

There are two levels of logic:

本项目包含两个层面的逻辑：

1. **Local turn geometry**
   - Compute feasible turn path between two lines.
   - Respect fixed turning radius, `Run out`, and `Run in`.

   **局部换线几何**
   - 计算两条测线之间的可行换线路径；
   - 满足固定转弯半径、`Run out`、`Run in` 约束。

2. **Global route planning**
   - Optimize the visit order of many lines.
   - Use spatial partitioning, same-direction blocks, interleaving, SA, and DP.

   **全局路线规划**
   - 优化多条测线的访问顺序；
   - 使用空间分块、块内同向、交错访问、SA 和 DP。

Do not reduce the project description to “just Dubins path between two lines”.

不要把项目简化描述成“仅仅是两条测线之间的 Dubins 路径”。

---

## 3. Non-Negotiable Geometry Conventions / 不可破坏的几何约定

### 3.1 Survey lines are vertical / 测线为竖直线

Current implementation assumes all survey lines are vertical.

当前实现假设所有测线均为竖直线。

Each line is represented by:

每条测线由以下信息表示：

- `x`
- `y_start`
- `y_end`

---

### 3.2 Side convention / 侧别约定

The repository uses side-based line entry and exit semantics:

仓库采用基于侧别的进出线语义：

- `AB` side = lower endpoint = `y_start`
- `CD` side = upper endpoint = `y_end`

And:

并且：

- Enter from `AB` $\rightarrow$ move upward along the line
- Enter from `CD` $\rightarrow$ move downward along the line

即：

- 从 `AB` 进入 $\rightarrow$ 沿测线上行
- 从 `CD` 进入 $\rightarrow$ 沿测线下行

Do not silently invert this convention.

不要在未明确说明的情况下颠倒这一约定。

---

### 3.3 EOL / SOL geometric points / EOL / SOL 几何点

Turn geometry is planned between **geometric points**, not directly between data points.

换线几何是在**几何点之间**求解的，而不是直接在数据点之间求解。

- `EOL geometric point`:
  obtained by moving from the current line’s acquisition end point along the line heading by `L_out`.

- `SOL geometric point`:
  obtained by moving backward from the target line’s acquisition start point against the target heading by `L_in`.

即：

- `EOL 几何点`：
  从当前测线采集结束点沿当前航向前进 `L_out` 得到；

- `SOL 几何点`：
  从目标测线采集开始点沿目标航向反方向回退 `L_in` 得到。

Do not redefine `Run out` / `Run in` as pure visualization concepts.  
They are part of the actual path model.

不要把 `Run out` / `Run in` 重新定义成纯可视化概念。  
它们是实际路径模型的一部分。

---

## 4. Three Local Turn Scenarios / 三类局部换线场景

The local solver must preserve the following scenario split:

局部换线求解必须保留以下三类场景划分：

### 4.1 `$W > 2R$`
Use Dubins CSC geometry.

使用 Dubins CSC 几何。

Current implementation mainly compares:

当前实现主要比较：

- `LSL`
- `RSR`

---

### 4.2 `$W = 2R$`
Use the semicircle turn model.

使用半圆换线模型。

---

### 4.3 `$W < 2R$`
Use the fixed symmetric three-arc model.

使用固定对称三圆弧模型。

Typical types:

典型类型：

- `LRL`
- `RLR`

Do not collapse these three cases into one generic solver unless the replacement fully preserves current behavior.

不要在未完整保留现有行为的前提下，把这三类情况粗暴合并成一个通用求解器。

---

## 5. Global Planner Truth / 全局规划器的真实结构

### 5.1 The planner is not block-internal TSP / 规划器不是“块内 TSP”

Do not describe or rewrite the planner as:

不要把当前规划器描述或改写成：

- “partition into blocks, then solve a small TSP inside each block”

即不要写成：

- “先分块，再在每个块里做一个小 TSP”

That is **not** the current intended model.

这**不是**当前项目想要的模型。

---

### 5.2 What the planner actually does / 当前规划器实际做什么

The planner currently works in this spirit:

当前规划器的核心思路是：

1. Partition lines into spatial blocks by expert rule.
   先按专家经验法进行空间分块。

2. Refine blocks if needed so they can be paired feasibly.
   如有必要，对空间块进一步细分，使其可配对。

3. Enforce same direction inside each refined block.
   每个细分块内部方向统一。

4. Build work units from neighboring paired blocks.
   由相邻配对细分块构造工作单元。

5. Interleave access between the paired blocks.
   在成对块之间进行交错访问。

6. Optimize work-unit order globally using SA.
   使用 SA 优化工作单元顺序。

7. For a fixed unit order, use DP to choose the best candidate chain.
   对固定单元顺序，用 DP 选择最优候选链。

This architecture should be preserved unless an explicit redesign is requested.

除非明确要求重构，否则应保持这一架构思想。

---

## 6. File-Level Responsibilities / 文件级职责

### `common.py`
Base geometry types, tolerances, and sampling helpers.

基础几何类型、公差常量、路径采样辅助函数。

Typical examples:

典型示例：

- `VerticalLine`
- `EPS`
- `sample_arc`
- `sample_line`

---

### `dubins_csc.py`
Solver for the `$W > 2R$` case.

用于 `$W > 2R$` 场景的求解器。

---

### `semicircle.py`
Solver for the `$W = 2R$` case.

用于 `$W = 2R$` 场景的求解器。

---

### `three_arc.py`
Solver for the `$W < 2R$` case.

用于 `$W < 2R$` 场景的求解器。

---

### `planner.py`
Main orchestration layer for global planning.

全局规划主调度模块。

Likely contains logic for:

通常包含以下逻辑：

- distance caching
- spatial partitioning
- refined block generation
- work-unit candidate generation
- block interleaving
- unit connection
- SA + DP optimization
- final route assembly

---

### `main.py`
Demo entry, printing, and visualization.

示例入口、打印输出、可视化绘图。

If plots change, verify legends, colors, and `zorder`.

若修改绘图，请检查图例、颜色和 `zorder`。

---

## 7. Safe Modification Rules / 安全修改规则

### 7.1 When modifying local geometry / 修改局部几何时

Preserve:

请保持：

- heading continuity
- tangent consistency
- scenario split correctness
- use of `L_out` and `L_in`
- path length consistency

即：

- 航向连续
- 切线一致
- 场景划分正确
- `L_out` / `L_in` 正确参与路径
- 路径长度计算一致

Do not “simplify” away engineering constraints.

不要为了代码看起来简洁而删掉工程约束。

---

### 7.2 When modifying planner logic / 修改规划器时

Preserve these ideas unless explicitly instructed otherwise:

除非明确要求，否则请保留以下思想：

- same-direction inside refined blocks
- interleaving between paired neighboring blocks
- same-side connection assumption
- SA + DP global optimization structure

即：

- 细分块内同向
- 相邻配对块间交错访问
- 同侧连接假设
- SA + DP 全局结构

---

### 7.3 When modifying output or docs / 修改输出或文档时

Always align docs with **actual code behavior**.

文档必须始终与**当前代码真实行为**一致。

If code and docs disagree:

如果代码与文档冲突：

- trust the code first
- then update docs

优先以代码为准，再更新文档。

---

## 8. Visualization Notes / 可视化说明

Current plots may include:

当前图中可能包含：

- survey lines
- turn paths
- `Run out`
- `Run in`
- start marker
- bar chart of turn distances

When editing plotting code:

修改绘图代码时请注意：

1. `Run out` and `Run in` are semantic overlays, not random decorations.
   `Run out` 和 `Run in` 是有明确含义的覆盖层，不是随意装饰。

2. Pay attention to `zorder`.
   注意 `zorder`。

3. Keep direction colors stable unless explicitly redesigning the style.
   除非明确重新设计风格，否则不要随意改动方向颜色约定。

Typical convention used recently:

近期常用约定通常是：

- blue = line upward
- red = line downward
- green = turn path
- gray / dark gray = `Run out`
- orange = `Run in`

---

## 9. Running and Verification / 运行与验证

Typical run command:

典型运行命令：

```bash
python main.py
```



After changes, verify:

修改后建议检查：

1. The code runs without crash.
   代码能正常运行。

2. The chosen route is feasible.
   路线可行。

3. `Run out` / `Run in` are still geometrically meaningful.
   `Run out` / `Run in` 仍然具有正确几何意义。

4. The planner still respects same-direction block behavior.
   规划器仍满足块内同向行为。

5. Visualization still matches route semantics.
   可视化仍与路线语义一致。

---

## 10. Dependencies / 依赖

Primary external dependency:

当前主要外部依赖：

- `matplotlib`

Install with:

安装方式：

```bash
pip install matplotlib
```

The rest is expected to rely mostly on the Python standard library.

其余部分通常主要依赖 Python 标准库。

---

## 11. Common Mistakes to Avoid / 常见误区

Do **not** make the following incorrect assumptions:

不要做出以下错误假设：

- “This repo is only about two-line path solving.”
- “`main.py` contains everything.”
- “Blocks are solved as internal TSPs.”
- “`Run out` and `Run in` are just for plotting.”
- “Direction is stored only as arrow color.”
- “The planner is Q-learning-based.”

即不要误以为：

- “本仓库只做两条线之间的换线求解”
- “所有代码都在 `main.py`”
- “分块后每块内部做 TSP”
- “`Run out` 和 `Run in` 只是画图用的”
- “方向只是图上的箭头颜色”
- “当前规划器是基于 Q-learning 的”

The current planner is **SA + DP + block interleaving**, not Q-learning.

当前规划器是**SA + DP + 区块交错访问**，不是 Q-learning。

---

## 12. Preferred Agent Behavior / 建议代理行为

When asked to modify code:

当收到代码修改请求时：

1. First identify whether the request affects:
   - local geometry
   - global planning
   - visualization
   - docs

   先判断请求影响的是：
   - 局部几何
   - 全局规划
   - 可视化
   - 文档

2. Avoid broad rewrites if a local patch is enough.
   如果局部修补即可解决问题，不要大范围重写。

3. Preserve public function names when possible.
   尽量保持公开函数名不变。

4. If changing semantics, explicitly state the semantic change.
   如果改变了语义，必须明确说明。

5. If generating docs, keep them consistent with current repository structure.
   若生成文档，必须与当前仓库结构一致。

---

## 13. If You Need to Add New Features / 若需扩展新功能

Prefer incremental extension over destructive rewrite.

优先增量式扩展，不要破坏式重写。

Examples of acceptable extension directions:

可接受的扩展方向包括：

- support for more turn templates
- support for non-vertical lines
- obstacle-aware planning
- better tests
- cleaner plotting layers
- richer route statistics

If adding a feature that changes assumptions, update:

若新增功能改变了现有假设，请同步更新：

- `README.md`
- `CLAUDE.md`
- `AGENTS.md`

---

## 14. Documentation Priority / 文档优先级

When multiple docs exist, interpret them in this order:

当多个文档并存时，建议按以下优先级理解：

1. actual source code
2. `AGENTS.md`
3. `CLAUDE.md`
4. `README.md`

If inconsistency exists, prefer current code behavior and update the docs.

若存在不一致，以当前代码行为为准，并更新文档。