# 本版检测流程与公式

1. 每条 PFF 边的所有系数保持原定义，J=∫(1+uᵀRu)dt、自由时间求解、完整终端状态 rewiring 不变。PFF 仍可使用 double；float8 加速对象是同一边的离散时间点。
2. 令 M=ceil(T/Δt)、N=M+1、B=ceil(N/8)。第 i 批的 lane j 对应 index=i+jB；index<N 的 lane 才有效，t=T·index/M。包含 t=0 和 t=T，任一有效 lane 失败立即拒绝边。
3. 为减少高阶多项式在 float32 中的消减，预处理把每个导数的幂基系数转成归一化时间 s=t/T 上的 Bernstein 系数。a_i=c_(i+d)(i+d)!/i!·T^i，b_l=Σ_(i≤l) a_i C(l,i)/C(degree,i)。八点 De Casteljau 求值，最高阶导数来自 FlatControl，未改 PFF 最优控制问题。
4. Uni：speed=hypot(dx,dy)，theta=atan2(dy,dx)，omega=(dx·ddy-dy·ddx)/speed²；Reverse 在 theta 上加 π 并令 v=-speed，branch 一条边不变。采样点 speed≤epsilon 拒绝。没有连续区间非零速度证明。
5. Quad2D：F=(ax,az+g)，theta=atan2(-ax,az+g)，T=m||F||。omega=(ax·jz-(az+g)·jx)/||F||²；alpha 是此式的时间导数，利用 snap；tau=I·alpha；u0=(T+tau/l)/(mg)，u1=(T-tau/l)/(mg)。控制布局和重力符号保持 DynoBench source 约定。
6. Quad3D：由 F=a+g e3 及 heading=(cosψ,sinψ,0) 构建 R，利用 jerk/snap 与 yaw derivatives 构建 Rdot/Rddot；omega=vee(RᵀRdot)，alpha=vee(RᵀRddot)，tau=J alpha+omega×Jomega。RdotᵀRdot 为对称矩阵，vee 为零，故与 scalar reference 的完整表达式等价。四元数输出 xyzw；physical body rates、world velocity；wrench 由 adapter 的逆分配矩阵映射到 actuator units。
7. 每个有效点都检查 DF 非奇异、速度/角速度/推力等 DF 约束、物理状态边界、实际 actuator bounds 和碰撞。严格要求所有有效 lane 通过。NaN/Inf、奇异推力、非法 branch、超过采样预算均拒绝，尾 lane 不影响结果。
8. 球体几何：机器人是配置指定单球；输入 AABB 障碍用小格外接球并集覆盖。中心 c、半径 r 的障碍，与机器人中心 p、半径 rR 的碰撞判据为 ||p-c||²≤(r+rR)²。接触属于碰撞。浮点计算及 1e-5 m 几何 margin 明确记录。
9. BVH 节点 AABB 包含其所有障碍球膨胀 robot_radius 后的范围；节点测试也同时处理 8 个点。叶球逐个与同一批时间点比较，已碰撞 lane 可提前退出。层级遍历和 mask 归约属于控制流，不是标量几何验证。
10. 论文 Algorithm 3 写碰撞在其他约束之前；此实现先计算可行性 mask，再为仍可能被接受的 lane 访问几何。逻辑是同一组采样点上 DF/状态/控制/碰撞谓词的合取；提早拒绝不改变接受结果。

## 边界

- 论文 VII-A 使用球体机器人/球体障碍，但公开正文没有给出全部半径与采样间隔。本版对应参数是明确适配值，不能称为与作者数据完全一致。
- 不保证采样点之间没有碰撞、奇异点或约束超限；不提供连续时间认证，不再计算 DynoBench step residual。
- float32 在约束边界处可能与 double 做出不同判断；测试按合理数值容差比较 DF，并在非边界样本比较有效性。
- 只含碰撞筛选而仍调用 FCL 的旧版本已替换。当前六系统 benchmark 没有链接 FCL/DynoBench 库，data adapter 也没有验证 API，因此无法偷偷调用这些路径。
- 初始/目标 lifting 的完整性和 FlatState、metadata、树连接检查保留；它们不是轨迹物理空间逐点的标量验证。

## 计数

`simd_validation_queries`：候选边/最终输出轨迹的 SIMD 验证次数；`simd_validation_batches`：实际访问的 8 lane 时间批次数；`simd_flat_samples`：有效采样 lane 数；`collision_checks`：叶球与时间批次的测试数。`simd_collision_samples` 记录进入整合验证的有效 lane 数，不表示每个点都访问了叶球（约束和 BVH 可提前排除）。

`dynobench_validations`/`dynobench_time` 为零。DF、几何与约束是融合 SIMD 内核，不再把其耗时拆成原先 `df_time` 和 FCL 时间；`simd_time` 包括该内核以及原 PFF/NN SIMD 计时范围，不应将各重叠时间相加。

源依据：<https://arxiv.org/html/2603.16059v4> V-D、Algorithm 3、VII-A；已有 `dynobench-main/src/unicycle1.cpp`、`planar_rotor.cpp`、`quadrotor.cpp` 的 state/control layout。
