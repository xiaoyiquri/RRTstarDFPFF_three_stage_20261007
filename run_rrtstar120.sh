#!/usr/bin/env bash
# ==============================================
# 合并脚本：先跑 RRT* 实验，再自动画图
# ==============================================

set -o pipefail

# ==============================================
# 1. 路径设置
# ==============================================

project_dir="/home/cxt/MYFILE/newfile/RRTstar/RRTstarDFPFF_paper_r2_float32_strict_20261007"

dynobench_dir="/home/cxt/MYFILE/newfile/dynobench-main"

# 兼容嵌套的 DynoBench 目录
if [ ! -f "$dynobench_dir/dynobench/utils/viewer/robot_viewer.py" ]; then
    if [ -f "$dynobench_dir/dynobench-main/dynobench/utils/viewer/robot_viewer.py" ]; then
        dynobench_dir="$dynobench_dir/dynobench-main"
    fi
fi

# ==============================================
# 2. 运行实验（生成新的 experiment_dir）
# ==============================================

echo "===== 开始运行实验脚本 ====="
bash "$HOME/run_rrtstar120.sh" 2>&1 | tee "$HOME/rrtstar120_debug.log"
echo "实验脚本退出码：${PIPESTATUS[0]}"

# ==============================================
# 3. 自动探测最新的 experiment_dir
#    形如：results/rrtstar120_rewire_shortcut_YYYYMMDD_HHMMSS_xxxx
# ==============================================

results_root="$project_dir/results"
experiment_dir="$(ls -1dt "$results_root"/rrtstar120_rewire_shortcut_* 2>/dev/null | head -n 1)"

if [ -z "$experiment_dir" ] || [ ! -d "$experiment_dir" ]; then
    echo "错误：未能在 $results_root 下找到 rrtstar120_rewire_shortcut_* 实验目录"
    exit 1
fi

echo "实验目录：$experiment_dir"
echo "DynoBench：$dynobench_dir"

# ==============================================
# 4. 检查文件是否存在
# ==============================================

ls -l "$experiment_dir/manifest.json"
ls -l "$experiment_dir/uni_bugtrap/trajectory_first_0.yaml"
ls -l "$dynobench_dir/dynobench/utils/viewer/robot_viewer.py"

# ==============================================
# 5. 设置 Python 环境
# ==============================================

export PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONHOME

cd "$project_dir"

# ==============================================
# 6. 画首解轨迹
# ==============================================

/usr/bin/python3 -s plot_successful_results_paper.py \
    --experiment "$experiment_dir" \
    --dynobench-source "$dynobench_dir" \
    --trajectory first \
    --run 0 \
    --mode dynobench \
    --skip-failed \
    --output "$experiment_dir/figures_first"

# ==============================================
# 7. 画120秒 RRT* + Rewire 后的轨迹
# ==============================================

/usr/bin/python3 -s plot_successful_results_paper.py \
    --experiment "$experiment_dir" \
    --dynobench-source "$dynobench_dir" \
    --trajectory raw \
    --run 0 \
    --mode dynobench \
    --skip-failed \
    --output "$experiment_dir/figures_rewire"

# ==============================================
# 8. 画 Algorithm 4 Shortcut 后的轨迹
# ==============================================

/usr/bin/python3 -s plot_successful_results_paper.py \
    --experiment "$experiment_dir" \
    --dynobench-source "$dynobench_dir" \
    --trajectory shortcut \
    --run 0 \
    --mode dynobench \
    --skip-failed \
    --output "$experiment_dir/figures_shortcut"

# ==============================================
# 9. 打开可视化结果
# ==============================================

xdg-open "$experiment_dir/figures_first/index.html"
xdg-open "$experiment_dir/figures_rewire/index.html"
xdg-open "$experiment_dir/figures_shortcut/index.html"

# ==============================================
# 10. 显示整体退出码
# ==============================================

echo "全部完成。实验目录：$experiment_dir"
