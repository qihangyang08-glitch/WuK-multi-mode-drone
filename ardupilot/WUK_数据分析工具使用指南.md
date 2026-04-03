# WuK 数据分析工具快速使用指南

**更新**: 2025-12-12  
**版本**: v2.0 - 统一JSONL格式数据流

---

## 📦 工具概览

WuK项目包含两个数据分析工具：

1. **wuk_analyzer.cpp** (C++)：解析SITL JSONL日志 → CSV
2. **wuk_plotter.py** (Python)：CSV数据 → 4张分析图表

**数据流**:
```
SITL仿真 → wuk_gb.jsonl → wuk_analyzer → wuk_analysis.csv → wuk_plotter → wuk_plot.png
```

---

## 🔧 环境准备

### 1. 编译C++分析器
```bash
cd ~/ardupilot/tools
g++ -std=c++11 -O2 -o wuk_analyzer wuk_analyzer.cpp

# 验证
./wuk_analyzer
# 应输出: 用法: ./wuk_analyzer <input.jsonl> <output.csv>
```

### 2. 安装Python依赖
```bash
pip3 install pandas numpy matplotlib

# 验证
python3 -c "import pandas, numpy, matplotlib; print('✓ 依赖已安装')"
```

---

## 🚀 基本使用

### 完整分析流程（3步）

```bash
# 步骤1: 运行SITL仿真
cd ~/ardupilot
./test_wuk_interactive.sh
# ... 进行飞行测试，完成后退出SITL
# 输出日志: wuk_gb.jsonl

# 步骤2: 解析日志为CSV
cd tools
./wuk_analyzer ../wuk_gb.jsonl wuk_analysis.csv

# 步骤3: 生成图表
python3 wuk_plotter.py wuk_analysis.csv wuk_plot.png
```

**输出文件**:
- `wuk_analysis.csv`: 22列数据，包含电机倾角、推力、姿态、高度
- `wuk_plot.png`: 4张分析图表（200 DPI高清）

---

## 📊 输出图表说明

### 图1: 电机推力方向相对机体Roll轴的倾角
- **内容**: 4个电机推力在机体坐标系中的倾角变化
- **理想表现**: 0° → 45° (MORPH模式) 或 90° (GROUND模式)
- **参考线**: 45°（紫色）、90°（棕色）
- **诊断**: 如倾角不变化，说明伺服控制失效

### 图2: 电机推力方向和机体姿态相对世界坐标系Roll轴
- **内容**: 电机倾角（世界坐标系） + 机体Roll角
- **理想表现**: 电机倾角（世界）保持稳定，机体Roll角在±5°内
- **诊断**: 如机体Roll角突变，说明推力补偿不足

### 图3: 电机推力、总推力和垂直分力变化
- **内容**: 
  - 4个电机推力（彩色）
  - 总推力（黑色实线）
  - 垂直分力（绿色虚线）
- **理想表现**: 
  - 总推力随倾角增大而增大（补偿水平分量损失）
  - 垂直分力保持稳定（≈ 机体重力）
- **诊断**: 
  - 垂直分力下降 → 推力补偿不足 → 高度下降
  - 总推力异常波动 → PID控制不稳定

### 图4: 机体高度变化
- **内容**: 高度-时间曲线，带5m参考线
- **理想表现**: 变构过程高度保持稳定（±0.5m内）
- **诊断**: 
  - 快速下降（橙色虚线标记）→ 推力补偿失效
  - 缓慢爬升/下降 → 积分调参需优化

### 事件标注
- **红色虚线**: 变构开始时刻（events.morph_start）
- **橙色虚线**: 机体快速下降时刻（下降速率 > 1.5 m/s）

---

## 🔍 数据字段说明

### CSV输出列（22列）

| 列名 | 说明 | 单位 |
|------|------|------|
| time_s | 仿真时间 | 秒 |
| fc_angle | 飞控设定的变构角度 | 度 |
| morph_active | 是否处于MORPH模式 | 0/1 |
| roll_deg, pitch_deg, yaw_deg | 机体姿态（世界坐标系） | 度 |
| altitude_m | 机体高度 | 米 |
| m1_tilt_body ~ m4_tilt_body | 电机倾角（机体坐标系） | 度 |
| m1_tilt_world ~ m4_tilt_world | 电机倾角（世界坐标系） | 度 |
| m1_thrust ~ m4_thrust | 各电机推力 | 牛顿 |
| total_thrust | 总推力 | 牛顿 |
| vertical_thrust | 垂直分力 | 牛顿 |
| morph_start | 变构开始标记 | 0/1 |
| altitude_drop | 快速下降标记 | 0/1 |

**关键计算**:
- `motor_tilt_body[i]` = atan2(|motor_vx[i]|, |motor_vz[i]|) × 180/π
- `motor_tilt_world[i]` = motor_tilt_body[i] + roll_deg
- `vertical_thrust` = Σ(-motor_vz[i]) （SITL中-z为向上）

---

## 🛠️ 高级用法

### 仅分析特定时间段
```bash
# 手动编辑CSV，删除不需要的行
head -1 wuk_analysis.csv > filtered.csv
awk '$1 >= 10 && $1 <= 30' wuk_analysis.csv >> filtered.csv
python3 wuk_plotter.py filtered.csv filtered_plot.png
```

### 导出高分辨率图表
```bash
# 修改wuk_plotter.py第219行：
# plt.savefig(output_file, dpi=200, ...)  改为 dpi=300
python3 wuk_plotter.py wuk_analysis.csv wuk_plot_hd.png
```

### 批量处理多次测试
```bash
for log in wuk_test*.jsonl; do
    base=$(basename "$log" .jsonl)
    ./wuk_analyzer "$log" "${base}.csv"
    python3 wuk_plotter.py "${base}.csv" "${base}.png"
done
```

---

## ❓ 常见问题

### Q1: JSONL文件为空或缺失
**原因**: SITL未正确输出日志  
**解决**: 
1. 检查 `SIM_Frame.cpp` 中 `wuk_json` 文件是否打开成功
2. 确认日志路径：`/home/qwssx2/ardupilot/wuk_gb.jsonl`
3. 运行SITL时确保有变构动作（RC9=1900）

### Q2: CSV列数不匹配
**原因**: 使用了旧版wuk_parser.cpp  
**解决**: 删除旧版，重新编译wuk_analyzer.cpp

### Q3: 图表中事件标记缺失
**原因**: JSONL中未检测到变构事件  
**解决**: 
1. 检查SITL日志中 `"events":{"morph_start":true}`
2. 确认MORPH模式已激活（mode MORPH命令）

### Q4: 电机倾角全为0
**原因**: 推力向量计算错误或电机未倾转  
**解决**: 
1. 检查JSONL中 `per_motor.vx/vy/vz` 是否有非零值
2. 确认伺服控制生效（RC9通道）

### Q5: Python绘图窗口无法显示
**原因**: SSH环境无GUI支持  
**解决**: 
```bash
export MPLBACKEND=Agg
python3 wuk_plotter.py wuk_analysis.csv output.png
```

---

## 📚 相关文档

- **WUK_改动详情.md**: 完整代码改动说明
- **WUK_测试命令集.md**: SITL测试流程
- **WUK_项目介绍与进展.md**: 项目总览

---

**郑州大学 无人机项目组**  
最后更新: 2025-12-12
