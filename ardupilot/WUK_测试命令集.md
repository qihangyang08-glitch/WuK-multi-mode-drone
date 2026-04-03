# WuK变构四旋翼 - 测试命令集

**更新**: 2025-12-11 | **环境**: SITL仿真 + 真机测试

---

## 1️⃣ 环境准备

### Python依赖
```bash
pip3 install numpy matplotlib scipy
```

### 验证工具
```bash
cd ~/ardupilot/tools
ls -lh wuk_analyzer.cpp wuk_plotter.py
# 应看到两个文件存在

# 编译分析器
g++ -std=c++11 -O2 -o wuk_analyzer wuk_analyzer.cpp
```

---

## 2️⃣ SITL编译与启动

### 完整编译
```bash
cd ~/ardupilot/
./waf clean
./waf configure --board sitl
./waf copter -j4
```

### 启动SITL
```bash
cd ~/ardupilot

# 方法1: 使用测试脚本（推荐）
./test_wuk_interactive.sh

# 方法2: 手动启动
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w \
    --add-param-file=mav.parm
```

---

## 3️⃣ SITL交互测试

### 基本飞行
```bash
mode LOITER          # 切换到LOITER模式
arm throttle         # 解锁
rc 3 1600            # 起飞（油门上升）
rc 3 1500            # 悬停
mode LAND            # 降落
```

### MORPH模式测试（核心功能）
```bash
# === 阶段1: 起飞 ===
mode LOITER
arm throttle
rc 3 1600            # 起飞到2米

# === 阶段2: 触发变构 ===
rc 9 1900            # RC9>1800触发MORPH

# 观察:
# - 终端显示 "Mode MORPH"
# - 角度0°→45°（约3秒）
# - 高度保持稳定（±0.5m）
# - 电机PWM逐渐增大（补偿生效）

# === 阶段3: 驻留期 ===
# 停留2秒后自动进入GROUND模式
# 终端显示 "Mode GROUND"

# === 阶段4: GROUND模式差速测试 ===
rc 2 1600            # 俯仰杆→前进
rc 1 1600            # 横滚杆→右转
rc 2 1500            # 回中
rc 1 1500            # 回中

# === 阶段5: 返回LOITER ===
rc 9 1100            # RC9<1200回到LOITER

# === 阶段6: 降落 ===
mode LAND
```

### 串口通信测试
```bash
rc 7 1000            # 测试RC7透传（ACT1）
# 应显示: "WuK RC7: 1000 -> ACT1"

rc 8 1500            # 测试RC8透传（ACT2）
# 应显示: "WuK RC8: 1500 -> ACT2"
```

### 推力补偿验证
```bash
# 1. 记录LOITER基线
mode LOITER
arm throttle
rc 3 1550            # 悬停1米，观察PWM（约1400）

# 2. 触发变构
rc 9 1900
# 观察: PWM应增大到约1980
#      高度保持稳定

# 3. 日志会记录所有推力数据
```

---

## 4️⃣ 日志分析流程

### 收集日志
```bash
cd ~/ardupilot

# 检查SITL日志
ls -lh wuk_gb.log wuk_gb.jsonl

# 检查.bin日志
ls -lht logs/*.bin | head -1
```

### 解析日志并生成图表
```bash
cd ~/ardupilot/tools

# 步骤1: 解析SITL JSONL日志
./wuk_analyzer ../wuk_gb.jsonl wuk_analysis.csv

# 检查CSV输出
head -5 wuk_analysis.csv
wc -l wuk_analysis.csv

# 步骤2: 生成4张分析图表
python3 wuk_plotter.py wuk_analysis.csv wuk_plot.png

# 检查输出
ls -lh wuk_plot.png
```

**输出图表说明**:
1. **图1 - 电机倾角（机体坐标系）**: 验证电机推力方向是否正确变化（0°→45°或90°）
2. **图2 - 电机倾角&姿态（世界坐标系）**: 验证推力补偿是否抵消姿态变化
3. **图3 - 推力变化**: 总推力、垂直分力、各电机推力，验证推力补偿效果
4. **图4 - 高度变化**: 验证变构过程高度是否稳定

**关键指标**:
- 红色虚线：变构开始时刻
- 橙色虚线：机体快速下降时刻（如有，表示控制失稳）
- 图1中电机倾角应平滑变化至目标角度（45°或90°）
- 图3中垂直推力应保持稳定（约等于机体重力）
- 图4中高度应无明显下降

---

## 5️⃣ 真机测试准备

### 固件编译
```bash
cd ~/ardupilot/ArduCopter

# 查看支持的飞控板
./waf list_boards | grep -i pixhawk

# 配置目标板（示例：Pixhawk4）
./waf configure --board Pixhawk4

# 编译
./waf copter

# 固件位置
ls -lh ../build/Pixhawk4/bin/arducopter.apj
```

### 参数配置
创建真机参数文件 `wuk_realflight.parm`:
```
FRAME_CLASS,1              # Quad
FRAME_TYPE,1               # X型
WUK_MORPH_RATE,10.0        # 变构速率（真机更慢）
WUK_MORPH_DWELL,3000       # 驻留3秒
WUK_COMP_EN,1              # 启用补偿
SERIAL4_PROTOCOL,28        # Arduino通信
SERIAL4_BAUD,57600
FS_BATT_ENABLE,1           # 电池failsafe
FS_GCS_ENABLE,1            # GCS failsafe
```

### 首飞检查
```bash
# === 硬件检查 ===
□ 电机转向正确（M1/M4顺时针，M2/M3逆时针）
□ 螺旋桨安装正确
□ 伺服运动正常（RC9: 1100→0°, 1900→45°）
□ 电池充满（>11.5V）
□ GPS定位（卫星数>10）

# === 软件检查 ===
□ 参数加载成功
□ 遥控器校准
□ 姿态水平（<5°）
□ Failsafe测试

# === 测试顺序 ===
1. 地面解锁测试
2. 悬停测试（1分钟）
3. 手动控制测试
4. 变构测试（5米以上高度）
5. 紧急降落准备
```

---

## 6️⃣ 故障排查

### SITL无法启动
```bash
# 检查依赖
pip3 install --upgrade pymavlink MAVProxy

# 重新配置
cd ~/ardupilot/ArduCopter
./waf distclean
./waf configure --board sitl
./waf copter
```

### MORPH模式高度下降
```bash
# 检查补偿是否启用
param show WUK_COMP_EN
# 应为1

# 检查日志推力数据
# 查看图表subplot 1
# 垂直推力应保持稳定
```

### GROUND模式转向反向
```bash
# 检查电机布局
grep -A 10 "set_ground_thrust" libraries/AP_Motors/AP_MotorsMatrix.cpp

# 应看到:
# _thrust_rpyt_out[0] = right_thrust;  // M1
# _thrust_rpyt_out[1] = left_thrust;   // M2
# _thrust_rpyt_out[2] = left_thrust;   // M3
# _thrust_rpyt_out[3] = right_thrust;  // M4
```

### 日志解析失败
```bash
# 检查JSONL日志格式
head -5 wuk_gb.jsonl
# 应看到JSON格式数据，包含 "per_motor", "attitude_deg", "altitude_m" 等字段

# 重新编译分析器
cd tools
g++ -std=c++11 -O2 -o wuk_analyzer wuk_analyzer.cpp

# 调试运行
./wuk_analyzer ../wuk_gb.jsonl test_output.csv
```

### 绘图失败
```bash
# 检查Python依赖
python3 -c "import pandas, numpy, matplotlib; print('OK')"

# 安装缺失依赖
pip3 install pandas numpy matplotlib

# 使用非GUI后端
MPLBACKEND=Agg python3 wuk_plotter.py wuk_analysis.csv output.png
```

### CSV列不匹配
```bash
# 检查CSV头部
head -2 wuk_analysis.csv

# 应包含这些列：
# time_s,fc_angle,morph_active,roll_deg,pitch_deg,yaw_deg,altitude_m,
# m1_tilt_body,m2_tilt_body,m3_tilt_body,m4_tilt_body,
# m1_tilt_world,m2_tilt_world,m3_tilt_world,m4_tilt_world,
# m1_thrust,m2_thrust,m3_thrust,m4_thrust,
# total_thrust,vertical_thrust,morph_start,altitude_drop

# 如果列名不对，重新运行分析器
./tools/wuk_analyzer wuk_gb.jsonl wuk_analysis.csv
```

---

## 7️⃣ 常用命令速查

```bash
# === 编译 ===
cd ~/ardupilot/ArduCopter && ./waf configure --board sitl && ./waf copter

# === 启动SITL ===
cd ~/ardupilot && ./test_wuk_interactive.sh

# === MAVProxy ===
mode LOITER         # 切换模式
arm throttle        # 解锁
rc 9 1900           # 触发变构
param show WUK_*    # 查看参数

# === 分析 ===
cd ~/ardupilot/tools
./wuk_analyzer ../wuk_gb.jsonl wuk_analysis.csv
python3 wuk_plotter.py wuk_analysis.csv wuk_plot.png
```

---

## 🔗 相关文档

- **WUK_项目介绍与进展.md** - 项目总览
- **WUK_改动详情.md** - 详细代码说明
- **WUK改动位置索引.txt** - 代码快速索引
