# WuK SITL 测试完整流程

## 一、编译

### 1.1 配置编译环境
```bash
cd ~/ardupilot
./waf configure --board sitl
```

**期望输出**:
```
Setting board to                         : sitl
Using toolchain                          : native
Checking for 'g++' (C++ compiler)        : /usr/lib/ccache/g++
Checking for 'gcc' (C compiler)          : /usr/lib/ccache/gcc
...
'configure' finished successfully (3.7s)
```

### 1.2 编译ArduCopter
```bash
./waf copter
```

**期望输出**:
```
Waf: Entering directory `/home/qwssx2/ardupilot/build/sitl'
[1338/1398] Compiling ArduCopter/mode_morph.cpp
[1392/1398] Compiling ArduCopter/Copter.cpp
[ 922/1398] Compiling libraries/SITL/SIM_Frame.cpp
[1398/1398] Linking build/sitl/bin/arducopter
Waf: Leaving directory `/home/qwssx2/ardupilot/build/sitl'
'build' finished successfully (XX.XXXs)
```

### 1.3 验证编译产物
```bash
ls -lh build/sitl/bin/arducopter
```

**期望输出**:
```
-rwxr-xr-x 1 qwssx2 qwssx2 4.3M Dec 10 22:53 build/sitl/bin/arducopter
```

---

## 二、启动SITL

```bash
./test_wuk_interactive.sh
```
*或手动:*
```bash
sim_vehicle.py -v ArduCopter --console --map -w \
    --add-param-file=Tools/autotest/default_params/wuk.parm
```
### 步骤1: 测试
**在MAVProxy中执行**:
```
mode LOITER
arm throttle
rc 3 1800      # 起飞(1600可能飞不动)
# 等待升到5米
rc 3 1500      # 悬停
# 等待稳定
rc 7 1900      # 测试RC7,应看到: WuK RC7: 1900 -> ACT1
rc 8 1100      # 测试RC8,应看到: WuK RC8: 1100 -> ACT2
rc 7 1500      # 归中RC7
rc 8 1500      # 归中RC8
rc 9 1900      # 触发变形,进入MORPH模式
# 观察终端输出: SITL WuK: FC angle=X.X° -> PWM=XXXX
# 等待5-6秒完成变形
```

**观察点**:
1. ✅ RC7/8改变时MAVProxy显示UART消息
2. ✅ RC9>1800触发MORPH模式切换
3. ✅ 终端显示FC angle从0°增加到90°
4. ✅ 角度变化: 0-45°(慢) → 停留2秒 → 45-90°(慢)
5. ✅ 完成后自动切换到GROUND模式

### 步骤2: 退出SITL
在MAVProxy中输入:
```
exit
```

或按 Ctrl+C


# 手动分析
./tools/wuk_log_parser_v3 wuk_gb.log wuk_parsed_v3.csv
python3 tools/wuk_plot_v3.py wuk_parsed_v3.csv wuk_gb.jsonl

# 查看结果
ls -lh wuk_analysis_v3.png  # 201KB图表

### 步骤3: 自动后处理
脚本会自动:
1. 检查日志文件存在性
2. 运行 `./tools/wuk_log_parser wuk_gb.log`
3. 生成 `wuk_parsed.csv`
4. 运行 `python3 tools/wuk_plot.py` (如果存在)
5. 生成分析图表

---

## 验证检查清单

### 编译验证
- [ ] `./waf copter` 编译成功
- [ ] `build/sitl/bin/arducopter` 大小约4.3MB

### 启动验证
- [ ] SITL启动无Connection refused错误
- [ ] MAVProxy控制台打开
- [ ] 地图窗口显示

### 功能验证
- [ ] `mode LOITER` 切换成功
- [ ] `arm throttle` 解锁成功
- [ ] `rc 3 1800` 能正常起飞
- [ ] `rc 7 1900` 显示UART消息
- [ ] `rc 9 1900` 触发MORPH模式
- [ ] 终端显示 `SITL WuK: FC angle=...`

### 日志验证
- [ ] `wuk_gb.log` 文件生成
- [ ] `wuk_gb.jsonl` 文件生成
- [ ] `head -1 wuk_gb.jsonl | python3 -m json.tool` 无错误
- [ ] `./tools/wuk_log_parser wuk_gb.log` 解析成功
- [ ] `wuk_parsed.csv` 文件生成

### 数据验证
- [ ] CSV包含angle列(0-90范围)
- [ ] target_pwm从1000变化到2000
- [ ] JSON格式正确可解析

---

**期望看到4个图像文件**:
- `wuk_attitude.png` - Roll/Pitch/Yaw vs 时间
- `wuk_thrust_delta.png` - 推力增量 vs 时间
- `wuk_tilt.png` - 倾转角度 vs 时间
- `wuk_total_vertical.png` - 垂直推力 vs 时间

---


## 三、问题排查

### 9.1 SITL启动失败
**症状1**: `Unknown frame type: wuk`  
**原因**: 不应该使用`-f wuk`参数  
**解决**: 使用默认frame启动(不加`-f`参数)

**症状2**: `[Errno 111] Connection refused sleeping`  
**原因**: 使用了`-f wuk`导致frame查找失败  
**解决**: 
```bash
# 错误: Tools/autotest/sim_vehicle.py -v ArduCopter -f wuk ...
# 正确: Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w ...
```

**验证机型配置**:
```bash
grep -n "wuk_motors" libraries/SITL/SIM_Frame.cpp | head -10
```

**期望输出**:
```
48:static Motor wuk_motors[] =
411:    Frame("wuk", 4, wuk_motors),
412:    Frame("+",         4, wuk_motors),
413:    Frame("quad",      4, wuk_motors),
414:    Frame("copter",    4, wuk_motors),
415:    Frame("x",         4, wuk_motors),
```

说明所有主要quad机型都已使用wuk_motors配置。

### 9.2 RC9不触发变形
**症状**: `rc 9 1900` 后没有模式变化

**检查**:
```bash
# 在MAVProxy中
param show RC9
```

**期望看到**: RC9相关参数

**调试**: 在Copter.cpp的process_wuk_switches()中添加打印:
```cpp
hal.console->printf("RC9=%d\n", pwm9);
```

重新编译并测试。

### 9.3 SITL无角度输出
**症状**: SITL终端没有 `SITL WuK: FC angle=...`

**检查1**: 是否在MORPH模式
```
status
```

**检查2**: 查看全局变量是否链接成功
```bash
nm build/sitl/bin/arducopter | grep g_wuk
```

**期望输出**:
```
00000000012345678 B g_wuk_morph_active
00000000012345679 B g_wuk_target_angle
00000000012345680 B g_wuk_last_update_ms
```

### 9.4 消息不显示
**症状**: RC7/8改变但没有STATUSTEXT消息

**检查**: MAVProxy的消息过滤
```
set moddebug 3
```

或查看日志:
```bash
ls -lh logs/
mavlogdump.py --types STATUSTEXT logs/00000001.BIN
```

---

## 十、测试检查清单

### 编译阶段
- [ ] `./waf configure --board sitl` 成功
- [ ] `./waf copter` 编译无错误
- [ ] `build/sitl/bin/arducopter` 存在且约4.3MB

### 启动阶段
- [ ] SITL启动成功,显示 `Using frame: wuk`
- [ ] MAVProxy控制台打开
- [ ] 地图窗口显示
- [ ] 当前模式为LOITER (或可手动切换)

### 基础功能
- [ ] `arm throttle` 解锁成功
- [ ] `rc 3 1600` 起飞成功
- [ ] 高度稳定在5米左右

### UART消息
- [ ] `rc 7 1900` 显示 `WuK RC7: 1900 -> ACT1`
- [ ] `rc 8 1500` 显示 `WuK RC8: 1500 -> ACT2`
- [ ] 不同值都能触发消息

### 变形功能
- [ ] `rc 9 1900` 触发模式切换到MORPH
- [ ] SITL终端显示 `SITL WuK: FC angle=...`
- [ ] 角度从0°逐渐增加到45° (约3秒)
- [ ] 在45°停留约2秒
- [ ] 角度从45°增加到90° (约3秒)
- [ ] 到达90°自动切换到GROUND模式
- [ ] PWM值从1000变化到2000

### 日志记录
- [ ] `wuk_gb.log` 生成且包含 `angle=` 字段
- [ ] `wuk_gb.jsonl` 生成且JSON格式正确
- [ ] 日志包含 `fc_angle` 和 `fc_morph_active` 字段
- [ ] 日志记录完整变形过程

### 数据分析
- [ ] 日志解析器编译成功
- [ ] 生成CSV数据文件
- [ ] 生成4张图表
- [ ] 图表显示合理的数据趋势

---

## 十一、完整测试脚本

### 11.1 一键测试脚本
保存为 `full_test.sh`:
```bash
#!/bin/bash
# WuK SITL完整测试脚本

set -e
cd ~/ardupilot

echo "========================================="
echo "WuK SITL 完整测试流程"
echo "========================================="
echo ""

# 1. 编译
echo "[1/5] 编译SITL..."
./waf configure --board sitl > /dev/null 2>&1
./waf copter

if [ ! -f "build/sitl/bin/arducopter" ]; then
    echo "❌ 编译失败"
    exit 1
fi
echo "✅ 编译成功: $(ls -lh build/sitl/bin/arducopter | awk '{print $5}')"
echo ""

# 2. 启动SITL
echo "[2/5] 启动SITL (后台运行)..."
Tools/autotest/sim_vehicle.py -v ArduCopter -f wuk --console \
    --add-param-file=Tools/autotest/default_params/copter-wuk.parm \
    > sitl_output.log 2>&1 &
SITL_PID=$!
echo "✅ SITL PID: $SITL_PID"
sleep 15
echo ""

# 3. 执行自动化测试
echo "[3/5] 运行自动化测试..."
python3 Tools/autotest/sitl_wuk_test.py
echo ""

# 4. 检查日志
echo "[4/5] 检查日志文件..."
if [ -f "wuk_gb.log" ] && [ -f "wuk_gb.jsonl" ]; then
    echo "✅ 日志文件生成成功"
    echo "  - wuk_gb.log: $(wc -l < wuk_gb.log) 行"
    echo "  - wuk_gb.jsonl: $(wc -l < wuk_gb.jsonl) 行"
else
    echo "❌ 日志文件未生成"
fi
echo ""

# 5. 停止SITL
echo "[5/5] 停止SITL..."
kill $SITL_PID 2>/dev/null || true
echo "✅ 测试完成"
echo ""

echo "========================================="
echo "查看日志: cat sitl_output.log"
echo "查看测试数据: cat wuk_gb.log"
echo "========================================="
```

### 11.2 运行
```bash
chmod +x full_test.sh
./full_test.sh
```

---

## 十二、关键指标

### 性能指标
- **编译时间**: ~3-5秒 (增量编译)
- **SITL启动时间**: ~10-15秒
- **变形总时长**: ~5-6秒
  - Phase 1 (0-45°): ~3秒 (15°/s)
  - Phase 2 (停留): 2秒
  - Phase 3 (45-90°): ~3秒 (15°/s)

### 数据输出
- **SITL打印频率**: 每100ms (10Hz)
- **日志记录频率**: 每200帧
- **JSON日志大小**: 约500KB/次完整测试

### 通信延迟
- **RC输入 → UART消息**: <10ms
- **FC角度更新 → SITL读取**: <1ms (全局变量)
- **PWM计算 → 电机响应**: <1帧 (~1ms)

---

## 附录A: 常用命令速查

```bash
# 编译
./waf configure --board sitl && ./waf copter

# 启动SITL (使用修正后的脚本)
./test_wuk_sitl_fixed.sh

# 或手动启动 (不使用-f wuk!)
sim_vehicle.py -v ArduCopter --console --map -w \
    --add-param-file=Tools/autotest/default_params/wuk.parm

# MAVProxy基本命令
mode LOITER          # 切换模式
arm throttle         # 解锁
rc 3 1600           # 油门起飞
rc 7 1900           # 测试RC7
rc 9 1900           # 触发变形
status              # 查看状态
disarm              # 上锁
exit                # 退出

# 日志查看
cat wuk_gb.log | less
python3 -m json.tool < wuk_gb.jsonl | less
grep "angle=45" wuk_gb.log

# 数据分析
./tools/wuk_log_parser wuk_gb.log
python3 tools/wuk_plot.py
```

---

## 附录B: 预期时间轴

```
T=0s     : rc 9 1900 触发
T=0.1s   : 模式切换到MORPH
T=0.2s   : FC angle=0°, SITL开始响应
T=0.3s   : FC angle=1.5°
T=1.0s   : FC angle=15°
T=2.0s   : FC angle=30°
T=3.0s   : FC angle=45° (到达)
T=3.0-5.0s : 停留在45° (2秒)
T=5.1s   : FC angle=46.5° (开始继续)
T=6.0s   : FC angle=60°
T=7.0s   : FC angle=75°
T=8.0s   : FC angle=90° (到达)
T=8.1s   : 模式切换到GROUND
```

---

**文档版本**: v1.0  
**最后更新**: 2025年12月10日  
**适用版本**: ArduPilot 4.x with WuK modifications
