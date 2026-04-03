# WuK变构四旋翼 - 改动详情

**更新**: 2025-12-12 | **标识**: 所有改动带[WuK]注释

---

## 📍 快速定位

```bash
# 搜索所有改动
grep -rn "\[WuK" ArduCopter/ libraries/ | grep -v "Binary"

# 按模块搜索
grep -rn "\[WuK-MORPH\]" ArduCopter/      # MORPH模式
grep -rn "\[WuK-GROUND\]" ArduCopter/     # GROUND模式
grep -rn "\[WuK-Comms\]" ArduCopter/      # 串口通信
grep -rn "\[WuK\]" libraries/AP_Motors/   # 电机混控
grep -rn "\[WuK-SITL\]" libraries/SITL/   # 仿真环境
```

---

## 🆕 新增文件

### 1. mode_morph.cpp/h
**功能**: MORPH模式（0-45°渐变，LOITER高度控制）

**核心逻辑**:
```cpp
void ModeMorph::run() {
    // 使用LOITER控制保持高度（✅已修正，原为STABILIZE）
    float target_climb_rate = get_pilot_desired_climb_rate(
        channel_throttle->get_control_in()
    );
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    pos_control->update_z_controller();
    
    // 渐变角度 0° → 45°（默认15°/s）
    morph_angle_deg += rate * dt;
    
    // 更新电机补偿
    motors->set_morph_angle(morph_angle_deg);
}
```

---

### 2. mode_ground.cpp/h
**功能**: GROUND模式（90°水平构型差速转向 + 航向稳定）

**差速控制 + 航向保持**:
```cpp
void ModeGround::run() {
    float forward = pitch_in / 4500.0f;   // 俯仰→前进
    float diff = roll_in / 4500.0f;       // 横滚→转向
    
    // ✅航向稳定：Roll回中时P控制器保持航向
    if (fabsf(roll_in) < 100.0f) {
        if (!_yaw_locked) {
            _target_yaw_rad = ahrs.yaw;
            _yaw_locked = true;
        }
        float yaw_error = wrap_PI(ahrs.yaw - _target_yaw_rad);
        diff = constrain_float(-yaw_error * 2.0f, -1.0f, 1.0f);
    } else {
        _yaw_locked = false;
    }
    
    motors->set_ground_thrust(forward, diff);
}
```

---

### 3. WuK_Comms.cpp/h
**功能**: 飞控↔Arduino串口通信

**协议**: `[0xAA][MsgId][Len][Payload][CRC8]`
- CMD_ARM_SERVO (0x01): 舵机角度控制
- CMD_ACT1/ACT2 (0x02/0x03): RC7/RC8透传
- RSP_STATUS (0x82): Arduino状态反馈

---

### 4. tools/wuk_analyzer.cpp
**功能**: SITL数据分析器（C++）v2.0

**输入**: wuk_gb.jsonl（SITL仿真输出的JSONL格式日志）

**处理流程**:
1. 解析JSONL → 提取电机推力向量(vx,vy,vz)、姿态(roll,pitch,yaw)、高度
2. 计算电机倾角：
   - `motor_tilt_body`: 推力相对机体roll轴的倾角 = atan2(|vx|, |vz|)
   - `motor_tilt_world`: 推力相对世界坐标系 = motor_tilt_body + roll_deg
3. 检测关键事件：
   - 变构开始：events.morph_start == true
   - 快速下降：高度下降速率 < -1.5 m/s
4. 高斯平滑滤波：窗口=7, σ=1.5（消除PID控制抖动）

**输出CSV**: 
```
time_s,fc_angle,morph_active,roll_deg,pitch_deg,yaw_deg,altitude_m,
m1_tilt_body,m2_tilt_body,m3_tilt_body,m4_tilt_body,
m1_tilt_world,m2_tilt_world,m3_tilt_world,m4_tilt_world,
m1_thrust,m2_thrust,m3_thrust,m4_thrust,
total_thrust,vertical_thrust,morph_start,altitude_drop
```

**编译**: `g++ -o wuk_analyzer wuk_analyzer.cpp -std=c++11`

---

### 5. tools/wuk_plotter.py
**功能**: 数据可视化工具（Python）v2.0

**输入**: wuk_analyzer.cpp输出的CSV文件

**生成4张图表**:

1. **图1 - 电机倾角（机体坐标系）**
   - 显示4个电机推力方向相对机体roll轴的倾角变化
   - 理想情况：0° → 45°（MORPH）或 90°（GROUND）
   - 参考线：45°（目标）, 90°（GROUND模式）

2. **图2 - 电机倾角&姿态（世界坐标系）**
   - 电机倾角（世界） = 电机倾角（机体）+ 机体Roll角
   - 机体Roll角（黑色虚线）
   - 验证推力补偿是否正确抵消姿态变化

3. **图3 - 推力变化**
   - 4电机推力（彩色线）
   - 总推力（黑色实线）
   - 垂直分力（绿色虚线）
   - 理想：总推力随倾角增大，垂直分力保持稳定

4. **图4 - 高度变化**
   - 机体高度（紫色，带填充）
   - 参考线：5m悬停高度
   - 验证高度控制是否稳定

**事件标注**:
- 红色虚线：变构开始时刻
- 橙色虚线：机体快速下降时刻（如有）

**输出**: wuk_analysis.png (200 DPI高清图)

**使用**: `python3 wuk_plotter.py wuk_analysis.csv wuk_plot.png`

---

## 🔧 核心修改

### 1. 推力补偿 (AP_MotorsMatrix.cpp)

#### 垂直推力补偿（仅0-45°范围）
```cpp
// ✅修正：使用 < 45.0f 排除GROUND模式(90°)
bool apply_morph = _morph_comp_enabled && 
                   _morph_angle_deg > 0.01f && 
                   _morph_angle_deg < 45.0f;  // 原为 <=

if (apply_morph) {
    throttle_thrust /= cosf(angle_rad);  // 补偿垂直分量损失
    throttle_thrust = MIN(throttle_thrust, 1.0f);
}
```

#### Roll/Yaw因子旋转矩阵
```cpp
// 电机倾转后，Yaw因子部分转为Roll因子
for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
    float original_yaw = _original_yaw_factors[i];
    _yaw_factor[i] = original_yaw * cos_morph;
    _roll_factor[i] += original_yaw * sin_morph;
}
```

---

### 2. X型差速转向 (set_ground_thrust)

```cpp
float left_thrust = forward - diff * 0.5f;
float right_thrust = forward + diff * 0.5f;

// ✅已修正：标准X型布局
_thrust_rpyt_out[0] = right_thrust;  // M1 前右(45°)
_thrust_rpyt_out[1] = left_thrust;   // M2 前左(-45°)
_thrust_rpyt_out[2] = left_thrust;   // M3 后左(-135°)
_thrust_rpyt_out[3] = right_thrust;  // M4 后右(135°)
```

---

### 3. SITL电机布局 (SIM_Frame.cpp)

**wuk_motors定义** (✅已修正):
```cpp
Motor(AP_MOTORS_MOT_1,   45, ..., 8, ...),  // M1 前右
Motor(AP_MOTORS_MOT_2,  -45, ..., 8, ...),  // M2 前左 ✅
Motor(AP_MOTORS_MOT_3, -135, ..., 8, ...),  // M3 后左 ✅
Motor(AP_MOTORS_MOT_4,  135, ..., 8, ...),  // M4 后右
```

**伺服仿真** (20°/s速率限制):
```cpp
float target_pwm = 1000 + (g_wuk_target_angle / 90.0f) * 1000.0f;
// 平滑追踪目标PWM，模拟伺服延迟
```

---

### 4. 模式切换安全 (Copter.cpp)

#### 快速循环中的角度重置（✅关键修复）
```cpp
void Copter::fast_loop() {
    // ... 原有代码 ...
    
    // [WuK] 非MORPH模式重置电机角度，防止补偿残留
    AP_MotorsMatrix *motors_matrix = (AP_MotorsMatrix*)motors;
    if (motors_matrix) {
        if (flightmode->mode_number() == Mode::Number::MORPH) {
            // MORPH模式内部管理角度，不重置
        } else {
            // 其他模式：GROUND=90°, 其余=0°
            float reset_angle = (flightmode->mode_number() == Mode::Number::GROUND) 
                                ? 90.0f : 0.0f;
            motors_matrix->set_morph_angle(reset_angle);
        }
    }
}
```

**修复问题**: RC3=1500坠毁bug
- **原因**: LOITER→MORPH→LOITER切换后，_morph_angle_deg保留45°
- **影响**: 非MORPH模式仍应用推力补偿，油门降低时补偿放大，导致失控
- **解决**: 每个fast_loop周期强制重置，确保补偿只在MORPH模式生效

---

## 📋 参数配置

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| WUK_MORPH_RATE | 15.0 | 5-30 | 变构速率(°/s) |
| WUK_MORPH_DWELL | 2000 | 1000-5000 | 45°驻留时间(ms) |
| WUK_COMP_EN | 1 | 0/1 | 推力补偿使能 |
| SERIAL4_PROTOCOL | 28 | - | USER协议(Arduino通信) |
| SERIAL4_BAUD | 57600 | - | 串口波特率 |

---

## ✅ 关键修正记录

1. **MORPH模式高度不稳** ✅
   - 原因: 使用STABILIZE控制，依赖手动油门
   - 修正: 改为LOITER控制，使用pos_control自动高度保持
   - 位置: mode_morph.cpp

2. **GROUND模式转向错误** ✅
   - 原因: 使用+型布局，M2/M3映射错误
   - 修正: 改为X型布局（M1/M4右侧，M2/M3左侧）
   - 位置: AP_MotorsMatrix.cpp - set_ground_thrust()

3. **SITL电机映射不一致** ✅
   - 原因: M2/M3位置与飞控定义不符
   - 修正: M2=-45°(前左), M3=-135°(后左)
   - 位置: SIM_Frame.cpp - wuk_motors定义

4. **推力补偿仅LOITER有效** ✅
   - 设计: 通过get_throttle_hover()判断是否为高度控制模式
   - 原因: STABILIZE模式手动油门，补偿会导致突变
   - 位置: AP_MotorsMatrix.cpp - output_armed_stabilizing()

5. **RC3=1500坠毁bug** ✅
   - 原因: MORPH→LOITER切换后_morph_angle_deg残留45°，补偿仍生效
   - 症状: 降油门时补偿放大(throttle/=cos(45°))导致失控
   - 修正: Copter.cpp fast_loop()每周期重置非MORPH模式角度
   - 位置: ArduCopter/Copter.cpp - fast_loop()

6. **补偿条件排除GROUND模式** ✅
   - 原因: 原条件 <= 45.0f 会在45°时仍启用补偿
   - 修正: 改为 < 45.0f，确保90° GROUND模式不启用
   - 位置: libraries/AP_Motors/AP_MotorsMatrix.cpp - line 327

7. **GROUND模式缺少航向稳定** ✅
   - 原因: Roll回中后无自动航向保持，需持续手动修正
   - 修正: 添加P控制器(Kp=2.0)，Roll死区内自动保持航向
   - 位置: ArduCopter/mode_ground.cpp + mode_ground.h

8. **数据分析工具格式不匹配** ✅
   - 原因: parser输出列名与plot期望不一致(time vs time_s等)
   - 修正: 统一CSV格式(time_s, roll_deg, m1_thrust_N等)
   - 位置: tools/wuk_parser.cpp - write_csv()函数

---

## 🔗 相关文档

- **WUK_项目介绍与进展.md** - 项目总览
- **WUK_测试命令集.md** - SITL测试流程
- **WUK改动位置索引.txt** - 代码快速索引
