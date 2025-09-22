
# Parameter List

本文件列出无人机电控系统中所有可调参数，包括 **Arduino 控制板** 与 **ArduPilot Lua 脚本**，用于实机调试与功能扩展。

---

## 1. Arduino 程序参数

| 参数名 | 全称 | 功能 | 默认值 | 推荐范围 | 调整效果 |
|--------|------|------|--------|----------|----------|
| `servoDelay` | Servo Step Delay | 舵机动作分步延时（控制转速） | 50 ms | 20–100 ms | 增大 → 动作更平稳但耗时更长；减小 → 响应更快但抖动风险增加 |
| `pushrodTime` | Pushrod Duration | 电推杆伸缩动作持续时间 | 2000 ms | 1000–4000 ms | 需与电推杆实际速度匹配，过短会导致不到位，过长浪费时间 |
| `gearWaitTime` | Gear Wait Duration | 起落架收放完成的等待时间 | 2000 ms | 1500–4000 ms | 确保起落架完全收放后再执行下一步动作 |
| `pre_angle1` | Servo1 Initial Angle | 电机臂舵机初始角度 | 90° | 80–100° | 根据实际安装位置设定，保证初始时臂处于飞行姿态 |
| `pre_angle2` | Servo2 Initial Angle | 电机方向舵机初始角度 | 80° | 70–90° | 根据实际安装位置设定，保证初始推力方向向下 |
| `delta_angle` | Servo Delta Angle | 舵机动作幅度（切换角度） | 90° | 70–100° | 控制电机臂由飞行到地面/水面时的转动角度 |

---

## 2. Lua 脚本参数

| 参数名 | 全称 | 功能 | 默认值 | 推荐范围 | 调整效果 |
|--------|------|------|--------|----------|----------|
| `PWM_FLIGHT_MAX` | Flight Mode PWM Max | 判定飞行模式的最大 PWM 值 | 1200 μs | 1100–1250 μs | 决定 CH7 在低位时的判定范围 |
| `PWM_WATER_MIN` | Water Mode PWM Min | 判定水面模式的最小 PWM 值 | 1300 μs | 1250–1350 μs | 与 `PWM_WATER_MAX` 共同确定中位区间 |
| `PWM_WATER_MAX` | Water Mode PWM Max | 判定水面模式的最大 PWM 值 | 1700 μs | 1650–1750 μs | 与 `PWM_WATER_MIN` 共同确定中位区间 |
| `PWM_GROUND_MIN` | Ground Mode PWM Min | 判定地面模式的最小 PWM 值 | 1800 μs | 1750–1850 μs | 决定 CH7 在高位时的判定范围 |
| `WATER_THROTTLE_SCALE` | Water Throttle Scale | 水面模式油门输出缩放系数 | 1.0 | 0.5–1.5 | 增大 → 推进更强，减小 → 推进更弱 |
| `WATER_THROTTLE_EXP` | Water Throttle Exponent | 水面模式油门响应指数 | 1.0 | 0.8–2.0 | >1：慢起步快加速；<1：快起步但尾部不灵敏 |
| `WATER_PITCH_LIMIT` | Water Pitch Limit | 水面模式允许的最大俯仰角 | 20° | 10–30° | 过大角度时强制停桨，避免失稳 |
| `WATER_THROTTLE_THRESHOLD` | Water Throttle Threshold | 水面模式油门激活门槛 | 1200 μs | 1100–1300 μs | 决定从低油门到启用推进的阈值 |
| `WATER_BASE_THROTTLE` | Water Base Throttle | 水面模式最低油门 | 1100 μs | 1050–1200 μs | 控制水面航行的最低推力 |
| `WATER_MAX_THROTTLE` | Water Max Throttle | 水面模式最大油门 | 1150 μs | 1120–1250 μs | 控制水面航行的最大推力 |
| `GROUND_BASE_THROTTLE` | Ground Base Throttle | 地面模式最低油门 | 1010 μs | 1000–1100 μs | 控制地面低速行驶时的推力 |
| `GROUND_MAX_THROTTLE` | Ground Max Throttle | 地面模式最大油门 | 1050 μs | 1020–1150 μs | 控制地面高速行驶时的推力 |
| `GROUND_THROTTLE_THRESHOLD` | Ground Throttle Threshold | 地面模式油门激活门槛 | 1200 μs | 1100–1300 μs | 低于此值电机保持最小输出，避免乱转 |
| `debug_counter` | Debug Counter Interval | 调试输出间隔计数器 | 100 | ≥10 | 减小数值输出更频繁，增大数值减少刷屏 |

---

