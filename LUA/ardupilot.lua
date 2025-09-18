-- 跨介质可变构无人机控制脚本
-- 支持飞行、地面、水面三种模式
-- 版本：1.6 (与Arduino统一版)

-- ==========================================================
-- 常量定义
-- ==========================================================
local SCRIPT_NAME = "Multi-Mode Drone"
local SCRIPT_VERSION = "1.6"

-- 通道和功能号定义
local CH_THROTTLE = 3 -- 油门遥控通道
local CH_ROLL = 1     -- 横滚遥控通道
local CH_PITCH = 2    -- 俯仰遥控通道
local CH_YAW = 4      -- 偏航遥控通道
local CH_MODE = 7     -- 模式切换遥控通道（三态）

-- 为电机分配的脚本功能号 (Scripting Functions)
local FN_MOTOR1 = 94 -- Scripting 1
local FN_MOTOR2 = 95 -- Scripting 2
local FN_MOTOR3 = 96 -- Scripting 3
local FN_MOTOR4 = 97 -- Scripting 4

-- ArduPilot原生的电机功能号
local FN_AP_MOTOR1 = 36 -- Motor1
local FN_AP_MOTOR2 = 33 -- Motor2
local FN_AP_MOTOR3 = 34 -- Motor3
local FN_AP_MOTOR4 = 35 -- Motor4

-- 未使用的SERVO端口作为临时绑定
local TEMP_SERVO1 = 13
local TEMP_SERVO2 = 14
local TEMP_SERVO3 = 15
local TEMP_SERVO4 = 16

-- 模式定义（注意与Arduino保持一致）
local MODE_FLIGHT = 1
local MODE_WATER  = 2
local MODE_GROUND = 3

-- PWM阈值 (低=飞行，中=水面，高=地面)
local PWM_FLIGHT_MAX = 1200
local PWM_WATER_MIN  = 1300
local PWM_WATER_MAX  = 1700
local PWM_GROUND_MIN = 1800

-- 水面模式参数
local WATER_PITCH_LIMIT = 20
local WATER_THROTTLE_THRESHOLD = 1200
local WATER_BASE_THROTTLE = 1100
local WATER_MAX_THROTTLE = 1150

-- 地面模式参数
local GROUND_BASE_THROTTLE = 1010
local GROUND_MAX_THROTTLE = 1050
local GROUND_THROTTLE_THRESHOLD = 1200

-- ==========================================================
-- 全局变量
-- ==========================================================
local current_mode = MODE_FLIGHT
local last_mode = MODE_FLIGHT
local update_rate_ms = 50
local script_control_active = false
local debug_counter = 0

-- ==========================================================
-- 辅助函数
-- ==========================================================
function get_channel_pwm(channel)
    if not rc then
        return 1500
    end
    local pwm = rc:get_pwm(channel)
    if pwm and type(pwm) == "number" and pwm >= 900 and pwm <= 2100 then
        return pwm
    else
        return 1500
    end
end

-- 获取当前模式（与Arduino一致）
function get_current_mode()
    local mode_pwm = get_channel_pwm(CH_MODE)
    if debug_counter % 100 == 0 then
        gcs:send_text(0, string.format("LUA_DEBUG: CH%d PWM=%d", CH_MODE, math.floor(mode_pwm)))
    end
    
    if mode_pwm <= PWM_FLIGHT_MAX then
        return MODE_FLIGHT
    elseif mode_pwm >= PWM_WATER_MIN and mode_pwm <= PWM_WATER_MAX then
        return MODE_WATER
    else
        return MODE_GROUND
    end
end

function set_motor_pwm(function_num, pwm_value)
    pwm_value = math.floor(math.max(1000, math.min(2000, pwm_value)))
    if SRV_Channels then
        SRV_Channels:set_output_pwm(function_num, pwm_value)
    end
end

-- ==========================================================
-- 模式处理核心逻辑
-- ==========================================================
function handle_flight_mode()
    if script_control_active then
        param:set('SERVO1_FUNCTION', FN_AP_MOTOR1)
        param:set('SERVO2_FUNCTION', FN_AP_MOTOR2)
        param:set('SERVO3_FUNCTION', FN_AP_MOTOR3)
        param:set('SERVO4_FUNCTION', FN_AP_MOTOR4)
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO1), 0)
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO2), 0)
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO3), 0)
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO4), 0)
        script_control_active = false
        gcs:send_text(0, "LUA_INFO: 飞行模式激活，电机控制权已归还")
    end
end

function handle_ground_mode()
    local throttle_pwm = get_channel_pwm(CH_THROTTLE)
    local roll_pwm = get_channel_pwm(CH_ROLL)
    local base_throttle = 1000
    if throttle_pwm > GROUND_THROTTLE_THRESHOLD then
        local factor = (throttle_pwm - GROUND_THROTTLE_THRESHOLD) / (2000 - GROUND_THROTTLE_THRESHOLD)
        base_throttle = GROUND_BASE_THROTTLE + (GROUND_MAX_THROTTLE - GROUND_BASE_THROTTLE) * factor
    end
    local yaw_value = roll_pwm - 1500
    local turn = yaw_value * 0.1
    set_motor_pwm(FN_MOTOR1, base_throttle + turn)
    set_motor_pwm(FN_MOTOR2, base_throttle - turn)
    set_motor_pwm(FN_MOTOR3, base_throttle - turn)
    set_motor_pwm(FN_MOTOR4, base_throttle + turn)
end

function handle_water_mode()
    local throttle_pwm = get_channel_pwm(CH_THROTTLE)
    local roll_pwm = get_channel_pwm(CH_ROLL)
    local pitch_rad = ahrs:get_pitch()
    local pitch_deg = pitch_rad and math.deg(pitch_rad) or 0
    local base_throttle = 1000
    if throttle_pwm > WATER_THROTTLE_THRESHOLD and math.abs(pitch_deg) <= WATER_PITCH_LIMIT then
        local factor = (throttle_pwm - WATER_THROTTLE_THRESHOLD) / (2000 - WATER_THROTTLE_THRESHOLD)
        base_throttle = WATER_BASE_THROTTLE + (WATER_MAX_THROTTLE - WATER_BASE_THROTTLE) * factor
    end
    local yaw_value = roll_pwm - 1500
    local turn = yaw_value * 0.05
    set_motor_pwm(FN_MOTOR1, base_throttle + turn)
    set_motor_pwm(FN_MOTOR2, base_throttle - turn)
    set_motor_pwm(FN_MOTOR3, base_throttle - turn)
    set_motor_pwm(FN_MOTOR4, base_throttle + turn)
end

-- ==========================================================
-- 模式切换逻辑
-- ==========================================================
function on_mode_change(new_mode)
    if new_mode == MODE_FLIGHT then
        handle_flight_mode()
    else
        if not script_control_active then
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO1), FN_AP_MOTOR1)
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO2), FN_AP_MOTOR2)
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO3), FN_AP_MOTOR3)
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO4), FN_AP_MOTOR4)
            param:set('SERVO1_FUNCTION', FN_MOTOR1)
            param:set('SERVO2_FUNCTION', FN_MOTOR2)
            param:set('SERVO3_FUNCTION', FN_MOTOR3)
            param:set('SERVO4_FUNCTION', FN_MOTOR4)
            script_control_active = true
            gcs:send_text(0, "LUA_INFO: 脚本控制模式已激活")
        end
    end
end

-- ==========================================================
-- 主循环
-- ==========================================================
function update()
    debug_counter = debug_counter + 1
    current_mode = get_current_mode()
    if current_mode ~= last_mode then
        on_mode_change(current_mode)
        last_mode = current_mode
    end
    if arming:is_armed() and script_control_active then
        if current_mode == MODE_GROUND then
            handle_ground_mode()
        elseif current_mode == MODE_WATER then
            handle_water_mode()
        end
    end
    return update, update_rate_ms
end

-- ==========================================================
-- 初始化
-- ==========================================================
function init()
    gcs:send_text(0, string.format("LUA_INFO: %s v%s 初始化完成", SCRIPT_NAME, SCRIPT_VERSION))
    return update, update_rate_ms
end

return init()
