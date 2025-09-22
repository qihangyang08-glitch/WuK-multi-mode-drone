-- 跨介质可变构无人机控制脚本
-- 支持飞行、地面、水面三种模式
-- 版本：1.7 (增强调试版)
-- 变更说明（最小化）:
-- 1) CH7 三态映射：低=飞行, 中=水面, 高=地面  （[修改]）
-- 2) 水面油门映射增加参数 WATER_THROTTLE_SCALE, WATER_THROTTLE_EXP （[修改]）
-- 3) 统一调试输出前缀，但保留所有原有调试调用（[修改]）

local SCRIPT_NAME = "Multi-Mode Drone"
local SCRIPT_VERSION = "1.8"

-- 通道定义
local CH_THROTTLE = 3
local CH_ROLL = 1
local CH_PITCH = 2
local CH_YAW = 4
local CH_MODE = 7

-- 为电机分配的脚本功能号 (Scripting Functions)
local FN_MOTOR1 = 94 -- Scripting 1
local FN_MOTOR2 = 95 -- Scripting 2
local FN_MOTOR3 = 96 -- Scripting 3
local FN_MOTOR4 = 97 -- Scripting 4

-- ArduPilot原生的电机功能号
local FN_AP_MOTOR1 = 36
local FN_AP_MOTOR2 = 33
local FN_AP_MOTOR3 = 34
local FN_AP_MOTOR4 = 35

-- 未使用的SERVO端口作为临时绑定（通常SERVO13-16不会被使用）
local TEMP_SERVO1 = 13
local TEMP_SERVO2 = 14
local TEMP_SERVO3 = 15
local TEMP_SERVO4 = 16

-- 模式定义
local MODE_FLIGHT = 1
local MODE_WATER  = 2
local MODE_GROUND = 3

-- PWM 阈值（默认值；与 Arduino 保持一致）
local PWM_FLIGHT_MAX = 1200
local PWM_WATER_MIN  = 1300
local PWM_WATER_MAX  = 1700
local PWM_GROUND_MIN = 1800

-- [修改] 水面油门映射可调参数（保守默认）
-- SCALE 控制最大输出缩放（0.0 ~ 1.5 推荐）
-- EXP 控制输入响应曲线（1.0 为线性，大于1 变慢起步、敏感尾部）
local WATER_THROTTLE_SCALE = 1.0
local WATER_THROTTLE_EXP   = 1.0

-- 水面模式参数（保持原始默认）
local WATER_PITCH_LIMIT = 20
local WATER_THROTTLE_THRESHOLD = 1200 -- 降低启动阈值
local WATER_BASE_THROTTLE = 1100      -- 降低基础油门
local WATER_MAX_THROTTLE = 1150       -- 大幅降低最大油门

-- 地面模式参数
local GROUND_BASE_THROTTLE = 1000     -- 地面模式基础油门
local GROUND_MAX_THROTTLE = 1050      -- 地面模式最大油门
local GROUND_THROTTLE_THRESHOLD = 1200 -- 地面模式启动阈值

-- 全局状态
local current_mode = MODE_FLIGHT
local last_mode = MODE_FLIGHT
local update_rate_ms = 50
local script_control_active = false
local debug_counter = 0 -- 调试计数器

-- ==========================================================
-- 辅助函数(标准化)
-- ==========================================================

-- 获取通道PWM值，增加健壮性和调试
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

-- 获取当前模式（[修改]：低=飞行, 中=水, 高=地）
function get_current_mode()
    local mode_pwm = get_channel_pwm(CH_MODE)

    -- 每5秒输出一次模式检测信息
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

-- 安全输出电机 PWM
function set_motor_pwm(function_num, pwm_value)
    -- 检查参数
    if not function_num or type(function_num) ~= "number" then
        gcs:send_text(4, "LUA_ERROR: function_num 无效")
        return false
    end
    if not pwm_value or type(pwm_value) ~= "number" then
        gcs:send_text(4, "LUA_ERROR: pwm_value 无效")
        return false
    end
    -- 规范化映射pwm
    pwm_value = math.floor(math.max(1000, math.min(2000, pwm_value)))
    function_num = math.floor(function_num)

    if SRV_Channels then
        SRV_Channels:set_output_pwm(function_num, pwm_value)
        return true
    else
        gcs:send_text(4, "LUA_ERROR: SRV_Channels 不可用")
        return false
    end
end



function debug_channels()
    local throttle = get_channel_pwm(CH_THROTTLE)
    local roll = get_channel_pwm(CH_ROLL)
    local pitch = get_channel_pwm(CH_PITCH)
    local yaw = get_channel_pwm(CH_YAW)
    local mode = get_channel_pwm(CH_MODE)
    gcs:send_text(0, string.format("LUA_DEBUG: RC通道 - T:%d R:%d P:%d Y:%d M:%d",
        math.floor(throttle), math.floor(roll), math.floor(pitch), math.floor(yaw), math.floor(mode)))
end

function debug_servo_functions()
    local s1 = param:get('SERVO1_FUNCTION') or -1
    local s2 = param:get('SERVO2_FUNCTION') or -1
    local s3 = param:get('SERVO3_FUNCTION') or -1
    local s4 = param:get('SERVO4_FUNCTION') or -1
    gcs:send_text(0, string.format("LUA_DEBUG: SERVO功能 - S1:%d S2:%d S3:%d S4:%d",
        math.floor(s1), math.floor(s2), math.floor(s3), math.floor(s4)))
end

function debug_system_status()
    local armed = false
    if arming and arming.is_armed then
        -- 兼容不同 API 访问方式
        pcall(function() armed = arming:is_armed() end)
    end
    local mode_text = {"飞行", "水面", "地面"}
    local current_mode_text = mode_text[current_mode] or "未知"
    gcs:send_text(0, string.format("LUA_DEBUG: 系统状态 - 解锁:%s 模式:%s 脚本控制:%s",
        armed and "是" or "否", current_mode_text, script_control_active and "是" or "否"))
end




-- ==========================================================
-- 模式处理核心逻辑
-- ==========================================================
function handle_flight_mode()
    if script_control_active then
        gcs:send_text(0, "LUA_INFO: 切换到飞行模式")
        -- 恢复到正常的电机功能号
        param:set('SERVO1_FUNCTION', FN_AP_MOTOR1)
        param:set('SERVO2_FUNCTION', FN_AP_MOTOR2)
        param:set('SERVO3_FUNCTION', FN_AP_MOTOR3)
        param:set('SERVO4_FUNCTION', FN_AP_MOTOR4)
        -- 清理临时端口
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO1), 0)
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO2), 0)
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO3), 0)
        param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO4), 0)
        
        script_control_active = false -- 调试输出
        debug_servo_functions()
        gcs:send_text(0, "LUA_INFO: 飞行模式激活，电机控制权已归还")
    end
end

-- 地面模式处理：脚本接管电机控制
function handle_ground_mode()
    local throttle_pwm = get_channel_pwm(CH_THROTTLE)
    local roll_pwm = get_channel_pwm(CH_ROLL)

    if not throttle_pwm or not roll_pwm then
        gcs:send_text(4, "LUA_ERROR: 地面模式 - 无法读取控制信号")
        return
    end

    -- 地面模式低速控制逻辑
    local base_throttle = 1000 -- 默认停转
    local throttle_active = throttle_pwm > GROUND_THROTTLE_THRESHOLD

    if throttle_active then
        -- 计算油门映射：从阈值到最大值映射到基础转速到最大转速
        local throttle_range = 2000 - GROUND_THROTTLE_THRESHOLD             --信号值范围
        local throttle_input = throttle_pwm - GROUND_THROTTLE_THRESHOLD     --超出阈值的信号值
        local throttle_factor = throttle_input / throttle_range             --油门比例
        --设置油门阈值：油门达到一定值时才会开始响应，防止误差
        base_throttle = GROUND_BASE_THROTTLE + (GROUND_MAX_THROTTLE - GROUND_BASE_THROTTLE) * throttle_factor
        base_throttle = math.max(GROUND_BASE_THROTTLE, math.min(GROUND_MAX_THROTTLE, base_throttle))
    end

    -- 转向控制
    local yaw_value = roll_pwm - 1500
    local turn_factor = yaw_value * 0.1

    local motor1 = base_throttle + turn_factor -- 右前
    local motor2 = base_throttle - turn_factor -- 左前
    local motor3 = base_throttle - turn_factor -- 左后
    local motor4 = base_throttle + turn_factor -- 右后

    -- 限制PWM范围，暂时屏蔽反转
    motor1 = math.max(1000, math.min(GROUND_MAX_THROTTLE, motor1))
    motor2 = math.max(1000, math.min(GROUND_MAX_THROTTLE, motor2))
    motor3 = math.max(1000, math.min(GROUND_MAX_THROTTLE, motor3))
    motor4 = math.max(1000, math.min(GROUND_MAX_THROTTLE, motor4))

    --输出到电机
    set_motor_pwm(FN_MOTOR1, motor1)
    set_motor_pwm(FN_MOTOR2, motor2)
    set_motor_pwm(FN_MOTOR3, motor3)
    set_motor_pwm(FN_MOTOR4, motor4)
    --调试信息，2秒一次
    if debug_counter % 40 == 0 then
        gcs:send_text(6, string.format("LUA_INFO: 地面模式 - RC油门:%d 激活:%s 基础:%d",
            math.floor(throttle_pwm), throttle_active and "是" or "否", math.floor(base_throttle)))
        gcs:send_text(6, string.format("LUA_INFO: 地面电机 - M1:%d M2:%d M3:%d M4:%d",
            math.floor(motor1), math.floor(motor2), math.floor(motor3), math.floor(motor4)))
    end
end

-- 水面模式处理：脚本接管电机控制
function handle_water_mode()
    local throttle_pwm = get_channel_pwm(CH_THROTTLE)
    local roll_pwm = get_channel_pwm(CH_ROLL)

    local pitch_rad = ahrs:get_pitch()
    local pitch_deg = pitch_rad and math.deg(pitch_rad) or 0

    -- 水面模式低速控制逻辑
    local base_throttle = 1000 -- 默认停转
    local throttle_active = throttle_pwm > WATER_THROTTLE_THRESHOLD

    if throttle_active then
        -- 若俯仰角过大则停止电机基础转速等待恢复
        if math.abs(pitch_deg) > WATER_PITCH_LIMIT then
            base_throttle = 1000
            if debug_counter % 20 == 0 then     --降低调试频率
                gcs:send_text(4, string.format("LUA_WARN: 俯仰角过大(%.1f°)，停止推进", pitch_deg))
            end
        else
            -- 计算油门映射： [修改] 支持缩放与响应曲线
            local throttle_range = 2000 - WATER_THROTTLE_THRESHOLD
            local throttle_input = throttle_pwm - WATER_THROTTLE_THRESHOLD
            local throttle_factor = math.max(0, math.min(1, throttle_input / throttle_range))

            -- v1.7后加入：缩放与响应曲线
            local adj_factor = math.pow(throttle_factor, WATER_THROTTLE_EXP) * WATER_THROTTLE_SCALE

            base_throttle = WATER_BASE_THROTTLE + (WATER_MAX_THROTTLE - WATER_BASE_THROTTLE) * adj_factor
            base_throttle = math.max(WATER_BASE_THROTTLE, math.min(WATER_MAX_THROTTLE, base_throttle))
        end
    end

    --转向控制
    local yaw_value = roll_pwm - 1500
    local turn_factor = yaw_value * 0.05

    local motor1 = base_throttle + turn_factor
    local motor2 = base_throttle - turn_factor
    local motor3 = base_throttle - turn_factor
    local motor4 = base_throttle + turn_factor

    motor1 = math.max(1000, math.min(WATER_MAX_THROTTLE, motor1))
    motor2 = math.max(1000, math.min(WATER_MAX_THROTTLE, motor2))
    motor3 = math.max(1000, math.min(WATER_MAX_THROTTLE, motor3))
    motor4 = math.max(1000, math.min(WATER_MAX_THROTTLE, motor4))

    set_motor_pwm(FN_MOTOR1, motor1)
    set_motor_pwm(FN_MOTOR2, motor2)
    set_motor_pwm(FN_MOTOR3, motor3)
    set_motor_pwm(FN_MOTOR4, motor4)

    if debug_counter % 40 == 0 then
        gcs:send_text(6, string.format("LUA_INFO: 水面模式 - RC油门:%d 激活:%s 基础:%d SCALE:%.2f EXP:%.2f",
            math.floor(throttle_pwm), throttle_active and "是" or "否", math.floor(base_throttle),
            WATER_THROTTLE_SCALE, WATER_THROTTLE_EXP))
        gcs:send_text(6, string.format("LUA_INFO: 水面电机 - M1:%d M2:%d M3:%d M4:%d",
            math.floor(motor1), math.floor(motor2), math.floor(motor3), math.floor(motor4)))
    end
end

-- ==========================================================
-- 模式切换逻辑（保留接管逻辑）
-- ==========================================================
function on_mode_change(new_mode)
    local mode_names = {"飞行", "水面", "地面"}
    gcs:send_text(0, string.format("LUA_INFO: 模式切换: %s -> %s",
        mode_names[last_mode] or "未知", mode_names[new_mode] or "未知"))

    if new_mode == MODE_FLIGHT then
        handle_flight_mode()
    else -- 地面或水面模式
        if not script_control_active then
            gcs:send_text(0, "LUA_INFO: 激活脚本控制模式...")

            -- 先将电机绑定到临时端口（避免电机检查）
            gcs:send_text(0, "LUA_DEBUG: 设置临时电机端口...")
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO1), FN_AP_MOTOR1)
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO2), FN_AP_MOTOR2)
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO3), FN_AP_MOTOR3)
            param:set(string.format('SERVO%d_FUNCTION', TEMP_SERVO4), FN_AP_MOTOR4)

            -- 将主端口设置为脚本控制
            gcs:send_text(0, "LUA_DEBUG: 设置脚本控制端口...")
            param:set('SERVO1_FUNCTION', FN_MOTOR1)
            param:set('SERVO2_FUNCTION', FN_MOTOR2)
            param:set('SERVO3_FUNCTION', FN_MOTOR3)
            param:set('SERVO4_FUNCTION', FN_MOTOR4)

            script_control_active = true
            debug_servo_functions()
            gcs:send_text(0, "LUA_INFO: 脚本控制模式已激活")
        end
    end
end

-- ==========================================================
-- 主循环与初始化
-- ==========================================================
-- 主更新函数
function update()
    debug_counter = debug_counter + 1

    -- 每10秒输出一次系统状态（无论是否解锁）
    if debug_counter % 200 == 0 then
        debug_system_status()
        debug_channels()
    end

    -- 如果未解锁，仍然检测模式但不执行电机控制
    current_mode = get_current_mode()

    -- 模式切换检测（解锁与否都要检测）
    if current_mode ~= last_mode then
        gcs:send_text(0, string.format("LUA_INFO: 检测到模式变化 %d -> %d", last_mode, current_mode))
        on_mode_change(current_mode)
        last_mode = current_mode
    end

    if arming:is_armed() then
        if script_control_active then
            if current_mode == MODE_GROUND then
                handle_ground_mode()
            elseif current_mode == MODE_WATER then
                handle_water_mode()
            end
        end

        if debug_counter % 100 == 0 then
            if script_control_active then
                gcs:send_text(0, string.format("LUA_INFO: 脚本正在控制电机 - 模式:%s",
                    current_mode == MODE_GROUND and "地面" or "水面"))
            else
                gcs:send_text(0, "LUA_INFO: 飞控正在控制电机 - 飞行模式")
            end
        end
    end

    return update, update_rate_ms
end

-- 脚本初始化
function init()
    gcs:send_text(0, string.format("LUA_INFO: %s v%s 初始化开始", SCRIPT_NAME, SCRIPT_VERSION))

    -- 检查核心API
    local apis = {
        {name = "rc", obj = rc},
        {name = "SRV_Channels", obj = SRV_Channels},
        {name = "ahrs", obj = ahrs},
        {name = "param", obj = param},
        {name = "arming", obj = arming},
        {name = "millis", obj = millis},
        {name = "gcs", obj = gcs}
    }

    for _, api in ipairs(apis) do
        if not api.obj then
            gcs:send_text(0, string.format("LUA_DEBUG: API %s 尚未就绪，1秒后重试...", api.name))
            return init, 1000
        end
    end

    -- 显示配置信息
    gcs:send_text(0, string.format("LUA_INFO: 模式切换通道: CH%d", CH_MODE))
    gcs:send_text(0, string.format("LUA_INFO: PWM阈值 - 飞行:<=%d 水面:%d-%d 地面:>=%d",
        PWM_FLIGHT_MAX, PWM_WATER_MIN, PWM_WATER_MAX, PWM_GROUND_MIN))

    -- 显示当前SERVO功能设置
    debug_servo_functions()
    gcs:send_text(0, "LUA_INFO: 初始化完成，开始主循环")
    gcs:send_text(0, "LUA_INFO: 请切换模式开关测试模式检测功能")

    return update, update_rate_ms
end

-- 启动脚本
return init()
