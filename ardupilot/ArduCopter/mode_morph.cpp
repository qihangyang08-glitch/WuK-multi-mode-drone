// [WuK-MORPH] 变构模式实现 - 控制机臂从0°→45°渐进式变构
// 功能: 低高度变构过程中维持姿态稳定,到达45°后悬停等待,然后切换到GROUND模式
// 改动位置: 新增文件

#include "Copter.h"
#include "mode_morph.h"
#include <AP_Motors/AP_MotorsMatrix.h>

#define Corner_Threshold 45.0f// 变构角度阈值，单位：度
#define Morph_Hold45_ms 1000U // 到达45°后短暂维持飞行约1秒

bool ModeMorph::init(bool ignore_checks)
{
    _current_morph_angle = 0.0f;
    _holding_45 = false;
    _dwelling = false;
    _hold_start_ms = 0;
    _dwell_start_ms = 0;
    _morph_start_ms = AP_HAL::millis();
    _last_arm_cmd_send_ms = 0;
    _last_enqueue_warn_ms = 0;
    _last_arm_cmd_value = 0xFFFF;
    _last_key_bucket = 0xFF;
    _pending_key_repeats = 0;
    _pending_key_value = 0;
    // _dwell_start_ms == 0 表示尚未开始切换（90°）阶段
    // 确保开始时姿态控制有效
    attitude_control->reset_rate_controller_I_terms();
    attitude_control->reset_yaw_target_and_rate();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WuK: Switched to MORPH Mode");

    return true;
}

void ModeMorph::run()
{
    static uint32_t last_log_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();

    if (!_dwelling) {
        // [阶段1] 0° → 45° 渐变过程，使用LOITER控制
        if (!_holding_45) {
            float rate = g2.wuk_morph_rate;
            if (rate <= 0) rate = 15.0f; // 安全默认值

            // 使用计时器定义角度，降低循环频率抖动对进度速度的影响
            const float elapsed_s = (now_ms - _morph_start_ms) * 0.001f;
            _current_morph_angle = MIN(Corner_Threshold, rate * elapsed_s);

            if (_current_morph_angle >= Corner_Threshold) {
                // 达到45°，先进入短暂45°维持阶段（仍保持飞行控制）
                _current_morph_angle = Corner_Threshold;
                _holding_45 = true;
                _hold_start_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WuK MORPH hold45 start");
                hal.console->printf("WuK MORPH hold45 start\n");
            }
        } else {
            // [阶段1.5] 45°短暂暂留：维持45°，继续飞行控制约1秒
            _current_morph_angle = Corner_Threshold;
            if (now_ms - _hold_start_ms >= Morph_Hold45_ms) {
                // 暂留完成，进入怠速+90°阶段
                _holding_45 = false;
                _dwelling = true;
                _dwell_start_ms = now_ms;

                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
                attitude_control->set_throttle_out(0.0f, true, 0.0f);
                copter.wuk_comms.enqueue_cmd(WuK_Comms::CmdID::ARM_SERVO, 90);

                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WuK MORPH hold45 done -> idle + 90 cmd");
                hal.console->printf("WuK MORPH hold45 done -> idle + 90 cmd\n");
            }
        }

        // 调试输出：0-45° 变构进度
        if (now_ms - last_log_ms > 200) { // 5Hz 防刷屏
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WuK MORPH run angle=%.1f rate=%.1f", (double)_current_morph_angle, (double)rate);
            //hal.console->printf("WuK MORPH run angle=%.1f rate=%.1f\n", (double)_current_morph_angle, (double)rate);
            last_log_ms = now_ms;
        }

        if (!_dwelling) {
            // 变构过渡期间（< 45°）的正常飞行行为
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            const uint16_t angle_cmd = (uint16_t)constrain_float(_current_morph_angle + 0.5f, 0.0f, Corner_Threshold);
            const uint8_t key_bucket = (uint8_t)(angle_cmd / 5U); // 每5度一个关键角
            if (key_bucket != _last_key_bucket) {
                _last_key_bucket = key_bucket;
                _pending_key_repeats = 2; // 关键角额外重复2次
                _pending_key_value = angle_cmd;
            }

            bool should_send = false;
            uint16_t send_value = angle_cmd;

            // 正常发送：角度变化或超时保活
            if ((angle_cmd != _last_arm_cmd_value && now_ms - _last_arm_cmd_send_ms >= 80) ||
                (now_ms - _last_arm_cmd_send_ms >= 300)) {
                should_send = true;
            }

            // 关键角重复：缩短丢包空窗
            if (!should_send && _pending_key_repeats > 0 && now_ms - _last_arm_cmd_send_ms >= 40) {
                should_send = true;
                send_value = _pending_key_value;
            }

            if (should_send) {
                if (copter.wuk_comms.enqueue_cmd(WuK_Comms::CmdID::ARM_SERVO, send_value)) {
                    _last_arm_cmd_send_ms = now_ms;
                    _last_arm_cmd_value = send_value;
                    if (_pending_key_repeats > 0) {
                        _pending_key_repeats--;
                    }
                } else if (now_ms - _last_enqueue_warn_ms > 1000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WuK MORPH ARM cmd queue full");
                    _last_enqueue_warn_ms = now_ms;
                }
            }

            // 更新全局变量供 SITL 使用
            // extern float g_wuk_target_angle;
            // g_wuk_target_angle = _current_morph_angle;

            // 更新电机混控的变构角度
            AP_MotorsMatrix *motors_matrix = (AP_MotorsMatrix *)motors;
            motors_matrix->set_morph_angle(_current_morph_angle);

            // [WuK] 模仿LOITER模式的姿态和高度保持逻辑
            // 参考: mode_loiter.cpp - 保留姿态稳定、高度控制和位置锁定
            
            // 1. 设置高度控制器的速度和加速度限制（LOITER必备）
            pos_control->set_max_speed_accel_U_m(get_pilot_speed_dn_ms(), 
                get_pilot_speed_up_ms(), get_pilot_accel_U_mss());
            
            // 2. 获取飞手Roll/Pitch输入（使用LOITER的角度限制）
            float target_roll_rad, target_pitch_rad;
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, 
                loiter_nav->get_angle_max_rad(), 
                attitude_control->get_althold_lean_angle_max_rad());
            
            // 3. [WuK-FIX] 添加位置控制器，防止水平漂移（可选：注释掉则允许漂移）
            loiter_nav->update();
            
            // 4. 获取Yaw输入
            float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

            // 5. 姿态控制器（设置目标角度和角速度）
            // ⚠️ 电机补偿在attitude_control链条后的motors->output()中生效，不会被绕过
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
                target_roll_rad,
                target_pitch_rad,
                target_yaw_rate_rads
            );

            // 6. 高度控制（LOITER的核心：RC3=1500维持，>1500爬升，<1500下降）
            float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
            target_climb_rate_ms = constrain_float(target_climb_rate_ms, 
                -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());
            
            // 发送爬升率到高度控制器
            pos_control->set_pos_target_U_from_climb_rate_ms(target_climb_rate_ms);
            
            // 运行高度控制器（在姿态控制之后调用，符合LOITER流程）
            pos_control->update_U_controller();
        }
    } else {
        // [阶段2] 驻留阶段：持续发送怠速和90°命令，等待舵机到位
        
        // [WuK-FIX] 更新电机层角度到90°（重要！飞控内部需要知道实际角度）
        AP_MotorsMatrix *motors_matrix = (AP_MotorsMatrix *)motors;
        motors_matrix->set_morph_angle(90.0f);
        
        // [WuK-FIX] 保持姿态水平，防止坠落时翻滚
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(0, 0, 0);
        
        // 持续设置怠速
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        
        // 周期发送90°命令（防止丢包且避免队列压力过大）
        if (now_ms - _last_arm_cmd_send_ms >= 120) {
            if (copter.wuk_comms.enqueue_cmd(WuK_Comms::CmdID::ARM_SERVO, 90)) {
                _last_arm_cmd_send_ms = now_ms;
                _last_arm_cmd_value = 90;
            } else if (now_ms - _last_enqueue_warn_ms > 1000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WuK MORPH 90 cmd queue full");
                _last_enqueue_warn_ms = now_ms;
            }
        }
        
        // 持续更新SITL全局变量
        // extern float g_wuk_target_angle;
        // g_wuk_target_angle = 90.0f;

        // 调试输出：驻留阶段
        if (now_ms - last_log_ms > 200) { // 5Hz
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WuK MORPH dwell angle=90");
            //hal.console->printf("WuK MORPH dwell angle=90\n");
            last_log_ms = now_ms;
        }
        
        // 检查是否到达驻留时间
        float dwell_time = g2.wuk_morph_dwell;
        if (dwell_time <= 0) dwell_time = 2.0f; // 安全默认值（秒）
        
        uint32_t dwell_time_ms = (uint32_t)(dwell_time * 1000.0f);
        if (AP_HAL::millis() - _dwell_start_ms >= dwell_time_ms) {
            // [WuK-FIX] 驻留时间到，切换GROUND
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WuK MORPH -> GROUND after dwell %.2fs", (double)dwell_time);
            //hal.console->printf("WuK MORPH -> GROUND after dwell %.2fs\n", (double)dwell_time);
            set_mode(Mode::Number::GROUND, ModeReason::RC_COMMAND);
        }
    }
}
