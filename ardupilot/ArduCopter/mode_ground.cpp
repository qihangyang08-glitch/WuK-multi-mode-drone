// [WuK-GROUND] 地面控制模式实现 - 90°构型下的RC车式控制
// 功能: Pitch摇杆控制前进推力, Roll摇杆控制差速转向
// 改动位置: 新增文件

#include "Copter.h"
#include "mode_ground.h"
#include <AP_Motors/AP_MotorsMatrix.h>

bool ModeGround::init(bool ignore_checks)
{
    // 确保电机处于安全状态
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    
    // [WuK] 初始化航向保持：记录当前航向作为目标
    _target_yaw_rad = ahrs.get_yaw();
    _yaw_locked = false;
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WuK: Switched to GROUND Mode");

    return true;
}

void ModeGround::run()
{
    // [WuK-GROUND] 姿态稳定控制：保持Roll/Pitch水平
    float target_roll_rad = 0.0f;   // 保持Roll=0（不允许侧倾）
    float target_pitch_rad = 0.0f;  // 保持Pitch=0（不允许俯仰）

    // [WuK] 航向控制逻辑：RC4回中时保持航向，RC4偏移时手动转向
    RC_Channel *steer_channel = RC_Channels::rc_channel(CH_4);  // 固定使用RC4做方向输入
    float diff_thrust = steer_channel != nullptr ? -steer_channel->norm_input() : 0.0f;
    
    float target_yaw_rate_rads = 0.0f;               // 目标偏航角速率
    
    const float roll_deadzone = 0.1f;  // RC4方向摇杆死区（10%）
    
    if (fabsf(diff_thrust) > roll_deadzone) {
        // [模式A] RC4方向摇杆有输入：手动差速转向模式
        // 使用差速推力控制转向，同时更新目标航向
        _target_yaw_rad = ahrs.get_yaw();  // 跟随当前航向
        _yaw_locked = false;
        
        // 将RC4输入映射为期望的偏航角速率（用于姿态控制器）
        // 这样可以协同差速转向和姿态稳定
        target_yaw_rate_rads = diff_thrust * radians(90.0f);  // 最大90°/s
    } else {
        // [模式B] RC4方向摇杆回中：航向保持模式
        // 使用Yaw PID控制器保持航向稳定
        diff_thrust = 0.0f;  // 清零差速推力
        
        if (!_yaw_locked) {
            // 首次进入航向保持：锁定当前航向
            _target_yaw_rad = ahrs.get_yaw();
            _yaw_locked = true;
        }
        
        // 计算航向误差（考虑±180°环绕）
        float yaw_error_rad = wrap_PI(_target_yaw_rad - ahrs.get_yaw());
        
        // 使用P控制器计算修正的偏航角速率
        // 增益可调：较大值响应快但可能震荡，较小值稳定但响应慢
        const float yaw_p_gain = 2.0f;  // P增益（建议范围1.0~5.0）
        target_yaw_rate_rads = yaw_error_rad * yaw_p_gain;
        
        // 限制最大修正角速率（防止过激反应）
        const float max_yaw_correction_rads = radians(45.0f);  // 最大45°/s
        target_yaw_rate_rads = constrain_float(target_yaw_rate_rads, 
            -max_yaw_correction_rads, max_yaw_correction_rads);
    }
    
    // 运行姿态控制器（Roll/Pitch稳定 + Yaw航向保持/手动控制）
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads  // ← 动态Yaw控制
    );
    
    // 读取飞手前进输入
    // Pitch 摇杆 -> 前进推力（正向控制：摇杆前推=推力增大）
    float fwd_thrust = channel_pitch->norm_input();
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "fwd_thrust=%.2f ", fwd_thrust);
    // 将前进推力限制为仅正值（不反向）
    if (fwd_thrust < 0.0f) fwd_thrust = 0.0f;
    
    // 设置电机状态
    if (motors->armed() && fwd_thrust > 0.0f) {
        // 当有推力需求时，允许无限制油门
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // 否则保持怠速状态
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    
    // 调用电机 API（叠加差速控制到姿态控制输出）
    AP_MotorsMatrix *motors_matrix = (AP_MotorsMatrix *)motors;
    
    motors_matrix->set_ground_thrust(fwd_thrust, diff_thrust);
    
    // 注意：不能直接set_throttle_out(0)，否则会覆盖姿态控制器的输出
    // 姿态控制器已经通过input_euler_angle调用链设置了throttle
}

// [WuK-GROUND] 覆盖输出函数，直接使用set_ground_thrust()的推力输出
// 不使用标准的output_armed_stabilizing()混控逻辑
void ModeGround::output_to_motors()
{
    // [WuK] 跳过标准混控(output_armed_stabilizing)，直接输出地面模式推力
    // 因为set_ground_thrust()已经在run()中设置了_thrust_rpyt_out
    
    // 调用AP_MotorsMulticopter的public方法，该方法会跳过output_armed_stabilizing()
    motors->output_direct_to_motors();
}
