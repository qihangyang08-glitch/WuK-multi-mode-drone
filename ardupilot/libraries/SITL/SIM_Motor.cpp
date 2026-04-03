/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple electric motor simulator class
*/

#include "SIM_Motor.h"
#include <AP_Motors/AP_Motors.h>
#include <cmath>
#include <stdio.h>

using namespace SITL;

// calculate rotational accel and thrust for a motor
// 计算单个电机的旋转加速度和推力
// 参数说明见函数签名：读取输入舵机、位置、气流速度、陀螺仪、空气密度、电压等
void Motor::calculate_forces(const struct sitl_input &input,
                             uint8_t motor_offset,
                             Vector3f &torque,
                             Vector3f &thrust,
                             const Vector3f &velocity_air_bf,
                             const Vector3f &gyro,
                             float air_density,
                             float voltage,
                             bool use_drag)
{

    const float pwm = input.servos[motor_offset+servo];
    float command = pwm_to_command(pwm);
    float voltage_scale = voltage / voltage_max;

    if (voltage_scale < 0.1) {
        // battery is dead
        torque.zero();
        thrust.zero();
        current = 0;
        return;
    }

    // apply slew limiter to command
    // 应用指令速率限制（slew limiter），防止命令突变
    uint64_t now_us = AP_HAL::micros64();
    if (last_calc_us != 0 && slew_max > 0) {
        float dt = (now_us - last_calc_us)*1.0e-6;
        float slew_max_change = slew_max * dt;
        command = constrain_float(command, last_command-slew_max_change, last_command+slew_max_change);
    }
    last_calc_us = now_us;
    last_command = command;

    // velocity of motor through air
    // 电机相对于气流的速度（机体参考系下）
    Vector3f motor_vel = velocity_air_bf;

    // add velocity of motor about center due to vehicle rotation
    // 加上由于机体角速度在电机位置产生的线速度 (omega x r)
    motor_vel += -(position % gyro);

    // protect thrust_vector: if it's zero or contains non-finite components
    // then reset to safe default to avoid division-by-zero inside projection
    // 保护推力向量：若向量为零或包含非有限值，则重置为安全默认 (0,0,-1)
    if (!std::isfinite(thrust_vector.x) || !std::isfinite(thrust_vector.y) || !std::isfinite(thrust_vector.z) || thrust_vector.is_zero()) {
        fprintf(stderr, "WUK_ERROR: motor servo=%u bad thrust_vector (%.6f,%.6f,%.6f) - resetting to (0,0,-1)\n",
                (unsigned)servo, thrust_vector.x, thrust_vector.y, thrust_vector.z);
        thrust_vector.x = 0.0f;
        thrust_vector.y = 0.0f;
        thrust_vector.z = -1.0f;
    }

    // calculate velocity into prop, clipping at zero
    // 计算进入桨盘的来流速度（投影到推力方向），并截断为非负
    Vector3f proj = motor_vel.projected(thrust_vector);
    float velocity_in = MAX(0, -proj.z);

    // get thrust for untilted motor
    // 计算在不倾斜（桨盘未旋转）情况下的推力
    float motor_thrust = calc_thrust(command, air_density, velocity_in, voltage_scale);

    /* periodic debug output to aid debugging of tilt/command mapping; low frequency */
    /* 周期性调试输出，用于帮助调试倾斜与命令的映射；低频率打印 */
    static int wuk_motor_dbg_cnt = 0;
    if ((++wuk_motor_dbg_cnt % 200) == 0) {
        fprintf(stderr, "WUK_MOTOR: servo=%u pwm=%.0f cmd=%.3f vin=%.3f thrust=%.3f tv=(%.3f,%.3f,%.3f)\n",
                (unsigned)servo, pwm, command, velocity_in, motor_thrust, thrust_vector.x, thrust_vector.y, thrust_vector.z);
        fflush(stderr);
    }

    // the yaw torque of the motor
    // 电机产生的偏航力矩（与推力相关的自旋导致的反扭矩）
    const float yaw_scale = 0.05 * diagonal_size * motor_thrust;
    Vector3f rotor_torque = thrust_vector * yaw_factor * command * yaw_scale * -1.0;

    // thrust in bodyframe NED
    // 推力在机体系（NED）中的表示
    thrust = thrust_vector * motor_thrust;

    // work out roll and pitch of motor relative to it pointing straight up
    // 计算电机相对于垂直向上方向的滚转和俯仰角（用于舵机控制的角度转换）
    float roll = 0, pitch = 0;

    uint64_t now = AP_HAL::micros64();
    
    // possibly roll and/or pitch the motor
    // 根据舵机输入（或外部覆盖值）计算当前的滚转/俯仰角，并通过 update_servo 应用舵机速率限制
    if (roll_servo >= 0) {
        uint16_t servoval;
        if (external_roll_pwm >= 0) {
            servoval = update_servo((uint16_t)external_roll_pwm, now, last_roll_value);
        } else {
            servoval = update_servo(input.servos[roll_servo+motor_offset], now, last_roll_value);
        }
        if (roll_min < roll_max) {
            roll = constrain_float(roll_min + (servoval-1000)*0.001*(roll_max-roll_min), roll_min, roll_max);
        } else {
            roll = constrain_float(roll_max + (2000-servoval)*0.001*(roll_min-roll_max), roll_max, roll_min);
        }
    }
    if (pitch_servo >= 0) {
        uint16_t servoval;
        if (external_pitch_pwm >= 0) {
            servoval = update_servo((uint16_t)external_pitch_pwm, now, last_pitch_value);
        } else {
            servoval = update_servo(input.servos[pitch_servo+motor_offset], now, last_pitch_value);
        }
        if (pitch_min < pitch_max) {
            pitch = constrain_float(pitch_min + (servoval-1000)*0.001*(pitch_max-pitch_min), pitch_min, pitch_max);
        } else {
            pitch = constrain_float(pitch_max + (2000-servoval)*0.001*(pitch_min-pitch_max), pitch_max, pitch_min);
        }
    }
    last_change_usec = now;

    // possibly rotate the thrust vector and the rotor torque
    // 若 roll/pitch 非零，则构造旋转矩阵并旋转推力向量与转子力矩
    if (!is_zero(roll) || !is_zero(pitch)) {
        Matrix3f rotation;
        rotation.from_euler(radians(roll), radians(pitch), 0);
        thrust = rotation * thrust;
        rotor_torque = rotation * rotor_torque;
        current_thrust_vector = rotation * thrust_vector;
    } else {
        current_thrust_vector = thrust_vector;
    }

    if (use_drag) {
        // calculate momentum drag per motor
        // 计算每个电机的动量阻力（近似项），用于补偿推力随来流速度变化的不准确建模
        const float momentum_drag_factor = momentum_drag_coefficient * sqrtf(air_density * true_prop_area);
        Vector3f momentum_drag;
        momentum_drag.x = momentum_drag_factor * motor_vel.x * (sqrtf(fabsf(thrust.y)) + sqrtf(fabsf(thrust.z)));
        momentum_drag.y = momentum_drag_factor * motor_vel.y * (sqrtf(fabsf(thrust.x)) + sqrtf(fabsf(thrust.z)));
        // The application of momentum drag to the Z axis is a 'hack' to compensate for incorrect modelling
        // of the variation of thust with inflow velocity. If not applied, the vehicle will
        // climb at an unrealistic rate during operation in STABILIZE. TODO replace prop and motor model in
        // with one based on DC motor, momentum disc and blade element theory.
        // 将动量阻力应用于Z轴是一个“权宜之计”，用于补偿当前模型在来流速度对推力影响建模上的不准确性。
        // 若不应用该修正，飞行器在 STABILIZE 模式下会出现不现实的爬升。TODO: 用基于直流电机、动量盘和叶片元件理论的模型替换此实现。
        momentum_drag.z = momentum_drag_factor * motor_vel.z * (sqrtf(fabsf(thrust.x)) + sqrtf(fabsf(thrust.y)) + sqrtf(fabsf(thrust.z)));

        thrust -= momentum_drag;
    }

    // calculate total torque in newton-meters
    // 计算合力矩（位置叉乘推力 + 转子自带的反扭矩）
    torque = (position % thrust) + rotor_torque;

    // calculate current
    // 计算当前电流：通过功率因子将推力映射为功率，再除以电压
    float power = power_factor * fabsf(motor_thrust);
    current = power / MAX(voltage, 0.1);
}

/*
  update and return current value of a servo. Calculated as 1000..2000
  更新并返回舵机的当前值，范围为 1000..2000
  - 对于带速率限制的舵机，会根据 `servo_rate` 和时间差限制变化速率
  - 对 retract 类型的舵机，有特殊阈值处理以避免中间状态
 */
uint16_t Motor::update_servo(uint16_t demand, uint64_t time_usec, float &last_value) const
{
    if (servo_rate <= 0) {
        return demand;
    }
    if (servo_type == SERVO_RETRACT) {
        // handle retract servos
        // 处理收放舵机的特殊逻辑：将中间区间映射为保持上一次值
        if (demand > 1700) {
            demand = 2000;
        } else if (demand < 1300) {
            demand = 1000;
        } else {
            demand = last_value;
        }
    }
    demand = constrain_int16(demand, 1000, 2000);
    float dt = (time_usec - last_change_usec) * 1.0e-6f;
    // assume servo moves through 90 degrees over 1000 to 2000
    // 假设舵机在 PWM 从 1000 -> 2000 对应 90 度的移动，计算允许的最大 PWM 变化量
    float max_change = 1000 * (dt / servo_rate) * 60.0f / 90.0f;
    last_value = constrain_float(demand, last_value-max_change, last_value+max_change);
    return uint16_t(last_value+0.5);
}


// calculate current and voltage
float Motor::get_current(void) const
{
    return current;
}

// setup PWM ranges for this motor
void Motor::setup_params(uint16_t _pwm_min, uint16_t _pwm_max, float _spin_min, float _spin_max, float _expo, float _slew_max,
                         float _diagonal_size, float _power_factor, float _voltage_max, float _effective_prop_area,
                         float _velocity_max, Vector3f _position, Vector3f _thrust_vector, float _yaw_factor, 
                         float _true_prop_area, float _momentum_drag_coefficient)
{
    mot_pwm_min = _pwm_min;
    mot_pwm_max = _pwm_max;
    mot_spin_min = _spin_min;
    mot_spin_max = _spin_max;
    mot_expo = _expo;
    slew_max = _slew_max;
    power_factor = _power_factor;
    voltage_max = _voltage_max;
    effective_prop_area = _effective_prop_area;
    max_outflow_velocity = _velocity_max;
    true_prop_area = _true_prop_area;
    momentum_drag_coefficient = _momentum_drag_coefficient;
    diagonal_size = _diagonal_size;

    if (!_position.is_zero()) {
        position = _position;
    } else {
        position.x = cosf(radians(angle)) * _diagonal_size;
        position.y =  sinf(radians(angle)) * _diagonal_size;
        position.z = 0;
    }

    if (!_thrust_vector.is_zero()) {
        thrust_vector = _thrust_vector;
    }
    if (!is_zero(_yaw_factor)) {
        yaw_factor = _yaw_factor;
    }
}

/*
  convert a PWM value to a command value from 0 to 1
*/
float Motor::pwm_to_command(float pwm) const
{
    const float pwm_thrust_max = mot_pwm_min + mot_spin_max * (mot_pwm_max - mot_pwm_min);
    const float pwm_thrust_min = mot_pwm_min + mot_spin_min * (mot_pwm_max - mot_pwm_min);
    const float pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;
    return constrain_float((pwm-pwm_thrust_min)/pwm_thrust_range, 0, 1);
}

/*
  calculate thrust given a command value
*/
float Motor::calc_thrust(float command, float air_density, float velocity_in, float voltage_scale) const
{
    float velocity_out = voltage_scale * max_outflow_velocity * sqrtf((1-mot_expo)*command + mot_expo*sq(command));
    float ret = 0.5 * air_density * effective_prop_area * (sq(velocity_out) - sq(velocity_in));
#if 0
    if (command > 0) {
        ::printf("air_density=%f effective_prop_area=%f velocity_in=%f velocity_max=%f\n",
                 air_density, effective_prop_area, velocity_in, voltage_scale * max_outflow_velocity);
        ::printf("calc_thrust %.3f %.3f\n", command, ret);
    }
#endif
    return ret;
}
