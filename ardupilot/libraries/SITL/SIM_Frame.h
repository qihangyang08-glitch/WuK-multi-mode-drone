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
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"
#include <AP_JSON/AP_JSON.h>

#ifndef SIM_FRAME_MAX_ACTUATORS
#define SIM_FRAME_MAX_ACTUATORS 32
#endif

namespace SITL {

/*
  class to describe a multicopter frame type
 */
class Frame {
public:
    const char *name;
    uint8_t num_motors;
    Motor *motors;

    Frame(const char *_name,
          uint8_t _num_motors,
          Motor *_motors) :
          name(_name),
          num_motors(_num_motors),
          motors(_motors) {}

#if AP_SIM_ENABLED
    // find a frame by name
    static Frame *find_frame(const char *name);
    
    // initialise frame
    void init(const char *frame_str, Battery *_battery);

    // calculate rotational and linear accelerations
    void calculate_forces(const Aircraft &aircraft,
                          const struct sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel, float* rpm,
                          bool use_drag=true);
#endif // AP_SIM_ENABLED

    float terminal_velocity;
    float terminal_rotation_rate;
    uint8_t motor_offset;

    // calculate current and voltage
    void current_and_voltage(float &voltage, float &current);

    // get mass in kg
    float get_mass(void) const {
        return mass;
    }

    // set mass in kg
    void set_mass(float new_mass) {
        mass = new_mass;
    }
    
private:
    /*
      parameters that define the multicopter model. Can be loaded from
      a json file to give a custom model
     */
    const struct Model {
        // 模型质量 kg（整机重量）
        float mass = 5.5;

        // 模型对角尺寸 m（臂展/直径，用于惯量与力矩估算）
        float diagonal_size = 0.8;

        /*
          参考值：固定姿态测试得到的气动参考，用于估算阻力/电流缩放
         */
        float refSpd = 15.08; // 参考速度 m/s
        float refAngle = 45;  // 参考攻角 度
        float refVoltage = 22.20; // 参考电压 V
        float refCurrent = 40.0; // 参考电流 A
        float refAlt = 593; // 参考海拔 AMSL
        float refTempC = 25; // 参考温度 摄氏度
        float refBatRes = 0.01; // 参考电池内阻 欧姆

        // 单节满电电压（例：3 节电池则 4.2*3）
        float maxVoltage = 4.2*6;

        // 电池容量 Ah（0 表示无限容量仿真）
        float battCapacityAh = 0.0;

        // 悬停时的推力输出比（CTUN.ThO）在 refAlt 条件下
        float hoverThrOut = 0.35;

        // 螺旋桨推力曲线指数 MOT_THST_EXPO（0~1，越大低油门越柔和）
        float propExpo = 0.65;

        // 偏航响应缩放系数，度/秒（用于 yaw 力矩标定）
        float refRotRate = 120;

        // 以下为电机/PWM 相关参数（参考测试得出）
        float pwmMin = 1000;   // MOT_PWM_MIN 最小 PWM
        float pwmMax = 2000;   // MOT_PWM_MAX 最大 PWM
        float spin_min = 0.15; // MOT_SPIN_MIN 电机最小转速比例
        float spin_max = 0.95; // MOT_SPIN_MAX 电机最大转速比例

        // 电机最大变化率（slew，单位 PWM 每秒）
        float slew_max = 150;

        // 单个转子盘面积 m^2（同轴算一个盘），用于动量模型
        float disc_area = 0.130;

        // 动量阻力系数（越大横向阻尼越强）
        float mdrag_coef = 0.2;

        // 三轴惯量，若为零则按 mass/diagonal_size 自动估算
        Vector3f moment_of_inertia;

        // 每个电机在机体坐标系的位置（米）
        Vector3f motor_pos[SIM_FRAME_MAX_ACTUATORS];
        // 每个电机推力方向的单位向量（机体系）
        Vector3f motor_thrust_vec[SIM_FRAME_MAX_ACTUATORS];
        // 每个电机的偏航力矩符号系数
        float yaw_factor[SIM_FRAME_MAX_ACTUATORS] {0,};

        // 电机数量
        float num_motors = 4;

    } default_model;

protected:
    // load frame parameters from a json model file
    void load_frame_params(const char *model_json);

    // get air density in kg/m^3
    float get_air_density(float alt_amsl) const;

    struct Model model;

private:
    // exposed area times coefficient of drag
    float areaCd;
    float mass;
    float last_param_voltage;
#if AP_SIM_ENABLED
    Battery *battery;
#endif

    // json parsing helpers
    void parse_float(AP_JSON::value val, const char* label, float &param);
    void parse_vector3(AP_JSON::value val, const char* label, Vector3f &param);
};
}
