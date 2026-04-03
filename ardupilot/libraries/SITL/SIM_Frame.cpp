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
  multicopter frame simulator class
*/

#include "SIM_Frame.h"
#include <AP_Motors/AP_Motors.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Math/AP_Math.h>
#include "SIM_Aircraft.h"
#include <AP_HAL/AP_HAL.h>

#include "SIM_config.h"
#include "WuK_SITL_Globals.h"

#include <stdio.h>
#include <sys/stat.h>
#include <cmath>

using namespace SITL;
extern const AP_HAL::HAL& hal;

// motor definitions for supported frames

// 定义您的变形无人机，我们称之为 "my_transformer"
// Motor 构造参数顺序（单位/含义）：
// 1) servo：输出通道号（AP_MOTORS_MOT_* 常量）；
// 2) angle：机臂方位角，度，正前方为 0 度，顺时针为正；
// 3) yaw_factor：偏航方向系数（顺时针为正，逆时针为负）；
// 4) display_order：地面站显示顺序（1..n 顺时针）；
// 5) roll_servo：横滚倾转所用的舵机通道索引（-1 表示无）；
// 6) roll_min / roll_max：横滚倾转角范围（度，负值向左/顺时针）；
// 7) pitch_servo：俯仰倾转所用的舵机通道索引（-1 表示无）；
// 8) pitch_min / pitch_max：俯仰倾转角范围（度，正值向前俯仰）。
// [WuK-SITL] ========== WuK变构四旋翼物理模型 START ==========
// [WuK-SITL] 功能: 支持机臂0°→90°变构的四旋翼SITL仿真
// [WuK-SITL] 特性: 1) 伺服延迟仿真 2) 推力矢量旋转计算
static Motor wuk_motors[] =
{
    // [WuK-SITL] 标准X型布局 - 与飞控mapping一致
    //       前(0°)
    //    M2      M1     M1: 前右 45°  (右侧组, 逆时针)
    //      \    /      M2: 前左 -45° (左侧组, 逆时针)
    //       \  /       M3: 后左 -135°(左侧组, 顺时针)
    //        \/        M4: 后右 135° (右侧组, 顺时针)
    //        /
    //       /  \       变构: 左右电机以机体前后轴(Roll轴)向内旋转
    //      /    \      90°时: 所有电机水平向后推进
    //    M3      M4
    //       后(180°)
    
    // M1: 前右 (45°), 右侧组, 逆时针旋转, roll_servo=8控制, 向左收折(0->-90°)
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, 8, 0, -90, -1, 0, 0),
    // M2: 后左 (-135°), 左侧组, 逆时针旋转, roll_servo=8控制, 向右收折(0->+90°)
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3, 8, 0, 90, -1, 0, 0),
    // M3: 前左 (-45°), 左侧组, 顺时针旋转, roll_servo=8控制, 向右收折(0->+90°)
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4, 8, 0, 90, -1, 0, 0),
    // M4: 后右 (135°), 右侧组, 顺时针旋转, roll_servo=8控制, 向左收折(0->-90°)
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, 8, 0, -90, -1, 0, 0),
};

static Motor quad_plus_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
};

static Motor quad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order to match betaflight conventions
// See: https://fpvfrenzy.com/betaflight-motor-order/
static Motor quad_bf_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4),
};

// motor order to match betaflight conventions, reversed direction
static Motor quad_bf_x_rev_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
};

// motor order to match DJI conventions
// See: https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
static Motor quad_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

// motor order so that test order matches motor order ("clockwise X")
static Motor quad_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
};

#if AP_SIM_FRAME_COPTER_DOTRIACONTA_OCTAQUAD_X_ENABLED
static Motor dotriaconta_octaquad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  17),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   25),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    9),

    Motor(AP_MOTORS_MOT_5,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2),
    Motor(AP_MOTORS_MOT_6, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   18),
    Motor(AP_MOTORS_MOT_7,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  26),
    Motor(AP_MOTORS_MOT_8,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  10),

    Motor(AP_MOTORS_MOT_9,    45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_10, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 19),
    Motor(AP_MOTORS_MOT_11,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  27),
    Motor(AP_MOTORS_MOT_12,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  11),

    Motor(AP_MOTORS_MOT_13,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_14, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  20),
    Motor(AP_MOTORS_MOT_15,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 28),
    Motor(AP_MOTORS_MOT_16,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 12),

    Motor(AP_MOTORS_MOT_17,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_18, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 21),
    Motor(AP_MOTORS_MOT_19,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  29),
    Motor(AP_MOTORS_MOT_20,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  13),

    Motor(AP_MOTORS_MOT_21,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_22, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  22),
    Motor(AP_MOTORS_MOT_23,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 30),
    Motor(AP_MOTORS_MOT_24,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 14),

    Motor(AP_MOTORS_MOT_25,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_26, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 23),
    Motor(AP_MOTORS_MOT_27,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  31),
    Motor(AP_MOTORS_MOT_28,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  15),

    Motor(AP_MOTORS_MOT_29,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_30, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  24),
    Motor(AP_MOTORS_MOT_31,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 32),
    Motor(AP_MOTORS_MOT_32,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 16),
};
#endif  // AP_SIM_FRAME_COPTER_DOTRIACONTA_OCTAQUAD_X_ENABLED


static Motor tiltquad_h_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 7, 10, -90),
};

static Motor tiltquad[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
};

static Motor hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor hexax_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
};

static Motor hexa_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_3,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_6,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2)
};

static Motor hexa_cw_x_motors[] = 
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6,  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static Motor octa_dji_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8),
    Motor(AP_MOTORS_MOT_3,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_4, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_5, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_7,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_8,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2)
};

static Motor octa_cw_x_motors[] = 
{
    Motor(AP_MOTORS_MOT_1,   22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5, -157.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6, -112.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_7,  -67.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_8,  -22.5f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8)
};

static Motor octa_quad_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

static Motor octa_quad_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_3,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_6, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_7,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7),
    Motor(AP_MOTORS_MOT_8,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8)
};

static Motor dodeca_hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,   30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3),
    Motor(AP_MOTORS_MOT_4,   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4),
    Motor(AP_MOTORS_MOT_5,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7),
    Motor(AP_MOTORS_MOT_8, -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8),
    Motor(AP_MOTORS_MOT_9,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10),
    Motor(AP_MOTORS_MOT_11, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11),
    Motor(AP_MOTORS_MOT_12, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12)
};

static Motor hexadeca_octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1),
    Motor(AP_MOTORS_MOT_2,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2),
    Motor(AP_MOTORS_MOT_3,    45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,    45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5),
    Motor(AP_MOTORS_MOT_6,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6),
    Motor(AP_MOTORS_MOT_7,   135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,   135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   9),
    Motor(AP_MOTORS_MOT_10,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 10),
    Motor(AP_MOTORS_MOT_11, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 11),
    Motor(AP_MOTORS_MOT_12, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  12),
    Motor(AP_MOTORS_MOT_13,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  13),
    Motor(AP_MOTORS_MOT_14,  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 14),
    Motor(AP_MOTORS_MOT_15,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 15),
    Motor(AP_MOTORS_MOT_16,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  16)
};

static Motor hexadeca_octa_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1),
    Motor(AP_MOTORS_MOT_2,    22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2),
    Motor(AP_MOTORS_MOT_3,    67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,    67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,   112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5),
    Motor(AP_MOTORS_MOT_6,   112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6),
    Motor(AP_MOTORS_MOT_7,   157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,   157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,  -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   9),
    Motor(AP_MOTORS_MOT_10, -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 10),
    Motor(AP_MOTORS_MOT_11, -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 11),
    Motor(AP_MOTORS_MOT_12, -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  12),
    Motor(AP_MOTORS_MOT_13,  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  13),
    Motor(AP_MOTORS_MOT_14,  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 14),
    Motor(AP_MOTORS_MOT_15,  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 15),
    Motor(AP_MOTORS_MOT_16,  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  16)
};

static Motor deca_motors[] =
{
    Motor(AP_MOTORS_MOT_1,     0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,    36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,    72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,   108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,   144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,   180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7,  -144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,  -108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,   -72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10,  -36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10)
};

static Motor deca_cw_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    18, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1),
    Motor(AP_MOTORS_MOT_2,    54, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2),
    Motor(AP_MOTORS_MOT_3,    90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3),
    Motor(AP_MOTORS_MOT_4,   126, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4),
    Motor(AP_MOTORS_MOT_5,   162, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5),
    Motor(AP_MOTORS_MOT_6,  -162, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6),
    Motor(AP_MOTORS_MOT_7,  -126, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7),
    Motor(AP_MOTORS_MOT_8,   -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8),
    Motor(AP_MOTORS_MOT_9,   -54, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9),
    Motor(AP_MOTORS_MOT_10,  -18, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  10)
};

static Motor tri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, AP_MOTORS_MOT_8, 0, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, AP_MOTORS_MOT_7, 60, -60, -1, 0, 0),
};

static Motor tilttri_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2,  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2)
};

static Motor y6_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_6, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3)
};

/*
  FireflyY6 is a Y6 with front motors tiltable using servo on channel 9 (output 8)
 */
static Motor firefly_motors[] =
{
    Motor(AP_MOTORS_MOT_1, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_2,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_3, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_5,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2, -1, 0, 0, 6, 0, -90),
    Motor(AP_MOTORS_MOT_6, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6, -1, 0, 0, 6, 0, -90)
};

/*
  table of supported frame types. String order is important for
  partial name matching
 */
static Frame supported_frames[] =
{
    Frame("wuk", 4, wuk_motors),
    Frame("+",         4, wuk_motors),
    Frame("quad",      4, wuk_motors),
    Frame("copter",    4, wuk_motors),
    Frame("x",         4, wuk_motors),
    Frame("bfxrev",    4, wuk_motors),
    Frame("bfx",       4, wuk_motors),
#if AP_SIM_FRAME_COPTER_DOTRIACONTA_OCTAQUAD_X_ENABLED
    Frame("dotriaconta", 32, dotriaconta_octaquad_x_motors),
#endif  // AP_SIM_FRAME_COPTER_DOTRIACONTA_OCTAQUAD_X_ENABLED
    Frame("djix",      4, quad_dji_x_motors),
    Frame("cwx",       4, quad_cw_x_motors),
    Frame("tilthvec",  4, tiltquad_h_vectored_motors),
    Frame("hexadeca-octa", 16, hexadeca_octa_motors),
    Frame("hexadeca-octa-cwx", 16, hexadeca_octa_cw_x_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("hexa-cwx",  6, hexa_cw_x_motors),
    Frame("hexa-dji",  6, hexa_dji_x_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("octa-cwx",  8, octa_cw_x_motors),
    Frame("octa-dji",  8, octa_dji_x_motors),
    Frame("octa-quad-cwx",8, octa_quad_cw_x_motors),
    Frame("octa-quad", 8, octa_quad_motors),
    Frame("octa",      8, octa_motors),
    Frame("deca",     10, deca_motors),
    Frame("deca-cwx", 10, deca_cw_x_motors),
    Frame("dodeca-hexa", 12, dodeca_hexa_motors),
    Frame("tri",       3, tri_motors),
    Frame("tilttrivec",3, tilttri_vectored_motors),
    Frame("tilttri",   3, tilttri_motors),
    Frame("y6",        6, y6_motors),
    Frame("firefly",   6, firefly_motors),
    Frame("tilt",      4, tiltquad),
};

// get air density in kg/m^3
float Frame::get_air_density(float alt_amsl) const
{
    return AP_Baro::get_air_density_for_alt_amsl(alt_amsl);
}

/*
  load frame specific parameters from a json file if available
 */
void Frame::load_frame_params(const char *model_json)
{
    char *fname = nullptr;
    struct stat st;
    if (AP::FS().stat(model_json, &st) == 0) {
        fname = strdup(model_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", model_json));
        if (AP::FS().stat(model_json, &st) != 0) {
            AP_HAL::panic("%s failed to load", model_json);
        }
    }
    if (fname == nullptr) {
        AP_HAL::panic("%s failed to load", model_json);
    }
    AP_JSON::value *obj = AP_JSON::load_json(model_json);
    if (obj == nullptr) {
        AP_HAL::panic("%s failed to load", model_json);
    }

    enum class VarType {
        FLOAT,
        VECTOR3F,
    };

    struct json_search {
        const char *label;
        void *ptr;
        VarType t;
    };
    
    json_search vars[] = {
#define FRAME_VAR(s) { #s, &model.s, VarType::FLOAT }
        FRAME_VAR(mass),
        FRAME_VAR(diagonal_size),
        FRAME_VAR(refSpd),
        FRAME_VAR(refAngle),
        FRAME_VAR(refVoltage),
        FRAME_VAR(refCurrent),
        FRAME_VAR(refAlt),
        FRAME_VAR(refTempC),
        FRAME_VAR(maxVoltage),
        FRAME_VAR(battCapacityAh),
        FRAME_VAR(refBatRes),
        FRAME_VAR(propExpo),
        FRAME_VAR(refRotRate),
        FRAME_VAR(hoverThrOut),
        FRAME_VAR(pwmMin),
        FRAME_VAR(pwmMax),
        FRAME_VAR(spin_min),
        FRAME_VAR(spin_max),
        FRAME_VAR(slew_max),
        FRAME_VAR(disc_area),
        FRAME_VAR(mdrag_coef),
        {"moment_inertia", &model.moment_of_inertia, VarType::VECTOR3F},
        FRAME_VAR(num_motors),
    };

    for (uint8_t i=0; i<ARRAY_SIZE(vars); i++) {
        auto v = obj->get(vars[i].label);
        if (v.is<AP_JSON::null>()) {
            // use default value
            continue;
        }
        if (vars[i].t == VarType::FLOAT) {
            parse_float(v, vars[i].label, *((float *)vars[i].ptr));

        } else if (vars[i].t == VarType::VECTOR3F) {
            parse_vector3(v, vars[i].label, *(Vector3f *)vars[i].ptr);

        }
    }

    json_search per_motor_vars[] = {
        {"position", &model.motor_pos, VarType::VECTOR3F},
        {"vector", &model.motor_thrust_vec, VarType::VECTOR3F},
        {"yaw", &model.yaw_factor, VarType::FLOAT},
    };
    char label_name[20];
    for (uint8_t i=0; i<ARRAY_SIZE(per_motor_vars); i++) {
        for (uint8_t j=0; j<SIM_FRAME_MAX_ACTUATORS; j++) {
            snprintf(label_name, 20, "motor%i_%s", j+1, per_motor_vars[i].label);
            auto v = obj->get(label_name);
            if (v.is<AP_JSON::null>()) {
                // use default value
                continue;
            }
            if (per_motor_vars[i].t == VarType::FLOAT) {
                parse_float(v, label_name, *(((float *)per_motor_vars[i].ptr) + j));

            } else if (per_motor_vars[i].t == VarType::VECTOR3F) {
                parse_vector3(v, label_name, *(((Vector3f *)per_motor_vars[i].ptr) + j));
            }
        }
    }

    delete obj;

    ::printf("Loaded model params from %s\n", model_json);
}

void Frame::parse_float(AP_JSON::value val, const char* label, float &param) {
    if (!val.is<double>()) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    param = val.get<double>();
}

void Frame::parse_vector3(AP_JSON::value val, const char* label, Vector3f &param) {
    if (!val.is<AP_JSON::value::array>() || !val.contains(2) || val.contains(3)) {
        AP_HAL::panic("Bad json type for %s: %s", label, val.to_str().c_str());
    }
    for (uint8_t j=0; j<3; j++) {
        parse_float(val.get(j), label, param[j]);
    }
}

#if AP_SIM_ENABLED

/*
  initialise the frame
 */
void Frame::init(const char *frame_str, Battery *_battery)
{
    model = default_model;
    battery = _battery;

    const char *colon = strchr(frame_str, ':');
    size_t slen = strlen(frame_str);
    if (colon != nullptr && slen > 5 && strcmp(&frame_str[slen-5], ".json") == 0) {
        load_frame_params(colon+1);
    }
    mass = model.mass;

    const float drag_force = model.mass * GRAVITY_MSS * tanf(radians(model.refAngle));

    const float cos_tilt = cosf(radians(model.refAngle));
    const float airspeed_bf = model.refSpd * cos_tilt;
    const float ref_thrust = model.mass * GRAVITY_MSS / cos_tilt;
    float ref_air_density = get_air_density(model.refAlt);

    const float momentum_drag = cos_tilt * model.mdrag_coef * airspeed_bf * sqrtf(ref_thrust * ref_air_density * model.disc_area);

    if (momentum_drag > drag_force) {
        model.mdrag_coef *= drag_force / momentum_drag;
        areaCd = 0.0;
        ::printf("Suggested EK3_DRAG_BCOEF_* = 0, EK3_DRAG_MCOEF = %.3f\n", (momentum_drag / (model.mass * airspeed_bf)) * sqrtf(1.225f / ref_air_density));
    } else {
        areaCd = (drag_force - momentum_drag) / (0.5f * ref_air_density * sq(model.refSpd));
        ::printf("Suggested EK3_DRAG_BCOEF_* = %.3f, EK3_DRAG_MCOEF = %.3f\n", model.mass / areaCd, (momentum_drag / (model.mass * airspeed_bf)) * sqrtf(1.225f / ref_air_density));
    }

    terminal_rotation_rate = model.refRotRate;

    float hover_thrust = mass * GRAVITY_MSS;
    float hover_power = model.refCurrent * model.refVoltage;
    float hover_velocity_out = 2 * hover_power / hover_thrust;
    float effective_disc_area = hover_thrust / (0.5 * ref_air_density * sq(hover_velocity_out));
    float velocity_max = hover_velocity_out / sqrtf(model.hoverThrOut);
    float effective_prop_area = effective_disc_area / num_motors;
    float true_prop_area = model.disc_area / num_motors;

    // power_factor is ratio of power consumed per newton of thrust
    float power_factor = hover_power / hover_thrust;

    battery->setup(model.battCapacityAh, model.refBatRes, model.maxVoltage);

    if (uint8_t(model.num_motors) != num_motors) {
        ::printf("Warning model expected %u motors and got %u\n", uint8_t(model.num_motors), num_motors);
    }

    for (uint8_t i=0; i<num_motors; i++) {
        motors[i].setup_params(model.pwmMin, model.pwmMax, model.spin_min, model.spin_max, model.propExpo, model.slew_max,
                               model.diagonal_size, power_factor, model.maxVoltage, effective_prop_area, velocity_max,
                               model.motor_pos[i], model.motor_thrust_vec[i], model.yaw_factor[i], true_prop_area,
                               model.mdrag_coef);
    }

    if (is_zero(model.moment_of_inertia.x) || is_zero(model.moment_of_inertia.y) || is_zero(model.moment_of_inertia.z)) {
        // if no inertia provided, assume 50% of mass on ring around center
        model.moment_of_inertia.x = model.mass * 0.25 * sq(model.diagonal_size*0.5);
        model.moment_of_inertia.y = model.moment_of_inertia.x;
        model.moment_of_inertia.z = model.mass * 0.5 * sq(model.diagonal_size*0.5);
    }

    // setup reasonable defaults for battery
    AP_Param::set_default_by_name("SIM_BATT_VOLTAGE", model.maxVoltage);
    AP_Param::set_default_by_name("SIM_BATT_CAP_AH", model.battCapacityAh);
    if (model.battCapacityAh > 0) {
        AP_Param::set_default_by_name("BATT_CAPACITY", model.battCapacityAh*1000);
    }
}

/*
  find a frame by name
 */
Frame *Frame::find_frame(const char *name)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        // do partial name matching to allow for frame variants
        if (strncasecmp(name, supported_frames[i].name, strlen(supported_frames[i].name)) == 0) {
            return &supported_frames[i];
        }
    }
    return nullptr;
}

// calculate rotational and linear accelerations
void Frame::calculate_forces(const Aircraft &aircraft,
                             const struct sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel,
                             float* rpm,
                             bool use_drag)
{
    // 函数说明（中文注释）:
    // 1) 目的：计算机体在当前仿真帧下的角加速度 `rot_accel` 和线加速度 `body_accel`，
    //    同时根据电机模型累加推力与扭矩并可写出模拟的 `rpm` 值。
    // 2) 输入：
    //    - `aircraft`：包含位置、速度、姿态、陀螺仪等飞行器状态的结构。
    //    - `input`：SITL 的遥控/伺服输入（包含 `servos[]` 等通道）。
    //    - `use_drag`：是否启用机身线性阻力计算（通常为 true）。
    // 3) 输出：
    //    - `rot_accel`：绕机体三轴的角加速度（rad/s^2）。
    //    - `body_accel`：机体参考系下的线性加速度（m/s^2）。
    //    - `rpm`：可选，写入每个电机的模拟转速供外部使用/记录。
    // 4) 主要步骤概览：
    //    a) 读取空气密度、惯性矩阵、姿态、速度等状态变量；
    //    b) 基于 RC 通道（本实现使用 channel 9）计算并平滑舵机目标 PWM，用于驱动电机 tilt；
    //    c) 对每个电机调用 `Motor::calculate_forces`，累加单个电机产生的推力与扭矩；
    //    d) 计算总扭矩除以惯量得到角加速度，并加上旋转阻尼项；
    //    e) 计算并减去机身空气阻力（若启用），最终得到线加速度。

    // 注意：此函数内保留大量调试/日志/JSON 输出，用于观测变形和推力向量；这些不是力学关键路径，可按需关闭。

    Vector3f thrust; // newtons
    Vector3f torque;

    // 获取空气密度（kg/m^3）用于推力/阻力计算：
    // `aircraft.get_location().alt` 的单位为厘米，因此乘以 0.01 得到米并传入估算函数。
    const float air_density = get_air_density(aircraft.get_location().alt*0.01);
    // 读取陀螺仪测量（rad/s），用于旋转阻尼计算等
    const Vector3f gyro = aircraft.get_gyro();

    // 姿态矩阵（方向余弦矩阵）及在机体参考系中的风速：
    // - dcm.transposed() 将惯性系（earth-fixed）速度投影到机体基座（body-fixed）
    const Matrix3f &dcm = aircraft.get_dcm();
    Vector3f vel_air_bf = dcm.transposed() * aircraft.get_velocity_air_ef();
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    dcm.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const float roll_deg = degrees(roll_rad);
    const float pitch_deg = degrees(pitch_rad);
    const float yaw_deg = degrees(yaw_rad);
        /*
         * Frame-level manual override of motors[].thrust_vector removed.
         * Motor::calculate_forces already implements tilt driven by
         * roll_servo/pitch_servo and reads servos using the motor_offset.
         * Removing this direct write avoids double-rotation, servo index
         * mismatches and potential NaN propagation when a zero vector is used
         * in vector projection. If frame-level control is required in future,
         * implement a controlled API on Motor (e.g. Motor::set_external_tilt()).
         */
        // 简单的调试计数器，用于节流打印/日志（避免每帧都输出）
        static int debug_counter = 0;
        //---变形相关--- 开始
        // 以下为变形/倾转控制（tilt/morph）使用的持久化变量与日志变量。
        // 这些变量用于：
        //  1) 将 RC9 的通道输入映射为电机倾转舵机的目标 PWM；
        //  2) 对目标 PWM 做帧级别的平滑（防止瞬时突变）；
        //  3) 下发平滑后的 PWM 到 Motor 实例，驱动 Motor 内部的 thrust_vector 旋转（即推力方向变形）。
        // 另外包括一些仅用于调试/事件检测的文件日志/JSON 输出变量。

        // external_target_pwm_f: 每个电机的目标 PWM（浮点），由 RC9 按映射规则计算得到
        // external_current_pwm_f: 每个电机的当前（平滑后）PWM，逐帧趋近于 target
        static float external_target_pwm_f[32] = {0.0f};
        static float external_current_pwm_f[32] = {0.0f};
        static bool external_inited = false;

        // logging file pointer (opened once per run, overwrite existing file)
        // 文本日志：wuk_gb.log，用于人工查看；JSON 行日志：wuk_gb.jsonl，用于程序化分析
        static FILE *wuk_log = nullptr;
        static FILE *wuk_json = nullptr;

        // event detection state（仅日志/事件判定使用）
        static uint64_t wuk_start_time_us = 0;
        static bool wuk_takeoff = false;
        static bool wuk_hover_start = false;
        static bool wuk_morph_start = false;
        static bool wuk_controlled_descent = false;
        static float wuk_last_alt_m = 0.0f;
        static bool wuk_prev_morph_active = false; // 用于检测FC变形模式的状态变化

        // tilt speed parameter (deg/sec), default 20 deg/sec; can adjust here
        // 倾转速率（度/秒），用于将角速度限制转换为 PWM 变化速率
        static float tilt_speed_deg_per_sec = 20.0f;

        // mapping: servo pwm 1000..2000 corresponds to approx 90 degrees for these motors
        // PWM->角度 映射比例（PWM/度），假设 1000 PWM 对应 90° 变化
        const float pwm_per_deg = 1000.0f / 90.0f;
        //---变形相关--- 变量定义部分 结束
    const auto *_sitl = AP::sitl();
    // store per-motor thrust to print later if debugging
    Vector3f per_motor_thrust[32];
    Vector3f per_motor_thrust_vec[32];
    uint64_t now_us = AP_HAL::micros64();
    static uint64_t last_time_us = 0;
    float dt = 0.0f;
    if (last_time_us == 0) {
        dt = 0.0f;
    } else {
        dt = (now_us - last_time_us) * 1.0e-6f;
    }
    last_time_us = now_us;

    if (!external_inited) {
        for (uint8_t i=0;i<32;i++) { external_target_pwm_f[i] = 1000.0f; external_current_pwm_f[i] = 1000.0f; }
        external_inited = true;
    }

    // ensure log file open (truncate at first open)
    if (wuk_log == nullptr) {
        const char *logpath = "/home/qwssx2/ardupilot/wuk_gb.log";
        wuk_log = fopen(logpath, "w");
        if (wuk_log == nullptr) {
            fprintf(stderr, "WUK_ERROR: failed to open %s for logging\n", logpath);
        }
    }
    // ensure json file open (truncate at first open)
    if (wuk_json == nullptr) {
        const char *jsonpath = "/home/qwssx2/ardupilot/wuk_gb.jsonl";
        wuk_json = fopen(jsonpath, "w");
        if (wuk_json == nullptr) {
            fprintf(stderr, "WUK_ERROR: failed to open %s for json logging\n", jsonpath);
        }
    }
    // ========== WuK SITL: Read FC morph angle command ==========
    // 原实现：从RC9读取并映射到PWM
    // 新实现：从FC全局变量读取目标角度(0-90度),然后映射到PWM(1000-2000)
    // 这样SITL变形逻辑由FC的ModeMorph驱动,而不是直接响应RC9
    // Global variables declared in WuK_SITL_Globals.h
    
    // 将FC角度(0-90°)映射到PWM(1000-2000)
    // angle: 0° -> PWM: 1000, angle: 90° -> PWM: 2000
    int target_pwm_global;
    float target_angle = g_wuk_target_angle;  // [WuK-SITL] 读取飞控设置的目标变构角度
    
    // 限制角度范围
    if (target_angle < 0.0f) target_angle = 0.0f;
    if (target_angle > 90.0f) target_angle = 90.0f;
    
    // 角度到PWM的线性映射: PWM = 1000 + (angle/90) * 1000
    target_pwm_global = 1000 + (int)(target_angle * (1000.0f / 90.0f) + 0.5f);
    
    // 调试输出(每100ms输出一次,避免刷屏)
    static uint64_t last_print_us = 0;
    if (now_us - last_print_us > 100000) { // 100ms
        if (g_wuk_morph_active) {
            printf("SITL WuK: FC angle=%.1f° -> PWM=%d (morph active)\n", 
                   target_angle, target_pwm_global);
        }
        last_print_us = now_us;
    }

    //---变形相关--- 计算本帧允许的 PWM 变化量（平滑速率限制）
    // 说明：
    //  - `pwm_per_deg` 单位为 PWM/度（例如 1000/90）
    //  - `tilt_speed_deg_per_sec` 单位为 度/秒（deg/s）
    //  - 因此 `pwm_change_per_sec` = pwm_per_deg * tilt_speed_deg_per_sec，单位为 PWM/秒
    //  - 将其乘以本帧时长 `dt`（秒）得到本帧允许的最大 PWM 变化量 `max_pwm_change`（PWM）
    // 注意：首次帧 `dt == 0` 时 `max_pwm_change == 0`，会导致第一次帧外部 PWM 不发生骤变（安全行为）
    float pwm_change_per_sec = pwm_per_deg * tilt_speed_deg_per_sec;
    float max_pwm_change = pwm_change_per_sec * dt; // may be 0 for first frame
    //---变形相关--- 速率计算结束

    //---变形相关--- 平滑目标 PWM 并更新当前值（对每个电机）
    // 作用：把 `target_pwm_global` 设为每个电机的目标值，然后按 `max_pwm_change` 逐帧逼近。
    // 这样可以用度/秒的倾转速度限制来平滑 PWM，从而限制实际角速度变化。
    for (uint8_t i=0; i<num_motors; i++) {
        // 将全局目标下发到每个电机（当前实现为每个电机相同；也可以自定义为 per-motor）
        external_target_pwm_f[i] = (float)target_pwm_global;
        // 使用浮点累加器向目标逐步逼近，避免数值抖动
        if (max_pwm_change > 0.0f) {
            float diff = external_target_pwm_f[i] - external_current_pwm_f[i];
            if (fabsf(diff) <= max_pwm_change) {
                // 若目标差值小于本帧允许的变化量，则直接设为目标
                external_current_pwm_f[i] = external_target_pwm_f[i];
            } else if (diff > 0.0f) {
                // 增加方向：当前值增加最多 max_pwm_change
                external_current_pwm_f[i] = external_current_pwm_f[i] + max_pwm_change;
            } else {
                // 减少方向：当前值减少最多 max_pwm_change
                external_current_pwm_f[i] = external_current_pwm_f[i] - max_pwm_change;
            }
        }
    }
    //---变形相关--- 平滑结束

    //---变形相关--- 将平滑后的 PWM 下发到 Motor 并计算每个电机的推力/扭矩
    // 说明：Motor 内部会优先使用 external_roll/pitch_pwm（若设置 >=0）来计算舵机角度，
    // 并在其 `calculate_forces` 中旋转 `thrust_vector`，从而实现推力方向的变形。
    for (uint8_t i=0; i<num_motors; i++) {
        Vector3f mtorque, mthrust;
        // 下发 roll 轴覆盖 PWM（整数），pitch 覆盖置为 -1（表示不覆盖）
        motors[i].set_external_roll_pwm((int)roundf(external_current_pwm_f[i]));
        motors[i].set_external_pitch_pwm(-1);

        // 此处调用 Motor::calculate_forces：该函数会读取 external_*_pwm 并
        // 将其通过 update_servo -> 角度映射 -> 构造旋转矩阵，最终旋转 thrust_vector。
        // 因此在 Frame 层只需负责把平滑后的 PWM 下发即可，不需要在 Frame 内部手动旋转向量。
        motors[i].calculate_forces(input, motor_offset, mtorque, mthrust, vel_air_bf, gyro, air_density, battery->get_voltage(), use_drag);

        // 累加单电机的贡献
        torque += mtorque;
        thrust += mthrust;
        // store for debug print
        if (i < 32) {
            per_motor_thrust[i] = mthrust;
            per_motor_thrust_vec[i] = motors[i].current_thrust_vector;
        }
        // simulate motor rpm
        if (!is_zero(_sitl->vibe_motor)) {
            rpm[motor_offset+i] = motors[i].get_command() * AP::sitl()->vibe_motor * 60.0f;
        }
    }
    //---变形相关--- 应用到电机并计算推力段 结束

    // debug output: print servo inputs and per-motor thrust vectors every 50 frames (console)
    if ((debug_counter++ % 50) == 0) {
        // print some relevant servo channels (common mapping: channels 9/10 used by this frame)
        int servo9 = input.servos[8];
        int servo10 = input.servos[9];
        fprintf(stderr, ">>> WUK_FRAME: servo9=%d servo10=%d num_motors=%u\n", servo9, servo10, (unsigned)num_motors);
        for (uint8_t i=0; i<num_motors; i++) {
            Vector3f &mt = per_motor_thrust[i];
            Vector3f &mt_vec = per_motor_thrust_vec[i];
            float mag = sqrtf(sq(mt.x) + sq(mt.y) + sq(mt.z));
            int rs = motors[i].roll_servo;
            int ps = motors[i].pitch_servo;
            int pwm_rs = (rs >= 0) ? input.servos[rs] : -1;
            int pwm_ps = (ps >= 0) ? input.servos[ps] : -1;
            int applied_pwm = (int)roundf(external_current_pwm_f[i]);
            float motor_cmd = motors[i].get_command();
            float motor_cur = motors[i].get_current();
            fprintf(stderr, "  motor%u: roll_servo=%d pwm=%d pitch_servo=%d pwm=%d applied_pwm=%d cmd=%.3f cur=%.3f thrust_N=%.3f vec=(%.3f,%.3f,%.3f)\n",
                (unsigned)(i+1), rs, pwm_rs, ps, pwm_ps, applied_pwm, motor_cmd, motor_cur, mag, mt_vec.x, mt_vec.y, mt_vec.z);
        }
        fflush(stderr);
    }

    // file log: lower frequency (every 200 frames) and writes to wuk_gb.log (overwrite per run)
    if (wuk_log != nullptr && (debug_counter % 200) == 0) {
        fprintf(wuk_log, "WUK_LOG: time_us=%llu angle=%.1f target_pwm=%d dt=%.6f roll_deg=%.3f pitch_deg=%.3f yaw_deg=%.3f\n",
                (unsigned long long)now_us, target_angle, target_pwm_global, dt, roll_deg, pitch_deg, yaw_deg);
        float voltage = battery->get_voltage();
        for (uint8_t i=0; i<num_motors; i++) {
            Vector3f &mt = per_motor_thrust[i];
            Vector3f &mt_vec = per_motor_thrust_vec[i];
            float mag = sqrtf(sq(mt.x) + sq(mt.y) + sq(mt.z));
            int applied_pwm = (int)roundf(external_current_pwm_f[i]);
            float motor_cmd = motors[i].get_command();
            float motor_cur = motors[i].get_current();
            fprintf(wuk_log, " motor%u applied_pwm=%d cmd=%.3f cur=%.3f volt=%.3f thrust_N=%.3f vec=(%.3f,%.3f,%.3f)\n",
                    (unsigned)(i+1), applied_pwm, motor_cmd, motor_cur, voltage, mag, mt_vec.x, mt_vec.y, mt_vec.z);
        }
        fflush(wuk_log);
    }

    // JSONL write: same frequency as text log (keeps IO comparable)
    if (wuk_json != nullptr && (debug_counter % 200) == 0) {
        // initialize start time
        if (wuk_start_time_us == 0) wuk_start_time_us = now_us;
        float elapsed_s = (now_us - wuk_start_time_us) * 1e-6f;
        // compute altitude in meters (existing convention: alt is in cm)
        float altitude_m = (float)aircraft.get_location().alt * 0.01f;
        // compute total and vertical thrust
        double total_thrust = 0.0;
        double vertical_thrust = 0.0;
        for (uint8_t i=0; i<num_motors; i++) {
            Vector3f &mt = per_motor_thrust[i];
            total_thrust += sqrtf(sq(mt.x) + sq(mt.y) + sq(mt.z));
            vertical_thrust += -mt.z; // sim uses negative z for up
        }
        // detect events: takeoff when total thrust becomes >0, morph_start when FC enters MORPH mode, hover_start when altitude>=5 and vertical speed small
        // takeoff
        if (!wuk_takeoff && total_thrust > 0.001) wuk_takeoff = true;
        // morph_start: detect when FC enters MORPH mode (rising edge)
        if (!wuk_morph_start) {
            if (!wuk_prev_morph_active && g_wuk_morph_active) wuk_morph_start = true;
        }
        // controlled_descent: detect when target_angle reaches 90 degrees
        if (!wuk_controlled_descent && target_angle >= 90.0f) wuk_controlled_descent = true;

        // hover_start: when altitude >= 5.0 m and vertical speed approx 0
        float vertical_speed = 0.0f;
        if (dt > 0.0f) vertical_speed = (altitude_m - wuk_last_alt_m) / dt;
        if (!wuk_hover_start && altitude_m >= 5.0f && fabsf(vertical_speed) < 0.2f) wuk_hover_start = true;

        // write JSON line
        // basic manual formatting without external JSON dependency
        fprintf(wuk_json, "{");
        fprintf(wuk_json, "\"time_us\":%llu,", (unsigned long long)now_us);
        fprintf(wuk_json, "\"elapsed_s\":%.6f,", elapsed_s);
        fprintf(wuk_json, "\"dt\":%.6f,", dt);
        fprintf(wuk_json, "\"fc_angle\":%.1f,", target_angle);
        fprintf(wuk_json, "\"fc_morph_active\":%s,", g_wuk_morph_active ? "true" : "false");
        fprintf(wuk_json, "\"target_pwm\":%d,", target_pwm_global);
        // per_motor arrays
        fprintf(wuk_json, "\"per_motor\":{");
        // applied_pwm
        fprintf(wuk_json, "\"applied_pwm\":[");
        for (uint8_t i=0;i<num_motors;i++) {
            int applied = (int)roundf(external_current_pwm_f[i]);
            fprintf(wuk_json, "%d%s", applied, (i+1==num_motors)?"":",");
        }
        fprintf(wuk_json, "],");
        // cmd
        fprintf(wuk_json, "\"cmd\":[");
        for (uint8_t i=0;i<num_motors;i++) {
            fprintf(wuk_json, "%.6f%s", motors[i].get_command(), (i+1==num_motors)?"":",");
        }
        fprintf(wuk_json, "],");
        // cur
        fprintf(wuk_json, "\"cur\":[");
        for (uint8_t i=0;i<num_motors;i++) {
            fprintf(wuk_json, "%.6f%s", motors[i].get_current(), (i+1==num_motors)?"":",");
        }
        fprintf(wuk_json, "],");
        // volt
        float voltage_now = battery->get_voltage();
        fprintf(wuk_json, "\"volt\":[");
        for (uint8_t i=0;i<num_motors;i++) {
            fprintf(wuk_json, "%.3f%s", voltage_now, (i+1==num_motors)?"":",");
        }
        fprintf(wuk_json, "],");
        // thrust_N
        fprintf(wuk_json, "\"thrust_N\":[");
        for (uint8_t i=0;i<num_motors;i++) {
            Vector3f &mt = per_motor_thrust[i];
            float mag = sqrtf(sq(mt.x) + sq(mt.y) + sq(mt.z));
            fprintf(wuk_json, "%.6f%s", mag, (i+1==num_motors)?"":",");
        }
        fprintf(wuk_json, "],");
        // vx,vy,vz arrays
        fprintf(wuk_json, "\"vx\":[");
        for (uint8_t i=0;i<num_motors;i++) { fprintf(wuk_json, "%.6f%s", per_motor_thrust_vec[i].x, (i+1==num_motors)?"":","); }
        fprintf(wuk_json, "],");
        fprintf(wuk_json, "\"vy\":[");
        for (uint8_t i=0;i<num_motors;i++) { fprintf(wuk_json, "%.6f%s", per_motor_thrust_vec[i].y, (i+1==num_motors)?"":","); }
        fprintf(wuk_json, "],");
        fprintf(wuk_json, "\"vz\":[");
        for (uint8_t i=0;i<num_motors;i++) { fprintf(wuk_json, "%.6f%s", per_motor_thrust_vec[i].z, (i+1==num_motors)?"":","); }
        fprintf(wuk_json, "],");

        // totals and altitude
        fprintf(wuk_json, "\"total_thrust_N\":%.6f,", total_thrust);
        fprintf(wuk_json, "\"vertical_thrust_N\":%.6f,", vertical_thrust);
        fprintf(wuk_json, "\"altitude_m\":%.6f", altitude_m);
        fprintf(wuk_json, "},");  // Close per_motor object with comma
        fprintf(wuk_json, "\"attitude_deg\":{\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":%.6f},", roll_deg, pitch_deg, yaw_deg);
        // events
        fprintf(wuk_json, "\"events\":{\"takeoff\":%s,\"hover_start\":%s,\"morph_start\":%s,\"controlled_descent\":%s}",
                wuk_takeoff?"true":"false", wuk_hover_start?"true":"false", wuk_morph_start?"true":"false", wuk_controlled_descent?"true":"false");
        fprintf(wuk_json, "}\n");
        fflush(wuk_json);

        // update state for next frame
        wuk_last_alt_m = altitude_m;
        wuk_prev_morph_active = g_wuk_morph_active;
    }

    // 计算总角加速度：
    // rot_accel = 总扭矩 / 惯性矩（model.moment_of_inertia）
    // 该计算假定惯量非零（init 中已为零值做估算）。
    rot_accel.x = torque.x / model.moment_of_inertia.x;
    rot_accel.y = torque.y / model.moment_of_inertia.y;
    rot_accel.z = torque.z / model.moment_of_inertia.z;

    if (terminal_rotation_rate > 0) {
        // rotational air resistance
        rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;
    }

    if (use_drag) {
        // use the model params to calculate drag
        Vector3f drag_bf;
        // 线性阻力计算（机体参考系）：使用 areaCd（已在 init 中计算）与局部空气速度，
        // 并保持阻力方向与速度方向相反（通过判断速度符号处理）。
        drag_bf.x = areaCd * 0.5f * air_density * sq(vel_air_bf.x);
        if (is_negative(vel_air_bf.x)) {
            drag_bf.x = -drag_bf.x;
        }

        drag_bf.y = areaCd * 0.5f * air_density * sq(vel_air_bf.y);
        if (is_negative(vel_air_bf.y)) {
            drag_bf.y = -drag_bf.y;
        }

        drag_bf.z = areaCd * 0.5f * air_density * sq(vel_air_bf.z);
        if (is_negative(vel_air_bf.z)) {
            drag_bf.z = -drag_bf.z;
        }

        // 从总推力中减去机身阻力（使线加速度更真实），注意 thrust 的 z 分量在该代码中向下为正
        thrust -= drag_bf;
    }

    body_accel = thrust/aircraft.gross_mass();
}


// calculate current and voltage
void Frame::current_and_voltage(float &voltage, float &current)
{
    float param_voltage = AP::sitl()->batt_voltage;
    if (!is_equal(last_param_voltage,param_voltage)) {
        battery->init_voltage(param_voltage);
        last_param_voltage = param_voltage;
    }
    voltage = battery->get_voltage();
    current = 0;
    for (uint8_t i=0; i<num_motors; i++) {
        current += motors[i].get_current();
    }
}
#endif // AP_SIM_ENABLED
