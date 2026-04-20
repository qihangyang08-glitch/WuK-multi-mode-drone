#include <Arduino.h>
#include <Servo.h>

// ============================================================================
// WuK 项目 - Arduino 从机固件
// 角色：通过 UART 接收来自 ArduPilot 飞控的指令并驱动执行器
// 平台：Arduino Mega 2560
// 通信模式：单工 (Simplex)，仅接收飞控指令，不回传 (避免 5V/3.3V 电平冲突)
// ============================================================================

// --- 硬件配置 ---
// 引脚分配基于旧版 'HardwareConfig'
// 根据需求移除了起落架（引脚 12）。

const uint8_t PIN_ARM_SERVO_0 = 11;    // 基础舵机（电机臂servo0（左））舵机板4号
const uint8_t PIN_ARM_SERVO_1 = 13;    // 基础舵机（电机方向servo1）

const uint8_t PIN_MOTOR_SERVO_0 = 7;   // 电机舵机1-左上-1
const uint8_t PIN_MOTOR_SERVO_1 = 6;   // 电机舵机2-左下-2
const uint8_t PIN_MOTOR_SERVO_2 = 5;   // 电机舵机3-右上-3
const uint8_t PIN_MOTOR_SERVO_3 = 4;   // 电机舵机4-右下-4

const uint8_t PIN_ACTUATOR_0 = 8;      // 电推杆1
const uint8_t PIN_ACTUATOR_1 = 9;      // 电推杆2

// --- 舵机角度映射参数 ---
const int ARM_LOGIC_MIN_ANGLE = 0;
const int ARM_LOGIC_MAX_ANGLE = 90;
const int MOTOR_LOGIC_MIN_ANGLE = 0;
const int MOTOR_LOGIC_MAX_ANGLE = 90;

const int ARM_SERVO_0_FLIGHT_DEG = 0;   // 逻辑0° -> 实际0°
const int ARM_SERVO_0_GROUND_DEG = 90;  // 逻辑90° -> 实际90°
const int ARM_SERVO_1_FLIGHT_DEG = 115;  // 逻辑0° -> 实际115°
const int ARM_SERVO_1_GROUND_DEG = 15;   // 逻辑90° -> 实际15°

const int MOTOR_SERVO_0_FLIGHT_DEG = 83;
const int MOTOR_SERVO_0_GROUND_DEG = 173;
const int MOTOR_SERVO_1_FLIGHT_DEG = 82;
const int MOTOR_SERVO_1_GROUND_DEG = 172;

const int MOTOR_SERVO_2_FLIGHT_DEG = 169;
const int MOTOR_SERVO_2_GROUND_DEG = 79;
const int MOTOR_SERVO_3_FLIGHT_DEG = 175;//少了3度
const int MOTOR_SERVO_3_GROUND_DEG = 85;//过了2度

// 舵机对象
Servo armServos[2];
Servo motorServos[4];
Servo armActuators[2];

// --- 通信协议 ---
// 定义在 WuK_Comms.h 中
// 使用 Serial3 (RX: Pin 15, TX: Pin 14)
// 注意：我们只使用 RX (Pin 15) 接收飞控数据
#define FC_SERIAL Serial3      
#define DEBUG_SERIAL Serial    // USB 调试 UART
#define FC_BAUD 9600         // 必须与 ArduPilot 中的 WUK_BAUD 参数匹配
#define DEBUG_BAUD 9600

const uint8_t HEADER_BYTE = 0xAA;
const uint8_t MAX_PAYLOAD_LEN = 16;

// 指令 ID (飞控 -> Arduino)
enum CmdID : uint8_t {
    CMD_ARM_SERVO = 0x01,
    CMD_ACT1 = 0x02,
    CMD_ACT2 = 0x03,
    CMD_STATUS_REQ = 0x04
};

// 解析器状态机
enum RxState {
    WAIT_HEADER,
    WAIT_MSG_ID,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CRC
};

RxState rxState = WAIT_HEADER;
uint8_t rxMsgID;
uint8_t rxLen;
uint8_t rxPayload[MAX_PAYLOAD_LEN];
uint8_t rxPayloadIdx;
uint8_t rxCRC;

// --- 调试信息缓存 ---
struct DebugInfo {
    unsigned long lastValidTime = 0;   // 最后一次有效报文时间
    char lastCmdName[15] = "None";     // 最后一个非心跳命令名称
    uint16_t lastCmdVal = 0;           // 最后一个非心跳命令值
    bool crcError = false;             // 最近是否有CRC/长度错误
    uint32_t totalBytes = 0;           // 总接收字节数
    bool msgReceived = false;          //最近是否收到有效报文
} debugInfo;

// --- 状态机与控制变量 ---
enum SystemMode {
    MODE_MANUAL_SERVO,      // 由 CMD_ARM_SERVO 直接控制
    MODE_FLIGHT,            // 静态飞行状态 (0°)
    MODE_GROUND,            // 静态地面状态 (90°)
    TRANSITION_TO_GROUND,   // 正在切换到地面模式
    TRANSITION_TO_FLIGHT    // 正在切换到飞行模式
};

SystemMode currentMode = MODE_FLIGHT;
float currentArmAngle = 0.0;   // 逻辑角度 0.0 (Flight) - 90.0 (Ground)
float currentMotorAngle = 0.0; // 逻辑角度 0.0 (Flight) - 90.0 (Ground)
float manualTargetArmAngle = 0.0; // MANUAL_SERVO 目标角度（由飞控报文更新）
unsigned long lastArmCmdMs = 0;   // 最近收到 ARM_SERVO 指令时间
bool pendingGroundTransition = false; // MANUAL阶段收到ACT1高位时的延迟切换标记
float manualBaseSpeed = 4.0;      // 0-45° 基础预测速度
float manualTrackSpeed = 4.0;     // 当前闭环跟踪速度
unsigned long lastLoopTime = 0;

// 速度定义 (度/秒)
const float SPEED_NORMAL = 30.0; 
const float SPEED_SLOW = 10.0;   // 用于 90->45 的慢速阶段
const float MORPH_DIRECT_MAX_ANGLE = 45.0;
const float MANUAL_BASE_SPEED_INIT = 4.0;   // 首次进入0-45控制时基础速度
const float MANUAL_BASE_SPEED_MIN = 2.5;    // 超前时可下调的基础速度下限
const float MANUAL_BASE_SPEED_MAX = 5.0;    // 允许回升的基础速度上限
const float MANUAL_SPEED_MIN = 2.0;         // 闭环最小速度
const float MANUAL_SPEED_MAX = 8.0;         // 闭环最大速度
const float MANUAL_TIMEOUT_SPEED = 3.5;     // 丢包续行速度
const float MANUAL_KP = 0.35;               // 误差到速度的比例系数
const float MANUAL_ERR_DEADBAND = 0.5;      // 误差死区
const float MANUAL_BASE_DROP_STEP = 0.25;   // 超前时基础速度下调步长
const uint16_t ARM_CMD_TIMEOUT_MS = 220;     // 超过该时间认为更新中断
const uint16_t ARM_CMD_AUTORUN_MAX_MS = 1500; // 丢包后最大自动续行时间
const uint16_t SERIAL_BYTES_BUDGET_PER_LOOP = 96; // 单次循环最多处理字节，避免控制更新被串口读取饿死

// --- 函数原型 ---
void handle_arm_servo_direct(uint16_t target_angle);
void handle_mode_switch_sequence(uint16_t pwm_val);
void handle_pushrod_action(uint16_t pwm_val);
void send_debug_status();
void printLog(bool isEvent, const char* eventName, uint16_t eventVal); 
uint8_t crc8_update(uint8_t crc, uint8_t data);
void processSerial();
void updateTransitions();
void updateServos();

// ============================================================================
// 初始化与循环
// ============================================================================

void setup() {
    // 1. 初始化串口
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    FC_SERIAL.begin(FC_BAUD); // 初始化 Serial3 (Pin 15 RX)
    
    delay(1000); // 等待串口稳定
    
    DEBUG_SERIAL.println(F("======================================"));
    DEBUG_SERIAL.println(F("[WuK] Arduino 从机已初始化 (v2.0)"));
    DEBUG_SERIAL.println(F("======================================"));
    DEBUG_SERIAL.print(F("[Config] RX Pin: 15 (Serial3)"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print(F("[Config] Baud Rate: "));
    DEBUG_SERIAL.println(FC_BAUD);
    DEBUG_SERIAL.print(F("[Config] Protocol: Header=0x"));
    DEBUG_SERIAL.print(HEADER_BYTE, HEX);
    DEBUG_SERIAL.println(F(" CRC8-Maxim"));
    DEBUG_SERIAL.println(F("[Status] 等待飞控指令..."));
    DEBUG_SERIAL.println(F("======================================"));

    // 2. 初始化舵机
    armServos[0].attach(PIN_ARM_SERVO_0);
    armServos[1].attach(PIN_ARM_SERVO_1);
    
    motorServos[0].attach(PIN_MOTOR_SERVO_0);
    motorServos[1].attach(PIN_MOTOR_SERVO_1);
    motorServos[2].attach(PIN_MOTOR_SERVO_2);
    motorServos[3].attach(PIN_MOTOR_SERVO_3);
    
    armActuators[0].attach(PIN_ACTUATOR_0);
    armActuators[1].attach(PIN_ACTUATOR_1);

    // 3. 设置初始状态 (飞行模式)
    currentMode = MODE_FLIGHT;
    currentArmAngle = 0.0;
    currentMotorAngle = 0.0;
    manualTargetArmAngle = 0.0;
    pendingGroundTransition = false;
    manualBaseSpeed = MANUAL_BASE_SPEED_INIT;
    manualTrackSpeed = MANUAL_BASE_SPEED_INIT;
    lastArmCmdMs = millis();
    
    // 写入初始位置
    updateServos();
    
    armActuators[0].writeMicroseconds(1500);
    armActuators[1].writeMicroseconds(1500);
    
    lastLoopTime = millis();
}

void loop() {
    // 1. 处理传入的串口数据
    processSerial();

    // 2. 更新状态机与舵机位置
    updateTransitions();

    // 3. 定期调试输出 (每1000ms输出一次)
    static uint32_t lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        printLog(false, nullptr, 0); // 周期性输出状态
        lastDebugTime = millis();
    }
}

// ============================================================================
// 状态机逻辑
// ============================================================================

void updateTransitions() {
    unsigned long now = millis();
    float dt = (now - lastLoopTime) / 1000.0; // 秒
    lastLoopTime = now;
    
    if (dt > 0.1) dt = 0.1; // 防止循环卡顿导致的时间跳变

    bool changed = false;

    if (currentMode == TRANSITION_TO_GROUND) {
        // 动作1: 0° -> 90°
        // 顺序: 先转机臂，到位后转电机
        
        // 1. 机臂 0 -> 90
        if (currentArmAngle < 90.0) {
            currentArmAngle += SPEED_NORMAL * dt;
            if (currentArmAngle > 90.0) currentArmAngle = 90.0;
            changed = true;
        } 
        // 2. 电机 0 -> 90 (机臂到位后)
        else if (currentMotorAngle < 90.0) {
            currentMotorAngle += SPEED_NORMAL * dt;
            if (currentMotorAngle > 90.0) currentMotorAngle = 90.0;
            changed = true;
        }
        else {
            currentMode = MODE_GROUND;
        }
    }
    else if (currentMode == TRANSITION_TO_FLIGHT) {
        // 动作2: 90° -> 0°
        // 顺序: 先转机臂，到位后转电机
        
        // 1. 机臂 90 -> 0
        if (currentArmAngle > 0.0) {
            // 90°到45°区间速度较慢
            float speed = (currentArmAngle > 45.0) ? SPEED_SLOW : SPEED_NORMAL;
            currentArmAngle -= speed * dt;
            if (currentArmAngle < 0.0) currentArmAngle = 0.0;
            changed = true;
        }
        // 2. 电机 90 -> 0 (机臂到位后)
        else if (currentMotorAngle > 0.0) {
            currentMotorAngle -= SPEED_NORMAL * dt;
            if (currentMotorAngle < 0.0) currentMotorAngle = 0.0;
            changed = true;
        }
        else {
            currentMode = MODE_FLIGHT;
        }
    }
    else if (currentMode == MODE_MANUAL_SERVO) {
        // 飞控短时丢包时，按预设慢速续行，缩短主从进度空窗
        const unsigned long since_cmd_ms = now - lastArmCmdMs;
        if (manualTargetArmAngle <= MORPH_DIRECT_MAX_ANGLE) {
            const float diff = manualTargetArmAngle - currentArmAngle;

            if (since_cmd_ms <= ARM_CMD_TIMEOUT_MS) {
                // 0-45°正常报文期间：直接按报文指定角度控制
                if (diff > 0.01f || diff < -0.01f) {
                    currentArmAngle = manualTargetArmAngle;
                    changed = true;
                }
            } else {
                // 超时后进入续行/追赶逻辑，仅用于丢包容错
                if (since_cmd_ms < ARM_CMD_AUTORUN_MAX_MS && manualTargetArmAngle < MORPH_DIRECT_MAX_ANGLE) {
                    manualTargetArmAngle += MANUAL_TIMEOUT_SPEED * dt;
                    if (manualTargetArmAngle > MORPH_DIRECT_MAX_ANGLE) {
                        manualTargetArmAngle = MORPH_DIRECT_MAX_ANGLE;
                    }
                }

                const float track_speed = min(manualTrackSpeed, MANUAL_TIMEOUT_SPEED);
                const float max_step = track_speed * dt;
                const float timeout_diff = manualTargetArmAngle - currentArmAngle;

                if (timeout_diff > MANUAL_ERR_DEADBAND) {
                    currentArmAngle += min(timeout_diff, max_step);
                    changed = true;
                } else if (timeout_diff < -MANUAL_ERR_DEADBAND) {
                    currentArmAngle -= min(-timeout_diff, max_step);
                    changed = true;
                }
            }
        } else {
            // >45° 指令（例如90°）保持直接跟踪，不启用0-45预测逻辑
            const float max_step = SPEED_SLOW * dt;
            const float diff = manualTargetArmAngle - currentArmAngle;
            if (diff > 0.01f) {
                currentArmAngle += min(diff, max_step);
                changed = true;
            } else if (diff < -0.01f) {
                currentArmAngle -= min(-diff, max_step);
                changed = true;
            }
        }

        // MORPH阶段仅控制机臂，电机维持飞行位
        if (currentMotorAngle != 0.0f) {
            currentMotorAngle = 0.0f;
            changed = true;
        }

        // 若ACT1高位在MANUAL阶段被延迟，机臂到位后自动切换到地面过渡流程
        if (pendingGroundTransition && currentArmAngle >= 89.5f) {
            pendingGroundTransition = false;
            currentMode = TRANSITION_TO_GROUND;
        }
    }
    else if (currentMode == MODE_GROUND) {
        // 持续维持地面模式角度 (保险起见)
        currentArmAngle = 90.0;
        currentMotorAngle = 90.0;
        changed = true;
    }
    else if (currentMode == MODE_FLIGHT) {
        // 持续维持飞行模式角度 (保险起见)
        currentArmAngle = 0.0;
        currentMotorAngle = 0.0;
        changed = true;
    }
    
    if (changed) {
        updateServos();
    }
}

void updateServos() {
    // 映射逻辑角度 (0-90) 到实际舵机角度
    
    // 机臂舵机
    // Servo 0 (电机臂): 0°(Flight) -> 20°, 90°(Ground) -> 130°
    // Servo 1 (电机方向): 0°(Flight) -> 155°, 90°(Ground) -> 45°
    int s0 = map((long)currentArmAngle,
                 ARM_LOGIC_MIN_ANGLE,
                 ARM_LOGIC_MAX_ANGLE,
                 ARM_SERVO_0_FLIGHT_DEG,
                 ARM_SERVO_0_GROUND_DEG);
    int s1 = map((long)currentArmAngle,
                 ARM_LOGIC_MIN_ANGLE,
                 ARM_LOGIC_MAX_ANGLE,
                 ARM_SERVO_1_FLIGHT_DEG,
                 ARM_SERVO_1_GROUND_DEG);
    armServos[0].write(s0);
    armServos[1].write(s1);
    
    // 电机舵机（4路独立映射）
    int m0 = map((long)currentMotorAngle,
                 MOTOR_LOGIC_MIN_ANGLE,
                 MOTOR_LOGIC_MAX_ANGLE,
                 MOTOR_SERVO_0_FLIGHT_DEG,
                 MOTOR_SERVO_0_GROUND_DEG);
    int m1 = map((long)currentMotorAngle,
                 MOTOR_LOGIC_MIN_ANGLE,
                 MOTOR_LOGIC_MAX_ANGLE,
                 MOTOR_SERVO_1_FLIGHT_DEG,
                 MOTOR_SERVO_1_GROUND_DEG);
    int m2 = map((long)currentMotorAngle,
                 MOTOR_LOGIC_MIN_ANGLE,
                 MOTOR_LOGIC_MAX_ANGLE,
                 MOTOR_SERVO_2_FLIGHT_DEG,
                 MOTOR_SERVO_2_GROUND_DEG);
    int m3 = map((long)currentMotorAngle,
                 MOTOR_LOGIC_MIN_ANGLE,
                 MOTOR_LOGIC_MAX_ANGLE,
                 MOTOR_SERVO_3_FLIGHT_DEG,
                 MOTOR_SERVO_3_GROUND_DEG);

    motorServos[0].write(m0);
    motorServos[1].write(m1);
    motorServos[2].write(m2);
    motorServos[3].write(m3);
    
    // 舵机更新已通过 [STATUS] 输出显示，此处不再重复输出
}

// ============================================================================
// 通信解析器
// ============================================================================

void processSerial() {
    // 循环查询引脚是否有报文
    uint16_t bytesProcessed = 0;
    while (FC_SERIAL.available() > 0 && bytesProcessed < SERIAL_BYTES_BUDGET_PER_LOOP) {
        uint8_t b = FC_SERIAL.read();
        bytesProcessed++;
        
        // 仅在首次收到数据时提示，避免刷屏
        if (debugInfo.totalBytes == 0) {
             DEBUG_SERIAL.println(F("[系统] 检测到数据输入..."));
        }
        debugInfo.totalBytes++;

        switch (rxState) {
            case WAIT_HEADER:
                if (b == HEADER_BYTE) {
                    rxState = WAIT_MSG_ID;
                    rxCRC = 0; // Reset CRC
                    // DEBUG_SERIAL.println(F("[RX] Found Header"));
                } 
                // 注意：由于启用了 MAVLink2，串口会有大量非 0xAA 数据
                // 我们这只静默丢弃，等待我们的协议头
                break;

            case WAIT_MSG_ID:
                rxMsgID = b;
                rxCRC = crc8_update(rxCRC, b);
                rxState = WAIT_LEN;
                break;

            case WAIT_LEN:
                rxLen = b;
                if (rxLen > MAX_PAYLOAD_LEN) {
                    // 长度无效，重置
                    rxState = WAIT_HEADER;
                    debugInfo.crcError = true;
                } else {
                    rxCRC = crc8_update(rxCRC, b);
                    rxPayloadIdx = 0;
                    if (rxLen == 0) {
                        rxState = WAIT_CRC;
                    } else {
                        rxState = WAIT_PAYLOAD;
                    }
                }
                break;

            case WAIT_PAYLOAD:
                rxPayload[rxPayloadIdx++] = b;
                rxCRC = crc8_update(rxCRC, b);
                if (rxPayloadIdx == rxLen) {
                    rxState = WAIT_CRC;
                }
                break;

            case WAIT_CRC:
                if (b == rxCRC) {
                    // 收到有效数据包
                    uint16_t val = 0;
                    if (rxLen >= 2) {
                        val = rxPayload[0] | (rxPayload[1] << 8);
                    }
                    
                    // 记录最近的有效报文时间
                    debugInfo.lastValidTime = millis();
                    debugInfo.msgReceived = true;

                    const char* actionName = "UNKNOWN";
                    switch ((CmdID)rxMsgID) {
                        case CMD_ARM_SERVO:
                            actionName = "ARM_SERVO";
                            handle_arm_servo_direct(val);
                            break;
                        case CMD_ACT1:
                            actionName = "MODE_SWITCH";
                            handle_mode_switch_sequence(val);
                            break;
                        case CMD_ACT2:
                            actionName = "PUSHROD";
                            handle_pushrod_action(val);
                            break;
                        case CMD_STATUS_REQ:
                            actionName = "STATUS_REQ";
                            send_debug_status();
                            break;
                        default:
                            break;
                    }
                    
                    // 仅当不是心跳包时，更新最后指令记录并打印
                    if (rxMsgID != CMD_STATUS_REQ) {
                        strncpy(debugInfo.lastCmdName, actionName, sizeof(debugInfo.lastCmdName) - 1);
                        debugInfo.lastCmdVal = val;
                        printLog(true, actionName, val); // 即时打印事件日志
                    }

                } else {
                    debugInfo.crcError = true;
                    // 不需要立即打印错误，周期性状态会显示 [ERROR]
                }
                rxState = WAIT_HEADER;
                break;
        }
    }

}

// ============================================================================
// 指令处理程序
// ============================================================================

// 由 MsgID 0x01 触发
// 直接控制机臂角度 (用于 Morph 模式)
void handle_arm_servo_direct(uint16_t target_angle) {
    if (target_angle > 90) target_angle = 90;

    const float target = (float)target_angle;
    
    // 切换到手动模式，停止自动转换
    currentMode = MODE_MANUAL_SERVO;

    bool servoChanged = false;

    if (target <= MORPH_DIRECT_MAX_ANGLE) {
        // 0-45°：严格跟随报文角度，保证与飞控侧同步
        const float err = target - currentArmAngle;
        if (err > MANUAL_ERR_DEADBAND) {
            // 落后于报文目标：提高跟踪速度
            manualBaseSpeed = min(MANUAL_BASE_SPEED_MAX, manualBaseSpeed + 0.05f);
            manualTrackSpeed = constrain(manualBaseSpeed + MANUAL_KP * err, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
        } else if (err < -MANUAL_ERR_DEADBAND) {
            // 超前于报文目标：降低基础速度并减速
            manualBaseSpeed = max(MANUAL_BASE_SPEED_MIN, manualBaseSpeed - MANUAL_BASE_DROP_STEP);
            manualTrackSpeed = constrain(manualBaseSpeed + MANUAL_KP * err, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
        } else {
            manualTrackSpeed = constrain(manualBaseSpeed, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX);
        }

        if (currentArmAngle > target + 0.01f || currentArmAngle < target - 0.01f) {
            currentArmAngle = target;
            servoChanged = true;
        }
    } else {
        // >45°：恢复到默认基础速度，避免影响后续流程
        manualBaseSpeed = MANUAL_BASE_SPEED_INIT;
        manualTrackSpeed = MANUAL_BASE_SPEED_INIT;

        // 45°后允许飞控直接给 90°，这里立即到位，避免长时间缓慢爬升
        if (target >= 89.5f && currentArmAngle < 89.5f) {
            currentArmAngle = 90.0f;
            servoChanged = true;
        }
    }

    manualTargetArmAngle = target;
    lastArmCmdMs = millis();

    // 保证 MORPH 阶段电机维持飞行位
    if (currentMotorAngle != 0.0f) {
        currentMotorAngle = 0.0f;
        servoChanged = true;
    }

    if (servoChanged) {
        updateServos();
    }
}

// 由 MsgID 0x02 (ACT1) 触发
// 触发模式切换序列
void handle_mode_switch_sequence(uint16_t pwm_val) {
    if (pwm_val > 1800) {
        // MANUAL阶段若机臂尚未到位，不丢弃请求，改为挂起等待到位后执行
        if (currentMode == MODE_MANUAL_SERVO && currentArmAngle < 89.5f) {
            pendingGroundTransition = true;
            DEBUG_SERIAL.print(F("[PEND]  | CMD_ACT1 queued: wait arm to 90 (A:"));
            DEBUG_SERIAL.print(currentArmAngle, 0);
            DEBUG_SERIAL.println(F(")"));
            return;
        }

        pendingGroundTransition = false;
        // 切换到地面模式
        if (currentMode != MODE_GROUND && currentMode != TRANSITION_TO_GROUND) {
            currentMode = TRANSITION_TO_GROUND;
        }
    } else if (pwm_val < 1200) {
        pendingGroundTransition = false;
        // 切换到飞行模式
        if (currentMode != MODE_FLIGHT && currentMode != TRANSITION_TO_FLIGHT) {
            currentMode = TRANSITION_TO_FLIGHT;
        }
    }
}

// 由 MsgID 0x03 (ACT2) 触发
// 推杆控制逻辑
void handle_pushrod_action(uint16_t pwm_val) {
    if (pwm_val > 1800) {
        // 伸张 (Extending)
        armActuators[0].writeMicroseconds(2000);
        armActuators[1].writeMicroseconds(2000);
    } else if (pwm_val < 1200) {
        // 收缩 (Retracting)
        armActuators[0].writeMicroseconds(1000);
        armActuators[1].writeMicroseconds(1000);
    } else {
        // 停止 (Stopped)
        armActuators[0].writeMicroseconds(1500);
        armActuators[1].writeMicroseconds(1500);
    }
}

// 由 MsgID 0x04 触发或定期触发
void send_debug_status() {
    // 仅在 USB 调试口输出，不向飞控回传数据
    // DEBUG_SERIAL.println(F("[CMD] Heartbeat"));
}

// ============================================================================
// 格式化输出函数
// ============================================================================

void printLog(bool isEvent, const char* eventName, uint16_t eventVal) {
    // 字段1: 心跳/连接状态
    // 如果超过 2000ms 没收到有效报文 -> LOST
    // 如果最近有 CRC 错误 -> ERROR
    // 否则 -> LIVE
    if (millis() - debugInfo.lastValidTime > 2000) {
        DEBUG_SERIAL.print(F("[LOST]  "));
    } else if (debugInfo.crcError) {
        DEBUG_SERIAL.print(F("[ERROR] "));
        debugInfo.crcError = false; // 打印一次后清除错误标志
    } else {
        DEBUG_SERIAL.print(F("[LIVE]  "));
    }

    DEBUG_SERIAL.print(F("| "));

    // 字段2: 报文内容 (心跳不显示)
    // 格式固定宽度，方便对齐
    if (isEvent) {
        DEBUG_SERIAL.print(F("RX: "));
        DEBUG_SERIAL.print(eventName);
        DEBUG_SERIAL.print(F("("));
        DEBUG_SERIAL.print(eventVal);
        DEBUG_SERIAL.print(F(")"));
    } else {
        // 周期性显示: 显示最后一次收到的非心跳指令
        DEBUG_SERIAL.print(F("Last: "));
        DEBUG_SERIAL.print(debugInfo.lastCmdName);
        DEBUG_SERIAL.print(F("("));
        DEBUG_SERIAL.print(debugInfo.lastCmdVal);
        DEBUG_SERIAL.print(F(")"));
    }

    // 补齐空格以保持对齐 (假设最长输出约 25 字符)
    // 简单起见不计算精确长度，直接打印分隔符
    DEBUG_SERIAL.print(F(" | "));

    // 字段3: 控制状态信息
    switch (currentMode) {
        case MODE_MANUAL_SERVO:    DEBUG_SERIAL.print(F("MANUAL")); break;
        case MODE_FLIGHT:          DEBUG_SERIAL.print(F("FLIGHT")); break;
        case MODE_GROUND:          DEBUG_SERIAL.print(F("GROUND")); break;
        case TRANSITION_TO_GROUND: DEBUG_SERIAL.print(F(" > GND")); break;
        case TRANSITION_TO_FLIGHT: DEBUG_SERIAL.print(F(" > FLY")); break;
    }
    
    DEBUG_SERIAL.print(F(" A:"));
    DEBUG_SERIAL.print(currentArmAngle, 0);
    DEBUG_SERIAL.print(F(" M:"));
    DEBUG_SERIAL.print(currentMotorAngle, 0);
    
    DEBUG_SERIAL.println();

    // 如果长时间没收到有效报文，且有数据进来，提示检查波特率/协议
    static uint32_t lastWarnTime = 0;
    if (debugInfo.totalBytes > 100 && !debugInfo.msgReceived && (millis() - lastWarnTime > 10000)) {
       // 仅在周期性检查时才触发警告，避免事件打印触发误报
       if (!isEvent) {
           DEBUG_SERIAL.println(F("======================================"));
           DEBUG_SERIAL.println(F("[警告] 收到数据但无法解析(无有效帧头)。请检查："));
           DEBUG_SERIAL.println(F("1. 波特率是否匹配 (当前 9600)"));
           DEBUG_SERIAL.println(F("2. 帧头是否为 0xAA"));
           DEBUG_SERIAL.println(F("3. 飞控是否输出 MavLink (本程序不支持 MavLink)"));
           DEBUG_SERIAL.println(F("4. 飞控是否真正从 UART 发送数据"));
           DEBUG_SERIAL.println(F("======================================"));
           lastWarnTime = millis();
       }
    }
    
    // 修复：仅在周期性报告结束时重置标志位
    // 避免因为中间插入了事件打印(isEvent=true)而导致周期性检查时误判为无数据
    if (!isEvent) {
        debugInfo.msgReceived = false;
    }

    DEBUG_SERIAL.println(F("======================================"));
    DEBUG_SERIAL.println();
}

uint8_t crc8_update(uint8_t crc, uint8_t data) {
    uint8_t i = (data ^ crc) & 0xff;
    crc = 0;
    if (i & 1) crc ^= 0x5e;
    if (i & 2) crc ^= 0xbc;
    if (i & 4) crc ^= 0x61;
    if (i & 8) crc ^= 0xc2;
    if (i & 16) crc ^= 0x9d;
    if (i & 32) crc ^= 0x23;
    if (i & 64) crc ^= 0x46;
    if (i & 128) crc ^= 0x8c;
    return crc;
}
