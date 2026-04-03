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

const uint8_t PIN_ARM_SERVO_0 = 11;    // 基础舵机（电机臂servo0）
const uint8_t PIN_ARM_SERVO_1 = 10;    // 基础舵机（电机方向servo1）

const uint8_t PIN_MOTOR_SERVO_0 = 4;   // 电机舵机1
const uint8_t PIN_MOTOR_SERVO_1 = 5;   // 电机舵机2
const uint8_t PIN_MOTOR_SERVO_2 = 6;   // 电机舵机3
const uint8_t PIN_MOTOR_SERVO_3 = 7;   // 电机舵机4

const uint8_t PIN_ACTUATOR_0 = 8;      // 电推杆1
const uint8_t PIN_ACTUATOR_1 = 9;      // 电推杆2

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
#define FC_BAUD 115200         // 必须与 ArduPilot 中的 WUK_BAUD 参数匹配
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
unsigned long lastLoopTime = 0;

// 速度定义 (度/秒)
const float SPEED_NORMAL = 30.0; 
const float SPEED_SLOW = 10.0;   // 用于 90->45 的慢速阶段

// --- 函数原型 ---
void handle_arm_servo_direct(uint16_t target_angle);
void handle_mode_switch_sequence(uint16_t pwm_val);
void handle_pushrod_action(uint16_t pwm_val);
void send_debug_status();
void printDebugInfo();
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
    
    DEBUG_SERIAL.println(F("[WuK] Arduino 从机已初始化 (单工模式 - RX Pin 15)"));
    DEBUG_SERIAL.println(F("[WuK] 等待飞控指令..."));

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

    // 3. 定期调试输出
    static uint32_t lastDebugTime = 0;
    if (millis() - lastDebugTime > 500) {
        printDebugInfo();
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
            DEBUG_SERIAL.println(F("[STATE] Reached GROUND Mode"));
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
            DEBUG_SERIAL.println(F("[STATE] Reached FLIGHT Mode"));
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
    int s0 = map((long)currentArmAngle, 0, 90, 20, 130);
    int s1 = map((long)currentArmAngle, 0, 90, 155, 45);
    armServos[0].write(s0);
    armServos[1].write(s1);
    
    // 电机舵机
    // Motor 0,1 (S2, S3): 0°(Flight) -> 0°, 90°(Ground) -> 90°
    // Motor 2,3 (S4, S5): 0°(Flight) -> 170°, 90°(Ground) -> 80°
    int m01 = map((long)currentMotorAngle, 0, 90, 0, 90);
    int m23 = map((long)currentMotorAngle, 0, 90, 170, 80);
    
    motorServos[0].write(m01);
    motorServos[1].write(m01);
    motorServos[2].write(m23);
    motorServos[3].write(m23);
}

// ============================================================================
// 通信解析器
// ============================================================================

void processSerial() {
    // 循环查询引脚是否有报文
    while (FC_SERIAL.available() > 0) {
        uint8_t b = FC_SERIAL.read();

        switch (rxState) {
            case WAIT_HEADER:
                if (b == HEADER_BYTE) {
                    rxState = WAIT_MSG_ID;
                    rxCRC = 0; // Reset CRC
                }
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
                    DEBUG_SERIAL.println(F("[ERR] 载荷过长"));
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

                    switch ((CmdID)rxMsgID) {
                        case CMD_ARM_SERVO:
                            handle_arm_servo_direct(val);
                            break;
                        case CMD_ACT1:
                            handle_mode_switch_sequence(val);
                            break;
                        case CMD_ACT2:
                            handle_pushrod_action(val);
                            break;
                        case CMD_STATUS_REQ:
                            send_debug_status();
                            break;
                        default:
                            DEBUG_SERIAL.print(F("[WARN] Unknown MsgID: 0x"));
                            DEBUG_SERIAL.println(rxMsgID, HEX);
                            break;
                    }
                } else {
                    DEBUG_SERIAL.println(F("[ERR] CRC Mismatch"));
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
    
    // 切换到手动模式，停止自动转换
    currentMode = MODE_MANUAL_SERVO;
    currentArmAngle = (float)target_angle;
    currentMotorAngle = 0.0; // 保险起见：手动控制机臂时，强制电机回正 (0度)
    
    // 立即更新舵机
    updateServos();

    DEBUG_SERIAL.print(F("[CMD] Arm Servo Target: "));
    DEBUG_SERIAL.println(target_angle);
}

// 由 MsgID 0x02 (ACT1) 触发
// 触发模式切换序列
void handle_mode_switch_sequence(uint16_t pwm_val) {
    if (pwm_val > 1800) {
        // 切换到地面模式
        if (currentMode != MODE_GROUND && currentMode != TRANSITION_TO_GROUND) {
            currentMode = TRANSITION_TO_GROUND;
            DEBUG_SERIAL.println(F("[CMD] Mode Switch: -> GROUND"));
        }
    } else if (pwm_val < 1200) {
        // 切换到飞行模式
        if (currentMode != MODE_FLIGHT && currentMode != TRANSITION_TO_FLIGHT) {
            currentMode = TRANSITION_TO_FLIGHT;
            DEBUG_SERIAL.println(F("[CMD] Mode Switch: -> FLIGHT"));
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
        // DEBUG_SERIAL.println(F("[CMD] Pushrod: EXTENDING"));
    } else if (pwm_val < 1200) {
        // 收缩 (Retracting)
        armActuators[0].writeMicroseconds(1000);
        armActuators[1].writeMicroseconds(1000);
        // DEBUG_SERIAL.println(F("[CMD] Pushrod: RETRACTING"));
    } else {
        // 停止 (Stopped)
        armActuators[0].writeMicroseconds(1500);
        armActuators[1].writeMicroseconds(1500);
        // DEBUG_SERIAL.println(F("[CMD] Pushrod: STOPPED"));
    }
}

// 由 MsgID 0x04 触发或定期触发
void send_debug_status() {
    // 仅在 USB 调试口输出，不向飞控回传数据
    // DEBUG_SERIAL.println(F("[CMD] Heartbeat"));
}

// ============================================================================
// 辅助函数
// ============================================================================

void printDebugInfo() {
    DEBUG_SERIAL.print(F("Mode: "));
    DEBUG_SERIAL.print(currentMode);
    DEBUG_SERIAL.print(F(" Arm: "));
    DEBUG_SERIAL.print(currentArmAngle);
    DEBUG_SERIAL.print(F(" Motor: "));
    DEBUG_SERIAL.println(currentMotorAngle);
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
