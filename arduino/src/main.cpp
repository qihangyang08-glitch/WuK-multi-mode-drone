#include <Servo.h>

//===可调参数配置区域===
// 注意：这些参数可在飞行测试时快速调整，无需修改复杂逻辑
struct TimingParameters {
  uint16_t pushrodExtendTime = 2000;   // 电推杆伸出时长 (ms) [1000-4000]
  uint16_t pushrodRetractTime = 2000;  // 电推杆缩回时长 (ms) [1000-4000]
  uint16_t gearWaitTime = 2000;        // 起落架等待时长 (ms) [1000-4000] - 陆转空模式下手动操作间隔
  uint16_t servoTransitionDelay = 20;  // 舵机转动步进延时 (ms)
  uint8_t servoTransitionSteps = 100;  // 舵机转动总步数（控制转动速度）
};

// 全局时序参数实例
TimingParameters timing;

//===硬件配置===
struct HardwareConfig {
  // 输入引脚配置
  struct {
    uint8_t modeLock = 45;           // 通道10（SdWA）- 模式锁定
    uint8_t modeSwitch = 2;          // 通道7(SWD) - 模式切换
    uint8_t reservedCH8 = 3;         // 通道8 - 预留（原电推杆手动控制）
    uint8_t landingGearControl = 13; // 通道9 - 起落架控制
  } inputPins;

  // 输出引脚配置
  struct {
    struct {
      uint8_t motorArm = 11;         // 电机臂舵机（servo0）
      uint8_t motorDirection = 10;   // 电机方向舵机（servo1）
      uint8_t auxiliaryServos[4] = { 4, 5, 6, 7 }; // 辅助舵机（servo2-5）
      uint8_t pushrodActuators[2] = { 8, 9 };      // 电推杆（servo6-7）
      uint8_t landingGear = 12;      // 起落架舵机（servo8）
    } servos;
  } outputPins;

  // PWM信号参数
  struct {
    uint16_t min = 1000;
    uint16_t max = 2000;
    uint16_t center = 1500;
    uint16_t deadzone = 50;
  } pwm;
};

//===全局硬件对象===
HardwareConfig HW;
Servo servos[9];  // 舵机数组：0-电机臂，1-电机方向，2-5辅助，6-7推杆，8-起落架

//===系统状态枚举===
enum SystemMode {
  MODE_FLIGHT,    // 飞行模式：电机臂朝上，推力向下
  MODE_LAND,      // 陆地模式：电机臂朝下，推力水平，推杆调整
  MODE_WATER      // 水面模式：电机臂朝下，推力水平，无推杆动作
};

enum LandingGearState {
  GEAR_RETRACTED, // 起落架收起
  GEAR_EXTENDED   // 起落架放下
};

enum PushrodState {
  PUSHROD_STOPPED,    // 推杆停止
  PUSHROD_EXTENDING,  // 推杆伸出
  PUSHROD_RETRACTING  // 推杆缩回
};

enum ControllerSource {
  CONTROL_SIGNAL,  // 遥控信号控制
  CONTROL_SERIAL   // 串口命令控制
};

// 模式切换状态机（用于复杂的自动切换流程）
enum TransitionState {
  TRANSITION_IDLE,           // 空闲状态
  TRANSITION_FLIGHT_TO_LAND, // 空转陆流程
  TRANSITION_LAND_TO_FLIGHT, // 陆转空流程
  TRANSITION_FLIGHT_TO_WATER,// 空转水流程
  TRANSITION_WATER_TO_FLIGHT // 水转空流程
};

// 具体的切换步骤枚举
enum TransitionStep {
  STEP_IDLE = 0,
  // 空 → 陆 流程步骤
  STEP_F2L_EXTEND_PUSHROD = 1,    // 1. 伸电推杆
  STEP_F2L_LOWER_ARM = 2,         // 2. 放下电机臂
  STEP_F2L_ROTATE_MOTOR = 3,      // 3. 转动电机推力方向
  STEP_F2L_RETRACT_PUSHROD = 4,   // 4. 缩电推杆
  STEP_F2L_COMPLETE = 5,          // 5. 完成
  
  // 陆 → 空 流程步骤
  STEP_L2F_RETRACT_PUSHROD = 11,  // 1. 缩电推杆
  STEP_L2F_WAIT_GEAR = 12,        // 2. 等待起落架操作
  STEP_L2F_RAISE_ARM = 13,        // 3. 抬起电机臂
  STEP_L2F_ROTATE_MOTOR = 14,     // 4. 转动电机推力方向
  STEP_L2F_EXTEND_PUSHROD = 15,   // 5. 伸电推杆
  STEP_L2F_COMPLETE = 16,         // 6. 完成
  
  // 空 ↔ 水 流程步骤（简化版）
  STEP_SIMPLE_ARM = 21,           // 1. 转动电机臂
  STEP_SIMPLE_MOTOR = 22,         // 2. 转动电机方向
  STEP_SIMPLE_COMPLETE = 23       // 3. 完成
};

//===系统状态结构体===
struct SystemStatus {
  // 基础状态
  bool systemLocked = true;                    // 系统锁定状态
  SystemMode currentMode = MODE_FLIGHT;        // 当前系统模式
  SystemMode targetMode = MODE_FLIGHT;         // 目标模式
  ControllerSource controller = CONTROL_SIGNAL; // 控制源
  
  // 硬件状态
  LandingGearState landingGear = GEAR_RETRACTED;
  PushrodState pushrodState = PUSHROD_STOPPED;
  
  // 切换状态机
  TransitionState transitionState = TRANSITION_IDLE;
  TransitionStep currentStep = STEP_IDLE;
  uint32_t stepStartTime = 0;                  // 当前步骤开始时间
  uint8_t servoTransitionProgress = 0;         // 舵机转动进度
  
  // 调试与控制
  bool debugMode = true;                       // 调试模式开关
  bool autoTransitionActive = false;           // 自动切换激活标志
};

SystemStatus status;

//===滤波器实现===
template<size_t N>
class MovingAverageFilter {
private:
  int buffer[N] = { 0 };
  size_t index = 0;
  bool initialized = false;

public:
  int filter(int newValue) {
    buffer[index] = newValue;
    index = (index + 1) % N;
    if (!initialized && index == 0) initialized = true;

    long sum = 0;
    size_t count = initialized ? N : index;
    for (size_t i = 0; i < count; i++) {
      sum += buffer[i];
    }
    return count > 0 ? sum / count : newValue;
  }
};

MovingAverageFilter<5> signalFilters[4];
static int filteredSignals[4];

//===函数声明===
void initializeHardware();                    // 硬件初始化
void readControlSignals();                    // 读取并处理遥控信号
void processSerialCommands();                 // 处理串口命令
void executeCommand(const String& cmd);       // 执行具体命令
void updateSystemState();                     // 主状态更新
void processAutoTransition();                 // 处理自动切换流程
void startModeTransition(SystemMode target);  // 启动模式切换
void executeTransitionStep();                 // 执行切换步骤
void controlPushrod(PushrodState state);      // 控制电推杆
void controlLandingGear(LandingGearState state); // 控制起落架
void safeServoWrite(uint8_t index, int angle); // 安全舵机控制
void smoothServoTransition(uint8_t servoIndex, int fromAngle, int toAngle, uint8_t& progress);
void printSystemStatus();                     // 打印系统状态
void printTransitionStep(const char* stepName); // 打印切换步骤

//===主程序入口===
void setup() {
  Serial.begin(9600);
  Serial.println("=================================");
  Serial.println("Parametric Auto Control System V2");
  Serial.println("=================================");
  
  initializeHardware();
  
  // 打印参数配置
  Serial.println("Timing Parameters:");
  Serial.print("  Pushrod Extend: "); Serial.print(timing.pushrodExtendTime); Serial.println(" ms");
  Serial.print("  Pushrod Retract: "); Serial.print(timing.pushrodRetractTime); Serial.println(" ms");
  Serial.print("  Gear Wait Time: "); Serial.print(timing.gearWaitTime); Serial.println(" ms");
  Serial.println("System Ready!");
}

void loop() {
  static uint32_t lastMainUpdate = 0;
  static uint32_t lastDebugUpdate = 0;
  
  // 主控制循环 - 50Hz
  if (millis() - lastMainUpdate >= 20) {
    processSerialCommands();        // 串口命令优先处理
    
    if (status.controller == CONTROL_SIGNAL) {
      readControlSignals();         // 处理遥控信号
    }
    
    updateSystemState();            // 更新系统状态
    processAutoTransition();        // 处理自动切换流程
    
    lastMainUpdate = millis();
  }
  
  // 调试信息输出 - 5Hz
  if (status.debugMode && millis() - lastDebugUpdate >= 200) {
    printSystemStatus();
    lastDebugUpdate = millis();
  }
}

//===硬件初始化===
void initializeHardware() {
  // 配置输入引脚
  pinMode(HW.inputPins.modeLock, INPUT);
  pinMode(HW.inputPins.modeSwitch, INPUT);
  pinMode(HW.inputPins.reservedCH8, INPUT);           // 预留引脚
  pinMode(HW.inputPins.landingGearControl, INPUT);
  
  // 初始化舵机
  const uint8_t servoPins[] = {
    HW.outputPins.servos.motorArm,           // 0 - 电机臂
    HW.outputPins.servos.motorDirection,     // 1 - 电机方向
    HW.outputPins.servos.auxiliaryServos[0], // 2 - 辅助舵机1
    HW.outputPins.servos.auxiliaryServos[1], // 3 - 辅助舵机2
    HW.outputPins.servos.auxiliaryServos[2], // 4 - 辅助舵机3
    HW.outputPins.servos.auxiliaryServos[3], // 5 - 辅助舵机4
    HW.outputPins.servos.pushrodActuators[0], // 6 - 电推杆1
    HW.outputPins.servos.pushrodActuators[1], // 7 - 电推杆2
    HW.outputPins.servos.landingGear         // 8 - 起落架
  };
  
  // 连接所有舵机
  for (int i = 0; i < 9; i++) {
    servos[i].attach(servoPins[i]);
    Serial.print("Servo "); Serial.print(i); Serial.print(" attached to pin "); Serial.println(servoPins[i]);
  }
  
  // 设置初始位置（飞行模式）
  safeServoWrite(0, 130);  // 电机臂朝上
  safeServoWrite(1, 45);   // 电机方向向下
  safeServoWrite(2, 90);   // 辅助舵机初始位置
  safeServoWrite(3, 90);
  safeServoWrite(4, 80);
  safeServoWrite(5, 80);
  
  // 推杆和起落架初始位置
  servos[6].writeMicroseconds(HW.pwm.center); // 推杆1中位
  servos[7].writeMicroseconds(HW.pwm.center); // 推杆2中位
  servos[8].writeMicroseconds(HW.pwm.min);    // 起落架收起
  
  Serial.println("Hardware initialization complete");
}

//===串口命令处理===
void processSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    Serial.print("[CMD] Received: "); Serial.println(command);
    executeCommand(command);
  }
}

void executeCommand(const String& cmd) {
  // 控制权切换
  if (cmd == "ctrl_signal") {
    status.controller = CONTROL_SIGNAL;
    Serial.println("[OK] Switched to signal control");
  }
  else if (cmd == "ctrl_serial") {
    status.controller = CONTROL_SERIAL;
    Serial.println("[OK] Switched to serial control");
  }
  
  // 系统锁定控制
  else if (cmd == "lock") {
    status.systemLocked = true;
    Serial.println("[OK] System locked");
  }
  else if (cmd == "unlock") {
    status.systemLocked = false;
    Serial.println("[OK] System unlocked");
  }
  
  // 模式切换命令（仅在串口控制且解锁状态下有效）
  else if (status.controller == CONTROL_SERIAL && !status.systemLocked) {
    if (cmd == "mode_flight") {
      startModeTransition(MODE_FLIGHT);
      Serial.println("[OK] Transitioning to flight mode");
    }
    else if (cmd == "mode_land") {
      startModeTransition(MODE_LAND);
      Serial.println("[OK] Transitioning to land mode");
    }
    else if (cmd == "mode_water") {
      startModeTransition(MODE_WATER);
      Serial.println("[OK] Transitioning to water mode");
    }
    
    // 手动硬件控制
    else if (cmd == "gear_up") {
      controlLandingGear(GEAR_RETRACTED);
      Serial.println("[OK] Landing gear retracted");
    }
    else if (cmd == "gear_down") {
      controlLandingGear(GEAR_EXTENDED);
      Serial.println("[OK] Landing gear extended");
    }
    else if (cmd == "pushrod_extend") {
      controlPushrod(PUSHROD_EXTENDING);
      Serial.println("[OK] Pushrod extending");
    }
    else if (cmd == "pushrod_retract") {
      controlPushrod(PUSHROD_RETRACTING);
      Serial.println("[OK] Pushrod retracting");
    }
    else if (cmd == "pushrod_stop") {
      controlPushrod(PUSHROD_STOPPED);
      Serial.println("[OK] Pushrod stopped");
    }
    else {
      // 尝试处理扩展命令
      executeExtendedCommands(cmd);
    }
  }
  else if (status.systemLocked && (cmd.startsWith("mode_") || cmd.startsWith("gear_") || cmd.startsWith("pushrod_"))) {
    Serial.println("[ERROR] System is locked");
  }
}

//===遥控信号处理===
void readControlSignals() {
  static int rawSignals[4];
  
  // 读取PWM信号
  rawSignals[0] = pulseIn(HW.inputPins.modeLock, HIGH, 25000);
  rawSignals[1] = pulseIn(HW.inputPins.modeSwitch, HIGH, 25000);
  rawSignals[2] = pulseIn(HW.inputPins.reservedCH8, HIGH, 25000);        // 预留通道
  rawSignals[3] = pulseIn(HW.inputPins.landingGearControl, HIGH, 25000);
  
  // 信号滤波处理
  for (int i = 0; i < 4; i++) {
    if (rawSignals[i] > 500) {  // 有效信号检测
      rawSignals[i] = constrain(rawSignals[i], HW.pwm.min, HW.pwm.max);
    }
    filteredSignals[i] = signalFilters[i].filter(rawSignals[i]);
  }
  
  // 处理锁定信号
  if (filteredSignals[0] < 1100) {
    status.systemLocked = true;
  } else if (filteredSignals[0] > 1900) {
    status.systemLocked = false;
  }
  
  // 处理模式切换信号（仅在解锁状态下）
  if (!status.systemLocked && status.transitionState == TRANSITION_IDLE) {
    int modeSignal = filteredSignals[1];
    
    if (modeSignal < 1200 && status.currentMode != MODE_FLIGHT) {
      startModeTransition(MODE_FLIGHT);
    }
    else if (modeSignal > 1400 && modeSignal < 1600 && status.currentMode != MODE_LAND) {
      startModeTransition(MODE_LAND);
    }
    else if (modeSignal > 1800 && status.currentMode != MODE_WATER) {
      startModeTransition(MODE_WATER);
    }
  }
  
  // 处理起落架信号
  int gearSignal = filteredSignals[3];
  if (gearSignal > 1800) {
    controlLandingGear(GEAR_EXTENDED);
  } else if (gearSignal < 1200 && gearSignal > 500) {
    controlLandingGear(GEAR_RETRACTED);
  }
}

//===系统状态更新===
void updateSystemState() {
  // 更新推杆状态（非自动切换时的手动控制已移除）
  // 推杆现在仅由自动切换流程控制
}

//===自动切换流程处理===
void processAutoTransition() {
  if (status.transitionState == TRANSITION_IDLE) return;
  
  executeTransitionStep();
}

void startModeTransition(SystemMode target) {
  if (status.transitionState != TRANSITION_IDLE) {
    Serial.println("[WARN] Transition already in progress");
    return;
  }
  
  status.targetMode = target;
  status.autoTransitionActive = true;
  status.stepStartTime = millis();
  status.servoTransitionProgress = 0;
  
  // 确定切换类型
  if (status.currentMode == MODE_FLIGHT && target == MODE_LAND) {
    status.transitionState = TRANSITION_FLIGHT_TO_LAND;
    status.currentStep = STEP_F2L_EXTEND_PUSHROD;
    printTransitionStep("Starting Flight → Land transition");
  }
  else if (status.currentMode == MODE_LAND && target == MODE_FLIGHT) {
    status.transitionState = TRANSITION_LAND_TO_FLIGHT;
    status.currentStep = STEP_L2F_RETRACT_PUSHROD;
    printTransitionStep("Starting Land → Flight transition");
  }
  else if (status.currentMode == MODE_FLIGHT && target == MODE_WATER) {
    status.transitionState = TRANSITION_FLIGHT_TO_WATER;
    status.currentStep = STEP_SIMPLE_ARM;
    printTransitionStep("Starting Flight → Water transition");
  }
  else if (status.currentMode == MODE_WATER && target == MODE_FLIGHT) {
    status.transitionState = TRANSITION_WATER_TO_FLIGHT;
    status.currentStep = STEP_SIMPLE_ARM;
    printTransitionStep("Starting Water → Flight transition");
  }
}

void executeTransitionStep() {
  uint32_t stepElapsed = millis() - status.stepStartTime;
  
  switch (status.currentStep) {
    // ===== 空 → 陆 流程 =====
    case STEP_F2L_EXTEND_PUSHROD:
      printTransitionStep("Step 1/4: Extending pushrod");
      controlPushrod(PUSHROD_EXTENDING);
      if (stepElapsed >= timing.pushrodExtendTime) {
        status.currentStep = STEP_F2L_LOWER_ARM;
        status.stepStartTime = millis();
        status.servoTransitionProgress = 0;
      }
      break;
      
    case STEP_F2L_LOWER_ARM:
      printTransitionStep("Step 2/4: Lowering motor arm");
      smoothServoTransition(0, 130, 20, status.servoTransitionProgress);
      if (status.servoTransitionProgress >= timing.servoTransitionSteps) {
        status.currentStep = STEP_F2L_ROTATE_MOTOR;
        status.stepStartTime = millis();
        status.servoTransitionProgress = 0;
      }
      break;
      
    case STEP_F2L_ROTATE_MOTOR:
      printTransitionStep("Step 3/4: Rotating motor direction");
      smoothServoTransition(1, 45, 155, status.servoTransitionProgress);
      if (status.servoTransitionProgress >= timing.servoTransitionSteps) {
        status.currentStep = STEP_F2L_RETRACT_PUSHROD;
        status.stepStartTime = millis();
      }
      break;
      
    case STEP_F2L_RETRACT_PUSHROD:
      printTransitionStep("Step 4/4: Retracting pushrod");
      controlPushrod(PUSHROD_RETRACTING);
      if (stepElapsed >= timing.pushrodRetractTime) {
        status.currentStep = STEP_F2L_COMPLETE;
      }
      break;
      
    case STEP_F2L_COMPLETE:
      printTransitionStep("Flight → Land transition complete!");
      status.currentMode = MODE_LAND;
      status.transitionState = TRANSITION_IDLE;
      status.currentStep = STEP_IDLE;
      status.autoTransitionActive = false;
      controlPushrod(PUSHROD_STOPPED);
      break;
      
    // ===== 陆 → 空 流程 =====
    case STEP_L2F_RETRACT_PUSHROD:
      printTransitionStep("Step 1/5: Retracting pushrod");
      controlPushrod(PUSHROD_RETRACTING);
      if (stepElapsed >= timing.pushrodRetractTime) {
        status.currentStep = STEP_L2F_WAIT_GEAR;
        status.stepStartTime = millis();
      }
      break;
      
    case STEP_L2F_WAIT_GEAR:
      printTransitionStep("Step 2/5: Waiting for landing gear operation");
      controlPushrod(PUSHROD_STOPPED);
      Serial.println("[INFO] Manual landing gear retraction window");
      if (stepElapsed >= timing.gearWaitTime) {
        status.currentStep = STEP_L2F_RAISE_ARM;
        status.stepStartTime = millis();
        status.servoTransitionProgress = 0;
      }
      break;
      
    case STEP_L2F_RAISE_ARM:
      printTransitionStep("Step 3/5: Raising motor arm");
      smoothServoTransition(0, 20, 130, status.servoTransitionProgress);
      if (status.servoTransitionProgress >= timing.servoTransitionSteps) {
        status.currentStep = STEP_L2F_ROTATE_MOTOR;
        status.stepStartTime = millis();
        status.servoTransitionProgress = 0;
      }
      break;
      
    case STEP_L2F_ROTATE_MOTOR:
      printTransitionStep("Step 4/5: Rotating motor direction");
      smoothServoTransition(1, 155, 45, status.servoTransitionProgress);
      if (status.servoTransitionProgress >= timing.servoTransitionSteps) {
        status.currentStep = STEP_L2F_EXTEND_PUSHROD;
        status.stepStartTime = millis();
      }
      break;
      
    case STEP_L2F_EXTEND_PUSHROD:
      printTransitionStep("Step 5/5: Extending pushrod");
      controlPushrod(PUSHROD_EXTENDING);
      if (stepElapsed >= timing.pushrodExtendTime) {
        status.currentStep = STEP_L2F_COMPLETE;
      }
      break;
      
    case STEP_L2F_COMPLETE:
      printTransitionStep("Land → Flight transition complete!");
      status.currentMode = MODE_FLIGHT;
      status.transitionState = TRANSITION_IDLE;
      status.currentStep = STEP_IDLE;
      status.autoTransitionActive = false;
      controlPushrod(PUSHROD_STOPPED);
      break;
      
    // ===== 简化版空↔水流程 =====
    case STEP_SIMPLE_ARM:
      if (status.transitionState == TRANSITION_FLIGHT_TO_WATER) {
        printTransitionStep("Step 1/2: Lowering motor arm (Flight → Water)");
        smoothServoTransition(0, 130, 20, status.servoTransitionProgress);
      } else {
        printTransitionStep("Step 1/2: Raising motor arm (Water → Flight)");
        smoothServoTransition(0, 20, 130, status.servoTransitionProgress);
      }
      
      if (status.servoTransitionProgress >= timing.servoTransitionSteps) {
        status.currentStep = STEP_SIMPLE_MOTOR;
        status.servoTransitionProgress = 0;
      }
      break;
      
    case STEP_SIMPLE_MOTOR:
      if (status.transitionState == TRANSITION_FLIGHT_TO_WATER) {
        printTransitionStep("Step 2/2: Rotating motor direction (Flight → Water)");
        smoothServoTransition(1, 45, 155, status.servoTransitionProgress);
      } else {
        printTransitionStep("Step 2/2: Rotating motor direction (Water → Flight)");
        smoothServoTransition(1, 155, 45, status.servoTransitionProgress);
      }
      
      if (status.servoTransitionProgress >= timing.servoTransitionSteps) {
        status.currentStep = STEP_SIMPLE_COMPLETE;
      }
      break;
      
    case STEP_SIMPLE_COMPLETE:
      if (status.transitionState == TRANSITION_FLIGHT_TO_WATER) {
        printTransitionStep("Flight → Water transition complete!");
        status.currentMode = MODE_WATER;
      } else {
        printTransitionStep("Water → Flight transition complete!");
        status.currentMode = MODE_FLIGHT;
      }
      status.transitionState = TRANSITION_IDLE;
      status.currentStep = STEP_IDLE;
      status.autoTransitionActive = false;
      break;
  }
}

//===硬件控制函数===
void controlPushrod(PushrodState state) {
  status.pushrodState = state;
  
  switch (state) {
    case PUSHROD_EXTENDING:
      servos[6].writeMicroseconds(HW.pwm.max);
      servos[7].writeMicroseconds(HW.pwm.max);
      break;
    case PUSHROD_RETRACTING:
      servos[6].writeMicroseconds(HW.pwm.min);
      servos[7].writeMicroseconds(HW.pwm.min);
      break;
    default: // PUSHROD_STOPPED
      servos[6].writeMicroseconds(HW.pwm.center);
      servos[7].writeMicroseconds(HW.pwm.center);
  }
}

void controlLandingGear(LandingGearState state) {
  status.landingGear = state;
  
  switch (state) {
    case GEAR_EXTENDED:
      servos[8].writeMicroseconds(HW.pwm.max);
      break;
    case GEAR_RETRACTED:
      servos[8].writeMicroseconds(HW.pwm.min);
      break;
  }
}

void safeServoWrite(uint8_t index, int angle) {
  if (index < 9 && servos[index].attached()) {
    angle = constrain(angle, 0, 180);
    servos[index].write(angle);
  }
}

void smoothServoTransition(uint8_t servoIndex, int fromAngle, int toAngle, uint8_t& progress) {
  if (progress < timing.servoTransitionSteps) {
    int currentAngle = map(progress, 0, timing.servoTransitionSteps, fromAngle, toAngle);
    safeServoWrite(servoIndex, currentAngle);
    progress++;
    delay(timing.servoTransitionDelay);
  }
}

//===调试和状态输出函数===
void printSystemStatus() {
  Serial.println("\n========== SYSTEM STATUS ==========");
  
  // 基础系统状态
  Serial.print("System: ");
  Serial.print(status.systemLocked ? "LOCKED" : "UNLOCKED");
  Serial.print(" | Controller: ");
  Serial.println(status.controller == CONTROL_SIGNAL ? "SIGNAL" : "SERIAL");
  
  // 当前模式
  Serial.print("Mode: ");
  switch (status.currentMode) {
    case MODE_FLIGHT: Serial.print("FLIGHT"); break;
    case MODE_LAND: Serial.print("LAND"); break;
    case MODE_WATER: Serial.print("WATER"); break;
  }
  
  // 切换状态
  if (status.autoTransitionActive) {
    Serial.print(" → ");
    switch (status.targetMode) {
      case MODE_FLIGHT: Serial.print("FLIGHT"); break;
      case MODE_LAND: Serial.print("LAND"); break;
      case MODE_WATER: Serial.print("WATER"); break;
    }
    Serial.print(" (Step: "); Serial.print(status.currentStep); Serial.print(")");
  }
  Serial.println();
  
  // 硬件状态
  Serial.print("Landing Gear: ");
  Serial.print(status.landingGear == GEAR_EXTENDED ? "EXTENDED" : "RETRACTED");
  Serial.print(" | Pushrod: ");
  switch (status.pushrodState) {
    case PUSHROD_EXTENDING: Serial.print("EXTENDING"); break;
    case PUSHROD_RETRACTING: Serial.print("RETRACTING"); break;
    default: Serial.print("STOPPED"); break;
  }
  Serial.println();
  
  // 信号值（仅在信号控制模式下显示）
  if (status.controller == CONTROL_SIGNAL) {
    Serial.print("Signals: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(filteredSignals[i]); Serial.print(" ");
    }
    Serial.println();
  }
  
  Serial.println("==================================");
}

void printTransitionStep(const char* stepName) {
  Serial.print("[TRANSITION] "); Serial.println(stepName);
}

//===参数调整函数（通过串口动态修改）===
void adjustTimingParameters(const String& param, uint16_t value) {
  if (param == "extend_time") {
    timing.pushrodExtendTime = constrain(value, 1000, 4000);
    Serial.print("[PARAM] Pushrod extend time set to: "); Serial.println(timing.pushrodExtendTime);
  }
  else if (param == "retract_time") {
    timing.pushrodRetractTime = constrain(value, 1000, 4000);
    Serial.print("[PARAM] Pushrod retract time set to: "); Serial.println(timing.pushrodRetractTime);
  }
  else if (param == "gear_wait") {
    timing.gearWaitTime = constrain(value, 1000, 4000);
    Serial.print("[PARAM] Gear wait time set to: "); Serial.println(timing.gearWaitTime);
  }
  else if (param == "servo_steps") {
    timing.servoTransitionSteps = constrain(value, 50, 200);
    Serial.print("[PARAM] Servo transition steps set to: "); Serial.println(timing.servoTransitionSteps);
  }
  else if (param == "servo_delay") {
    timing.servoTransitionDelay = constrain(value, 10, 50);
    Serial.print("[PARAM] Servo transition delay set to: "); Serial.println(timing.servoTransitionDelay);
  }
  else {
    Serial.print("[ERROR] Unknown parameter: "); Serial.println(param);
  }
}

//===扩展命令处理===
// 在executeCommand函数中添加更多命令支持
void executeExtendedCommands(const String& cmd) {
  // 参数调整命令格式: "param_<name>_<value>"
  if (cmd.startsWith("param_")) {
    int firstUnderscore = cmd.indexOf('_', 6);
    int secondUnderscore = cmd.indexOf('_', firstUnderscore + 1);
    
    if (firstUnderscore > 0 && secondUnderscore > 0) {
      String paramName = cmd.substring(6, firstUnderscore);
      String valueStr = cmd.substring(firstUnderscore + 1);
      uint16_t value = valueStr.toInt();
      
      adjustTimingParameters(paramName, value);
    } else {
      Serial.println("[ERROR] Parameter format: param_<name>_<value>");
    }
  }
  
  // 调试控制
  else if (cmd == "debug_on") {
    status.debugMode = true;
    Serial.println("[OK] Debug mode enabled");
  }
  else if (cmd == "debug_off") {
    status.debugMode = false;
    Serial.println("[OK] Debug mode disabled");
  }
  
  // 系统信息
  else if (cmd == "status") {
    printSystemStatus();
  }
  else if (cmd == "params") {
    Serial.println("\n===== TIMING PARAMETERS =====");
    Serial.print("Pushrod Extend: "); Serial.print(timing.pushrodExtendTime); Serial.println(" ms");
    Serial.print("Pushrod Retract: "); Serial.print(timing.pushrodRetractTime); Serial.println(" ms");
    Serial.print("Gear Wait: "); Serial.print(timing.gearWaitTime); Serial.println(" ms");
    Serial.print("Servo Steps: "); Serial.println(timing.servoTransitionSteps);
    Serial.print("Servo Delay: "); Serial.print(timing.servoTransitionDelay); Serial.println(" ms");
    Serial.println("=============================");
  }
  
  // 紧急停止
  else if (cmd == "emergency_stop" || cmd == "estop") {
    Serial.println("[EMERGENCY] Stopping all operations");
    status.transitionState = TRANSITION_IDLE;
    status.currentStep = STEP_IDLE;
    status.autoTransitionActive = false;
    controlPushrod(PUSHROD_STOPPED);
    Serial.println("[OK] Emergency stop complete");
  }
  
  // 帮助信息
  else if (cmd == "help") {
    Serial.println("\n===== COMMAND REFERENCE =====");
    Serial.println("Control:");
    Serial.println("  ctrl_signal / ctrl_serial - Switch control source");
    Serial.println("  lock / unlock - System lock control");
    Serial.println("");
    Serial.println("Mode Transitions:");
    Serial.println("  mode_flight / mode_land / mode_water - Auto mode transition");
    Serial.println("");
    Serial.println("Manual Control:");
    Serial.println("  gear_up / gear_down - Landing gear control");
    Serial.println("  pushrod_extend / pushrod_retract / pushrod_stop");
    Serial.println("");
    Serial.println("Parameters (format: param_<name>_<value>):");
    Serial.println("  param_extend_time_2000 - Pushrod extend time (1000-4000ms)");
    Serial.println("  param_retract_time_2000 - Pushrod retract time (1000-4000ms)");
    Serial.println("  param_gear_wait_2000 - Gear wait time (1000-4000ms)");
    Serial.println("  param_servo_steps_100 - Servo transition steps (50-200)");
    Serial.println("  param_servo_delay_20 - Servo step delay (10-50ms)");
    Serial.println("");
    Serial.println("Debug:");
    Serial.println("  debug_on / debug_off - Debug output control");
    Serial.println("  status - Show current status");
    Serial.println("  params - Show timing parameters");
    Serial.println("  emergency_stop / estop - Emergency stop all operations");
    Serial.println("=============================");
  }
}
