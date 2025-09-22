#include <Servo.h>

//===硬件配置===
struct HardwareConfig {
  //输入引脚
  struct {
    uint8_t modeLock = 45;      //通道10（SdWA）
    uint8_t modeSwitch = 2;     //通道7(SWD)
    uint8_t armControl = 3;     //通道8(SWE)
    uint8_t landingGearControl = 13; //通道9（原airControl改为起落架控制）
  } inputPins;

  //输出引脚
  struct {
    struct {
      uint8_t baseServos[2] = { 11, 10 };    //基础舵机（电机臂servo0，电机方向servo1）
      uint8_t armServos[4] = { 4, 5, 6, 7 }; //电机舵机（保留但简化用途）
      uint8_t armActuators[2] = { 8, 9 };    //电推杆（保留）
      uint8_t landingGear = 12;              //起落架舵机（原pump引脚）
    } servos;
  } outputPins;

  //PWM参数
  struct {
    uint16_t min = 1000;
    uint16_t max = 2000;
    uint16_t deadzone = 50;  //死区
  } pwm;
};

//===全局对象===
HardwareConfig HW;
Servo servos[9];  //索引0-8对应各舵机

//===状态管理===
enum SystemState {  //系统状态
  STATE_FLIGHT_MODE,   //飞行模式：电机臂朝上，推力向下
  STATE_SURFACE_MODE   //陆/水模式：电机臂转下，推力水平
};

enum ArmState {  //推杆状态（保留）
  ARM_STOPPED,
  ARM_EXTENDING,
  ARM_RETRACTING
};

enum LandingGearState {  //起落架状态
  GEAR_RETRACTED,  //收起
  GEAR_EXTENDED    //放下
};

enum Controller {  //控制权归属
  SIGNAL_CT,
  SERIAL_CT
};

struct StatusBar {                           //状态机
  bool STATE_LOCKED = true;                  //默认锁定
  bool activation = false;                   //是否处于激活态
  SystemState system = STATE_FLIGHT_MODE;    //初始为飞行模式
  Controller controller = SIGNAL_CT;         //默认遥控
  ArmState arm = ARM_STOPPED;                //默认推杆停止
  LandingGearState landingGear = GEAR_RETRACTED; //默认起落架收起
  bool debugMode = true;                     //默认开启调试
};
StatusBar SystemStatus;

//===滤波器实现===
template<size_t N>
class MovingAverageFilter {
private:
  int buffer[N] = { 0 };
  size_t index = 0;

public:
  int filter(int newValue) {
    buffer[index] = newValue;
    index = (index + 1) % N;

    long sum = 0;
    for (size_t i = 0; i < N; i++) {
      sum += buffer[i];
    }
    return sum / N;
  }
};

MovingAverageFilter<5> inputFilters[4];
static int filteredValues[4];  // 存储滤波后的值

//===函数声明===
void hardwareInit();                                 //硬件初始化
void readControlSignals();                           //控制信号读取
void executeControlSignals();                        //识别信号指令
void SerialRead();                                   //串口命令读取
void executeCommand(const String& cmd);              //识别串口指令
void updateSystemState();                            //更新系统状态
void transitionToFlightMode();                       //飞行模式切换
void transitionToSurfaceMode();                      //陆/水模式切换
void manualModeChange();                             //手动模式切换
void manualArmOperation();                           //推杆控制
void manualLandingGearControl();                     //起落架控制
void safeServoWrite(uint8_t servoIndex, int value);  //舵机安全写入
void printDebugInfo();                               //调试输出

//===初始化===
void setup() {
  hardwareInit();
  Serial.begin(9600);
  Serial.println("Updated System Initialized - Two Mode Control");
}

//===主循环===
void loop() {
  static uint32_t beforetime = 0;
  //主函数
  if (millis() - beforetime >= 20) {  // 50Hz 更新频率
    SerialRead();                     //串口优先级高
    if (SystemStatus.controller == SIGNAL_CT) {
      readControlSignals();  //读取遥控信号
    }
    updateSystemState();    //更新状态
    beforetime = millis();  //更新计时
  }
  //调试信息，200ms刷新
  if (SystemStatus.debugMode) {
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= 200) {
      printDebugInfo();
      lastDebug = millis();
    }
  }
}

//===串口命令处理===
void SerialRead() {
  if (Serial.available() > 0) {
    String received = Serial.readStringUntil('\n');
    received.trim();  // 去除可能的多余空白字符
    received.toLowerCase();
    Serial.println(received);
    executeCommand(received);  //更新状态机
  }
}

//===更新状态机===
void executeCommand(const String& cmd) {
  //控制权限转让
  if (cmd == "ct_open") {
    SystemStatus.controller = SERIAL_CT;
    Serial.println("[OK] Serial control enabled");
  } else if (cmd == "ct_close") {
    SystemStatus.controller = SIGNAL_CT;
    Serial.println("[OK] Signal control enabled");
  }
  
  //指令执行
  if (SystemStatus.controller == SERIAL_CT) {
    if (cmd == "lock") {
      SystemStatus.STATE_LOCKED = true;
      Serial.println("[OK] System locked");
    } else if (cmd == "unlock") {
      SystemStatus.STATE_LOCKED = false;
      Serial.println("[OK] System unlocked");
    } else if (cmd == "mdchg") {
      manualModeChange();  //状态切换
      Serial.println("[OK] Mode changed");
    } else if (cmd == "armex") {
      SystemStatus.arm = ARM_EXTENDING;
      Serial.println("[OK] Arm extending");
    } else if (cmd == "armre") {
      SystemStatus.arm = ARM_RETRACTING;
      Serial.println("[OK] Arm retracting");
    } else if (cmd == "armst") {
      SystemStatus.arm = ARM_STOPPED;
      Serial.println("[OK] Arm stopped");
    } else if (cmd == "gearup") {
      SystemStatus.landingGear = GEAR_RETRACTED;
      Serial.println("[OK] Landing gear retracted");
    } else if (cmd == "geardown") {
      SystemStatus.landingGear = GEAR_EXTENDED;
      Serial.println("[OK] Landing gear extended");
    } else if (cmd == "flight") {
      SystemStatus.system = STATE_FLIGHT_MODE;
      SystemStatus.activation = true;
      Serial.println("[OK] Flight mode activated");
    } else if (cmd == "surface") {
      SystemStatus.system = STATE_SURFACE_MODE;
      SystemStatus.activation = true;
      Serial.println("[OK] Surface mode activated");
    } else {
      Serial.print("[ERROR] Unknown command: ");
      Serial.println(cmd);
    }
  }
}

//===更新模式状态机===
void manualModeChange() {
  if (!SystemStatus.STATE_LOCKED) {
    SystemStatus.activation = true;
    if (SystemStatus.system == STATE_FLIGHT_MODE) {
      SystemStatus.system = STATE_SURFACE_MODE;
    } else {
      SystemStatus.system = STATE_FLIGHT_MODE;
    }
  }
}

void manualArmOperation() {
  switch (SystemStatus.arm) {
    case ARM_EXTENDING:
      servos[6].writeMicroseconds(2000);
      servos[7].writeMicroseconds(2000);
      break;
    case ARM_RETRACTING:
      servos[6].writeMicroseconds(1000);
      servos[7].writeMicroseconds(1000);
      break;
    default:
      servos[6].writeMicroseconds(1500);
      servos[7].writeMicroseconds(1500);
  }
}

void manualLandingGearControl() {
  switch (SystemStatus.landingGear) {
    case GEAR_EXTENDED:  // 放下起落架
      servos[8].writeMicroseconds(2000);
      break;
    case GEAR_RETRACTED: // 收起起落架
      servos[8].writeMicroseconds(1000);
      break;
  }
}

//===遥控信号处理===
void executeControlSignals() {
  // 遥控器锁定控制
  if (filteredValues[0] < 1100) {
    SystemStatus.STATE_LOCKED = true;  //低值时上锁
  } else if(filteredValues[0] > 1900){
    SystemStatus.STATE_LOCKED = false; //高值时解锁
  }
  
  // 模式切换
  if (!SystemStatus.STATE_LOCKED) {
    int modeSignal = filteredValues[1];
    if (modeSignal < 1100 && SystemStatus.system != STATE_FLIGHT_MODE) {
      SystemStatus.system = STATE_FLIGHT_MODE;
      SystemStatus.activation = true;
    } else if (modeSignal > 1900 && SystemStatus.system != STATE_SURFACE_MODE) {
      SystemStatus.system = STATE_SURFACE_MODE;
      SystemStatus.activation = true;
    }
  }
  
  // 推杆控制（保留原逻辑）
  int armSignal = filteredValues[2];
  if (armSignal > 1900) {
    SystemStatus.arm = ARM_EXTENDING;
  } else if (armSignal < 1100 && armSignal > 500) {
    SystemStatus.arm = ARM_RETRACTING;
  } else {
    SystemStatus.arm = ARM_STOPPED;
  }
  
  // 起落架控制
  int gearSignal = filteredValues[3];
  if (gearSignal > 1900) {
    SystemStatus.landingGear = GEAR_EXTENDED;
  } else if (gearSignal < 1100 && gearSignal > 500) {
    SystemStatus.landingGear = GEAR_RETRACTED;
  }
}

//===系统状态更新===
void updateSystemState() {
  if (SystemStatus.activation) {
    if (SystemStatus.system == STATE_FLIGHT_MODE) {
      transitionToFlightMode();
    } else {
      transitionToSurfaceMode();
    }
  }
  manualArmOperation();         //处理推杆
  manualLandingGearControl();   //处理起落架
}

//===模式转换===
void transitionToFlightMode() {
  static uint8_t transitionStep = 0;
  static uint8_t motorStep = 0;
  const uint8_t totalSteps = 100;   //保留时延让转动更缓慢
  const uint8_t totalSteps2 = 90;

  if (transitionStep < totalSteps) {
    // 电机臂朝上，基础舵机转动
    int baseAngle = map(transitionStep, 0, totalSteps, 20, 130); //电机臂朝上
    int armAngle = map(transitionStep, 0, totalSteps, 155, 45);   //电机方向向下
    safeServoWrite(0, baseAngle);  //电机臂舵机
    safeServoWrite(1, armAngle);   //电机方向舵机
    Serial.println("[INFO] Flight mode transition - step 1");
    transitionStep++;
  } else {
    if (motorStep < totalSteps2) {
      // 电机舵机调整（简化，主要用于微调）
      int Angle1 = map(motorStep, 0, totalSteps2, 0, 90);
      int Angle2 = map(motorStep, 0, totalSteps2, 170, 80);
      safeServoWrite(2, Angle1);
      safeServoWrite(3, Angle1);
      safeServoWrite(4, Angle2);
      safeServoWrite(5, Angle2);
      motorStep++;
    } else {
      SystemStatus.system = STATE_FLIGHT_MODE;
      SystemStatus.activation = false;  //关闭激活态
      transitionStep = 0;
      motorStep = 0;
      Serial.println("[INFO] Flight mode activated");
    }
  }
}

void transitionToSurfaceMode() {
  static uint8_t transitionStep = 0;
  static uint8_t motorStep = 0;
  const uint8_t totalSteps1 = 100;  //保留时延让转动更缓慢
  const uint8_t totalSteps2 = 90;

  if (transitionStep < totalSteps1) {
    // 电机臂转下，推力方向水平
    int baseAngle = map(transitionStep, 0, totalSteps1, 130, 20); //电机臂转下
    int armAngle = map(transitionStep, 0, totalSteps1, 45, 155);   //电机方向水平
    safeServoWrite(0, baseAngle);  //电机臂舵机
    safeServoWrite(1, armAngle);   //电机方向舵机
    Serial.println("[INFO] Surface mode transition - step 1");
    transitionStep++;
  } else {
    if (motorStep < totalSteps2) {
      // 电机舵机调整（简化，主要用于微调）
      int Angle1 = map(motorStep, 0, totalSteps2, 90, 0);
      int Angle2 = map(motorStep, 0, totalSteps2, 80, 170);
      safeServoWrite(2, Angle1);
      safeServoWrite(3, Angle1);
      safeServoWrite(4, Angle2);
      safeServoWrite(5, Angle2);
      motorStep++;
    } else {
      SystemStatus.system = STATE_SURFACE_MODE;
      SystemStatus.activation = false;  //关闭激活态
      transitionStep = 0;
      motorStep = 0;
      Serial.println("[INFO] Surface mode activated");
    }
  }
}

//===安全舵机控制===
void safeServoWrite(uint8_t servoIndex, int value) {
  value = constrain(value, 0, 180);
  if (servoIndex < 9 && servos[servoIndex].attached()) {
    servos[servoIndex].write(value);
  }
}

//===调试信息输出===
void printDebugInfo() {
  Serial.println("\n==== System Status ====");
  Serial.print("Controller: ");
  Serial.println(SystemStatus.controller == SIGNAL_CT ? "SIGNAL" : "SERIAL");
  Serial.println(SystemStatus.STATE_LOCKED ? "LOCKED" : "UNLOCKED");
  Serial.print("Mode: ");
  switch (SystemStatus.system) {
    case STATE_FLIGHT_MODE: Serial.println("FLIGHT"); break;
    case STATE_SURFACE_MODE: Serial.println("SURFACE"); break;
  }

  Serial.print("Arm State: ");
  switch (SystemStatus.arm) {
    case ARM_STOPPED: Serial.println("STOPPED"); break;
    case ARM_EXTENDING: Serial.println("EXTENDING"); break;
    case ARM_RETRACTING: Serial.println("RETRACTING"); break;
  }

  Serial.print("Landing Gear: ");
  switch (SystemStatus.landingGear) {
    case GEAR_RETRACTED: Serial.println("RETRACTED"); break;
    case GEAR_EXTENDED: Serial.println("EXTENDED"); break;
  }
  
  Serial.print("Filtered Values: ");
  for(int i = 0; i < 4; i++) {
    Serial.print(filteredValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("=======================");
}

//===硬件初始化===
void hardwareInit() {
  // 初始化输入引脚
  pinMode(HW.inputPins.modeLock, INPUT);
  pinMode(HW.inputPins.modeSwitch, INPUT);
  pinMode(HW.inputPins.armControl, INPUT);
  pinMode(HW.inputPins.landingGearControl, INPUT);

  // 初始化舵机
  const uint8_t servoPins[] = {
    HW.outputPins.servos.baseServos[0],    //0 - 电机臂舵机
    HW.outputPins.servos.baseServos[1],    //1 - 电机方向舵机
    HW.outputPins.servos.armServos[0],     //2 - 电机舵机1
    HW.outputPins.servos.armServos[1],     //3 - 电机舵机2
    HW.outputPins.servos.armServos[2],     //4 - 电机舵机3
    HW.outputPins.servos.armServos[3],     //5 - 电机舵机4
    HW.outputPins.servos.armActuators[0],  //6 - 电推杆1
    HW.outputPins.servos.armActuators[1],  //7 - 电推杆2
    HW.outputPins.servos.landingGear       //8 - 起落架舵机
  };

  for (int i = 0; i < 9; i++) {
    if (servoPins[i] != 0) {
      servos[i].attach(servoPins[i]);
    }
  }
  
  // 初始位置设定（飞行模式）
  safeServoWrite(0, 130);  //电机臂朝上
  safeServoWrite(1, 45);   //电机方向向下
  safeServoWrite(2, 90);   //电机舵机初始位置
  safeServoWrite(3, 90);
  safeServoWrite(4, 80);
  safeServoWrite(5, 80);
  servos[6].writeMicroseconds(1500);  //推杆中位
  servos[7].writeMicroseconds(1500);  
  servos[8].writeMicroseconds(1000);  //起落架收起
  
  //状态机初始化
  SystemStatus.system = STATE_FLIGHT_MODE;
  SystemStatus.STATE_LOCKED = true;
  SystemStatus.arm = ARM_STOPPED;
  SystemStatus.landingGear = GEAR_RETRACTED;
  
  Serial.println("Hardware initialization complete");
}

//===信号处理===
void readControlSignals() {
  static int rawSignals[4];

  rawSignals[0] = pulseIn(HW.inputPins.modeLock, HIGH, 25000);
  rawSignals[1] = pulseIn(HW.inputPins.modeSwitch, HIGH, 25000);
  rawSignals[2] = pulseIn(HW.inputPins.armControl, HIGH, 25000);
  rawSignals[3] = pulseIn(HW.inputPins.landingGearControl, HIGH, 25000);

  // 信号处理与滤波
  for (int i = 0; i < 4; i++) {
    if (rawSignals[i] > 500) {
      rawSignals[i] = constrain(rawSignals[i], HW.pwm.min, HW.pwm.max);
    }
    filteredValues[i] = inputFilters[i].filter(rawSignals[i]);
  }
  
  executeControlSignals();
}
