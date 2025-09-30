#include <Servo.h>

//===硬件配置===
struct HardwareConfig {
  //输入引脚
  struct {
    uint8_t modeLock = 45;           //通道10（SdWA）
    uint8_t modeSwitch = 2;          //通道7(SWD)
    uint8_t armControl = 3;          //通道8(SWE)
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

//===状态追踪结构体（用于检测变化）===
struct OutputStateTracker {
  int servoAngles[6] = {-1, -1, -1, -1, -1, -1};  // Servo 0-5的角度值，初始-1表示未设置
  int actuatorPWM[2] = {-1, -1};                  // Servo 6-7的PWM值
  int landingGearPWM = -1;                        // Servo 8的PWM值
  
  // 更新并检测servo角度变化
  bool updateServoAngle(uint8_t index, int newAngle) {
    if (index >= 6) return false;
    if (servoAngles[index] != newAngle) {
      servoAngles[index] = newAngle;
      return true;  // 发生变化
    }
    return false;  // 无变化
  }
  
  // 更新并检测推杆PWM变化
  bool updateActuatorPWM(uint8_t index, int newPWM) {
    if (index >= 2) return false;
    if (actuatorPWM[index] != newPWM) {
      actuatorPWM[index] = newPWM;
      return true;
    }
    return false;
  }
  
  // 更新并检测起落架PWM变化
  bool updateLandingGearPWM(int newPWM) {
    if (landingGearPWM != newPWM) {
      landingGearPWM = newPWM;
      return true;
    }
    return false;
  }
};

OutputStateTracker outputTracker;

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
  bool debugMode = true;                     //默认开启调试（硬件调试版本固定为true）
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
void safeServoWrite(uint8_t servoIndex, int value);  //舵机安全写入（带变化检测）
void safeServoWriteMicroseconds(uint8_t servoIndex, int value); //PWM舵机写入（带变化检测）
void printDebugInfo();                               //调试输出
void printOutputChange(const char* componentName, uint8_t pin, const char* valueType, int value); //输出变化打印

//===初始化===
void setup() {
  Serial.begin(9600);
  Serial.println("========================================");
  Serial.println("Arduino Drone Control V3");
  Serial.println("Hardware Debug Version");
  Serial.println("========================================");
  Serial.println("功能说明：");
  Serial.println("- 实时监测所有输出通道变化");
  Serial.println("- 任何舵机/推杆/起落架动作都会即时打印");
  Serial.println("- 用于上主供电前的信号传递验证");
  Serial.println("========================================\n");
  
  hardwareInit();
  
  Serial.println("\n[系统就绪] 开始监听控制信号...\n");
}

//===主循环===
void loop() {
  static uint32_t beforetime = 0;
  static uint32_t lastSignalPrint = 0;
  
  //主函数 - 50Hz更新
  if (millis() - beforetime >= 20) {
    SerialRead();  //串口优先级高
    
    if (SystemStatus.controller == SIGNAL_CT) {
      readControlSignals();  //读取遥控信号
    }
    
    updateSystemState();  //更新状态
    beforetime = millis();
  }
  
  //输入信号监测 - 每500ms打印一次（不需要太频繁）
  if (SystemStatus.controller == SIGNAL_CT && millis() - lastSignalPrint >= 500) {
    Serial.print("[输入信号] CH10:");
    Serial.print(filteredValues[0]);
    Serial.print(" | CH7:");
    Serial.print(filteredValues[1]);
    Serial.print(" | CH8:");
    Serial.print(filteredValues[2]);
    Serial.print(" | CH9:");
    Serial.print(filteredValues[3]);
    Serial.print(" | 锁定:");
    Serial.print(SystemStatus.STATE_LOCKED ? "是" : "否");
    Serial.print(" | 模式:");
    Serial.println(SystemStatus.system == STATE_FLIGHT_MODE ? "飞行" : "陆/水");
    lastSignalPrint = millis();
  }
}

//===串口命令处理===
void SerialRead() {
  if (Serial.available() > 0) {
    String received = Serial.readStringUntil('\n');
    received.trim();
    received.toLowerCase();
    Serial.print("\n[串口命令] 接收: ");
    Serial.println(received);
    executeCommand(received);
  }
}

//===更新状态机===
void executeCommand(const String& cmd) {
  //控制权限转让
  if (cmd == "ct_open") {
    SystemStatus.controller = SERIAL_CT;
    Serial.println("[OK] 已切换到串口控制模式");
  } else if (cmd == "ct_close") {
    SystemStatus.controller = SIGNAL_CT;
    Serial.println("[OK] 已切换到遥控器控制模式");
  }
  
  //指令执行
  if (SystemStatus.controller == SERIAL_CT) {
    if (cmd == "lock") {
      SystemStatus.STATE_LOCKED = true;
      Serial.println("[OK] 系统已锁定");
    } else if (cmd == "unlock") {
      SystemStatus.STATE_LOCKED = false;
      Serial.println("[OK] 系统已解锁");
    } else if (cmd == "mdchg") {
      manualModeChange();
      Serial.println("[OK] 模式切换指令已执行");
    } else if (cmd == "armex") {
      SystemStatus.arm = ARM_EXTENDING;
      Serial.println("[OK] 推杆伸出指令");
    } else if (cmd == "armre") {
      SystemStatus.arm = ARM_RETRACTING;
      Serial.println("[OK] 推杆缩回指令");
    } else if (cmd == "armst") {
      SystemStatus.arm = ARM_STOPPED;
      Serial.println("[OK] 推杆停止指令");
    } else if (cmd == "gearup") {
      SystemStatus.landingGear = GEAR_RETRACTED;
      Serial.println("[OK] 起落架收起指令");
    } else if (cmd == "geardown") {
      SystemStatus.landingGear = GEAR_EXTENDED;
      Serial.println("[OK] 起落架放下指令");
    } else if (cmd == "flight") {
      SystemStatus.system = STATE_FLIGHT_MODE;
      SystemStatus.activation = true;
      Serial.println("[OK] 飞行模式激活");
    } else if (cmd == "surface") {
      SystemStatus.system = STATE_SURFACE_MODE;
      SystemStatus.activation = true;
      Serial.println("[OK] 陆/水模式激活");
    } else {
      Serial.print("[ERROR] 未知命令: ");
      Serial.println(cmd);
    }
  } else {
    if (cmd != "ct_open" && cmd != "ct_close") {
      Serial.println("[WARN] 当前为遥控器控制模式，请先执行 ct_open");
    }
  }
}

//===更新模式状态机===
void manualModeChange() {
  if (!SystemStatus.STATE_LOCKED) {
    SystemStatus.activation = true;
    if (SystemStatus.system == STATE_FLIGHT_MODE) {
      SystemStatus.system = STATE_SURFACE_MODE;
      Serial.println("[模式切换] 飞行模式 → 陆/水模式");
    } else {
      SystemStatus.system = STATE_FLIGHT_MODE;
      Serial.println("[模式切换] 陆/水模式 → 飞行模式");
    }
  } else {
    Serial.println("[WARN] 系统已锁定，无法切换模式");
  }
}

void manualArmOperation() {
  int pwmValue = HW.pwm.min + HW.pwm.max / 2;  // 默认中值
  
  switch (SystemStatus.arm) {
    case ARM_EXTENDING:
      pwmValue = HW.pwm.max;
      break;
    case ARM_RETRACTING:
      pwmValue = HW.pwm.min;
      break;
    default:
      pwmValue = (HW.pwm.min + HW.pwm.max) / 2;
  }
  
  safeServoWriteMicroseconds(6, pwmValue);
  safeServoWriteMicroseconds(7, pwmValue);
}

void manualLandingGearControl() {
  int pwmValue;
  
  switch (SystemStatus.landingGear) {
    case GEAR_EXTENDED:
      pwmValue = HW.pwm.max;
      break;
    case GEAR_RETRACTED:
      pwmValue = HW.pwm.min;
      break;
  }
  
  safeServoWriteMicroseconds(8, pwmValue);
}

//===遥控信号处理===
void executeControlSignals() {
  // 遥控器锁定控制
  bool previousLockState = SystemStatus.STATE_LOCKED;
  
  if (filteredValues[0] < 1100) {
    SystemStatus.STATE_LOCKED = true;
  } else if(filteredValues[0] > 1900){
    SystemStatus.STATE_LOCKED = false;
  }
  
  // 锁定状态变化时打印
  if (previousLockState != SystemStatus.STATE_LOCKED) {
    Serial.println(SystemStatus.STATE_LOCKED ? 
      "\n[状态变化] 系统已锁定\n" : 
      "\n[状态变化] 系统已解锁\n");
  }
  
  // 模式切换
  if (!SystemStatus.STATE_LOCKED && SystemStatus.activation == false) {
    int modeSignal = filteredValues[1];
    SystemState previousMode = SystemStatus.system;
    
    if (modeSignal < 1100 && SystemStatus.system != STATE_FLIGHT_MODE) {
      SystemStatus.system = STATE_FLIGHT_MODE;
      SystemStatus.activation = true;
      Serial.println("\n[模式切换触发] 切换到飞行模式\n");
    } else if (modeSignal > 1900 && SystemStatus.system != STATE_SURFACE_MODE) {
      SystemStatus.system = STATE_SURFACE_MODE;
      SystemStatus.activation = true;
      Serial.println("\n[模式切换触发] 切换到陆/水模式\n");
    }
  }
  
  // 推杆控制
  int armSignal = filteredValues[2];
  ArmState previousArmState = SystemStatus.arm;
  
  if (armSignal > 1900) {
    SystemStatus.arm = ARM_EXTENDING;
  } else if (armSignal < 1100 && armSignal > 500) {
    SystemStatus.arm = ARM_RETRACTING;
  } else {
    SystemStatus.arm = ARM_STOPPED;
  }
  
  // 推杆状态变化时打印
  if (previousArmState != SystemStatus.arm && SystemStatus.arm != ARM_STOPPED) {
    Serial.print("\n[推杆状态] ");
    Serial.println(SystemStatus.arm == ARM_EXTENDING ? "伸出" : "缩回");
  }
  
  // 起落架控制
  int gearSignal = filteredValues[3];
  LandingGearState previousGearState = SystemStatus.landingGear;
  
  if (gearSignal > 1900) {
    SystemStatus.landingGear = GEAR_EXTENDED;
  } else if (gearSignal < 1100 && gearSignal > 500) {
    SystemStatus.landingGear = GEAR_RETRACTED;
  }
  
  // 起落架状态变化时打印
  if (previousGearState != SystemStatus.landingGear) {
    Serial.print("\n[起落架状态] ");
    Serial.println(SystemStatus.landingGear == GEAR_EXTENDED ? "放下" : "收起");
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
  const uint8_t totalSteps = 100;
  const uint8_t totalSteps2 = 90;

  if (transitionStep < totalSteps) {
    int baseAngle = map(transitionStep, 0, totalSteps, 20, 130);
    int armAngle = map(transitionStep, 0, totalSteps, 155, 45);
    safeServoWrite(0, baseAngle);
    safeServoWrite(1, armAngle);
    transitionStep++;
  } else {
    if (motorStep < totalSteps2) {
      int Angle1 = map(motorStep, 0, totalSteps2, 0, 90);
      int Angle2 = map(motorStep, 0, totalSteps2, 170, 80);
      safeServoWrite(2, Angle1);
      safeServoWrite(3, Angle1);
      safeServoWrite(4, Angle2);
      safeServoWrite(5, Angle2);
      motorStep++;
    } else {
      SystemStatus.system = STATE_FLIGHT_MODE;
      SystemStatus.activation = false;
      transitionStep = 0;
      motorStep = 0;
      Serial.println("\n[模式切换完成] 飞行模式已激活\n");
    }
  }
}

void transitionToSurfaceMode() {
  static uint8_t transitionStep = 0;
  static uint8_t motorStep = 0;
  const uint8_t totalSteps1 = 100;
  const uint8_t totalSteps2 = 90;

  if (transitionStep < totalSteps1) {
    int baseAngle = map(transitionStep, 0, totalSteps1, 130, 20);
    int armAngle = map(transitionStep, 0, totalSteps1, 45, 155);
    safeServoWrite(0, baseAngle);
    safeServoWrite(1, armAngle);
    transitionStep++;
  } else {
    if (motorStep < totalSteps2) {
      int Angle1 = map(motorStep, 0, totalSteps2, 90, 0);
      int Angle2 = map(motorStep, 0, totalSteps2, 80, 170);
      safeServoWrite(2, Angle1);
      safeServoWrite(3, Angle1);
      safeServoWrite(4, Angle2);
      safeServoWrite(5, Angle2);
      motorStep++;
    } else {
      SystemStatus.system = STATE_SURFACE_MODE;
      SystemStatus.activation = false;
      transitionStep = 0;
      motorStep = 0;
      Serial.println("\n[模式切换完成] 陆/水模式已激活\n");
    }
  }
}

//===安全舵机控制（带变化检测和即时打印）===
void safeServoWrite(uint8_t servoIndex, int value) {
  value = constrain(value, 0, 180);
  
  if (servoIndex < 6 && servos[servoIndex].attached()) {
    // 检测是否发生变化
    if (outputTracker.updateServoAngle(servoIndex, value)) {
      // 发生变化，打印输出
      const char* servoNames[] = {
        "电机臂舵机",      // servo0
        "电机方向舵机",    // servo1
        "辅助舵机1",       // servo2
        "辅助舵机2",       // servo3
        "辅助舵机3",       // servo4
        "辅助舵机4"        // servo5
      };
      
      uint8_t pins[] = {11, 10, 4, 5, 6, 7};
      
      printOutputChange(servoNames[servoIndex], pins[servoIndex], "角度", value);
      servos[servoIndex].write(value);
    }
  }
}

void safeServoWriteMicroseconds(uint8_t servoIndex, int value) {
  value = constrain(value, HW.pwm.min, HW.pwm.max);
  
  if (servoIndex >= 6 && servoIndex <= 8 && servos[servoIndex].attached()) {
    bool changed = false;
    
    if (servoIndex == 6 || servoIndex == 7) {
      // 推杆控制
      changed = outputTracker.updateActuatorPWM(servoIndex - 6, value);
    } else if (servoIndex == 8) {
      // 起落架控制
      changed = outputTracker.updateLandingGearPWM(value);
    }
    
    if (changed) {
      const char* componentNames[] = {"电推杆1", "电推杆2", "起落架舵机"};
      uint8_t pins[] = {8, 9, 12};
      int nameIndex = servoIndex - 6;
      
      printOutputChange(componentNames[nameIndex], pins[nameIndex], "PWM", value);
      servos[servoIndex].writeMicroseconds(value);
    }
  }
}

//===输出变化打印函数===
void printOutputChange(const char* componentName, uint8_t pin, const char* valueType, int value) {
  Serial.print("【输出变化】");
  Serial.print(componentName);
  Serial.print(" - ");
  Serial.print(pin);
  Serial.print("引脚 - ");
  Serial.print(valueType);
  Serial.print(": ");
  Serial.println(value);
}

//===硬件初始化===
void hardwareInit() {
  Serial.println("[硬件初始化] 开始...");
  
  // 初始化输入引脚
  pinMode(HW.inputPins.modeLock, INPUT);
  pinMode(HW.inputPins.modeSwitch, INPUT);
  pinMode(HW.inputPins.armControl, INPUT);
  pinMode(HW.inputPins.landingGearControl, INPUT);
  Serial.println("[硬件初始化] 输入引脚配置完成");

  // 初始化舵机
  const uint8_t servoPins[] = {
    HW.outputPins.servos.baseServos[0],    //0 - 电机臂舵机
    HW.outputPins.servos.baseServos[1],    //1 - 电机方向舵机
    HW.outputPins.servos.armServos[0],     //2 - 辅助舵机1
    HW.outputPins.servos.armServos[1],     //3 - 辅助舵机2
    HW.outputPins.servos.armServos[2],     //4 - 辅助舵机3
    HW.outputPins.servos.armServos[3],     //5 - 辅助舵机4
    HW.outputPins.servos.armActuators[0],  //6 - 电推杆1
    HW.outputPins.servos.armActuators[1],  //7 - 电推杆2
    HW.outputPins.servos.landingGear       //8 - 起落架舵机
  };

  for (int i = 0; i < 9; i++) {
    if (servoPins[i] != 0) {
      servos[i].attach(servoPins[i]);
      Serial.print("[硬件初始化] Servo");
      Serial.print(i);
      Serial.print(" 连接到引脚 ");
      Serial.println(servoPins[i]);
    }
  }
  
  Serial.println("\n[初始位置设置] 开始设置初始位置（飞行模式）...\n");
  
  // 设置初始位置（飞行模式）
  safeServoWrite(0, 130);  //电机臂朝上
  safeServoWrite(1, 45);   //电机方向向下
  safeServoWrite(2, 90);   //辅助舵机初始位置
  safeServoWrite(3, 90);
  safeServoWrite(4, 80);
  safeServoWrite(5, 80);
  safeServoWriteMicroseconds(6, (HW.pwm.min + HW.pwm.max) / 2);  //推杆1中位
  safeServoWriteMicroseconds(7, (HW.pwm.min + HW.pwm.max) / 2);  //推杆2中位
  safeServoWriteMicroseconds(8, HW.pwm.min);                     //起落架收起
  
  //状态机初始化
  SystemStatus.system = STATE_FLIGHT_MODE;
  SystemStatus.STATE_LOCKED = true;
  SystemStatus.arm = ARM_STOPPED;
  SystemStatus.landingGear = GEAR_RETRACTED;
  
  Serial.println("\n[硬件初始化] 完成");
  Serial.println("[系统状态] 飞行模式 | 已锁定 | 遥控器控制\n");
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
    if (rawSignals[i] > 500) {  // 有效信号检测
      rawSignals[i] = constrain(rawSignals[i], HW.pwm.min, HW.pwm.max);
    }
    filteredValues[i] = inputFilters[i].filter(rawSignals[i]);
  }
  
  executeControlSignals();
}
