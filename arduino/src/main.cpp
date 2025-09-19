#include <Servo.h>
#include <Arduino.h> 

// =================== 舵机对象 ===================
Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9;

// =================== 输入引脚 ===================
// （保持原有映射）
const int pwmInput1 = 45;  // 通道10 -> 模式锁定
const int pwmInput2 = 2;   // 通道7  -> 模式切换
const int pwmInput3 = 3;   // 通道8  -> 电推杆手动
const int pwmInput4 = 13;  // 通道9  -> 预留（现在不用）

// =================== 输出引脚 ===================
const int servoPin1 = 11; // 摇臂舵机1
const int servoPin2 = 10; // 摇臂舵机2
const int servoPin3 = 4;  // 电机舵机A
const int servoPin4 = 5;  // 电机舵机B
const int servoPin5 = 6;  // 电机舵机C
const int servoPin6 = 7;  // 电机舵机D
const int servoPin7 = 8;  // 推杆控制1
const int servoPin8 = 9;  // 推杆控制2
const int servoPin9 = 12; // 起落架舵机（替换气泵）

// =================== 状态变量 ===================
bool Lock = 0;         // 解锁标志
int Mode = 2;          // 默认开机地面模式 (2=GROUND)
short Spl_agl = 0;     // 保留原有摇臂张角状态

// =================== 参数化设置 ===================
// （保留原始数值作为默认值）
int pre_angle1 = 90;
int pre_angle2 = 80;
int delta_angle = 90;

int servoDelay    = 50;    // 舵机转动延时 (ms)
int pushrodTime   = 2000;  // 推杆动作时间 (ms)
int gearWaitTime  = 2000;  // 起落架收放等待时间 (ms)

// =================== PWM滤波 ===================
#define FILTER_SIZE 5
int pwmBuffer[4][FILTER_SIZE];
int bufferIndex[4] = {0};

int filterSignal(int channel, int pwmValue) {
  pwmBuffer[channel][bufferIndex[channel]] = pwmValue;
  bufferIndex[channel] = (bufferIndex[channel] + 1) % FILTER_SIZE;

  long sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += pwmBuffer[channel][i];
  }
  return sum / FILTER_SIZE;
}

// =================== 初始化 ===================
void setup() {
  // 舵机 attach
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);
  servo7.attach(servoPin7);
  servo8.attach(servoPin8);
  servo9.attach(servoPin9);

  // 输入
  pinMode(pwmInput1, INPUT);
  pinMode(pwmInput2, INPUT);
  pinMode(pwmInput3, INPUT);
  pinMode(pwmInput4, INPUT);

  // 舵机初始位置（保持原有）
  servo1.write(45);
  servo2.write(130);
  servo3.write(pre_angle1);
  servo4.write(pre_angle1);
  servo5.write(pre_angle2);
  servo6.write(pre_angle2);
  servo7.writeMicroseconds(1500);
  servo8.writeMicroseconds(1500);
  servo9.writeMicroseconds(1500);

  Serial.begin(9600);
}

// =================== 辅助函数 ===================
long readPWM(int pin, int channel) {
  long v = pulseIn(pin, HIGH, 25000);
  if (v < 1000) v = 1000;
  if (v > 2000) v = 2000;
  return filterSignal(channel, v);
}

// 推杆动作
void pushrodExtend() {
  servo7.writeMicroseconds(2000);
  servo8.writeMicroseconds(2000);
  delay(pushrodTime);
  servo7.writeMicroseconds(1500);
  servo8.writeMicroseconds(1500);
}
void pushrodRetract() {
  servo7.writeMicroseconds(1000);
  servo8.writeMicroseconds(1000);
  delay(pushrodTime);
  servo7.writeMicroseconds(1500);
  servo8.writeMicroseconds(1500);
}

// 起落架动作
void gearDown() {
  servo9.writeMicroseconds(2000);
  delay(gearWaitTime);
}
void gearUp() {
  servo9.writeMicroseconds(1000);
  delay(gearWaitTime);
}

// =================== 模式切换函数 ===================
#define MODE_FLIGHT 0
#define MODE_GROUND 1
#define MODE_WATER  2

void switchToMode(int target) {
  if (target == Mode) return; // 已经在此模式

  if (target == MODE_GROUND) {
    // 飞行/水面 -> 地面
    gearDown();
    pushrodExtend();
    for (int i = 0; i <= delta_angle; i++) {
      servo3.write(i);
      servo4.write(i);
      servo5.write(170 - i);
      servo6.write(170 - i);
      delay(servoDelay);
    }
    pushrodRetract();
    gearUp();
  }
  else if (target == MODE_FLIGHT) {
    // 地面/水面 -> 飞行
    pushrodRetract();
    gearDown();
    pushrodExtend();
    for (int i = 0; i <= delta_angle; i++) {
      servo3.write(90 - i);
      servo4.write(90 - i);
      servo5.write(80 + i);
      servo6.write(80 + i);
      delay(servoDelay);
    }
    gearUp();
  }
  else if (target == MODE_WATER) {
    // 飞行/地面 -> 水面
    for (int i = 0; i <= delta_angle; i++) {
      servo3.write(i);
      servo4.write(i);
      servo5.write(170 - i);
      servo6.write(170 - i);
      delay(servoDelay);
    }
  }

  Mode = target;
  Serial.print("Switched to mode: ");
  Serial.println(Mode);
}

// =================== 主循环 ===================
void loop() {
  long signal1 = readPWM(pwmInput1, 0); // Lock
  long signal2 = readPWM(pwmInput2, 1); // Mode
  long signal3 = readPWM(pwmInput3, 2); // Pushrod
  long signal4 = readPWM(pwmInput4, 3); // 预留

  // 信号一 -> 模式锁定
  if (signal1 > 1900) Lock = 1;
  else if (signal1 < 1100) Lock = 0;

  // 信号二 -> 模式切换
  if (!Lock) {
    if (signal2 > 1800) switchToMode(MODE_GROUND);
    else if (signal2 >= 1300 && signal2 <= 1700) switchToMode(MODE_WATER);
    else if (signal2 < 1200) switchToMode(MODE_FLIGHT);
  }

  // 信号三 -> 电推杆手动控制
  if (signal3 < 1100) pushrodRetract();
  else if (signal3 > 1900) pushrodExtend();
  else {
    servo7.writeMicroseconds(1500);
    servo8.writeMicroseconds(1500);
  }

  // 串口调试输出
  Serial.print("Lock: "); Serial.print(Lock);
  Serial.print(" Mode: "); Serial.println(Mode);
}
