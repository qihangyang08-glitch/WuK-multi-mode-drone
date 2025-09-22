#include <Servo.h>

// 创建舵机对象
Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9;

// 定义输入引脚
const int pwmInput1 = 45;  // 信号1输入引脚 模式锁定 10通道 SWA
const int pwmInput2 = 2;   // 信号2输入引脚 模式切换 7通道  SWD
const int pwmInput3 = 3;   // 信号3输入引脚 推杆控制 8通道  SWE
const int pwmInput4 = 13;  // 信号4输入引脚 起落架控制 9通道 SWC

// 系统状态标志
bool Lock = 0;             // 解锁标志
bool Mode = 1;             // 模式标志 (1=飞行模式, 0=陆/水模式)
short pusher_ctrl = 0;     // 推杆控制状态 (0=停止, 1=伸长, 2=收缩)

// 舵机输出引脚定义
const int servoPin1 = 11;  // 电机臂舵机1
const int servoPin2 = 10;  // 电机臂舵机2 
const int servoPin3 = 4;   // 预留舵机3
const int servoPin4 = 5;   // 预留舵机4
const int servoPin5 = 6;   // 预留舵机5
const int servoPin6 = 7;   // 预留舵机6
const int servoPin7 = 8;   // 推杆控制舵机1
const int servoPin8 = 9;   // 推杆控制舵机2
const int servoPin9 = 12;  // 起落架控制舵机

// 舵机角度参数
int pre_angle1 = 90;  // 预设角度1
int pre_angle2 = 80;  // 预设角度2
int delta_angle = 90; // 角度变化量

// PWM信号范围
const int pwmMin = 1000;  // PWM最小值
const int pwmMax = 2000;  // PWM最大值

// 滤波参数
#define FILTER_SIZE 5           // 滤波窗口大小
int pwmBuffer[4][FILTER_SIZE];  // 存储每个通道的PWM信号
int bufferIndex[4] = {0};       // 每个通道当前索引位置

// 起落架状态
bool landing_gear_extended = false;  // 起落架状态标志

void setup() {
  // 初始化舵机连接
  servo1.attach(servoPin1);  // 电机臂舵机1
  servo2.attach(servoPin2);  // 电机臂舵机2
  servo3.attach(servoPin3);  // 预留舵机
  servo4.attach(servoPin4);  // 预留舵机
  servo5.attach(servoPin5);  // 预留舵机
  servo6.attach(servoPin6);  // 预留舵机
  servo7.attach(servoPin7);  // 推杆控制1
  servo8.attach(servoPin8);  // 推杆控制2
  servo9.attach(servoPin9);  // 起落架控制
  
  // 初始化输入引脚
  pinMode(pwmInput1, INPUT);
  pinMode(pwmInput2, INPUT);
  pinMode(pwmInput3, INPUT);
  pinMode(pwmInput4, INPUT);
  
  // 舵机初始化到安全位置
  servo1.write(45);           // 电机臂初始位置
  servo2.write(130);          // 电机臂初始位置
  servo3.write(pre_angle1);   // 预留舵机初始位置
  servo4.write(pre_angle1);   // 预留舵机初始位置
  servo5.write(pre_angle2);   // 预留舵机初始位置
  servo6.write(pre_angle2);   // 预留舵机初始位置
  servo7.writeMicroseconds(1500);  // 推杆中位
  servo8.writeMicroseconds(1500);  // 推杆中位
  servo9.writeMicroseconds(1500);  // 起落架中位
  
  // 初始化滤波缓冲区
  for(int ch = 0; ch < 4; ch++) {
    for(int i = 0; i < FILTER_SIZE; i++) {
      pwmBuffer[ch][i] = 1500;  // 初始化为中位值
    }
  }
  
  // 串口初始化
  Serial.begin(9600);
  Serial.println("跨介质无人机变形控制系统 v2.0 启动");
  Serial.println("CH1=模式锁定, CH2=模式切换, CH3=推杆控制, CH4=起落架控制");
  
  delay(1000);  // 等待系统稳定
}

// PWM信号滤波函数
int filterSignal(int channel, int pwmValue) {
  // 将新的PWM值存入缓冲区
  pwmBuffer[channel][bufferIndex[channel]] = pwmValue;
  bufferIndex[channel] = (bufferIndex[channel] + 1) % FILTER_SIZE;
  
  // 计算滤波后的平均值
  int sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += pwmBuffer[channel][i];
  }
  return sum / FILTER_SIZE;
}

// PWM信号范围规范化函数
int normalizeSignal(int signal) {
  if (signal < 1000) signal = 1000;
  if (signal > 1900) signal = 2000;
  return signal;
}

// 模式切换到飞行模式
void switchToFlightMode() {
  Serial.println("切换到飞行模式...");
  
  // 电机臂缓慢转动到飞行位置
  for (int i = 0; i <= 110; i++) {
    servo1.write(155 - i);  // servo1从155度转到45度
    servo2.write(20 + i);   // servo2从20度转到130度
    delay(50);              // 缓慢转动，避免冲击
  }
  
  // 预留舵机回到初始位置
  for (int i = 0; i <= delta_angle; i++) {
    servo3.write(i);
    servo4.write(i);
    servo5.write(170 - i);
    servo6.write(170 - i);
    delay(50);
  }
  
  Serial.println("飞行模式切换完成");
}

// 模式切换到陆地/水面模式
void switchToLandWaterMode() {
  Serial.println("切换到陆地/水面模式...");
  
  // 电机臂缓慢转动到水平位置
  for (int i = 0; i <= 110; i++) {
    servo1.write(45 + i);   // servo1从45度转到155度
    servo2.write(130 - i);  // servo2从130度转到20度
    delay(50);              // 缓慢转动，避免冲击
  }
  
  // 预留舵机转动到工作位置
  for (int i = 0; i <= delta_angle; i++) {
    servo3.write(90 - i);   // 转到0度
    servo4.write(90 - i);   // 转到0度
    servo5.write(80 + i);   // 转到170度
    servo6.write(80 + i);   // 转到170度
    delay(50);
  }
  
  Serial.println("陆地/水面模式切换完成");
}

// 起落架控制函数
void controlLandingGear(int signal) {
  if (signal < 1200 && landing_gear_extended) {
    // 收起起落架
    Serial.println("收起起落架");
    servo9.writeMicroseconds(1000);  // 收起位置
    landing_gear_extended = false;
  } 
  else if (signal > 1800 && !landing_gear_extended) {
    // 放下起落架
    Serial.println("放下起落架");
    servo9.writeMicroseconds(2000);  // 放下位置
    landing_gear_extended = true;
  } 
  else if (signal >= 1200 && signal <= 1800) {
    // 中位保持当前状态
    servo9.writeMicroseconds(1500);
  }
}

// 推杆控制函数
void controlPusher(int signal) {
  if (signal < 1100) {
    pusher_ctrl = 2;  // 推杆收缩
    servo7.writeMicroseconds(1000);
    servo8.writeMicroseconds(1000);
    Serial.println("推杆收缩");
  } 
  else if (signal > 1900) {
    pusher_ctrl = 1;  // 推杆伸长
    servo7.writeMicroseconds(2000);
    servo8.writeMicroseconds(2000);
    Serial.println("推杆伸长");
  } 
  else {
    pusher_ctrl = 0;  // 推杆停止
    servo7.writeMicroseconds(1500);
    servo8.writeMicroseconds(1500);
  }
}

void loop() {
  // 读取四路PWM信号
  long signal1 = pulseIn(pwmInput1, HIGH, 25000);  // 模式锁定
  long signal2 = pulseIn(pwmInput2, HIGH, 25000);  // 模式切换
  long signal3 = pulseIn(pwmInput3, HIGH, 25000);  // 推杆控制
  long signal4 = pulseIn(pwmInput4, HIGH, 25000);  // 起落架控制
  
  // 信号范围规范化
  signal1 = normalizeSignal(signal1);
  signal2 = normalizeSignal(signal2);
  signal3 = normalizeSignal(signal3);
  signal4 = normalizeSignal(signal4);
  
  // 对信号进行滤波处理
  int filteredValue1 = filterSignal(0, signal1);  // 模式锁定信号
  int filteredValue2 = filterSignal(1, signal2);  // 模式切换信号
  int filteredValue3 = filterSignal(2, signal3);  // 推杆控制信号
  int filteredValue4 = filterSignal(3, signal4);  // 起落架控制信号
  
  // 信号1 - 模式锁定控制
  if (filteredValue1 > 1900) {
    Lock = 1;  // 解锁模式切换
  } else if (filteredValue1 < 1100) {
    Lock = 0;  // 锁定模式切换
  }
  
  // 信号2 - 模式切换控制（仅在解锁状态下有效）
  if (Lock == 1) {
    if (filteredValue2 > 1900 && Mode == 0) {
      // 切换到飞行模式
      Mode = 1;
      switchToFlightMode();
    } 
    else if (filteredValue2 < 1100 && Mode == 1) {
      // 切换到陆地/水面模式
      Mode = 0;
      switchToLandWaterMode();
    }
  }
  
  // 信号3 - 推杆控制（独立于模式切换）
  controlPusher(filteredValue3);
  
  // 信号4 - 起落架控制（独立于模式切换）
  controlLandingGear(filteredValue4);
  
  // 调试信息输出
  Serial.print("Lock:");
  Serial.print(Lock);
  Serial.print(" Mode:");
  Serial.print(Mode);
  Serial.print(" Pusher:");
  Serial.print(pusher_ctrl);
  Serial.print(" LandingGear:");
  Serial.print(landing_gear_extended);
  Serial.print(" | PWM-> CH1:");
  Serial.print(filteredValue1);
  Serial.print(" CH2:");
  Serial.print(filteredValue2);
  Serial.print(" CH3:");
  Serial.print(filteredValue3);
  Serial.print(" CH4:");
  Serial.println(filteredValue4);
  
  delay(20);  // 主循环延时，避免过于频繁的处理
}
