# WuK 串口通信调试清单
**更新时间**: 2026-01-24  
**目标**: 诊断 ArduPilot → Arduino Mega2560 串口通信问题

---

## 📋 修改内容总结

### ArduPilot 端 (WuK_Comms.cpp)
✅ **降低心跳频率**: 200ms → **1000ms** (每秒1次)  
✅ **添加初始化日志**: 验证串口打开状态  
✅ **添加发送队列日志**: 跟踪每个命令入队和发送  
✅ **添加底层字节日志**: 显示实际发送的帧数据和CRC

### Arduino 端 (arduino.cpp)
✅ **增强启动信息**: 显示配置参数（波特率、协议头）  
✅ **添加接收监控**: 每秒报告缓冲区状态和接收速率  
✅ **添加状态机调试**: 显示每个解析步骤（HEADER/MsgID/Len/Payload/CRC）  
✅ **添加原始数据采样**: 每100字节输出最近10字节的HEX数据  
✅ **增加缓冲区溢出检测**: 警告 >50 字节的缓冲积压

---

## 🔍 验证步骤

### 第一步：编译并上传代码

**ArduPilot 端**:
```bash
cd ~/ardupilot/ArduCopter
./waf copter
# 上传到飞控板
```

**Arduino 端**:
1. 打开 Arduino IDE
2. 加载 `arduino.cpp`
3. 选择 **Arduino Mega 2560** 板型
4. 选择正确的 COM 口
5. 上传代码

---

### 第二步：检查 Arduino 启动信息

打开 Arduino **串口监视器** (9600 波特率)，应看到：

```
======================================
[WuK] Arduino 从机已初始化 (v2.0)
======================================
[Config] RX Pin: 15 (Serial3)
[Config] Baud Rate: 9600
[Config] Protocol: Header=0xAA CRC8-Maxim
[Status] 等待飞控指令...
======================================
```

✅ **确认**: 如果看到以上信息，Arduino 程序正常运行

---

### 第三步：检查 ArduPilot 初始化日志

连接地面站（Mission Planner / QGC），查看 **Messages** 面板，应看到：

```
[WuK] Initializing UART port=4 baud=9600
[WuK] UART4 init SUCCESS @ 9600 baud
```

❌ **如果看到错误**:
- `UART4 init FAILED - serial() returned nullptr` → 检查飞控板是否真的有 UART4
- `Invalid port number` → 检查 `WUK_PORT` 参数设置

💡 **验证参数**:
```
WUK_PORT = 4     (使用 UART4)
WUK_BAUD = 9600  (与 Arduino 一致)
```

---

### 第四步：监控发送报文

**ArduPilot 地面站日志**（每秒应看到类似内容）：

```
[WuK-TX] Heartbeat enqueued at 12345 ms
[WuK-TX] Sending frame #1: MsgID=0x04 Val=0
[WuK-LOW] Frame: [0xAA 04 02 00 00 ...]
[WuK-LOW] Wrote 6 bytes (expected 6), CRC=0x??
```

✅ **关键信息**:
- `Heartbeat enqueued` → 心跳定时器工作
- `Sending frame` → 队列处理正常
- `Wrote 6 bytes` → 实际发送到 UART TX

❌ **异常情况**:
- `Heartbeat queue FULL!` → 队列溢出（发送失败）
- `TX buffer full` → UART TX 缓冲区满

---

### 第五步：监控 Arduino 接收状态

**Arduino 串口监视器**（每秒输出）：

#### 情况 A：正常接收
```
[RX-Monitor] Buffer: 0 bytes | RX Rate: 6 B/s | State: WAIT_HEADER
[Parse] Found HEADER 0xAA
[Parse] MsgID: 0x04
[Parse] Len: 2
[Parse] Payload complete (2 bytes)
[Parse] CRC recv=0x?? calc=0x??
[SUCCESS] === Valid Frame Received ===
报文：ID:0x04 Val:0
```
✅ **结论**: 通信正常！

#### 情况 B：收不到任何数据
```
[RX-Monitor] Buffer: 0 bytes | RX Rate: 0 B/s | State: WAIT_HEADER
[STATUS] Mode: FLIGHT | Arm:0.0° Motor:0.0°
[COMM] Last second: NO VALID FRAME (Total RX: 0 bytes)
```
❌ **问题**: 物理连接有问题或飞控未发送

**排查**:
1. 检查接线：飞控 UART4 TX → Arduino Pin 15 (Serial3 RX)
2. 确认飞控 UART4 TX 引脚位置（参考飞控文档）
3. 使用万用表测试 TX 引脚是否有电平变化
4. 使用逻辑分析仪抓取波形（如果有设备）

#### 情况 C：收到数据但无法解析
```
[RX-Monitor] Buffer: 5 bytes | RX Rate: 120 B/s | State: WAIT_HEADER
[RAW-100] Last 10 bytes: FE 09 00 01 01 00 00 C8 00 00
[Parse] Non-header byte: 0xFE
```
❌ **问题**: 收到的不是 WuK 协议数据

**排查**:
1. 如果看到 `0xFE` → 可能是 **MAVLink 数据**！
   - 说明飞控在该串口上输出了 MAVLink 而不是 WuK 协议
   - 检查地面站参数：`SERIALx_PROTOCOL` 是否配置为 MAVLink（1）
   - 需要禁用该串口的 MAVLink 输出

2. 如果看到其他杂乱数据 → 波特率不匹配
   - 再次确认双方都是 **9600**
   - 重新上传 Arduino 程序

#### 情况 D：收到 0xAA 但 CRC 错误
```
[Parse] Found HEADER 0xAA
[Parse] MsgID: 0x04
[Parse] Len: 2
[Parse] Payload complete (2 bytes)
[Parse] CRC recv=0x3A calc=0x5B
[Parse-ERR] CRC mismatch!
```
❌ **问题**: 帧头正确但 CRC 校验失败

**排查**:
1. 检查 CRC 算法是否完全一致
2. 检查大小端字节序（Payload 中的 `uint16_t`）
3. 可能有电气干扰导致数据损坏

---

## 🛠️ 高级调试：逻辑分析仪验证

如果有逻辑分析仪/示波器：

1. **连接探头**:
   - CH1 → 飞控 UART4 TX
   - CH2 → Arduino Pin 15 (RX)
   - GND → 公共地

2. **触发设置**:
   - 波特率: 9600
   - 数据格式: 8N1 (8位数据, 无校验, 1位停止位)
   - 触发条件: 检测 `0xAA` 字节

3. **验证内容**:
   - 飞控 TX 是否有周期性的波形（1秒一次）
   - Arduino RX 是否能接收到相同波形
   - 两者波形是否完全一致

---

## 📝 常见问题排查表

| 症状 | 可能原因 | 解决方案 |
|------|---------|---------|
| ArduPilot 日志无 WuK 信息 | 未调用 `wuk_comms.init()` | 检查 `system.cpp` 是否有 `wuk_comms.init()` |
| `UART4 init FAILED` | 端口号错误或硬件不支持 | 修改 `WUK_PORT` 参数（尝试 0-6） |
| Arduino 收不到任何数据 | 物理连接断开 | 检查杜邦线、焊点、引脚定义 |
| Arduino 收到 0xFE 开头数据 | 串口被 MAVLink 占用 | 设置 `SERIAL4_PROTOCOL = -1` (禁用) |
| CRC 一直错误 | CRC 算法不一致 | 比对两端 `crc8_update()` 实现 |
| 缓冲区溢出警告 | Arduino 处理太慢 | 降低发送频率或优化 `loop()` |

---

## ✅ 成功标志

当看到以下内容时，通信已完全正常：

**ArduPilot 地面站**:
```
[WuK-TX] Sending frame #100: MsgID=0x04 Val=0
```

**Arduino 串口监视器**:
```
[SUCCESS] === Valid Frame Received ===
报文：ID:0x04 Val:0
[STATUS] Mode: FLIGHT | Arm:0.0° Motor:0.0°
[COMM] Last second: FRAME RECEIVED OK
```

---

## 📧 问题反馈

如果问题仍未解决，请记录以下信息：

1. **ArduPilot 地面站完整日志**（从启动到运行5秒）
2. **Arduino 串口监视器完整输出**（运行10秒）
3. **参数设置截图**:
   - `WUK_PORT`
   - `WUK_BAUD`
   - `SERIAL4_PROTOCOL`
4. **飞控型号和接线照片**

---

**祝调试顺利！🚁**
