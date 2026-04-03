// [WuK-Comms] 串口通信类 - 与Arduino控制板通信
// 功能: 发送机臂舵机角度命令(arm:X°), 接收状态反馈
// 协议: 0xAA [MsgID] [Len] [Payload] [CRC8]
// 改动位置: 新增文件

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class WuK_Comms {
public:
    WuK_Comms();

    void init();
    void update();

    // Command IDs
    enum class CmdID : uint8_t {
        ARM_SERVO = 0x01,
        ACT1 = 0x02,
        ACT2 = 0x03,
        STATUS_REQ = 0x04
    };

    // Response IDs
    enum class RspID : uint8_t {
        ACK = 0x80,
        STATUS = 0x81
    };

    // Enqueue a command to be sent
    bool enqueue_cmd(CmdID cmd_id, uint16_t value);

    // Parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_HAL::UARTDriver *_uart;
    
    // Parameters
    AP_Int8 _port;
    AP_Int32 _baud;

    // Protocol constants
    static const uint8_t HEADER_BYTE = 0xAA;
    static const uint8_t MAX_PAYLOAD_LEN = 16;
    static const uint8_t FRAME_OVERHEAD = 4; // Header, MsgID, Len, CRC

    // Ring buffer for outgoing commands
    struct QueueItem {
        CmdID cmd_id;
        uint16_t value;
    };
    static const uint8_t QUEUE_SIZE = 16;
    QueueItem _queue[QUEUE_SIZE];
    uint8_t _head;
    uint8_t _tail;

    // RX State machine
    enum class RxState {
        WAIT_HEADER,
        WAIT_MSG_ID,
        WAIT_LEN,
        WAIT_PAYLOAD,
        WAIT_CRC
    } _rx_state;

    uint8_t _rx_msg_id;
    uint8_t _rx_len;
    uint8_t _rx_payload[MAX_PAYLOAD_LEN];
    uint8_t _rx_payload_idx;
    uint8_t _rx_crc;

    // Helper functions
    void service_tx();
    void service_rx();
    void process_frame();
    uint8_t crc8_update(uint8_t crc, uint8_t data);
    void send_frame(uint8_t msg_id, const uint8_t* payload, uint8_t len);

    uint32_t _last_heartbeat_ms;
};
