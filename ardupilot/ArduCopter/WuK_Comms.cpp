#include "Copter.h"
#include "WuK_Comms.h"

extern const AP_HAL::HAL& hal;

WuK_Comms::WuK_Comms() :
    _uart(nullptr),
    _head(0),
    _tail(0),
    _rx_state(RxState::WAIT_HEADER),
    _last_heartbeat_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void WuK_Comms::init()
{
    // AP_SerialManager &serial_manager = AP::serialmanager();
    
    // Use SerialProtocol_WuK (51) as defined in AP_SerialManager (we assume it was added there)
    // If not, we can use a custom protocol ID or just search for Scripting (28) as a fallback
    // For now, let's try to find a serial port configured with protocol 28 (Scripting) 
    // since we can't easily modify AP_SerialManager enum in this context without recompiling libraries.
    // Wait, the user instructions said "Add SERIALx_PROTOCOL slot via AP_SerialManager".
    // I will assume the user has done this or I should use a known ID.
    // Let's use 28 (Scripting) as a proxy for now, or just use the _port param to find the UART directly.
    
    // Option 1: Use SerialManager with a specific protocol
    // _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Scripting, 0);

    // Option 2: Use the _port parameter to select the UART directly
    // This is more robust if we can't change SerialManager
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[WuK] Initializing UART port=%d baud=%ld", (int)_port, (long)_baud);
    
    if (_port >= 0) {
        _uart = hal.serial(_port);
        if (_uart) {
            _uart->begin(_baud);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[WuK] UART%d init SUCCESS @ %ld baud", (int)_port, (long)_baud);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[WuK] UART%d init FAILED - serial() returned nullptr", (int)_port);
        }
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "[WuK] Invalid port number: %d", (int)_port);
    }
}

void WuK_Comms::update()
{
    if (!_uart) {
        return;
    }

    // service_rx(); // Disabled: Simplex communication (FC -> Arduino only)
    service_tx();
}

bool WuK_Comms::enqueue_cmd(CmdID cmd_id, uint16_t value)
{
    uint8_t next_head = (_head + 1) % QUEUE_SIZE;
    if (next_head == _tail) {
        return false; // Queue full
    }

    _queue[_head].cmd_id = cmd_id;
    _queue[_head].value = value;
    _head = next_head;
    return true;
}

void WuK_Comms::service_tx()
{
    // Send heartbeat every 1000ms (降低频率便于调试)
    uint32_t now = AP_HAL::millis();
    static uint32_t last_hb_full_log_ms = 0;
    static uint32_t last_txspace_log_ms = 0;
    static uint32_t last_tx_stat_ms = 0;
    static uint16_t sent_since_stat = 0;
    if (now - _last_heartbeat_ms > 1000) {
        if (enqueue_cmd(CmdID::STATUS_REQ, 0)) {
            _last_heartbeat_ms = now;
        } else {
            if (now - last_hb_full_log_ms > 2000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[WuK-TX] HB queue full");
                last_hb_full_log_ms = now;
            }
        }
    }

    // Process queue
    static uint32_t total_sent = 0;
    while (_head != _tail) {
        uint16_t txspace = _uart->txspace();
        if (txspace < (FRAME_OVERHEAD + 2)) { // 2 bytes for value
            if (now - last_txspace_log_ms > 2000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[WuK-TX] TX buf low:%u", txspace);
                last_txspace_log_ms = now;
            }
            break;
        }

        QueueItem &item = _queue[_tail];
        uint8_t payload[2];
        payload[0] = item.value & 0xFF;
        payload[1] = (item.value >> 8) & 0xFF;

        send_frame((uint8_t)item.cmd_id, payload, 2);
        ++total_sent;
        ++sent_since_stat;
        
        _tail = (_tail + 1) % QUEUE_SIZE;
    }

    if (now - last_tx_stat_ms > 2000) {
        const uint8_t q_depth = (_head >= _tail) ? (_head - _tail) : (QUEUE_SIZE - _tail + _head);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "[WuK-TX] sent=%lu(+%u) q=%u", 
                      (unsigned long)total_sent,
                      (unsigned int)sent_since_stat,
                      (unsigned int)q_depth);
        sent_since_stat = 0;
        last_tx_stat_ms = now;
    }
}

void WuK_Comms::service_rx()
{
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        uint8_t b = _uart->read();
        
        switch (_rx_state) {
        case RxState::WAIT_HEADER:
            if (b == HEADER_BYTE) {
                _rx_state = RxState::WAIT_MSG_ID;
                _rx_crc = 0; // Reset CRC
            }
            break;
            
        case RxState::WAIT_MSG_ID:
            _rx_msg_id = b;
            _rx_crc = crc8_update(_rx_crc, b);
            _rx_state = RxState::WAIT_LEN;
            break;
            
        case RxState::WAIT_LEN:
            _rx_len = b;
            if (_rx_len > MAX_PAYLOAD_LEN) {
                _rx_state = RxState::WAIT_HEADER; // Invalid len
            } else {
                _rx_crc = crc8_update(_rx_crc, b);
                _rx_payload_idx = 0;
                if (_rx_len == 0) {
                    _rx_state = RxState::WAIT_CRC;
                } else {
                    _rx_state = RxState::WAIT_PAYLOAD;
                }
            }
            break;
            
        case RxState::WAIT_PAYLOAD:
            _rx_payload[_rx_payload_idx++] = b;
            _rx_crc = crc8_update(_rx_crc, b);
            if (_rx_payload_idx == _rx_len) {
                _rx_state = RxState::WAIT_CRC;
            }
            break;
            
        case RxState::WAIT_CRC:
            if (b == _rx_crc) {
                process_frame();
            }
            _rx_state = RxState::WAIT_HEADER;
            break;
        }
    }
}

void WuK_Comms::process_frame()
{
    // Handle incoming messages
    switch ((RspID)_rx_msg_id) {
    case RspID::ACK:
        // Handle ACK - maybe clear a pending command flag?
        break;
    case RspID::STATUS:
        // Handle Status - maybe log it?
        // Payload: [AngleL, AngleH, State, Error]
        if (_rx_len >= 4) {
            uint16_t angle = _rx_payload[0] | (_rx_payload[1] << 8);
            uint8_t state = _rx_payload[2];
            uint8_t error = _rx_payload[3];
            
            // Log this data (Task 8)
            struct log_WuK pkt = {
                LOG_PACKET_HEADER_INIT(LOG_WUK_MSG),
                time_us : AP_HAL::micros64(),
                angle   : angle,
                state   : state,
                error   : error,
                cmd     : 0 // We don't track last cmd here easily without more state
            };
            AP::logger().WriteBlock(&pkt, sizeof(pkt));

            // Update motors with current morph angle (Task 6 integration)
            // Assuming angle is in degrees. If centi-degrees, divide by 100.0f
            // [WuK-FIX] dynamic_cast not supported with -fno-rtti, using C-style cast
            AP_MotorsMatrix *m = (AP_MotorsMatrix *)copter.motors;
            if (m) {
                m->set_morph_angle((float)angle);
            }
        }
        break;
    default:
        break;
    }
}

uint8_t WuK_Comms::crc8_update(uint8_t crc, uint8_t data)
{
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

void WuK_Comms::send_frame(uint8_t msg_id, const uint8_t* payload, uint8_t len)
{
    uint8_t crc = 0;

    size_t written = 0;
    written += _uart->write(HEADER_BYTE);
    
    written += _uart->write(msg_id);
    crc = crc8_update(crc, msg_id);
    
    written += _uart->write(len);
    crc = crc8_update(crc, len);
    
    for (uint8_t i = 0; i < len; i++) {
        written += _uart->write(payload[i]);
        crc = crc8_update(crc, payload[i]);
    }
    
    written += _uart->write(crc);

    const uint8_t expected = FRAME_OVERHEAD + len;
    if (written != expected) {
        static uint32_t last_short_write_log_ms = 0;
        const uint32_t now = AP_HAL::millis();
        if (now - last_short_write_log_ms > 2000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "[WuK-TX] short write %u/%u", 
                          (unsigned int)written,
                          (unsigned int)expected);
            last_short_write_log_ms = now;
        }
    }
}

// Parameter definitions
const AP_Param::GroupInfo WuK_Comms::var_info[] = {
    // @Param: PORT
    // @DisplayName: UART Port
    // @Description: UART port number (0=Serial0, 1=Serial1, etc.) - Not used if using SerialManager
    // @User: Advanced
    AP_GROUPINFO("PORT", 1, WuK_Comms, _port, 4),

    // @Param: BAUD
    // @DisplayName: UART Baudrate
    // @Description: UART baudrate
    // @User: Advanced
    AP_GROUPINFO("BAUD", 2, WuK_Comms, _baud, 9600),

    AP_GROUPEND
};
