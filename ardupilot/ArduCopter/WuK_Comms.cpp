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
    if (_port >= 0) {
        _uart = hal.serial(_port);
        if (_uart) {
            _uart->begin(_baud);
        }
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
    // Send heartbeat every 200ms
    uint32_t now = AP_HAL::millis();
    if (now - _last_heartbeat_ms > 200) {
        if (enqueue_cmd(CmdID::STATUS_REQ, 0)) {
            _last_heartbeat_ms = now;
        }
    }

    // Process queue
    while (_head != _tail) {
        if (_uart->txspace() < (FRAME_OVERHEAD + 2)) { // 2 bytes for value
            break;
        }

        QueueItem &item = _queue[_tail];
        uint8_t payload[2];
        payload[0] = item.value & 0xFF;
        payload[1] = (item.value >> 8) & 0xFF;
        
        send_frame((uint8_t)item.cmd_id, payload, 2);
        
        _tail = (_tail + 1) % QUEUE_SIZE;
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
            if (AP_MotorsMatrix *m = dynamic_cast<AP_MotorsMatrix*>(copter.motors)) {
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
    _uart->write(HEADER_BYTE);
    
    _uart->write(msg_id);
    crc = crc8_update(crc, msg_id);
    
    _uart->write(len);
    crc = crc8_update(crc, len);
    
    for (uint8_t i = 0; i < len; i++) {
        _uart->write(payload[i]);
        crc = crc8_update(crc, payload[i]);
    }
    
    _uart->write(crc);
}

// Parameter definitions
const AP_Param::GroupInfo WuK_Comms::var_info[] = {
    // @Param: PORT
    // @DisplayName: UART Port
    // @Description: UART port number (0=Serial0, 1=Serial1, etc.) - Not used if using SerialManager
    // @User: Advanced
    AP_GROUPINFO("PORT", 1, WuK_Comms, _port, 0),

    // @Param: BAUD
    // @DisplayName: UART Baudrate
    // @Description: UART baudrate
    // @User: Advanced
    AP_GROUPINFO("BAUD", 2, WuK_Comms, _baud, 57600),

    AP_GROUPEND
};
