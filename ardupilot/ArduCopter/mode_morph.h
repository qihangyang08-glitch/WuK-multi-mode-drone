#pragma once

#include "mode.h"

class ModeMorph : public Mode {
public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    Mode::Number mode_number() const override { return Mode::Number::MORPH; }
    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }  // [WuK-FIX] 使用自动高度控制
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:
    const char *name() const override { return "MORPH"; }
    const char *name4() const override { return "MRPH"; }

private:
    float _current_morph_angle;
    uint32_t _hold_start_ms;
    uint32_t _dwell_start_ms;
    uint32_t _morph_start_ms;
    uint32_t _last_arm_cmd_send_ms;
    uint32_t _last_enqueue_warn_ms;
    uint16_t _last_arm_cmd_value;
    uint8_t _last_key_bucket;
    uint8_t _pending_key_repeats;
    uint16_t _pending_key_value;
    bool _holding_45;
    bool _dwelling;
};
