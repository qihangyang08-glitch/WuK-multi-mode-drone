#pragma once

#include "mode.h"

class ModeGround : public Mode {
public:
    // inherit constructor
    using Mode::Mode;

    bool init(bool ignore_checks) override;
    void run() override;
    Mode::Number mode_number() const override { return Mode::Number::GROUND; }
    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    bool is_autopilot() const override { return false; }

    // [WuK] 覆盖输出函数，跳过标准混控直接输出地面模式推力
    void output_to_motors() override;

protected:
    const char *name() const override { return "GROUND"; }
    const char *name4() const override { return "GRND"; }

private:
    // [WuK] 航向保持相关
    float _target_yaw_rad;  // 目标航向角（弧度）
    bool _yaw_locked;       // 航向是否已锁定
};
