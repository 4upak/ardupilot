#pragma once

#include <AP_Param/AP_Param.h>

class CFMotoDBW {
public:
    CFMotoDBW();
    static const AP_Param::GroupInfo var_info[];
    void setup_failsafe() const;
    void update(bool armed, bool failsafe, float throttle_pct, float ground_speed);

private:
    enum class Gear : uint8_t {
        park,
        reverse,
        neutral,
        high,
        low,
    };

    struct Outputs {
        uint16_t gas_pwm;
        uint16_t brake_pwm;
        uint16_t gear_pwm;
    };

    AP_Int8 _enable;
    AP_Int16 _gas_idle_pwm;
    AP_Int16 _gas_max_pwm;
    AP_Int16 _brake_release_pwm;
    AP_Int16 _brake_hold_pwm;
    AP_Int16 _gear_park_pwm;
    AP_Int16 _gear_reverse_pwm;
    AP_Int16 _gear_neutral_pwm;
    AP_Int16 _gear_high_pwm;
    AP_Int16 _gear_low_pwm;
    AP_Int8 _gear_rc_channel;
    AP_Float _shift_speed_max;
    AP_Float _throttle_deadzone;
    Gear _selected_gear;

    static constexpr uint16_t PWM_MIN = 800;
    static constexpr uint16_t PWM_MAX = 2200;

    [[nodiscard]] uint16_t constrained_pwm(const AP_Int16 &parameter) const;
    [[nodiscard]] uint16_t interpolate_pwm(uint16_t from_pwm, uint16_t to_pwm, float ratio) const;
    [[nodiscard]] Gear requested_gear() const;
    [[nodiscard]] uint16_t gear_pwm(Gear gear) const;
    [[nodiscard]] bool gear_allows_throttle() const;
    [[nodiscard]] Outputs safe_outputs() const;
    [[nodiscard]] Outputs active_outputs(float throttle_pct) const;
    void write_outputs(const Outputs &outputs) const;
};
