#pragma once

#include <AP_Param/AP_Param.h>

namespace AP_HAL {
class UARTDriver;
}

class CFMotoDBW {
public:
    CFMotoDBW();
    static const AP_Param::GroupInfo var_info[];
    void init();
    void setup_failsafe() const;
    void update(bool armed, bool failsafe, float throttle_pct, float ground_speed);

private:
    enum class Gear : uint8_t {
        park,
        reverse,
        neutral,
        high,
        low,
        unknown,
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
    AP_Int8 _telemetry_enable;
    AP_Int16 _telemetry_timeout_ms;
    AP_Int16 _dio_mask;
    AP_Int16 _dio_park;
    AP_Int16 _dio_reverse;
    AP_Int16 _dio_neutral;
    AP_Int16 _dio_high;
    AP_Int16 _dio_low;
    Gear _selected_gear;
    Gear _telemetry_gear = Gear::unknown;
    AP_HAL::UARTDriver *_telemetry_uart = nullptr;
    uint32_t _last_telemetry_ms = 0;
    uint8_t _telemetry_buffer[24]{};
    uint8_t _telemetry_offset = 0;
    bool _throttle_release_required = true;

    static constexpr uint16_t PWM_MIN = 800;
    static constexpr uint16_t PWM_MAX = 2200;
    static constexpr uint8_t TELEMETRY_PACKET_ID = 0x23;
    static constexpr uint8_t TELEMETRY_PACKET_LENGTH = 24;

    [[nodiscard]] uint16_t constrained_pwm(const AP_Int16 &parameter) const;
    [[nodiscard]] uint16_t interpolate_pwm(uint16_t from_pwm, uint16_t to_pwm, float ratio) const;
    [[nodiscard]] Gear requested_gear() const;
    [[nodiscard]] uint16_t gear_pwm(Gear gear) const;
    [[nodiscard]] bool gear_allows_throttle() const;
    [[nodiscard]] bool telemetry_enabled() const;
    [[nodiscard]] bool telemetry_healthy() const;
    [[nodiscard]] bool gear_feedback_matches() const;
    [[nodiscard]] Gear decode_gear(uint16_t dio) const;
    [[nodiscard]] Outputs safe_outputs() const;
    [[nodiscard]] Outputs active_outputs(float throttle_pct) const;
    void update_telemetry();
    void parse_telemetry_byte(uint8_t byte);
    void accept_telemetry_packet();
    void write_outputs(const Outputs &outputs) const;
};
