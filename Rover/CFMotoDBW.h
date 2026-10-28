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
    void update(bool armed, bool failsafe, float ground_speed);
    [[nodiscard]] bool enabled() const;
    [[nodiscard]] bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;
    [[nodiscard]] bool steering_controller_enabled() const;

private:
    enum class Gear : uint8_t {
        park,
        reverse,
        neutral,
        high,
        low,
        unknown,
    };

    enum class ShiftState : uint8_t {
        steady,
        braking,
        neutral_dwell,
        moving,
        verifying,
        fault,
    };

    enum class EngineState : uint8_t {
        off,
        ignition_delay,
        cranking,
        running,
        retry_delay,
        fault,
    };

    enum class CalibrationState : uint8_t {
        idle,
        running,
        complete,
        fault,
    };

    enum class CalibrationPhase : uint8_t {
        prepositioning,
        settling_start,
        sweeping,
        settling_end,
    };

    struct CalibrationInterval {
        uint16_t low_pwm = 0;
        uint16_t high_pwm = 0;
        uint8_t samples = 0;
    };

    struct CalibrationAggregate {
        uint32_t low_sum[2]{};
        uint32_t high_sum[2]{};
        uint8_t count[2]{};
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
    AP_Int16 _gear_input_park_pwm;
    AP_Int16 _gear_input_reverse_pwm;
    AP_Int16 _gear_input_neutral_pwm;
    AP_Int16 _gear_input_high_pwm;
    AP_Int16 _gear_input_low_pwm;
    AP_Int16 _gear_input_tolerance_pwm;
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
    AP_Int8 _steering_controller_enable;
    AP_Int16 _steering_failsafe_pwm;
    AP_Int16 _shift_brake_ms;
    AP_Int16 _shift_stop_ms;
    AP_Int16 _shift_neutral_ms;
    AP_Int16 _shift_timeout_ms;
    AP_Int8 _shift_rpm_instance;
    AP_Int32 _shift_rpm_max;
    AP_Int8 _engine_enable;
    AP_Int8 _ignition_rc_channel;
    AP_Int8 _starter_rc_channel;
    AP_Int8 _engine_rpm_instance;
    AP_Int32 _engine_running_rpm;
    AP_Int16 _engine_start_delay_ms;
    AP_Int16 _engine_crank_ms;
    AP_Int16 _engine_retry_delay_ms;
    AP_Int8 _engine_max_retries;
    AP_Int8 _fourwd_rc_channel;
    AP_Int8 _diff_lock_rc_channel;
    AP_Float _axle_shift_speed_max;
    AP_Int8 _axle_rpm_instance;
    AP_Int32 _axle_rpm_max;
    AP_Int8 _headlights_rc_channel;
    AP_Int8 _ew_rc_channel;
    AP_Int8 _gas_rc_channel;
    AP_Int8 _brake_rc_channel;
    AP_Int8 _throttle_limit_rc_channel;
    AP_Float _throttle_limit_pct;
    AP_Int8 _calibration_command;
    AP_Int8 _calibration_rc_channel;
    AP_Int16 _calibration_min_pwm;
    AP_Int16 _calibration_max_pwm;
    AP_Int8 _calibration_step_pwm;
    AP_Int16 _calibration_period_ms;
    Gear _selected_gear;
    Gear _target_gear = Gear::neutral;
    ShiftState _shift_state = ShiftState::steady;
    EngineState _engine_state = EngineState::off;
    Gear _telemetry_gear = Gear::unknown;
    Gear _telemetry_raw_gear = Gear::unknown;
    Gear _telemetry_last_stable_gear = Gear::unknown;
    int8_t _telemetry_travel_direction = 0;
    AP_HAL::UARTDriver *_telemetry_uart = nullptr;
    uint32_t _last_telemetry_ms = 0;
    uint8_t _telemetry_buffer[24]{};
    uint16_t _telemetry_dio = 0;
    uint16_t _telemetry_previous_dio = 0;
    uint16_t _telemetry_adc[7]{};
    uint16_t _telemetry_nano_rx_count = 0;
    uint32_t _telemetry_valid_packets = 0;
    uint32_t _telemetry_bad_packets = 0;
    uint32_t _last_telemetry_publish_ms = 0;
    uint8_t _telemetry_offset = 0;
    uint8_t _telemetry_sequence = 0;
    bool _throttle_release_required = true;
    bool _fourwd_enabled = false;
    bool _diff_lock_enabled = false;
    uint32_t _shift_started_ms = 0;
    uint32_t _shift_state_started_ms = 0;
    uint32_t _engine_state_started_ms = 0;
    uint8_t _engine_retry_count = 0;
    CalibrationState _calibration_state = CalibrationState::idle;
    CalibrationPhase _calibration_phase = CalibrationPhase::settling_start;
    int8_t _calibration_error = 0;
    int8_t _calibration_direction = 1;
    uint8_t _calibration_cycle = 0;
    uint16_t _calibration_pwm = 1500;
    uint32_t _calibration_next_step_ms = 0;
    uint32_t _calibration_hold_until_ms = 0;
    uint8_t _calibration_last_sequence = 0;
    bool _calibration_have_sequence = false;
    Gear _calibration_active_gear = Gear::unknown;
    CalibrationInterval _calibration_active_interval{};
    CalibrationInterval _calibration_leg_intervals[5]{};
    bool _calibration_leg_seen[5]{};
    Gear _calibration_leg_order[5]{};
    uint8_t _calibration_leg_order_count = 0;
    int8_t _calibration_forward_orientation = 0;
    CalibrationAggregate _calibration_aggregate[5]{};
    uint16_t _calibration_mean_low[5]{};
    uint16_t _calibration_mean_high[5]{};
    uint16_t _calibration_candidate[5]{};

    static constexpr uint16_t PWM_MIN = 800;
    static constexpr uint16_t PWM_MAX = 2200;
    static constexpr uint8_t TELEMETRY_PACKET_ID = 0x23;
    static constexpr uint8_t TELEMETRY_PACKET_LENGTH = 24;
    static constexpr uint16_t TELEMETRY_PUBLISH_PERIOD_MS = 500;
    static constexpr uint8_t CALIBRATION_CYCLES = 3;
    static constexpr uint16_t CALIBRATION_ENDPOINT_HOLD_MS = 1000;
    static constexpr uint8_t CALIBRATION_MIN_SAMPLES = 2;

    [[nodiscard]] uint16_t constrained_pwm(const AP_Int16 &parameter) const;
    [[nodiscard]] uint16_t interpolate_pwm(uint16_t from_pwm, uint16_t to_pwm, float ratio) const;
    [[nodiscard]] Gear requested_gear() const;
    [[nodiscard]] uint16_t gear_pwm(Gear gear) const;
    [[nodiscard]] bool gear_allows_throttle() const;
    [[nodiscard]] bool telemetry_enabled() const;
    [[nodiscard]] bool telemetry_healthy() const;
    [[nodiscard]] bool gear_feedback_matches() const;
    [[nodiscard]] bool gear_feedback_matches(Gear gear) const;
    [[nodiscard]] bool rc_switch_high(const AP_Int8 &channel_parameter) const;
    [[nodiscard]] float rc_channel_positive_pct(const AP_Int8 &channel_parameter) const;
    [[nodiscard]] float throttle_limit_pct() const;
    [[nodiscard]] bool rpm_below(const AP_Int8 &instance_parameter, const AP_Int32 &limit_parameter) const;
    [[nodiscard]] bool engine_running() const;
    [[nodiscard]] bool shift_in_progress() const;
    [[nodiscard]] bool shift_timed_out(uint32_t now_ms) const;
    [[nodiscard]] bool shift_requires_neutral(Gear from, Gear to) const;
    [[nodiscard]] Gear decode_gear(uint16_t dio) const;
    [[nodiscard]] Gear resolve_telemetry_gear(Gear raw_gear);
    [[nodiscard]] static int8_t gear_code(Gear gear);
    [[nodiscard]] static uint8_t gear_index(Gear gear);
    [[nodiscard]] Outputs safe_outputs() const;
    [[nodiscard]] Outputs active_outputs(float gas_pct, float brake_pct) const;
    void update_telemetry();
    void parse_telemetry_byte(uint8_t byte);
    void accept_telemetry_packet();
    void publish_telemetry(uint32_t now_ms);
    void publish_calibration() const;
    [[nodiscard]] bool telemetry_stream_fresh() const;
    [[nodiscard]] bool calibration_deadman() const;
    [[nodiscard]] bool calibration_engine_stopped() const;
    [[nodiscard]] bool calibration_safety_ok(bool armed, bool failsafe, bool throttle_is_neutral,
                                             float ground_speed, int8_t &error) const;
    void reset_calibration_leg();
    void start_calibration(uint32_t now_ms);
    void abort_calibration(int8_t error);
    void record_calibration_sample();
    [[nodiscard]] bool finish_calibration_interval();
    [[nodiscard]] bool finish_calibration_leg(uint32_t now_ms);
    [[nodiscard]] bool finish_calibration();
    [[nodiscard]] bool update_calibration(bool armed, bool failsafe, bool throttle_is_neutral,
                                          float ground_speed, uint32_t now_ms);
    void write_outputs(const Outputs &outputs) const;
    void start_shift(Gear target, uint32_t now_ms);
    void update_shift(Gear requested, bool throttle_is_neutral, float ground_speed, uint32_t now_ms);
    void update_engine(bool armed, bool failsafe, bool throttle_is_neutral, float ground_speed, uint32_t now_ms);
    void update_driveline(bool failsafe, bool throttle_is_neutral, float ground_speed);
    void update_accessories(bool failsafe);
    void set_relay(uint8_t relay_index, bool enabled) const;
};
