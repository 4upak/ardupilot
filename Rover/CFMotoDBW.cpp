#include "CFMotoDBW.h"

#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_Param::GroupInfo CFMotoDBW::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: CFMOTO drive-by-wire enable
    // @Description: Enables CFMOTO gas, brake and gearbox output coordination. Actuator PWM values must be calibrated before enabling.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, CFMotoDBW, _enable, 0),

    // @Param: GAS_IDLE
    // @DisplayName: Gas idle PWM
    // @Description: PWM command that places the gas servo at idle. SERVO1_TRIM must be set to the same value for safe hardware failsafe behaviour.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GAS_IDLE", 2, CFMotoDBW, _gas_idle_pwm, 1500),

    // @Param: GAS_MAX
    // @DisplayName: Gas maximum PWM
    // @Description: PWM command for maximum permitted gas. Set equal to GAS_IDLE until the linkage limits have been measured.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GAS_MAX", 3, CFMotoDBW, _gas_max_pwm, 1500),

    // @Param: BRK_REL
    // @DisplayName: Brake released PWM
    // @Description: PWM command that fully releases the service brake.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("BRK_REL", 4, CFMotoDBW, _brake_release_pwm, 1500),

    // @Param: BRK_HOLD
    // @DisplayName: Brake hold PWM
    // @Description: PWM command that applies the service brake for failsafe and disarmed holding. SERVO2_TRIM must be set to the same value.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("BRK_HOLD", 5, CFMotoDBW, _brake_hold_pwm, 1500),

    // @Param: GEAR_P
    // @DisplayName: Gearbox Park PWM
    // @Description: Gearbox actuator PWM for Park.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GEAR_P", 6, CFMotoDBW, _gear_park_pwm, 1500),

    // @Param: GEAR_R
    // @DisplayName: Gearbox Reverse PWM
    // @Description: Gearbox actuator PWM for Reverse.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GEAR_R", 7, CFMotoDBW, _gear_reverse_pwm, 1500),

    // @Param: GEAR_N
    // @DisplayName: Gearbox Neutral PWM
    // @Description: Gearbox actuator PWM for Neutral. SERVO4_TRIM must be set to the same value.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GEAR_N", 8, CFMotoDBW, _gear_neutral_pwm, 1500),

    // @Param: GEAR_H
    // @DisplayName: Gearbox High PWM
    // @Description: Gearbox actuator PWM for High.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GEAR_H", 9, CFMotoDBW, _gear_high_pwm, 1500),

    // @Param: GEAR_L
    // @DisplayName: Gearbox Low PWM
    // @Description: Gearbox actuator PWM for Low.
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GEAR_L", 10, CFMotoDBW, _gear_low_pwm, 1500),

    // @Param: GEAR_CH
    // @DisplayName: Gearbox RC input channel
    // @Description: RC input channel selecting P, R, N, H and L in five equal PWM bands. Zero always requests Neutral.
    // @Range: 0 16
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GEAR_CH", 11, CFMotoDBW, _gear_rc_channel, 8),

    // @Param: SHIFT_SPD
    // @DisplayName: Maximum gearbox shift speed
    // @Description: Maximum absolute vehicle speed at which a requested gearbox position change is accepted.
    // @Units: m/s
    // @Range: 0 2
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("SHIFT_SPD", 12, CFMotoDBW, _shift_speed_max, 0.1),

    // @Param: THR_DZ
    // @DisplayName: Gas and brake deadzone
    // @Description: Absolute throttle command below which gas is idle and the service brake is released. Positive commands operate gas and negative commands operate brake.
    // @Units: %
    // @Range: 0 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_DZ", 13, CFMotoDBW, _throttle_deadzone, 5.0),

    AP_GROUPEND
};

CFMotoDBW::CFMotoDBW() :
    _selected_gear(Gear::neutral)
{
    AP_Param::setup_object_defaults(this, var_info);
}

uint16_t CFMotoDBW::constrained_pwm(const AP_Int16 &parameter) const
{
    return static_cast<uint16_t>(constrain_int16(parameter.get(), PWM_MIN, PWM_MAX));
}

uint16_t CFMotoDBW::interpolate_pwm(uint16_t from_pwm, uint16_t to_pwm, float ratio) const
{
    const float constrained_ratio = constrain_float(ratio, 0.0f, 1.0f);
    return static_cast<uint16_t>(from_pwm + (static_cast<int32_t>(to_pwm) - from_pwm) * constrained_ratio);
}

CFMotoDBW::Gear CFMotoDBW::requested_gear() const
{
    const int8_t channel_number = _gear_rc_channel.get();
    if (channel_number < 1 || channel_number > 16) {
        return Gear::neutral;
    }

    const RC_Channel *gear_channel = ::rc().channel(static_cast<uint8_t>(channel_number - 1));
    if (gear_channel == nullptr) {
        return Gear::neutral;
    }

    const int16_t pwm = gear_channel->get_radio_in();
    if (pwm < RC_Channel::RC_MIN_LIMIT_PWM || pwm > RC_Channel::RC_MAX_LIMIT_PWM) {
        return Gear::neutral;
    }
    if (pwm < 1200) {
        return Gear::park;
    }
    if (pwm < 1400) {
        return Gear::reverse;
    }
    if (pwm < 1600) {
        return Gear::neutral;
    }
    if (pwm < 1800) {
        return Gear::high;
    }
    return Gear::low;
}

uint16_t CFMotoDBW::gear_pwm(Gear gear) const
{
    switch (gear) {
    case Gear::park:
        return constrained_pwm(_gear_park_pwm);
    case Gear::reverse:
        return constrained_pwm(_gear_reverse_pwm);
    case Gear::neutral:
        return constrained_pwm(_gear_neutral_pwm);
    case Gear::high:
        return constrained_pwm(_gear_high_pwm);
    case Gear::low:
        return constrained_pwm(_gear_low_pwm);
    }
    return constrained_pwm(_gear_neutral_pwm);
}

bool CFMotoDBW::gear_allows_throttle() const
{
    return _selected_gear == Gear::reverse ||
           _selected_gear == Gear::high ||
           _selected_gear == Gear::low;
}

CFMotoDBW::Outputs CFMotoDBW::safe_outputs() const
{
    return {
        constrained_pwm(_gas_idle_pwm),
        constrained_pwm(_brake_hold_pwm),
        gear_pwm(_selected_gear),
    };
}

CFMotoDBW::Outputs CFMotoDBW::active_outputs(float throttle_pct) const
{
    const uint16_t gas_idle = constrained_pwm(_gas_idle_pwm);
    const uint16_t brake_release = constrained_pwm(_brake_release_pwm);
    const float deadzone = constrain_float(_throttle_deadzone.get(), 0.0f, 30.0f);
    Outputs outputs{gas_idle, brake_release, gear_pwm(_selected_gear)};

    if (throttle_pct > deadzone && gear_allows_throttle()) {
        const float ratio = (throttle_pct - deadzone) / (100.0f - deadzone);
        outputs.gas_pwm = interpolate_pwm(gas_idle, constrained_pwm(_gas_max_pwm), ratio);
    } else if (throttle_pct < -deadzone) {
        const float ratio = (-throttle_pct - deadzone) / (100.0f - deadzone);
        outputs.brake_pwm = interpolate_pwm(brake_release, constrained_pwm(_brake_hold_pwm), ratio);
    }

    if (_selected_gear == Gear::park) {
        outputs.brake_pwm = constrained_pwm(_brake_hold_pwm);
    }
    return outputs;
}

void CFMotoDBW::write_outputs(const Outputs &outputs) const
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting1, outputs.gas_pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting2, outputs.brake_pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_scripting3, outputs.gear_pwm);
}

void CFMotoDBW::setup_failsafe() const
{
    SRV_Channels::set_failsafe_pwm(SRV_Channel::k_scripting1, constrained_pwm(_gas_idle_pwm));
    SRV_Channels::set_failsafe_pwm(SRV_Channel::k_scripting2, constrained_pwm(_brake_hold_pwm));
    SRV_Channels::set_failsafe_pwm(SRV_Channel::k_scripting3, 0);
}

void CFMotoDBW::update(bool armed, bool failsafe, float throttle_pct, float ground_speed)
{
    if (_enable.get() == 0) {
        _selected_gear = Gear::neutral;
        write_outputs(safe_outputs());
        return;
    }

    if (!armed) {
        // Do not command a gearbox shift merely because the vehicle was
        // disarmed while it was still moving.
        const bool stopped = fabsf(ground_speed) <= MAX(_shift_speed_max.get(), 0.0f);
        if (stopped) {
            _selected_gear = Gear::neutral;
        }
        write_outputs(safe_outputs());
        return;
    }

    if (failsafe) {
        write_outputs(safe_outputs());
        return;
    }

    const Gear requested = requested_gear();
    const float deadzone = constrain_float(_throttle_deadzone.get(), 0.0f, 30.0f);
    const bool throttle_is_neutral = fabsf(throttle_pct) <= deadzone;
    const bool speed_allows_shift = fabsf(ground_speed) <= MAX(_shift_speed_max.get(), 0.0f);
    if (requested != _selected_gear && throttle_is_neutral && speed_allows_shift) {
        _selected_gear = requested;
    }

    write_outputs(active_outputs(constrain_float(throttle_pct, -100.0f, 100.0f)));
}
