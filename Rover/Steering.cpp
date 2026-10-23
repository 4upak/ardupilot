#include "Rover.h"

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // send output signals to motors
    float speed = 0.0f;
    g2.attitude_control.get_forward_speed(speed);

    if (motor_test) {
        g2.cfmoto_dbw.update(false, true, speed);
        motor_test_output();
    } else {
        const bool drive_enabled = arming.is_armed() && hal.util->get_soft_armed();
        const bool failsafe_active = failsafe.bits != 0;
        const float dbw_throttle = g2.motors.get_throttle();

        // The external Arduino controller closes the steering position loop
        // locally. Command its calibrated centre whenever drive is disabled
        // or a Rover failsafe is active. The IO failsafe is configured
        // separately in CFMotoDBW::setup_failsafe() for an FMU failure.
        if (g2.cfmoto_dbw.steering_controller_enabled() && (!drive_enabled || failsafe_active)) {
            g2.motors.set_steering(0.0f);
        }
        g2.cfmoto_dbw.update(drive_enabled, failsafe_active, speed);

        // CFMotoDBW reads the separate CH2 gas and CH5 brake inputs directly.
        // Keep the stock motor layer non-reversing so it cannot invert the
        // steering output or contend with the dedicated service-brake output.
        g2.motors.set_throttle(MAX(dbw_throttle, 0.0f));

        g2.motors.output(arming.is_armed(), speed, G_Dt);
    }
}
