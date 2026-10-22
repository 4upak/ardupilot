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
        g2.cfmoto_dbw.update(false, true, 0.0f, speed);
        motor_test_output();
    } else {
        const bool drive_enabled = arming.is_armed() && hal.util->get_soft_armed();
        const bool failsafe_active = failsafe.bits != 0;
        const float dbw_throttle = g2.motors.get_throttle();
        g2.cfmoto_dbw.update(drive_enabled,
                             failsafe_active,
                             dbw_throttle,
                             speed);

        // Negative throttle operates the separate service brake. Do not let
        // AP_MotorsUGV interpret it as reverse motor torque and reverse the
        // steering output.
        g2.motors.set_throttle(MAX(dbw_throttle, 0.0f));

        g2.motors.output(arming.is_armed(), speed, G_Dt);
    }
}
