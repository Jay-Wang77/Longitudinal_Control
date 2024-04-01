#include "carla_shenlan_pid_controller/pid_controller.h"
#include <math.h>
#include <assert.h>
#include <iostream>
namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
}

// pid controller
double PIDController::Control(const double error, const double dt) {
    //ensure dt must be positive
    if (dt <= 0){
        return previous_output_;
    }\
    if (first_hit_) {
        first_hit_ = false;
        proportional_part = error;
        integral_part += error * dt;
        derivative_part = 0;
    } else {
        proportional_part = error;
        integral_part += error * dt;
        derivative_part = (error - previous_error_) / dt;
    }

    // protection, avoid to oscillation
    if (integral_part > 40.0){
        integral_part = 40.0;
    }
    if (integral_part < -40.0){
        integral_part = -40.0;
    }
    // std::cout << "integral_part: " << integral_part << std::endl;
    current_output = kp_ * proportional_part + ki_ * integral_part + kd_ * derivative_part;

    previous_error_ = error;
    previous_output_ = current_output;

    return current_output;
}

// /**to-do**/ reset pid param.
void PIDController::Reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    first_hit_ = true;
}

}    // namespace control
}    // namespace shenlan
