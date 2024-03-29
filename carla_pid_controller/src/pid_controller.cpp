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
    }
    //protection, avoid to oscillation
    if (std::fabs(integral_) > 5) {
        PIDController::Reset();
    }
    double current_output = 0.0;
    double diff;
    if (first_hit_){
        return first_hit_ = false;
    }else{
        diff = (error - previous_error_) / dt;
    }
    integral_ +=  error * dt;
    current_output = kp_ * error + kd_ * diff + ki_ * integral_;

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
