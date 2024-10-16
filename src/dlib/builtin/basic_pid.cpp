#include "pros/rtos.hpp"
#include "dlib/builtin/basic_pid.hpp"

namespace dlib {
    double BasicPID::update(double error) {
        uint32_t time = pros::millis();
        uint32_t delta_time = time - last_time;

        // calculate PID terms
        p = error * gains.kp;
        i = i + error * delta_time * gains.ki;
        d = (error - last_error) / delta_time * gains.kd;

        // scale the output and limit it to [-12000, 12000] millivolts
        double output_scale = 100;
        double output = std::clamp((p + i + d) * output_scale, -12000.0, 12000.0);

        // update PID state
        last_error    = error;
        last_time     = time;

        return output;
    }

    void BasicPID::reset() {
        p = 0;
        i = 0;
        d = 0;

        last_error  = 0;
        last_time   = 0;
    }

    const PIDGains BasicPID::get_gains() const {
        return gains;
    }

    void BasicPID::set_gains(PIDGains new_gains) {
        gains = new_gains;
    }

    const double BasicPID::get_setpoint() const {
        return setpoint;
    }

    void BasicPID::set_setpoint(double new_setpoint) {
        reset();
        setpoint = new_setpoint;
    }

    const double BasicPID::get_error() const {
        return last_error;
    }
}