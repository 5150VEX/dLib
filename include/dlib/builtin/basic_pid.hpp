#pragma once
#include "dlib/interfaces/pid.hpp"
#include <iostream>

namespace dlib {

class BasicPID : public PID {
    private:
        PIDGains gains;
        double setpoint = 0;

        double p = 0;
        double i = 0;
        double d = 0;

        double last_error = 0;
        uint32_t last_time = 0;
    public:
        BasicPID(PIDGains gains) : gains(gains) {};

        double update(double error);
        void reset();

        const PIDGains get_gains() const;
        void set_gains(PIDGains new_gains);

        const double get_setpoint() const;
        void set_setpoint(double new_setpoint);

        const double get_error() const;
};

}