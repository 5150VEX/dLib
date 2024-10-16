#pragma once

#include <cstdint>
#include <limits>

namespace dlib {
    struct PIDGains {
        double kp = 0;
        double ki = 0;
        double kd = 0;
    };

    struct PIDOptions {
        double error_threshold = 0;
        uint32_t settle_ms = 250;
        uint32_t timeout_ms = std::numeric_limits<uint32_t>::max();

        PIDOptions with_error_threshold(double threshold) {
            return { threshold, settle_ms, timeout_ms };
        }
        
        PIDOptions with_settle_ms(uint32_t time) {
            return { error_threshold, time, timeout_ms };
        }

        PIDOptions with_max_ms(uint32_t time) {
            return { error_threshold, settle_ms, time };
        }
    };

    class PID {
        public:
            virtual void reset() = 0;
            virtual double update(double error) = 0;
            
            virtual const PIDGains get_gains() const = 0;
            virtual void set_gains(PIDGains new_gains) = 0;

            virtual const double get_setpoint() const = 0;
            virtual void set_setpoint(double new_setpoint) = 0;

            virtual const double get_error() const = 0;
    };
}