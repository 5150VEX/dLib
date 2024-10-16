#pragma once
#include "dlib/interfaces/encoder.hpp"
#include "dlib/interfaces/chassis.hpp"
#include "pros/motor_group.hpp"

namespace dlib {
    class InternalMotorEncoders : public Encoder {
        dlib::Chassis& chassis;
    public:
        InternalMotorEncoders(dlib::Chassis& chassis) : chassis(chassis) {};
        
        void reset();
        double get_displacement();
        const uint32_t get_update_rate() const;
    };
}