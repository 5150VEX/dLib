#pragma once

#include "pros/motor_group.hpp"
#include <initializer_list>

namespace dlib {
    struct Motors {
        pros::MotorGroup left;
        pros::MotorGroup right;

        Motors(
            std::initializer_list<std::int8_t> left_ports, 
            std::initializer_list<std::int8_t> right_ports
        ) : left(left_ports), right(right_ports) {};
    };

    struct Specs {
        const double drive_rpm;
        const double wheel_diameter;

        Specs(
            double drive_rpm, double wheel_diameter
        ) : drive_rpm(drive_rpm), wheel_diameter(wheel_diameter) {};
    };

    class Chassis {
        public:
            virtual Motors& get_motors() = 0;
            virtual const Specs get_specs() const = 0;
    };
}