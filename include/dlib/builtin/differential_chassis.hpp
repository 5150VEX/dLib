#pragma once

#include "dlib/interfaces/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include <initializer_list>

namespace dlib {
class DifferentialChassis : public dlib::Chassis {
    Motors motors;
    Specs specs;
    
    public:
        DifferentialChassis(
            std::initializer_list<std::int8_t> left_ports, 
            std::initializer_list<std::int8_t> right_ports,
            double drive_rpm,
            double wheel_diameter,
            pros::MotorGearset motors_gearset
        ) : motors(left_ports, right_ports), specs(drive_rpm, wheel_diameter) {
            motors.left.set_gearing_all(motors_gearset);
            motors.right.set_gearing_all(motors_gearset);
        };

        Motors& get_motors();
        const Specs get_specs() const;

        void move(int32_t voltage);
        void turn(int32_t voltage);
        void arcade(int32_t power, int32_t turn);
};
}