#include "dlib/builtin/internal_motor_encoders.hpp"
#include "pros/motors.hpp"
#include <cmath>



namespace dlib {
    void InternalMotorEncoders::reset() {
        Motors& motors = chassis.get_motors();

        motors.left.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
        motors.right.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
        
        motors.left.tare_position_all();
        motors.right.tare_position_all();
    }

    double InternalMotorEncoders::get_displacement() {
        Motors& motors = chassis.get_motors();
        Specs specs = chassis.get_specs();

        auto diameter = specs.wheel_diameter;
        auto rpm = specs.drive_rpm;

        double sum = 0;

        auto left_gearsets = motors.left.get_gearing_all();
        auto left_positions = motors.left.get_position_all();

        for(int i = 0; i < left_positions.size(); i++) {
            double internal;

            switch (left_gearsets[i]) {
                case pros::MotorGears::red: internal = 100; break;
                case pros::MotorGears::green: internal = 200; break;
                case pros::MotorGears::blue: internal = 600; break;
                default: internal = 200; break;
            }

            sum += left_positions[i] * (diameter * M_PI) * (rpm / internal);
        }

        auto right_gearsets = motors.right.get_gearing_all();
        auto right_positions = motors.right.get_position_all();

        for(int i = 0; i < right_positions.size(); i++) {
            double internal;

            switch (right_gearsets[i]) {
                case pros::MotorGears::red: internal = 100; break;
                case pros::MotorGears::green: internal = 200; break;
                case pros::MotorGears::blue: internal = 600; break;
                default: internal = 200; break;
            }

            sum += right_positions[i] * (diameter * M_PI) * (rpm / internal);
        }
        
        return sum / (left_positions.size() + right_positions.size());
    }

    const uint32_t InternalMotorEncoders::get_update_rate() const {
        return 10;
    }
}