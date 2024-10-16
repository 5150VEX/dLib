#pragma once

#include "interfaces/chassis.hpp"
#include "interfaces/pid.hpp"
#include "interfaces/encoder.hpp"
#include "pros/rtos.hpp"

namespace dlib {
    inline void brake(Chassis& chassis) {
        Motors& motors = chassis.get_motors();
        motors.left.brake();
        motors.right.brake();
    }

    inline void move(Chassis& chassis, int32_t voltage) {
        Motors& motors = chassis.get_motors();
        motors.left.move(voltage);
        motors.right.move(voltage);
    }

    inline void turn(Chassis& chassis, int32_t voltage) {
        Motors& motors = chassis.get_motors();
        motors.left.move(-voltage);
        motors.right.move(voltage);
    }

    inline void move_voltage(Chassis& chassis, int32_t millivolts) {
        Motors& motors = chassis.get_motors();
        motors.left.move_voltage(millivolts);
        motors.right.move_voltage(millivolts);
    }

    inline void turn_voltage(Chassis& chassis, int32_t millivolts) {
        Motors& motors = chassis.get_motors();
        motors.left.move(-millivolts);
        motors.right.move(millivolts);
    }

    inline void arcade(Chassis& chassis, int32_t power, int32_t turn) {
        Motors& motors = chassis.get_motors();
        motors.left.move(power - turn);
        motors.right.move(power + turn);
    }

    inline void move_with_pid(Chassis& chassis, Encoder& encoder, PID& drive_pid, double distance, PIDOptions options) {
        auto interval = encoder.get_update_rate();
        
        auto start_displacement = encoder.get_displacement();
        auto target_displacement = start_displacement + distance;

        uint32_t movement_start = pros::millis();
        uint32_t settling_start = 0;
        bool is_settling = false;

        while (true) {
            uint32_t current_time = pros::millis();

            if (current_time - movement_start >= options.timeout_ms) {
                break;
            }

            if (!is_settling && std::abs(drive_pid.get_error()) < options.error_threshold) {
                is_settling = true;
                settling_start = pros::millis();
            }

            if (is_settling) {
                if (std::abs(drive_pid.get_error()) < options.error_threshold) {
                    if(current_time - settling_start > options.settle_ms) {
                        break;
                    }
                } else {
                    is_settling = false;
                }
            }

            double current_displacement = encoder.get_displacement();
            double error = target_displacement - current_displacement;
            double output_voltage = drive_pid.update(error);

            move_voltage(chassis, output_voltage);

            pros::delay(interval);
        }

        brake(chassis);
    }
}