#include "main.h"
#include "dlib/dlib.hpp"
#include "dlib/builtin.hpp"
#include <initializer_list>

// User-built class, constructed from dlib components
struct Robot {
    dlib::DifferentialChassis chassis;
    dlib::InternalMotorEncoders encoders;

    dlib::BasicPID move_pid;
    dlib::BasicPID turn_pid;

    Robot(
        std::initializer_list<int8_t> left_ports,
        std::initializer_list<int8_t> right_ports, 
        double drive_rpm,
        double wheel_diameter,
        pros::MotorGearset chassis_gearset,
        dlib::BasicPID move_pid, 
        dlib::BasicPID turn_pid
    ) : 
        chassis(left_ports, right_ports, drive_rpm, wheel_diameter, chassis_gearset), 
        encoders(chassis), move_pid(move_pid), turn_pid(turn_pid) {
    };

    void initialize() {
        encoders.reset();
    }

    // TODO: Try to find a way to eliminate the 'glue code' that delegates to builtin methods
    void move_with_pid(double distance, dlib::PIDOptions options) {
        dlib::move_with_pid(
            chassis,
            encoders, 
            move_pid, 
            distance, 
            options
        );
    }
};

// instantiate a Controller object
pros::Controller master(pros::E_CONTROLLER_MASTER);

dlib::BasicPID move_pid = dlib::BasicPID({5, 0, 0});
dlib::BasicPID turn_pid = dlib::BasicPID({5, 0, 0});

Robot robot = Robot(
    {18,19,17},
    {-14,-16,-11}, 
    450, 
    3.25, 
    pros::MotorGearset::blue,
    move_pid, 
    turn_pid
);

void initialize() {}

void disabled() {
}

void competition_initialize() {}

void autonomous() {
    // Initialize
    robot.initialize();

    // Try some movements!
    robot.move_with_pid(24, dlib::PIDOptions{ .error_threshold = 1, .settle_ms = 250, .timeout_ms = 0});
}

void opcontrol() {
    while(true){
        
        pros::delay(20);
    }
}