#include "api.h"
#include "dlib/chassis.hpp"
#include "dlib/pid.hpp"
#include "dlib/odom.hpp"
#include "pros/motors.h"
#include "dlib/imu.hpp"
#include <concepts>

namespace dlib {

template<typename Robot>
void set_brake_mode_brake(Robot& robot){
    auto chassis = robot.get_chassis();

    chassis.left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

template<typename Robot>
void set_brake_mode_coast(Robot& robot){
    auto chassis = robot.get_chassis();

    chassis.left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

template<typename Robot>
void set_brake_mode_hold(Robot& robot){
    auto chassis = robot.get_chassis();

    chassis.left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

template<typename Robot>
void calibrate(Robot& robot){
    auto chassis = robot.get_chassis();

    chassis.left.set_gearing_all(pros::E_MOTOR_GEAR_BLUE);
    chassis.right.set_gearing_all(pros::E_MOTOR_GEAR_BLUE);

    chassis.left.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    chassis.right.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

    robot.get_imu().imu.reset(true);
}

template<typename Robot>
void tare_position(Robot& robot){
    auto chassis = robot.get_chassis();

    chassis.left.tare_position_all();
    chassis.right.tare_position_all();
}

template<typename Robot>
void brake_motors(Robot& robot)  {
    auto chassis = robot.get_chassis();

    chassis.left.brake();
    chassis.right.brake();
}

template<typename Robot>
void move_voltage(Robot& robot, std::int32_t power) {
    auto chassis = robot.get_chassis();

    chassis.left.move_voltage(power);
    chassis.right.move_voltage(power);
}

// Turn at a given voltage
template<typename Robot>
void turn_voltage(Robot& robot, std::int32_t power) {
    auto chassis = robot.get_chassis();

    chassis.left.move_voltage(-power);
    chassis.right.move_voltage(power);
}

// Drive using the arcade scheme
template<typename Robot>
void arcade(Robot& robot, std::int8_t power, std::int8_t turn) {
    auto chassis = robot.get_chassis();

    chassis.left.move(power - turn);
    chassis.right.move(power + turn);
}

// Constrain the angle to -180 to 180 for efficient turns

inline double angle_within_180(double degrees){
    degrees = std::fmod(degrees, 360);

    if(degrees > 180){
        degrees -= 360;
    }

    return degrees;
}

// Get the current heading from -180 to 180 degrees
template<typename Robot>
double get_imu_heading(Robot& robot) {

    double angle = std::fmod(robot.get_imu().imu.get_heading(), 360.0);
    return angle;
}

template<typename Robot>
double get_imu_rotation(Robot& robot) {
    return robot.get_imu().imu.get_rotation();
}

inline double average(std::vector<double> values) {
    double sum = 0;

    for(auto value : values) {
        sum += value;
    }

    return sum / values.size();
}

// Get the average inches the drivebase motors have moved
template<typename Robot>
double get_ime_displacement(Robot& robot) {
    auto diameter = robot.get_chassis().wheel_diameter;
    auto rpm = robot.get_chassis().rpm;

    std::vector<double> distances;

    auto left_gearsets = robot.get_chassis().left.get_gearing_all();
    auto left_positions = robot.get_chassis().left.get_position_all();
    
    for(int i = 0; i < left_positions.size(); i++) {
        double in;

        switch (left_gearsets[i]) {
            case pros::MotorGears::red: in = 100; break;
            case pros::MotorGears::green: in = 200; break;
            case pros::MotorGears::blue: in = 600; break;
            default: in = 200; break;
        }

        distances.push_back(left_positions[i] * (diameter * M_PI) * (rpm / in));
    }

    auto right_gearsets = robot.get_chassis().right.get_gearing_all();
    auto right_positions = robot.get_chassis().right.get_position_all();

    for(int i = 0; i < right_positions.size(); i++) {
        double in;

        switch (right_gearsets[i]) {
            case pros::MotorGears::red: in = 100; break;
            case pros::MotorGears::green: in = 200; break;
            case pros::MotorGears::blue: in = 600; break;
            default: in = 200; break;
        }

        distances.push_back(right_positions[i] * (diameter * M_PI) * (rpm / in));
    }

    return average(distances);
};

template<typename Robot>
void update_odom(Robot& robot){
    auto odom = robot.get_odom();

    while(true){
        odom.update(get_ime_displacement(robot), get_imu_heading(robot) * (M_PI / 180.00)); 
        pros::delay(10);
    }
}

template<typename Robot>
void start_odom_update_loop(Robot& robot) {
    auto odom = robot.get_odom();

    // Capture the current class instance and call update_odom in a seperate task
    if (!odom.odom_started) {
        odom.odom_updater = std::make_unique<pros::Task>([&] { update_odom(robot); });
        odom.odom_started = true;
    }
}

template<typename Robot>
void move_pid(Robot& robot, double inches, Options options) {
    auto drive_pid = robot.get_drive_pid();

    long interval = drive_pid.get_interval();

    double starting_inches = get_ime_displacement(robot);
    double target_inches = starting_inches + inches;
    drive_pid.reset();

    uint32_t starting_time = pros::millis();
    
    bool is_settling = false;
    uint32_t settle_start = 0;
    
    while (true) {
        
        uint32_t current_time = pros::millis();

        if (current_time - starting_time > options.max_ms) {
            break;
        }

        if (!is_settling && std::abs(drive_pid.get_error()) < options.error_threshold) {
            is_settling = true;
            settle_start = pros::millis();
        }

        if (is_settling) {
            if (std::abs(drive_pid.get_error()) < options.error_threshold) {
                if(current_time - settle_start > options.settle_ms) {
                    break;
                }
            } 
            else {
                is_settling = false;
            }
        }

        double current_inches = get_ime_displacement(robot);
        double error = target_inches - current_inches;
        double output_voltage = drive_pid.update(error);
        
        if(std::abs(output_voltage) > options.max_voltage){
            if(output_voltage > 0){
                output_voltage = options.max_voltage;
            }
            else{
                output_voltage = -options.max_voltage;
            }
        }
        output_voltage += std::copysign(error, 900);

        move_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    brake_motors(robot);
}

// Turn to a given absolute angle using PID
template<typename Robot>
void turn_pid(Robot& robot, double degrees, const Options options) {
    auto turn_pid = robot.get_turn_pid();
    auto imu = robot.get_imu();
    
    turn_pid.reset();

    double target_angle = degrees;
    uint32_t starting_time = pros::millis();
    bool is_settling = false;
    uint32_t settle_start = 0;
    uint32_t interval = turn_pid.get_interval();

    while (true) {
        uint32_t current_time = pros::millis();

        if (current_time - starting_time > options.max_ms) {
            break;
        }

        if (!is_settling && std::abs(turn_pid.get_error()) < options.error_threshold) {
            is_settling = true;

            settle_start = pros::millis();
        }

        double current_angle = imu.get_heading();
        double error = std::remainder(target_angle - current_angle,360);

        if (is_settling) {
            if (std::abs(turn_pid.get_error()) < options.error_threshold) {
                if(current_time - settle_start > options.settle_ms) {
                    break;
                }
            } 
            else {
                is_settling = false;
            }
        } 

        double output_voltage = turn_pid.update(error);
        
        output_voltage += std::copysign(error, 2100);
        if (output_voltage > options.max_voltage) {
            output_voltage = options.max_voltage;
        }

        turn_voltage(robot, output_voltage);

        pros::delay(interval);
    }

    brake_motors(robot);
}

template<typename Robot>
// Calculate the angle to turn from the current position to a point
double angle_to(Robot& robot, double x, double y, bool reverse){
    double target_x = x;
    double target_y = y;

    Position new_position = robot.get_odom().get_position();

    double current_x = new_position.x;
    double current_y = new_position.y;

    double angle = std::atan2(target_y - current_y, target_x - current_x) * (180.0 / M_PI);

    // If reverse is true add 180 so that the bot faces the opposite direction
    if(reverse){
        angle += 180;
    }

    return angle_within_180(angle);
}

template<typename Robot>
// Calculate the distance from the current position to a given point
double dist_to(Robot& robot, double x, double y, bool reverse){
    double target_x = x;
    double target_y = y;

    Position new_position = robot.get_odom().get_position();
    double current_x = new_position.x;
    double current_y = new_position.y;
    
    double dist = std::hypot(target_x - current_x, target_y - current_y);
    // The bot will move in the opposite direction
    if(reverse){
        dist = -dist;
    }

    return dist;
}

// Get the robot's current x, y and theta
template<typename Robot>
Position get_position(Robot& robot, bool radians = false) {
    Position position = robot.get_odom().get_position();

    if (!radians) {
        position.theta *= 180.0 / M_PI;
    }

    return position;
}

// Set the robot's current x, y and theta
template<typename Robot>
void set_position(Robot& robot, Position new_position, bool radians) {
    if (radians) {
        new_position.theta *= M_PI / 180.0;
    }

    robot.get_odom().odom.set_position(new_position);
}

template<typename Robot>
// Turn to a coordinate point using Odometry and PID
void turn_to(Robot& robot, double x, double y, bool reverse, Options options){
    double target_angle = angle_to(robot, x, y, reverse);
    turn_pid(robot, target_angle,  options);
}

template<typename Robot>
// Move to a coordinate point using Odometry and PID
void move_to(Robot& robot, double x, double y, bool reverse, Options move_options, Options turn_options) {
    turn_to(robot, x, y, reverse, turn_options);
    double dist = dist_to(robot, x, y, reverse);
    move_pid(robot, dist, move_options);
}

}