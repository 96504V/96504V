#include "lemlib/api.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// MotorGroups
pros::MotorGroup leftMotors({5, 2, 12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({6, 11, 3}, pros::MotorGearset::blue);
pros::MotorGroup intakeMotors({7, 8}, pros::MotorGearset::blue);

// IMU and encoders
pros::Imu imu(20);
pros::Rotation horizontalEncoder(19);
pros::Rotation verticalEncoder(18);

// LEMLib pointers (will be initialized in initialize())
lemlib::TrackingWheel* horizontal;
lemlib::TrackingWheel* vertical;
lemlib::Drivetrain* drivetrain;
lemlib::Chassis* chassis;

// Robot pose
double heading;
double x;
double y;

// Callback for LCD center button
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        heading = imu.get_heading();
        x = chassis->getPose().x;
        y = chassis->getPose().y;
    } else {
        pros::lcd::clear_line(2);
    }
}

// Initialization
void initialize() {
    pros::lcd::initialize();

    // Reverse left motors
    // leftMotors.set_reversed(true);

    // Create tracking wheels
    horizontal = new lemlib::TrackingWheel(&horizontalEncoder, 2.75, -5.75);
    vertical = new lemlib::TrackingWheel(&verticalEncoder, 2.75, -2.5);

    // Create drivetrain
    drivetrain = new lemlib::Drivetrain(&leftMotors, &rightMotors, 10, 2.75, 450, 2);

    // Controller settings
    lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
    lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

    // Sensors for odometry
    lemlib::OdomSensors sensors(vertical, nullptr, horizontal, nullptr, &imu);

    // Input curves
    lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
    lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

    // Create chassis
    chassis = new lemlib::Chassis(*drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

    // Calibrate chassis
    chassis->calibrate(false);

    // Register LCD callback
    pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        // Arcade drive
        chassis->arcade(
            controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
            controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)
        );

        // Intake control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeMotors.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeMotors.move(-127);
        } else {
            intakeMotors.move(0);
        }

        pros::delay(20);
    }
}
