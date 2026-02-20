#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut loader('A');
pros::adi::DigitalOut wing('B');
pros::adi::DigitalOut middleScore('C', true);
pros::MotorGroup leftMotors({-2, -14, -13}, pros::MotorGearset::blue); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup rightMotors({5, 11, 12}, pros::MotorGearset::blue); 
pros::MotorGroup intake({4, -3});
pros::Motor score({6});
pros::Imu imu(20); // initializes an Inertial sensor on port 20
pros::Rotation horizontalEncoder(10); // Initializes a rotation sensor on port 19
pros::Rotation verticalEncoder(9); 
lemlib::TrackingWheel horizontal(&horizontalEncoder, 2, .875); //Initializes a tracking wheel object that uses the horizontal encoder from line 9, has a wheel diameter 2in, and an offset of 5.75 inches to the left
lemlib::TrackingWheel vertical(&verticalEncoder, 2, -2.25);
int wheel_size = lemlib::Omniwheel::NEW_275;
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, wheel_size, 450, 8);
lemlib::ControllerSettings linearController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(1.8, // proportional gain (kP)
                                              0,  // integral gain (kI)
                                              1.6, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
                                              );
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
bool loading = false;
bool clearing = false;
bool middle = true;
int autonCount = 0;

/*void tuneLinear() {
    chassis.lateralPID.kI = 0; // Sets kI to 0 for tuning
    chassis.lateralPID.kD = 0; // Sets kD to 0 for
    int oscillations = 0; // Counts the number of oscillations
    const double eps = .1; // Defines the error threshold for counting oscillations
    int targetDistance = 48; // Defines the target distance in inches
    bool debounce = false; // Debounce variable to prevent multiple counts for a single oscillation
    chassis.lateralPID.kP -= 0.3; // Decreases the initial kP value so that it can be increased at the start of the loop. Done so that we only have to check oscillations < 3 at loop end

    // Tuning Proportional Gain (kP)
    
    while (oscillations < 3) {
        chassis.turnToHeading(180, 50000); // Resets robot heading to 180 degrees to prevent imu drift from affecting movement
        chassis.waitUntilDone();
        debounce = false; // Resets debounce for each kP test
        oscillations = 0; // Resets oscillation count for each kP test
        chassis.lateralPID.kP += 0.3; // Increases the kP value by 0.3 for the next iteration
        chassis.setPose(0, 0, 0); // Redefines current position as the origin
        double lastError = 0; // Keeps track of the last error value
        chassis.moveToPoint(0, targetDistance, 5000000,  {.maxSpeed = 60}); // Moves the robot to the target distance at a max speed of ~50%
        while (chassis.isInMotion()) {
            lemlib::Pose pose = chassis.getPose(); //*
            double x = pose.x; // * 
            double y = pose.y; // *
            double distance = sqrt((x * x) + (y * y)); // Measures the distance traveled
            double error = targetDistance - distance; // Calculates the current error
            if (!debounce && ((lastError > eps && error < -eps) || (lastError < -eps && error > eps))) { // Checks if an oscillation has occurred by seeing if the last error was positive and the current negative and vice versa
                oscillations++;
                debounce = true;
            }
            if (debounce && (fabs(error - lastError) < 0.05 || fabs(error) < eps)) {
                debounce = false; // Resets debounce when within the error threshold
            }
            lastError = error;
            pros::delay(10);
        }
    }
    chassis.lateralPID.kP *= .55; // Damps oscillation by a Ziegler-Nichols factor of 0.55 to get the final kP value 
    pros::lcd::set_text(3, "Oscillations reached at kP: " + std::to_string(chassis.lateralPID.kP)); // Displays the tuned kP value on the LCD
    
    // Tuning Derivative Gain (kD)

    bool tuningStarted = false; // Used to enter the loop the first time
    chassis.setPose(0, 0, 0); // Redefines current position as the origin
    int maxOvershoot = 0; // Keeps track of the maximum overshoot
    double desiredOvershoot = .5; // Defines an acceptable overshoot of .5 inches
    chassis.lateralPID.kD -= .1; // Decreases the initial kD value so that it can be increased at the start of the loop. Done so that we only have to check overshoot > desiredOvershoot at loop end
    while (!tuningStarted || maxOvershoot > desiredOvershoot) {
        chassis.lateralPID.kD += .1; // Increases kD by 0.1 for the next iteration
        chassis.turnToHeading(180, 50000); // Resets robot heading to 180 degrees to prevent imu drift from affecting movement
        chassis.waitUntilDone();
        tuningStarted = true;
        maxOvershoot = 0; // Resets max overshoot for each kD test
        chassis.setPose(0, 0, 0); // Redefines current position as the origin
        chassis.moveToPoint(0, targetDistance, 5000000,  {.maxSpeed = 60}); // Moves the robot to the target distance at a max speed of ~50%
       int loops = 0;
        while (chassis.isInMotion()) {
            lemlib::Pose pose = chassis.getPose(); //*
            double x = pose.x; // * 
            double y = pose.y; // *
            double distance = sqrt((x * x) + (y * y)); // Measures the distance traveled
            if (distance - targetDistance > maxOvershoot) {
                maxOvershoot = distance - targetDistance; // Updates max overshoot if current overshoot is larger
            }
            pros::delay(10);
        }
   }
    pros::lcd::set_text(4, "Dampening achieved at kD: " + std::to_string(chassis.lateralPID.kD)); // Displays the tuned kD value on the LCD
}

void tuneAngular() {} */

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void load(double y) {
        intake.move(127);
        chassis.moveToPoint(0, y + 2, 1000);
        chassis.waitUntilDone();
        chassis.moveToPoint(0, y, 1000);
        chassis.waitUntilDone();
        pros::delay(70);
    intake.move(127);
    pros::delay(30);
    chassis.moveToPoint(0, 0, 1000);
    pros::delay(30);
}

void unload() {
    for (int i = 0; i < 5; i++) {
        intake.move(127);
        score.move(127);
        pros::delay(50);
    }
    intake.move(0);
    score.move(0);
}

void allianceAutonLeft() {
    double y = 14.8;
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 37.25, 1000, {.maxSpeed = 10000});
    chassis.waitUntilDone();
    loading = !loading;
    loader.set_value(loading);
    chassis.turnToHeading(-87, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    chassis.moveToPoint(0, y, 1000);
    pros::delay(200);
    load(y);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -32, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(50);
    loading = !loading;
    loader.set_value(loading);
    unload();
    chassis.waitUntilDone();
}

void allianceAutonRight() {
    double y = 14.8;
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 37.25, 1000, {.maxSpeed = 10000});
    chassis.waitUntilDone();
    loading = !loading;
    loader.set_value(loading);
    chassis.turnToHeading(87, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    chassis.moveToPoint(0, y, 1000);
    pros::delay(200);
    load(y);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -32, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(50);
    loading = !loading;
    loader.set_value(loading);
    unload();
    chassis.waitUntilDone();
}

void skillsAuton() {
    double y = 14.8;
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 37.25, 1000, {.maxSpeed = 10000});
    chassis.waitUntilDone();
    loading = !loading;
    loader.set_value(loading);
    chassis.turnToHeading(-87, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    pros::delay(200);
    chassis.moveToPoint(0, y, 1000);
    pros::delay(200);
    load(y);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, -32, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(50);
    loading = !loading;
    loader.set_value(loading);
    unload();
    chassis.waitUntilDone();
    pros::delay(3000);
    chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(-132, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 37, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(51, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 55, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(48, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 37, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -20, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(30);
    unload();
    score.move(0);
    pros::delay(30);
    loading = !loading;
    loader.set_value(loading);
    chassis.moveToPoint(0, 20, 1000);
    chassis.waitUntilDone();
    load(10);
    pros::delay(20);
    chassis.moveToPoint(0, -20, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(30);
    unload();
    pros::delay(30);
}

void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(true);
    pros::lcd::initialize();
    pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    chassis.setPose(0, 0, 0);
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        autonCount = 0;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        autonCount = 1;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        autonCount = 2;
    }
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // skillsAuton(10);
    if (autonCount == 0) {
        allianceAutonLeft();
    }
    if (autonCount == 1) {
        allianceAutonRight();
    }
    if (autonCount == 2) {
        skillsAuton();
    }
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    clearing = !clearing;
    wing.set_value(clearing);
    while (true) {   
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            chassis.setPose(0, 0, 0);
            skillsAuton();
            pros::delay(400000);
        }
        chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), false, 0.75);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            score.move(127);
            intake.move(127);
        } else {
            score.move(0);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            loading = !loading;
            loader.set_value(loading);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            clearing = !clearing;
            wing.set_value(clearing);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            middle = !middle;
            middleScore.set_value(middle);
        }
        pros::delay(20);// Run for 20 ms then update

    }
}