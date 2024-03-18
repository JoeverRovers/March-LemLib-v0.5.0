#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"

// controller

// drive motors
pros::Motor lF(3, pros::E_MOTOR_GEARSET_06, true); // left front motor. port 12, reversed
pros::Motor lM(12, pros::E_MOTOR_GEARSET_06, false); // left middle motor. port 11, reversed
pros::Motor lB(13, pros::E_MOTOR_GEARSET_06, true); // left back motor. port 1, reversed
pros::Motor rF(19, pros::E_MOTOR_GEARSET_06, false); // right front motor. port 2
pros::Motor rM(20, pros::E_MOTOR_GEARSET_06, true); // right middle motor. port 11
pros::Motor rB(9, pros::E_MOTOR_GEARSET_06, false); // right back motor. port 13

//intake
pros::Motor intake(5, pros::E_MOTOR_GEAR_BLUE, false);
pros::ADIDigitalOut backLeft('F');
pros::ADIDigitalOut backRight('H');
pros::ADIDigitalOut frontLeft('C');
pros::ADIDigitalOut frontRight('G');

// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

//DT Set Up
lemlib::Drivetrain drivetrain {
    &leftMotors, // left drivetrain motors
    &rightMotors, // right drivetrain motors
    12, // track width
    lemlib::Omniwheel::NEW_325, // wheel diameter
    400, // wheel rpm
	2 //chase power
};
pros::Controller master(pros::E_CONTROLLER_MASTER);
// Inertial Sensor on port 2
pros::Imu imu1(7);

// tracking wheels
// Encoder
pros::Rotation l_enc(15, false); // horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
// pros::Rotation r_enc(15, false);
pros::Rotation h_enc(11, true);


// horizontal tracking wheel. 2.00" diameter, 3.7" offset, back of the robot (negative)
lemlib::TrackingWheel l_tracking_wheel(&l_enc, 2, -2.5, 1);
// lemlib::TrackingWheel r_tracking_wheel(&r_enc, 2, -3, 1);
lemlib::TrackingWheel h_tracking_wheel(&h_enc, 2, -8.5, 1);

// odometry struct
lemlib::OdomSensors sensors {
    &l_tracking_wheel, //&l_tracking_wheel, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    &h_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &imu1, // inertial sensor
};

// forward/backward PID
lemlib::ControllerSettings lateralController {
    10, // proportional gain (kP)
	0, // integral gain (kI)
	25, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in inches
	1000, // small error range timeout, in milliseconds
	3, // large error range, in inches
	10000, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
};
 
// turning PID
lemlib::ControllerSettings angularController {
    1.5, // kP
	0, // kI
	10, // kD
	3, // anti windup
	1, // small error range, in degrees
	1000, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	10000, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
};

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

void intakeIn(){
	intake.move(127);
}

void intakeOut(){
	intake.move(-127);
}

void intakeStop() {
	intake.move(0);
}

void raiseBackWings(){
	backLeft.set_value(true);
	backRight.set_value(true);
}

void lowerBackWings(){
	backLeft.set_value(false);
	backLeft.set_value(false);
}

void raiseFrontWings(){
	frontLeft.set_value(true);
	frontRight.set_value(true);
}

void lowerFrontWings(){
	frontLeft.set_value(false);
	frontRight.set_value(false);
}

// Set up screen
void screen() {
    // loop forever
    // while (true) {
    //     lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
    //     pros::lcd::print(0, "x: %f", pose.x); // print the x position
    //     pros::lcd::print(1, "y: %f", pose.y); // print the y position
    //     pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
    //     pros::delay(10);

    // }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "imutheta: %f", imu1.get_rotation()); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    // chassis.setPose(0, 0, 0);
    chassis.setPose(34, -54, 0);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}


// Auton Lists

void auton_Skills() {
	// chassis.setPose(5.2, 10.333, 45); // X: 5.2, Y: 10.333, Heading: 87

}

ASSET(firstPathv2_txt);
ASSET(path2_txt);

void far_side_auton() {
    intakeIn();
    chassis.moveToPoint(23, 0, 3000, {.minSpeed = 5, .earlyExitRange = 5}, false);
    chassis.moveToPoint(36, -50, 4000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 15});
    chassis.turnToHeading(45, 1000, {.minSpeed = 5, .earlyExitRange = 10}, false);
    intakeOut();
    pros::delay(400);
    intakeStop();
    chassis.moveToPose(0, -60, -90, 4000, {.minSpeed = 70, .earlyExitRange = 3}, false);
    intakeIn();
    // chassis.follow(firstPathv2_txt, 9, 4000, false);
    chassis.moveToPoint(48, -58, 4000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 15});
    lowerBackWings();
    chassis.turnToHeading(-45, 1000, {.minSpeed = 20, .earlyExitRange = 10}, false);
    chassis.moveToPose(60, -20, -180, 3000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 3});

    intakeStop();
    chassis.moveToPoint(60, -43, 1000, {.forwards = true, .minSpeed = 10, .earlyExitRange = 15});
    chassis.turnToHeading(0, 1000, {.minSpeed = 5, .earlyExitRange = 10}, false);
    chassis.moveToPoint(60, -20, 4000, {.minSpeed = 20, .earlyExitRange = 3}, false);
    intakeOut();
    pros::delay(400);
    intakeStop();
    chassis.moveToPoint(58, -43, 1000, {.forwards = false, .minSpeed = 15, .earlyExitRange = 15}, false);
    chassis.turnToHeading(-60, 1000, {.minSpeed = 5, .earlyExitRange = 10}, false);

    chassis.moveToPoint(4, -20, 4000, {.forwards = true, .minSpeed = 25, .earlyExitRange = 15}, false);
    intakeIn();
    chassis.turnToHeading(90, 1000, {.minSpeed = 5, .earlyExitRange = 10}, false);
    intakeStop();
    chassis.moveToPoint(48, 0, 2000, {.forwards = true, .minSpeed = 25, .earlyExitRange = 15}, false);
    intakeOut();
    pros::delay(400);
    intakeStop();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 5, .earlyExitRange = 10});
    chassis.moveToPoint(4, 4, 4000, {.forwards = true, .minSpeed = 25, .earlyExitRange = 15}, false);
    intakeIn();
    chassis.turnToHeading(90, 1000, {.minSpeed = 5, .earlyExitRange = 10}, false);
    intakeStop();
    chassis.moveToPoint(48, 0, 2000, {.forwards = true, .minSpeed = 25, .earlyExitRange = 15}, false);
    intakeOut();
    pros::delay(400);
    intakeStop();

    
}

void close_side_auton() {}

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

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void autonomous() {
    far_side_auton();
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
void opcontrol(){
    while(true){
        chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*10, controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)*10);
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            intakeIn();
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intakeOut();
        } else {
            intakeStop();
        }
    }

}