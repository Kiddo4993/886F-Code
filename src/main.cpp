#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.hpp"

using namespace pros; // IWYU pragma: keep

pros::MotorGroup left_motors({-3, -7, -11}, pros::MotorGearset::blue); 
pros::MotorGroup right_motors({4, 6, 19}, pros::MotorGearset::blue); 
pros::MotorGroup liftMotor({-16, 20});
pros::Motor intakeMotors({5});
pros::Motor chainbar({18});
pros::Controller master(pros::E_CONTROLLER_MASTER); // create a controller object for the master controller

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);


pros::Imu imu(14);
pros::Rotation verticalrotation_sensor(-12);
pros::Rotation horizontalrotation_sensor(-13);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalrotation_sensor, lemlib::Omniwheel::NEW_2, 2);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&verticalrotation_sensor, lemlib::Omniwheel::NEW_2, 0);


lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// // horizontal tracking wheel
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// // vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// lateral PID controller

// );

// lemlib::ControllerSettings lateral_controller(9, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               40, // derivative gain (kD)
//                                               3, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               20 // maximum acceleration (slew)
// );

lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              4.8, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)

);
// angular PID controller
// lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               10, // derivative gain (kD)
//                                               3, // anti windup
//                                               1, // small error range, in degrees
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in degrees
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

lemlib::ControllerSettings angular_controller(1.9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(100);
        }
    });
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
// void initialize() {
// 	pros::lcd::initialize();
// 	pros::lcd::set_text(1, "Hello PROS User!");

// 	pros::lcd::register_btn1_cb(on_center_button);
// }

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

    // chassis.setPose(0, -59.5, 229.58);
    // chassis.moveToPoint(0, -63,800);

    // // chassis.setPose(0, 0, 0);
    
    // // chassis.turnToHeading(270, 1000);
    // // chassis.moveToPoint(0, -67, 1000);

    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 10000);
    // chassis.moveToPoint(0, 24, 1000);

    
    // chassis.setPose(0.00, -66.93, 180.0);         
    
    // chassis.moveToPoint(  0.00, -55,  700);                               // pt00  cm(0, -170) h=180
  
    // chassis.moveToPoint(  0.00, -66.93,  700);                               // pt02  cm(0, -170) h=180
    // chassis.waitUntilDone();
    // chassis.moveToPoint(  0.00, -57.09,  750);          // pt03  cm(0, -145) h=90
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-23.62, -57.09, 1300, {.forwards = false});          // pt04  cm(-60, -145) h=180
    // chassis.waitUntilDone();
    // chassis.turnToHeading(180.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-23.62, -51.18,  700, {.forwards = false});          // pt05  cm(-60, -130) h=180
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-23.62, -66.93, 1000, {.forwards = false});          // pt06  cm(-60, -170) h=0
    // chassis.waitUntilDone();
    // chassis.turnToHeading(180.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-23.62, -51.18, 1000, {.forwards = false});          // pt07  cm(-60, -130) h=180
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-23.62, -57.09,  700, {.forwards = false});          // pt08  cm(-60, -145) h=0
    // chassis.waitUntilDone();
    // chassis.moveToPoint(  0.00, -57.09, 1300);                               // pt09  cm(0, -145) h=0
    // chassis.waitUntilDone();
    // chassis.turnToHeading(180.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(  0.00, -25.59, 1600, {.forwards = false});          // pt10  cm(0, -65) h=180
    // chassis.waitUntilDone();
    // chassis.turnToHeading(316.2, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint( 19.48, -45.59, 1450, {.forwards = false});          // pt11  cm(49.485, -115.801) h=316.2
    // chassis.waitUntilDone();
    // chassis.moveToPoint( 12.74, -38.95,  750);                               // pt12  cm(32.363, -98.922) h=316.5
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint( 23.62, -57.09, 1200, {.forwards = false});          // pt13  cm(60, -145) h=0
    // chassis.waitUntilDone();
    // chassis.moveToPoint( 23.62, -66.93,  750, {.forwards = false});          // pt14  cm(60, -170) h=0
    // chassis.waitUntilDone();
    // chassis.turnToHeading(180.0, 800);
    // chassis.waitUntilDone();
    // chassis.moveToPoint( 23.62, -51.18, 1000, {.forwards = false});          // pt15  cm(60, -130) h=180
    // chassis.waitUntilDone();



    


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
	
    

	// while (true) {
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		// int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		// int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		// left_motors.move(dir + turn);                      // Sets left motor voltage
		// right_motors.move(dir - turn);                     // Sets right motor voltage
		// pros::delay(20);      

        // if (master.get_digital(DIGITAL_R1)) {
        //     liftMotor.move(127);
        // } else if (master.get_digital(DIGITAL_R2)) {
        //     liftMotor.move(-127);
        // } else if (master.get_digital(DIGITAL_L1)) {
        //     intakeMotors.move(127);
        // } else if (master.get_digital(DIGITAL_L2)) {
        //     intakeMotors.move(-127);
        // } else if (master.get_digital(DIGITAL_Y)) {
        //     chainbar.move(127);

        // } else if (master.get_digital(DIGITAL_B)) {
        //     chainbar.move(-127);
        // } else {
        //     liftMotor.move(0);
        //     intakeMotors.move(0);
        //     chainbar.brake();
        // }

    chainbar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (true) {
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        left_motors.move(dir + turn);
        right_motors.move(dir - turn);

        // Lift / intake — independent of chainbar now
        if (master.get_digital(DIGITAL_R1)) {
            liftMotor.move(127);
        } else if (master.get_digital(DIGITAL_R2)) {
            liftMotor.move(-127);
        } else {
            liftMotor.move(0);
        }

        if (master.get_digital(DIGITAL_L1)) {
            intakeMotors.move(127);
        } else if (master.get_digital(DIGITAL_L2)) {
            intakeMotors.move(-127);
        } else {
            intakeMotors.move(0);
        }

        // Chainbar — separate block, unaffected by any other button
        if (master.get_digital(DIGITAL_Y)) {
            chainbar.move(127);
        } else if (master.get_digital(DIGITAL_B)) {
            chainbar.move(-127);
        } else {
            chainbar.brake();  // actively holds position (HOLD brake mode)
        }

        pros::delay(20);

            



            
                                    // Run for 130 ms then update
    }
}


