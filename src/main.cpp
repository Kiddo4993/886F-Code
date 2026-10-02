#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.h"
#include "pros/rotation.hpp"

using namespace pros; // IWYU pragma: keep

pros::MotorGroup right_motors({1, 2, 3}, pros::MotorGearset::blue); 
pros::MotorGroup left_motors({-4, -5, -6}, pros::MotorGearset::blue); 
pros::MotorGroup liftMotor({-11, 12});
// pros::Motor intakeMotors({});
// pros::Motor chainbar({});
pros::Controller master(pros::E_CONTROLLER_MASTER); // create a controller object for the master controller
pros::adi::Pneumatics intake_piston('H', false, true); // create a piston object for the pneumatic piston on port 1
pros::Motor intake(13, pros::v5::MotorGears::blue);
pros::Motor clamp(7, pros::v5::MotorGears::blue);
// pros::Rotation chainbar_encoder(17);
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);


pros::Imu imu(14);
pros::Rotation verticalrotation_sensor(15);
pros::Rotation horizontalrotation_sensor(-16);
pros::Rotation liftRotation(18);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalrotation_sensor, 2.0, -1);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&verticalrotation_sensor, 2.0, 1.2);


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
                                              15, // derivative gain (kD)
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

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings  
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


static pros::Task* liftTask = nullptr;
static volatile bool liftStopRequested = false;
static volatile bool liftTaskFinished = true;

void StopLiftMacro() {
    liftStopRequested = true;
}

// Moves the lift to a target in rotation-sensor centidegrees without blocking.
void LiftToPosition(int targetPosition) {
    StopLiftMacro();

    if (liftTask != nullptr) {
        while (!liftTaskFinished) {
            pros::delay(5);
        }
        delete liftTask;
        liftTask = nullptr;
    }

    liftStopRequested = false;
    liftTaskFinished = false;

    liftTask = new pros::Task([targetPosition]() {
        constexpr double kP = 0.35;
        constexpr double kI = 0.0;
        constexpr double kD = 0.08;
        constexpr double errorThreshold = 100.0;
        constexpr int stableCyclesRequired = 5;
        constexpr uint32_t timeout = 2500;

        double previousError = 0;
        double integral = 0;
        int stableCycles = 0;
        const uint32_t startTime = pros::millis();

        while (!liftStopRequested && pros::millis() - startTime < timeout) {
            const double error = targetPosition - liftRotation.get_position();
            integral += error;
            const double derivative = error - previousError;
            double output = (kP * error) + (kI * integral) + (kD * derivative);

            if (output > 127) output = 127;
            if (output < -127) output = -127;
            liftMotor.move(output);

            if (error > -errorThreshold && error < errorThreshold) {
                stableCycles++;
                if (stableCycles >= stableCyclesRequired) break;
            } else {
                stableCycles = 0;
            }

            previousError = error;
            pros::delay(10);
        }

        liftMotor.brake();
        liftTaskFinished = true;
    });
}
// // initialize function. Runs on program startup
void initialize() {
    intake_piston.set_value(false);
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::initialize(); // initialize brain screen
    // chainbar_encoder.reset_position(); // zero the chainbar encoder wherever it is at power-on
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Lift Degrees: %f", liftRotation.get_position());
            // pros::lcd::print(3, "Chainbar: %d", chainbar_encoder.get_position()); // chainbar position, in encoder ticks (centidegrees)
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
//  pros::lcd::initialize();
//  pros::lcd::set_text(1, "Hello PROS User!");

//  pros::lcd::register_btn1_cb(on_center_button);
// }

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void autoskills() {

}

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

    // autoskills();
//    chassis.setPose(0,0,0);
//    chassis.moveToPoint(0, 24, 2000);
//    chassis.turnToHeading(90, 2000);



    // // Right side 
    // // chainbar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-62, 0, 270.0);  
    
    //  //chainbar to score position
    
    intake_piston.set_value(false);
    chassis.moveToPoint(-68,   0,  400, {.forwards = true}, false);          
    chassis.moveToPoint(-59,   0,  400, {.forwards = false, .minSpeed = 110}, false);          
    chassis.moveToPoint(-68,   0,  400, {.forwards = true}, false);
    chassis.moveToPoint(-59,   0,  400, {.forwards = false, .minSpeed = 110}, false);         
    chassis.moveToPoint(-68,   0,  400, {.forwards = true}, false);   
    //roller done
    // chassis.turnToHeading(0, 600, {}, false);


    // chassis.moveToPoint(-58.5,   -17,  1200, {.forwards = true}, false);

    // chassis.turnToHeading(302, 600, {}, false);
    // chassis.moveToPoint(-55,   -17.55,  800, {.forwards = true}, false);
    // delay(300);
    // intake_piston.set_value(true);
    // // score first pin
    // chassis.moveToPoint(-59,   -16.5,  1200, {.forwards = true}, false);

    // chassis.turnToHeading(42, 600, {}, false);
    // chassis.moveToPoint(-61,   -14.5,  800, {.forwards = false}, false);
    // // chainbarFunction(-94000);
    // delay(600);
    // intake_piston.set_value(false);
    // delay(500);
    // chassis.moveToPoint(-59,   -16.5,  1200, {.forwards = true}, false);
    // //got second cone

    // chassis.turnToHeading(180, 600, {}, false);
    
    // chassis.moveToPoint(-59,   16.5,  1200, {.forwards = false}, false);

    // // chainbarFunction(-80000);
    // intake_piston.set_value(true);

    // // put in second cone 

    // chassis.moveToPoint(-59,   16.5,  1200, {.forwards = false}, false);

    //left --------------------------------------------

    // chainbar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // chassis.setPose(0, -62, 180);  
    
    // intake_piston.set_value(false);
    // chassis.moveToPoint(0,   -58,  500, {.forwards = false}, false);          
    // chassis.moveToPoint(0,   -64,  500, {.forwards = true, .minSpeed = 60}, false);          
    // chassis.moveToPoint(0,   -58,  500, {.forwards = false}, false);
    // chassis.moveToPoint(0,   -64,  500, {.forwards = true, .minSpeed = 60}, false);         
    // chassis.moveToPoint(0,   -49.7,  1000, {.forwards = false}, false);   
    // chainbarFunction(-2500);
    // chassis.turnToHeading(271, 600, {}, false);
    // chassis.moveToPoint(-18.2,   -49.7,  900, {.forwards = true}, false);
    // chainbarFunction(0);
    // delay(300);
    // intake_piston.set_value(true);
    
    // delay(300);
    // chainbarFunction(0);
    // chassis.moveToPoint(-8,   -51,  900, {.forwards = false}, false); 
    // chassis.turnToHeading(323, 600, {}, false);
    // chassis.moveToPoint(-24.5,   -27,  1900, {.forwards = true, .maxSpeed = 30}, true); 
    // chassis.waitUntil(20.9);
    // intake_piston.set_value(false);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0, 600, {}, false);
    // chainbarFunction(-15000);
    // chassis.moveToPoint(-26,   -45,  1500, {.forwards = false}, false); 
    // chainbarFunction(-108000);
    // delay(2000);
    // intake_piston.set_value(true);
    //  chainbarFunction(-8000);

   


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

    // chainbar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (true) {
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X)*0.8;
        left_motors.move(dir + turn);
        right_motors.move(dir - turn);

        // Lift / intake — independexant of chainbar now
        if (master.get_digital(DIGITAL_L1)) {
            StopLiftMacro();
            liftMotor.move(127);
        } else if (master.get_digital(DIGITAL_L2)) {
            StopLiftMacro();
            liftMotor.move(-127);
        }

        if (master.get_digital(DIGITAL_R1)) {
            intake.move(127);
        } else if (master.get_digital(DIGITAL_R2)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        if (master.get_digital(DIGITAL_UP)) {
            clamp.move(127);
        } else if (master.get_digital(DIGITAL_DOWN)) {
            clamp.move(-127);
        } else {
            clamp.move(0);
        }

        // if (master.get_digital(DIGITAL_R2)) {
        //     chainbar.move(75);
        // } else if (master.get_digital(DIGITAL_R1)) {
        //     chainbar.move(-75);
        // } else {
        //     chainbar.brake();
        // }

        // Chainbar — separate block, unaffected by any other butto
        if(master.get_digital_new_press(DIGITAL_X)) {
            intake_piston.toggle(); // toggle piston state
        }
    
        // if (master.get_digital_new_press(DIGITAL_X)) {
        //     intake_piston.set_value(false); // retract piston
        //     chainbarFunction(-91427); // macro: drive chainbar to preset position
        // }
        // if(master.get_digital_new_press(DIGITAL_DOWN)) {
             
        //     chainbarFunction(-90000);
        //     intake_piston.set_value(true);
        // }
        // else if(master.get_digital_new_press(DIGITAL_UP)) {
        //     intake_piston.set_value(true); 
        //     chainbarFunction(-105600);
        // }
        pros::delay(20);
            
                                    // Run for 130 ms then update
    }
}
