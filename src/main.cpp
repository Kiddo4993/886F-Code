#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

// Hardware Setup
pros::MotorGroup leftMotors({-3, -4, -5}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({6, 7, 8}, pros::MotorGearset::blue);
pros::MotorGroup liftMotor({-11, 12});

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics wrist('H', false, true);
pros::adi::Pneumatics intakeLift('G', false, true);

pros::Motor intake(-13, pros::v5::MotorGears::blue);
pros::Motor clamp(-1, pros::v5::MotorGears::blue);

// Drivetrain and odometry
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    9.5,                         // Track width in inches
    lemlib::Omniwheel::NEW_275,
    450,                         // Drive motor RPM
    2                            // Drivetrain horizontal drift
);

pros::Imu imu(14);
pros::Rotation verticalRotationSensor(15);
pros::Rotation horizontalRotationSensor(16);
pros::Rotation liftRotation(-20);

// Odometry Set up

lemlib::TrackingWheel horizontalTrackingWheel(
    &horizontalRotationSensor, 2.0, 0.175
);
lemlib::TrackingWheel verticalTrackingWheel(
    &verticalRotationSensor, 2.0, 1.25
);

lemlib::OdomSensors odomSensors(
    &verticalTrackingWheel,
    nullptr,
    &horizontalTrackingWheel,
    nullptr,
    &imu
);

lemlib::ControllerSettings lateralController(
    10,     // kP: proportional gain
    0,      // kI: integral gain
    50,     // kD: derivative gain
    3,      // Small error threshold
    1,      // Small error timeout (ms)
    100,    // Large error threshold
    3,      // Large error timeout (ms)
    500,    // Slew rate
    0       // Optional setting
);

lemlib::ControllerSettings angularController(
    1,      // kP: proportional gain
    0,      // kI: integral gain
    7,      // kD: derivative gain
    3,      // Small error threshold
    1,      // Small error timeout (ms)
    100,    // Large error threshold
    3,      // Large error timeout (ms)
    500,    // Slew rate
    0       // Optional setting
);

lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    odomSensors
);

// Lift settings and PID state
constexpr int liftMaxHeight = 216000;
constexpr int liftMinHeight = 10000;
constexpr int manualLiftControl = -1;

static volatile int liftTarget = manualLiftControl;
static volatile int liftLoops = 0;
static volatile int liftLastTarget = manualLiftControl;
static volatile int liftOutput = 0;

void spinTurn(int speed, int theta, int error) {
    while (true) {
        double difference = theta - imu.get_heading();

        if (difference > 180) difference -= 360;
        if (difference < -180) difference += 360;

        if (std::abs(difference) <= error) break;

        if (difference > 0) {
            leftMotors.move(speed/8);
            rightMotors.move(-speed);
        }
        else {
            leftMotors.move(-speed);
            rightMotors.move(speed/8);
        }

        pros::delay(10);
    }

    leftMotors.move(0);
    rightMotors.move(0);
}

void liftToPosition(int targetPosition) {
    liftTarget = targetPosition;
}

void stopLiftMacro() {
    liftTarget = manualLiftControl;
}

enum class macroStepState {
    idle,
    singleClampBack,
    singleWaitUp,
    matchWaitDown
};

macroStepState macroStep = macroStepState::idle;
uint32_t macroStepStart = 0;

// Function for single pin scoring

void startSinglePin() {
    stopLiftMacro();
    wrist.retract();
    liftMotor.move(-127);
    pros::delay(250);

    wrist.extend();
    clamp.move(-127);
    liftMotor.move(0);
    pros::delay(500);

    wrist.retract();
    liftToPosition(10000);
    clamp.move(60);

    macroStep = macroStepState::idle;
}

// Function for Matchloading

void startMatchLoad() {
    stopLiftMacro();

    uint32_t startTime = pros::millis();
    while (liftRotation.get_position() > 0 &&
           pros::millis() - startTime < 1500) {
        liftMotor.move(-127);
        pros::delay(10);
    }
    liftMotor.brake();
    if (liftRotation.get_position() > 0) {
        return;
    }
    clamp.move(127);
    pros::delay(200);
    liftToPosition(10000);
}

// Function for auto for lift

void startIntakeLift(){
    intake.move(127);
    clamp.move(127);
    intakeLift.retract();
    pros::delay(1000);
    intakeLift.extend();
    leftMotors.move(-127);
    rightMotors.move(-127);
    pros::delay(100);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(1000);
    intake.move(0);
    clamp.move(60);

    macroStep = macroStepState::matchWaitDown;
    macroStepStart = pros::millis();
}

// Intializing all the functions for the start

void initialize() {

    wrist.set_value(false);
    intakeLift.set_value(false);
    liftMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    liftRotation.reset_position();
    pros::lcd::initialize();

    chassis.calibrate();

    pros::Task screenTask([] {
        // For testing purposes 
        while (true) {
            const auto pose = chassis.getPose();

            pros::lcd::print(0, "X: %.2f", pose.x);
            pros::lcd::print(1, "Y: %.2f", pose.y);
            pros::lcd::print(2, "Theta: %.2f", pose.theta);
            pros::lcd::print(3, "Lift: %d", liftRotation.get_position());
            pros::lcd::print(4, "Target: %d Last: %d", liftTarget, liftLastTarget);
            pros::lcd::print(5, "Output: %d Loops: %d", liftOutput, liftLoops);
            pros::lcd::print(6, "Motor velocity: %.0f", liftMotor.get_actual_velocity());

            pros::delay(100);
        }
    });

    // Lift Macro

    pros::Task liftTask([] {
        constexpr double kP = 0.012;
        constexpr double kI = 0.0;
        constexpr double kD = 0.01;
        constexpr double errorThreshold = 100.0;

        double previousError = 0;
        double integral = 0;
        int activeTarget = manualLiftControl;

        while (true) {
            const int target = liftTarget;

            if (target != manualLiftControl) {
                const double currentPosition = liftRotation.get_position();
                const double error = target - currentPosition;

                if (target != activeTarget) {
                    previousError = error;
                    integral = 0;
                    activeTarget = target;
                }

                integral += error;
                const double derivative = error - previousError;
                double output = kP * error + kI * integral + kD * derivative;
                constexpr double minOutput = 15.0;

                if (std::abs(error) > errorThreshold && std::abs(output) < minOutput) {
                    output = (output > 0) ? minOutput : -minOutput;
                }

                output = std::clamp(output, -127.0, 127.0);

                liftMotor.move(static_cast<int>(output));
                liftOutput = static_cast<int>(output);
                liftLastTarget = target;

                if (std::abs(error) < errorThreshold) {
                    liftMotor.brake();
                    liftTarget = manualLiftControl;
                }

                previousError = error;
            } else {
                activeTarget = manualLiftControl;
            }

            ++liftLoops;
            pros::delay(10);
        }
    });
    liftToPosition(10000);
}

void autoskills() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

    // controller commands 

    bool matchLoadReady = false;
    while (true) {
        const int drive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = static_cast<int>(
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.8
        );
        const int liftPosition = liftRotation.get_position();

        leftMotors.move(drive + turn);
        rightMotors.move(drive - turn);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            stopLiftMacro();
            macroStep = macroStepState::idle;
            wrist.retract();

            if (liftPosition < liftMaxHeight) {
                liftMotor.move(127);
            } else {
                liftMotor.brake();
            }
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            stopLiftMacro();
            macroStep = macroStepState::idle;

            if (liftPosition > 14000) {
                liftMotor.move(-127);
            } else if (liftPosition > 10000) {
                liftMotor.move(-15);
            } else {
                liftMotor.brake();
            }
        }
        else if (liftTarget == manualLiftControl) {
            liftMotor.brake();
        }
        // Intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            wrist.extend();
            intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        if (macroStep == macroStepState::idle) {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                clamp.move(127);
            } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                clamp.move(-127);
            } else {
                clamp.move(60);
            }
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            startIntakeLift();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            master.rumble(".");
            pros::lcd::print(7, "B triggered: %u", pros::millis());
            startSinglePin();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)
            && macroStep == macroStepState::idle) {
            if (!matchLoadReady) {
                liftToPosition(19000);
                wrist.retract();
                matchLoadReady = true;
            } else {
                startMatchLoad();
                matchLoadReady = false;
            }
        }
        pros::delay(20);
    }
}
