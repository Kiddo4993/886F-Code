#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include <algorithm>
#include <cmath>

using namespace pros;

// Motors and controller
MotorGroup right_motors({3, 4, 5}, MotorGearset::blue);
MotorGroup left_motors({-6, -7, -8}, MotorGearset::blue);
// Flipped motor signs so positive power (+127) moves lift UP
MotorGroup liftMotor({11, -12});
Motor intake(13, v5::MotorGears::blue);
Motor clamp(1, v5::MotorGears::blue);
Controller master(E_CONTROLLER_MASTER);

// Pneumatics
adi::Pneumatics wrist('H', false, true);
adi::Pneumatics intake_lift('G', false, true);

// Sensors
Imu imu(14);
Rotation verticalrotation_sensor(15);
Rotation horizontalrotation_sensor(-16);
// Reversing port sign (-20) ensures going UP yields positive readings
Rotation liftRotation(-20);

// Drivetrain and odometry
lemlib::Drivetrain drivetrain(
    &left_motors, &right_motors, 9.5,
    lemlib::Omniwheel::NEW_275, 450, 2
);

lemlib::TrackingWheel horizontal_tracking_wheel(
    &horizontalrotation_sensor, 2.0, -1
);
lemlib::TrackingWheel vertical_tracking_wheel(
    &verticalrotation_sensor, 2.0, 1.2
);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, nullptr,
    &horizontal_tracking_wheel, nullptr, &imu
);

lemlib::ControllerSettings lateral_controller(
    6, 0, 15, 0,
    0, 0, 0, 0, 0
);

lemlib::ControllerSettings angular_controller(
    2, 0, 10, 3,
    1, 100, 3, 500, 0
);

lemlib::Chassis chassis(
    drivetrain, lateral_controller, angular_controller, sensors
);

// Soft Limit Thresholds (Adjust based on LCD screen readings)
constexpr int LIFT_MAX_HEIGHT = 216000; // Peak position UP
constexpr int LIFT_MIN_HEIGHT = 0;      // Rest position DOWN

// Macro State Control (-1 = Manual Control Mode)
static volatile int liftTarget = -1;

// DEBUG: lift macro diagnostics shown on the brain screen
static volatile int liftLoops = 0;      // counts up while the PID task is alive
static volatile int liftLastTarget = -1; // last target the PID task worked on
static volatile int liftOutput = 0;     // last power the PID task sent

void LiftToPosition(int targetPosition) {
    liftTarget = targetPosition;
}

void StopLiftMacro() {
    liftTarget = -1;
}

void initialize() {
    wrist.set_value(false);
    intake_lift.set_value(false);
    // HOLD so the lift stays where the macro leaves it (move(0) applies brake mode).
    // _all: plain set_brake_mode on a MotorGroup only sets motor index 0.
    liftMotor.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

    liftRotation.reset_position();

    pros::lcd::initialize();
    chassis.calibrate();

    // Display Task
    pros::Task screenTask([] {
        while (true) {
            const auto pose = chassis.getPose();
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::lcd::print(3, "Lift: %d", liftRotation.get_position());
            // DEBUG
            pros::lcd::print(4, "Tgt: %d  Last: %d", liftTarget, liftLastTarget);
            pros::lcd::print(5, "Out: %d  Loops: %d", liftOutput, liftLoops);
            pros::lcd::print(6, "Motor vel: %.0f", liftMotor.get_actual_velocity());
            pros::delay(100);
        }
    });

    // Permanent PID Background Task
    pros::Task liftTask([] {
        constexpr double kP = 0.35;
        constexpr double kI = 0.0;
        constexpr double kD = 0.08;
        constexpr double errorThreshold = 100.0;

        double previousError = 0;
        double integral = 0;
        int activeTarget = -1;

        while (true) {
            // Read once: opcontrol can clear liftTarget mid-loop
            const int target = liftTarget;
            if (target != -1) {
                const double currentPos = liftRotation.get_position();
                const double error = target - currentPos;

                // Fresh PID state for each new target (avoids derivative kick)
                if (target != activeTarget) {
                    previousError = error;
                    integral = 0;
                    activeTarget = target;
                }
                integral += error;
                const double derivative = error - previousError;

                double output = kP * error + kI * integral + kD * derivative;
                output = std::clamp(output, -127.0, 127.0);

                liftMotor.move(output);
                liftOutput = static_cast<int>(output); // DEBUG
                liftLastTarget = target;               // DEBUG

                if (std::abs(error) < errorThreshold) {
                    liftMotor.brake();
                    liftTarget = -1; // Target reached, relinquish control
                }

                previousError = error;
            } else {
                activeTarget = -1;
            }
            liftLoops = liftLoops + 1; // DEBUG
            pros::delay(10);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autoskills() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        // Drivetrain control
        const int dir = -master.get_analog(ANALOG_LEFT_Y);
        const int turn = master.get_analog(ANALOG_RIGHT_X) * 0.8;

        left_motors.move(dir + turn);
        right_motors.move(dir - turn);

        // Manual Lift Control with Soft Limits
        const int current_height = liftRotation.get_position();

        if (master.get_digital(DIGITAL_L1)) {
            StopLiftMacro(); // Cancel macro on manual driver input
            if (current_height < LIFT_MAX_HEIGHT) {
                liftMotor.move(127); // Raise lift
            } else {
                liftMotor.move(0);
            }
        } else if (master.get_digital(DIGITAL_L2)) {
            StopLiftMacro(); // Cancel macro on manual driver input
            if (current_height > LIFT_MIN_HEIGHT) {
                liftMotor.move(-127); // Lower lift
            } else {
                liftMotor.move(0);
            }
        } else if (liftTarget == -1) {
            // Stop motor only when no macro is active
            liftMotor.move(0);
        }

        // Intake and Clamp Control
        if (master.get_digital(DIGITAL_R2)) {
            intake.move(127);
            clamp.move(127);
        } else if (master.get_digital(DIGITAL_R1)) {
            intake.move(-127);
            clamp.move(-127);
        } else {
            intake.move(0);
            clamp.move(0);
        }

        // Pneumatics Toggles
        if (master.get_digital_new_press(DIGITAL_X)) {
            wrist.toggle();
        }

        if (master.get_digital_new_press(DIGITAL_A)) {
            intake_lift.toggle();
        }

        // Macro Keybinds (Positive target values move lift higher)
        if (master.get_digital_new_press(DIGITAL_UP)) {
            LiftToPosition(35000);
        }
        if (master.get_digital_new_press(DIGITAL_DOWN)) {
            LiftToPosition(19000);
        }
        if (master.get_digital_new_press(DIGITAL_LEFT)) {
            LiftToPosition(14000);
        }

        pros::delay(20);
    }
}