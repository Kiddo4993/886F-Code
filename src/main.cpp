#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include <algorithm>
#include <cmath>

using namespace pros;

// Motors and controller
MotorGroup right_motors({3, 4, 5}, MotorGearset::blue);
MotorGroup left_motors({-6, -7, -8}, MotorGearset::blue);
MotorGroup liftMotor({-11, 12});
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
Rotation liftRotation(20);

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

// Lift position macro
static pros::Task* liftTask = nullptr;
static volatile bool liftStopRequested = false;
static volatile bool liftTaskFinished = true;

void StopLiftMacro() {
    liftStopRequested = true;
}

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

        while (!liftStopRequested &&
               pros::millis() - startTime < timeout) {
            const double error = targetPosition - liftRotation.get_position();
            integral += error;
            const double derivative = error - previousError;
            double output = kP * error + kI * integral + kD * derivative;

            output = std::clamp(output, -127.0, 127.0);
            liftMotor.move(output);

            if (std::abs(error) < errorThreshold) {
                if (++stableCycles >= stableCyclesRequired) break;
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

void initialize() {
    wrist.set_value(false);
    intake_lift.set_value(false);
    liftMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    liftRotation.reset_position();

    pros::lcd::initialize();
    chassis.calibrate();

    pros::Task screenTask([] {
        while (true) {
            const auto pose = chassis.getPose();
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::lcd::print(3, "Lift: %d", liftRotation.get_position());
            pros::delay(100);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autoskills() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        const int dir = -master.get_analog(ANALOG_LEFT_Y);
        const int turn = master.get_analog(ANALOG_RIGHT_X) * 0.8;

        left_motors.move(dir + turn);
        right_motors.move(dir - turn);

        if (master.get_digital(DIGITAL_L1)) {
            StopLiftMacro();
            liftMotor.move(127);
        } else if (master.get_digital(DIGITAL_L2)) {
            StopLiftMacro();
            liftMotor.move(-127);
        } else {
            liftMotor.move(0);
        }

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

        if (master.get_digital_new_press(DIGITAL_X)) {
            wrist.toggle();
        }

        if (master.get_digital_new_press(DIGITAL_A)) {
            intake_lift.toggle();
        }

        pros::delay(20);
    }
}