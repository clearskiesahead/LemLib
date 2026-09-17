<<<<<<< HEAD
#include "main.h" 
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "autons.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motor_group({3, 4}, pros::MotorGearset::blue);
// right side reversed (negative ports) so that a raw positive encoder/voltage direction means
// "this wheel spins to push the robot forward" on BOTH sides -- previously this mirroring was
// only patched downstream in opcontrol()'s joystick math, which fixed driving but left the raw
// encoders reporting a mismatched sign relationship, corrupting tracking-wheel-based odometry.
pros::MotorGroup right_motor_group({-1, -2}, pros::MotorGearset::blue);

// rpm is 257 (not the blue cartridge's native 600) to account for the drivetrain's 36:84 external
// gear reduction (600 * 36/84 ~= 257) -- matches the rpm already used for the Drivetrain below.
lemlib::TrackingWheel horizontal_tracking_wheel(&right_motor_group, lemlib::Omniwheel::NEW_4, 0, 257);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&left_motor_group, lemlib::Omniwheel::NEW_4, 0, 257);

pros::Motor intake(5);
pros::MotorGroup lift({6, -7}, pros::MotorGearset::green);
pros::Motor claw(8, pros::v5::MotorGears::red);
pros::Motor rotationMech(9, pros::v5::MotorGears::red);
pros::Distance backdistance('A');
pros::Distance frontdistance('B');
pros::Distance leftdistance('C');
pros::Distance rightdistance('D');

pros::Imu imu(-20);

lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 11, lemlib::Omniwheel::NEW_4, 257, 2);


enum class LiftState {
    Bottom,
    Low,
    High,
    Top
};

LiftState current_lift_state = LiftState::Bottom;

int rotationMechState = 0; // 0 = resting at 0 (down), matching the mechanism's actual position at boot
int clawState = 0;

// Closes the claw at full power until it stalls against the pin (or a timeout elapses),
// then backs off to a lower holding voltage so it doesn't keep fighting something it already has.
void closeClawUntilStall() {
    const double stallVelocityThreshold = 5;  // RPM; below this counts as "not moving"
    const int stallReadingsNeeded = 5;        // consecutive low-velocity readings before declaring a stall
    const int rampUpGraceMs = 150;            // ignore the initial near-zero velocity while it's still starting to move
    const int maxCloseMs = 1500;              // safety timeout in case nothing is actually gripped

    claw.move(-120);
    pros::delay(rampUpGraceMs);

    int stalledReadings = 0;
    int elapsedMs = rampUpGraceMs;
    while (elapsedMs < maxCloseMs) {
        double actualVelocity = claw.get_actual_velocity();
        if (actualVelocity < 0) actualVelocity = -actualVelocity;

        if (actualVelocity < stallVelocityThreshold) {
            stalledReadings++;
            if (stalledReadings >= stallReadingsNeeded) break;
        } else {
            stalledReadings = 0;
        }

        pros::delay(10);
        elapsedMs += 10;
}
}

// Asynchronous background task wrapper to keep your main opcontrol loop completely fluid
void intakeToClawMovement() {
    pros::Task macroTask([]() {
        const int POSITION_TOLERANCE = 15; // allowable encoder error margin to proceed to next step
        const int SAFTEY_TIMEOUT = 1500; 

        // Ensure claw is open, drop rotation mechanism to intake zone, and position lift
        clawState = 1;
        claw.move(120); // Actively open claw
        rotationMech.move_absolute(0, 100);
        lift.move_absolute(50, 100);

        int timeElapsed = 0;
        // Wait intelligently until lift and rotation mechanisms arrive at their handoff points
        while ((abs(rotationMech.get_position() - 0) > POSITION_TOLERANCE || 
                abs(lift.get_position() - 50) > POSITION_TOLERANCE) && 
               timeElapsed < SAFTEY_TIMEOUT) {
            pros::delay(10);
            timeElapsed += 10;
        }
        claw.brake(); // Stop opening claw once positions align

        // Brief stabilization pause instead of a long blind blind delay
        pros::delay(50); 

        // Close claw firmly around the game piece
        clawState = 0;
        claw.move(-120); 
        pros::delay(250);

        // move lift up and away from intake
        lift.move_absolute(300, 100);
        timeElapsed = 0;
        while (abs(lift.get_position() - 300) > POSITION_TOLERANCE && timeElapsed < SAFTEY_TIMEOUT) {
            pros::delay(10);
            timeElapsed += 10;
        }

        //move rotation mech to scoring position
        rotationMech.move_absolute(-270, 100);
        timeElapsed = 0;
        while (abs(rotationMech.get_position() - (-270)) > POSITION_TOLERANCE && timeElapsed < SAFTEY_TIMEOUT) {
            pros::delay(10);
            timeElapsed += 10;
        }

        rotationMech.brake();
    });
}


void toggleClaw() { // 0 for closed, 1 for open
    if (clawState == 0) {
        clawState = 1;
        claw.move(120);
        pros::delay(500);
        claw.brake();
    } else {
        clawState = 0;
        closeClawUntilStall();
    }
}

void toggleRotationMech() {
    if (rotationMechState == 0) {
        rotationMechState = 1;
        rotationMech.move_absolute(-270, 100);
    } else {
        rotationMechState = 0;
        rotationMech.move_absolute(0, 100);
    }
}



lemlib::OdomSensors sensors(nullptr,
                             nullptr, // no second vertical tracking wheel
                             nullptr, // no horizontal tracking wheel
                             nullptr, // no second horizontal tracking wheel
                             &imu);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              1, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              300, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);


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
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // actively hold position instead of coasting after claw.brake()
    claw.set_current_limit(1200); // cap current draw (default 2500mA) so sustained stall/hold pressure runs cooler
    rotationMech.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // resist gravity/external torque once at target

    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.calibrate(); 

    // forces the brain to pause here until the IMU is completely done calibrating
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    // NOTE: disabled while debugging debug_auton() -- this task also writes lines 0-2,
    // which made it impossible to tell whether debug_auton()'s own loop was actually
    // running (lines 0-2 would keep updating from THIS task even if debug_auton() died).
    // Re-enable once debug_auton() is confirmed working correctly.
    // pros::Task screen_task([&]() {
    //     while (true) {
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x);
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y);
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
    //         pros::delay(20);
    //     }
    // });
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
void competition_initialize() {}

// Diagnostic helper: does NOT drive the robot at all. It resets odometry to (0,0,0)
// and then just prints the live pose forever so you can push the robot around by hand
// and check whether the reported X/Y/Theta match what you actually did.
//
// Lines 0-2 show chassis.getPose() (LemLib's computed position/heading).
// Lines 3-5 show the RAW sensors underneath it, bypassing LemLib entirely: the IMU's
// own heading, and each drive motor group's own encoder position. If pose (0-2) is
// frozen but the raw readings (3-5) change when you move the robot, the sensors are
// fine and LemLib's internal odometry task isn't running/updating. If the raw readings
// are ALSO frozen, the problem is upstream of LemLib -- a sensor/wiring/port issue.
//
// How to use it:
//   1. Call this instead of your real autonomous routine (temporarily swap the call
//      in autonomous(), or just invoke it from initialize() for a bench test).
//   2. Put the robot on the ground with driver control NOT touching the joysticks
//      (motors should be idle -- this is a pure sensor readout, not a motion test).
//   3. Physically push the robot forward exactly 24 inches (measure it) and watch
//      the numbers. Given LemLib's compass-style heading (0 degrees = facing +Y),
//      pushing it "forward" should increase Y, not X, while X and Theta stay near 0.
//      If instead X changes and Y doesn't, that confirms the "forward is +Y" mixup
//      is exactly what happened in the runaway moveToPoint(24, 0, ...) call.
//   4. Then try pushing it sideways, and turning it in place, to check X and Theta
//      the same way. Each axis should only respond to the motion that should affect it.
void debug_auton() {
    chassis.setPose(0, 0, 0);
    // 0=degrees, 1=rotations, 2=counts -- settles whether "left pos"/"right pos" below are degrees or rotations
    int leftEncoderUnits = static_cast<int>(left_motor_group.get_encoder_units());
    int rightEncoderUnits = static_cast<int>(right_motor_group.get_encoder_units());
    printf("debug_auton: pose reset to (0, 0, 0). left encoder units: %d, right encoder units: %d (0=degrees, "
           "1=rotations, 2=counts)\n",
           leftEncoderUnits, rightEncoderUnits);
    printf("Push the robot by hand and watch the numbers below.\n");

    int heartbeat = 0;
    while (true) {
        heartbeat++;

        lemlib::Pose pose = chassis.getPose();
        double rawImuHeading = imu.get_heading(); // [0,360) -- yaw about the sensor's own hardcoded reference axis
        double rawPitch = imu.get_pitch();
        double rawRoll = imu.get_roll();
        double rawYaw = imu.get_yaw();
        double rawLeftPos = left_motor_group.get_position();
        double rawRightPos = right_motor_group.get_position();
        // status is a BITMASK, not a simple enum: bit 0 (value 1) set = still calibrating, 0xFF = hard error
        int imuStatus = static_cast<int>(imu.get_status());
        // orientation: 0=Z_UP, 1=Z_DOWN, 2=X_UP, 3=X_DOWN, 4=Y_UP, 5=Y_DOWN, 255=error/undetected
        int imuOrientation = static_cast<int>(imu.get_physical_orientation());

        pros::lcd::print(0, "X: %.2f  Y: %.2f", pose.x, pose.y);
        pros::lcd::print(1, "Theta: %.2f", pose.theta);
        pros::lcd::print(2, "raw heading: %.2f", rawImuHeading);
        pros::lcd::print(3, "pitch:%.1f roll:%.1f yaw:%.1f", rawPitch, rawRoll, rawYaw);
        pros::lcd::print(4, "left:%.2f right:%.2f", rawLeftPos, rawRightPos);
        pros::lcd::print(5, "status:%d orient:%d", imuStatus, imuOrientation);
        // if this number isn't climbing, the loop itself isn't running -- meaning something
        // above crashed/errored before reaching here, or you're looking at stale (not-just-rebuilt) code.
        pros::lcd::print(6, "heartbeat: %d", heartbeat);

        printf("pose X: %f, Y: %f, Theta: %f | raw heading: %f, pitch: %f, roll: %f, yaw: %f | left pos: %f, right "
               "pos: %f | status: %d, orientation: %d | heartbeat: %d\n",
               pose.x, pose.y, pose.theta, rawImuHeading, rawPitch, rawRoll, rawYaw, rawLeftPos, rawRightPos,
               imuStatus, imuOrientation, heartbeat);
        pros::delay(100); // slow enough to actually read while pushing the robot by hand
    }
}

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

 void ans_auton() {
    // set position to x:0, y:0, heading:0
    printf("Cord-x: %f\n", chassis.getPose().x);
    printf("Cord-y: %f\n", chassis.getPose().y);
    printf("Heading: %f\n", chassis.getPose().theta);
    pros::delay(20);

    chassis.setPose(0, 0, 0);
    printf("Cord-x: %f\n", chassis.getPose().x);
    printf("Cord-y: %f\n", chassis.getPose().y);
    printf("Heading: %f\n", chassis.getPose().theta);
    pros::delay(20);

    pros::delay(3000);
    // turn to face heading 90 with a very long timeout
    chassis.moveToPoint(24, 0, 5000, {.maxSpeed = 50});
    printf("Cord-x: %f\n", chassis.getPose().x);
    printf("Cord-y: %f\n", chassis.getPose().y);
    printf("Heading: %f\n", chassis.getPose().theta);
    pros::delay(20);

    chassis.turnToHeading(90, 5000);

    while (true) {
        printf("Cord-x: %f\n", chassis.getPose().x);
        printf("Cord-y: %f\n", chassis.getPose().y);
        printf("Heading: %f\n", chassis.getPose().theta);
    }
 }

void autonomous() {
        //start auton
    chassis.setPose(-14.5,-61.5, 135);
    chassis.moveToPoint(-24, -48, 2000, {.forwards = false});

    // lift.move_absolute(50, 70);
    // pros::delay(500);
    // lift.move_absolute(0, 70);
    // claw.move(-120);
    // chassis.swingToheading(90);
    // chassis.setPose(-72+ getLeft(), chassis.getPose().y, chassis.getPose().theta);
    // chassis.moveToPoint(0, -48, 2000);
    // chassis.turnToHeading(180, 2000);
    // chassis.setPose(-72 + getFront(), chassis.getPose().y, chassis.getPose().theta);
    // chassis.moveToPoint(0, -65, 2000);
    // chassis.moveToPoint(0, -60, 2000);
    // chassis.moveToPoint(0, -65, 2000);
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
    // loop forever
    while (true) {
        // get left y (throttle) and right x (turn) positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        // NOTE: right_motor_group's ports are now reversed at the declaration (the actual root cause
        // of the earlier turn-direction issue), so this no longer needs the -rightX negation that used
        // to compensate for it downstream. If turning comes out backwards now, re-add the negation here
        // instead of re-reversing the ports again -- don't stack two fixes for the same problem.
        chassis.arcade(leftY, -rightX);

        // control the intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(120);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(-120);
        } else {
            intake.brake();
        };


        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            closeClawUntilStall();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            toggleRotationMech();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move_velocity(100);  // Move Up (closed-loop RPM, consistent speed regardless of gravity)
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move_velocity(-70); // Move Down (closed-loop RPM, consistent speed regardless of gravity)
        }
        else {
            lift.brake();      // Automatically brakes due to HOLD mode
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            intakeToClawMovement();
        }

        // pros::lcd::print(0, "X: %f", chassis.getPose().x); 
        // pros::lcd::print(1, "Y: %f", chassis.getPose().y); 
        // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); 
        
        // printf("Cord-x: %f\n", chassis.getPose().x);
        // printf("Cord-y: %f\n", chassis.getPose().y);
        // printf("Heading: %f\n", chassis.getPose().theta);
        // delay to save resources
        pros::delay(20);

        //controller.screen.print("potato");
    }
}
=======
#include "main.h"
#include "lemlog/logger/sinks/terminal.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "lemlib/motions/turnTo.hpp"
#include "pros/llemu.hpp"

logger::Terminal terminal;

lemlib::MotorGroup rightDrive({8, 10}, 360_rpm);
lemlib::MotorGroup leftDrive({-1, 11, -12, 13}, 360_rpm);

lemlib::V5InertialSensor imu(1);

lemlib::TrackingWheel verticalTracker({'E', 'F'}, true, 2.75_in, 26.5_cm / 2);
lemlib::TrackingWheel horizontalTracker({'G', 'H'}, false, 2.75_in, -26.5_cm / 2);

lemlib::TrackingWheelOdometry odom({&imu}, {&verticalTracker}, {&horizontalTracker});

lemlib::PID pid(0.05, 0, 0);
lemlib::ExitCondition<AngleRange> exitCondition(1_stDeg, 2_sec);

void initialize() {
    terminal.setLoggingLevel(logger::Level::DEBUG);
    pros::lcd::initialize();

    imu.calibrate();
    pros::delay(3200);
    odom.startTask();
    pros::delay(100);
    pros::Task([&] {
        while (true) {
            auto p = odom.getPose();
            pros::lcd::print(0, "X: %f", to_in(p.x));
            pros::lcd::print(1, "Y: %f", to_in(p.y));
            pros::lcd::print(2, "Theta: %f", to_cDeg(p.orientation));
            pros::delay(10);
        }
    });
    lemlib::turnTo(90_cDeg, 100_sec, {.slew = 1},
                   {
                       .angularPID = pid,
                       .exitConditions = std::vector<lemlib::ExitCondition<AngleRange>>({exitCondition}),
                       .poseGetter = [] -> units::Pose { return odom.getPose(); },
                       .leftMotors = leftDrive,
                       .rightMotors = rightDrive,
                   });
}

void disabled() {}

void autonomous() {}

void opcontrol() {}
>>>>>>> master
