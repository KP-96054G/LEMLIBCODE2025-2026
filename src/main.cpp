#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/timer.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <chrono>
#include <ctime>
#include <thread>
#include <cmath>

using namespace pros;


pros::Controller controller(pros::E_CONTROLLER_MASTER);


//Default drivetrain
pros::MotorGroup DL({1, 2, 3}, pros::MotorGearset::blue); 
pros::MotorGroup DR({4, 5, 6}, pros::MotorGearset::blue);

// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-10);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -1.41, 1);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0, 1);
//Gear ratio is Default 1 


// Default IMU
pros::Imu imu(15);

//Drivetrain - trackWidth still needs to be set + i think rpm is 480
lemlib::Drivetrain drivetrain(&DL, // left motor group
                              &DR, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis

                              480, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// Need to tune 
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller,
						sensors // angular PID settings
                        
);

void initialize() {
	chassis.calibrate();
    pros::lcd::initialize(); // initialize brain screen
	vertical_tracking_wheel.reset();
	horizontal_tracking_wheel.reset();

	 pros::Task screen_task([&]() {
        while (true) {
            // print measurements from the rotation sensor
			//If it prints the opposite value u need to reverse the sensor buy multiplying port by -1
            pros::lcd::print(0, "Horizontal: %i", horizontal_encoder.get_position());
        
        pros::lcd::print(1, "Vertical: %i", vertical_encoder.get_position());
		//Prints total distance traveled
		pros::lcd::print(2, "Vertical dis: %i", vertical_tracking_wheel.getDistanceTraveled());
		pros::lcd::print(2, "Horizontal dis: %i", horizontal_tracking_wheel.getDistanceTraveled());
            // delay to save resources
            pros::delay(20);
            
        }
    });
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
	
	

	while (true) {
		// get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with arcade drive - can change later
        chassis.arcade(leftY, rightX);
		
		pros::delay(20);                               // Run for 20 ms then update
	}
}