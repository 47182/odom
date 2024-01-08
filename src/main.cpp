#include "main.h"
#include "lemlib/api.hpp"
#include "functions.hpp"

Motor leftFrontMotor(9, pros::E_MOTOR_GEARSET_36, true); // port 1, blue gearbox, not reversed
Motor leftMidMotor(8, pros::E_MOTOR_GEARSET_36, true); // port 2, blue gearbox, not reversed
Motor leftBackMotor(6, pros::E_MOTOR_GEARSET_36, true); // port 3, blue gearbox, reversed
Motor rightFrontMotor(19, pros::E_MOTOR_GEARSET_36, false); // port 4, blue gearbox, reversed
Motor rightMidMotor(20, pros::E_MOTOR_GEARSET_36, false); // port 4, blue gearbox, reversed
Motor rightBackMotor(18, pros::E_MOTOR_GEARSET_36, false); // port 4, blue gearbox, reversed
pros::Rotation vertTracking(21, false); // port 1, not reversed
ADIDigitalOut wings(1);
ADIDigitalOut hangpiston(2);
Motor catapult(7);
Motor intake(16);
Rotation rotation_sensor(17);
MotorGroup left_side_motors({leftFrontMotor, leftBackMotor,leftMidMotor});
MotorGroup right_side_motors({rightFrontMotor, rightBackMotor,rightMidMotor});
Imu inertial_sensor(10);

lemlib::Drivetrain_t drivetrain{
    &left_side_motors,// left drivetrain motors
    &right_side_motors,// right drivetrain motors
    11.25, // track width
    2.75, // wheel diameter
    450 // wheel rpm
};

lemlib::TrackingWheel vertical(&vertTracking, 2.75,0);
lemlib::OdomSensors_t sensors {
    &vertical, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial_sensor // inertial sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};

// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

/**

 * A callback function for LLEMU's center button.
 *
 * When this callback is fiblue, it will toggle line 2 of the LCD text between
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
	pros::lcd::initialize();
	chassis.calibrate();
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    chassis.setPose(5.2, 10.333, 87); // X: 5.2, Y: 10.333, Heading: 87
	pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(50);
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
    //AWP AND ELIM AUTO
	// chassis.follow("firsttriball.txt", 2000, 15);
    // chassis.follow("ArcTurnPush.txt", 2000, 15);
    // chassis.follow("TriballDescore.txt", 2000, 15);
    // timedintake(5000,127);  

    //Far side Auto

    chassis.follow("Farsidebeginning.txt",2000,10);
    timedintake(500,-127);
    chassis.turnTo(51.062,-2.605,1000,false,200,false);
    wings.set_value(false);
    chassis.follow("Farside1stpush.txt",2000,10);
    timedintake(500,127);
    wings.set_value(true);
    chassis.follow("FS1stintake.txt",2000,10);
    timedintake(500,-127);
    chassis.follow("FS3rdtriball.txt",2000,15);
    timedintake(500,127);
    chassis.follow("fsballdescore.txt",2000,15);
    wings.set_value(false);
    chassis.follow("fs2ballpush.txt",2000,15);
    timedintake(750,-127);
    chassis.follow("fsfinalmovement.txt",2000,15);
    intake = 127;






    







};

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	rotation_sensor.reset();
	bool toggle = false;
	bool tog = false;
	bool hang = false;
	bool shoot = false;
	while (true) {
		Controller master(pros::E_CONTROLLER_MASTER);
		int yaxis = master.get_analog(ANALOG_LEFT_Y);
		int xaxis = master.get_analog(ANALOG_RIGHT_X);

		Powerdrive(yaxis,xaxis);
		
		
	
		

		if(master.get_digital(DIGITAL_R2)){
			catapult = -127;
		}
		else{
			if((rotation_sensor.get_angle() < 12150)){
			catapult = -100;
			}
			else{
				catapult = 0;
			}

		}
		if(master.get_digital_new_press(DIGITAL_B)){
			hang = !hang;
			hangpiston.set_value(hang);
		}
		

		if(master.get_digital(DIGITAL_L1) == true){
			intake = 110;
		}
		else if(master.get_digital(DIGITAL_R1) == true){
			intake = -127;
		}
		else{
			intake = 0;
		}
		if (master.get_digital_new_press(DIGITAL_L2) == true){
			tog = !tog;
			wings.set_value(tog);

		}
		
		
		
		delay(20);
		
	}
}

