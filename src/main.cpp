#include "main.h"
#include "lemlib/api.hpp"

//controller 
Controller controller(pros::E_CONTROLLER_MASTER);

//drive motor 
Motor leftFrontMotor(17, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
Motor leftMidMotor(18, pros::E_MOTOR_GEARSET_06, true); // port 2, blue gearbox, not reversed
Motor leftBackMotor(8, pros::E_MOTOR_GEARSET_06, true); // port 3, blue gearbox, reversed
Motor rightFrontMotor(14, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed
Motor rightMidMotor(12, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed
Motor rightBackMotor(7, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed
Motor catapult(10);
Motor intake(20);

//motor groups 
MotorGroup leftMotors({leftFrontMotor, leftBackMotor,leftMidMotor});
MotorGroup rightMotors({rightFrontMotor, rightBackMotor,rightMidMotor});

//inertial sensor 
Imu inertial_sensor(1);

//tracking wheel 
pros::Rotation vertTracking(13, false); // port 1, not reversed
lemlib::TrackingWheel vertical(&vertTracking, lemlib::Omniwheel::NEW_275, 0);

//pneumatics 
ADIDigitalOut wings(1);
ADIDigitalOut hangpiston(2);

//cata sensor
Rotation rotation_sensor(2);


// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors, // left motor group
    &rightMotors, // right motor group
    10, // 10 inch track width
    lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
    450, // drivetrain rpm is 450
    8 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            25, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.9, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             5, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             20 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial_sensor // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);


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
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
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

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(test_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
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
    chassis.setPose(0,0,10.4);//Sets the robot position as the new pose
    chassis.moveToPose(6, 51, 4, 4000); 
    chassis.waitUntil(3);
    intake = -127;
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(0,0,1000,false);
    chassis.moveToPose(0,0,117,2000);
    chassis.moveToPoint(-24,16,2000,false,60);
    chassis.waitUntilDone(); 
    chassis.moveToPose(-24, 20, 179,2000);
    chassis.waitUntilDone(); 
    chassis.moveToPoint(-24,26,1000, false);
    chassis.moveToPose(-13,4.5,137, 2000);
   // hangpiston.set_value()
    chassis.moveToPose(33,-1,85, 2000);
    // // example movement: Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // // cancel the movement after it has travelled 10 inches
    // chassis.waitUntil(10);
    // chassis.cancelMotion();
    // // example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
    // // dont turn faster than 60 (out of a maximum of 127)
    // chassis.turnTo(45, -45, 1000, true, 60);
    // // example movement: Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // // following the path with the back of the robot (forwards = false)
    // // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // // wait until the chassis has travelled 10 inches. Otherwise the code directly after
    // // the movement will run immediately
    // // Unless its another movement, in which case it will wait
    // chassis.waitUntil(10);
    // pros::lcd::print(4, "Travelled 10 inches during pure pursuit!");
    // // wait until the movement is done
    // chassis.waitUntilDone();
    // pros::lcd::print(4, "pure pursuit finished!");
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
		if(master.get_digital_new_press(DIGITAL_L2)){
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
		if (master.get_digital_new_press(DIGITAL_A) == true){
			tog = !tog;
			wings.set_value(tog);

		}
		
		
		
		delay(20);
		
	}
}

