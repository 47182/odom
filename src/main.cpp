#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"


//controller 
Controller controller(pros::E_CONTROLLER_MASTER);

//drive motor 
Motor leftFrontMotor(12, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
Motor leftMidMotor(15, pros::E_MOTOR_GEARSET_06, true); // port 2, blue gearbox, not reversed
Motor leftBackMotor(14, pros::E_MOTOR_GEARSET_06, true); // port 3, blue gearbox, reversed
Motor rightFrontMotor(13, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed
Motor rightMidMotor(17, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed
Motor rightBackMotor(10, pros::E_MOTOR_GEARSET_06, false); // port 4, blue gearbox, reversed
Motor catapult(2);
Motor intake(16);

//motor groups 
MotorGroup leftMotors({leftFrontMotor, leftBackMotor,leftMidMotor});
MotorGroup rightMotors({rightFrontMotor, rightBackMotor,rightMidMotor});

//inertial sensor 
Imu inertial_sensor(6);

//tracking wheel 
pros::Rotation vertTracking(19, false); // port 1, not reversed
lemlib::TrackingWheel vertical(&vertTracking, lemlib::Omniwheel::NEW_275, 0);

//pneumatics 
ADIDigitalOut wings(3);
ADIDigitalOut hangpiston(1);
ADIDigitalOut backwings(2);
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
lemlib::ControllerSettings linearController(7.0, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            15, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            250, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             4, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             250, // large error range timeout, in milliseconds
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
 // '.' replaced with "_" to make c++ happy

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
    chassis.moveToPose(8, 54, 10, 3000); //drive to first triball
    intake = 127; //takes off rubberbadns
    chassis.waitUntil(1); 
    intake = -127; //intake the other way for triball
    chassis.waitUntil(5); 
    chassis.waitUntilDone();
    intake = 30; //turn off intake after triball is intaked
    chassis.moveToPoint(8, 45, 1000, false);//goes back by bit
    chassis.waitUntilDone();//finishes movetopoint
    chassis.moveToPose(8, 45, 90, 1000,{.forwards=true, .minSpeed = 20});
    chassis.waitUntilDone();
    chassis.moveToPoint(30, 48, 1000, true);//moves FORWARD
    wings.set_value(true);//
    chassis.waitUntilDone();//finishes move
    intake = 0;//turns off intake
    wings.set_value(false);//closes wings
    chassis.moveToPose(-12,-1,50,3500, {.forwards=false, .lead=0.08});//goes to the bar
    chassis.waitUntilDone();
    chassis.moveToPose(-12,-1,-75,1500, {.forwards=false, .lead=0.08});//turns to get corner triball
    backwings.set_value(1);//gets corner triball
    chassis.waitUntilDone();
    chassis.moveToPose(18,-3,94,3500, {.forwards=true, .lead=0.05});//touches hanging bar
    backwings.set_value(0);
    intake = 127;//turns on intake
    chassis.waitUntilDone();
    /*chassis.moveToPoint(0,0,1000,false); //
    intake = -127;
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPose(-10.5,2,-53,1000);
    delay(1000);
    chassis.moveToPose(-24 ,16.5, 0,2000);
        delay(1000);
    chassis.moveToPose(-20,5.5,-47,2000);
        delay(1000);
    chassis.moveToPoint(-13,0.5,1000,false,80);
        delay(1000);
    backwings.set_value(true);
    chassis.waitUntil(2);
    chassis.waitUntilDone();
    backwings.set_value(false);
   chassis.moveToPose(-13, 0.5, -79, 1000);
       delay(1000);
 chassis.moveToPose(-13, 0.5, -51, 1000);
     delay(1000);
chassis.moveToPose(3.5, 0, 82.8, 2000);
    delay(1000);
  chassis.moveToPose(33,-2,86, 2000);




   // chassis.moveToPose(3.5, 0, 119, 2000);
    // chassis.moveToPoint(-15,7,-52, false);
   // chassis.moveToPoint(-24, 16.8,1200, true);
  //  chassis.waitUntilDone();
   // chassis.moveToPoint(-24,30,1000, false);
   /*chassis.moveToPose(-13,4.5,137, 2000);
    chassis.moveToPose(-17.5,4.58,86, 2000);
    backwings.set_value(true);
    chassis.moveToPose(-11.5,4.58,90, 2000);
    chassis.waitUntil(4);
    backwings.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPose(33,-2,86, 2000);
    intake = 127;
    chassis.waitUntilDone();
    intake = 0;
    */
    //  1 minute skills auto
    /*
    chassis.setPose(0,0,0);
    chassis.moveToPose(10.14,18,-0.41,1500);
    intake = 100;
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(11,30,1000,true, 127);
    chassis.moveToPoint(10,8,2000,false,80);
    chassis.moveToPose(7.5,4.53,111,2000);
    chassis.waitUntil(3);
    chassis.waitUntilDone();
    hangpiston.set_value(true);
    //timedcata(30000,-127);
    timedcata(1000,-127);
    hangpiston.set_value(false);
    chassis.moveToPose(-12,-5,-88,2000);
    chassis.waitUntil(1);
    resetcata();
    chassis.waitUntilDone();
    chassis.moveToPoint(-80,-5,3000,true,95);
    intake = 127;
    chassis.waitUntilDone();
    chassis.moveToPose(-103,26.9,361,2000);
    intake = 90;
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(-105,30,1000,true,127);
    chassis.waitUntilDone();
    chassis.moveToPoint(-105,20,1000,false,127);
    chassis.waitUntilDone();
    chassis.moveToPoint(-105,30,1000,true,127);
    chassis.waitUntilDone();
    //before mid push
    chassis.moveToPoint(-105,16,1500,false,127);
    chassis.waitUntilDone();
    chassis.moveToPose(-90,20.5,92,1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-54,20.5,2000,true,100);
    chassis.waitUntilDone();
    chassis.moveToPose(-54,20.5,-5,1000);
    chassis.waitUntilDone();
    chassis.moveToPose(-95,55,-90,2000);
    hangpiston.set_value(true);
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    chassis.moveToPoint(-70,55,1000,false,127);
    chassis.moveToPoint(-100,55,1000,true,127);
    chassis.moveToPoint(-70,55,1000,false,127); // pushing into the goal 
    chassis.waitUntil(1);
    hangpiston.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPoint(-54,55,1000,false,127);
    chassis.moveToPose(-50,55,0,2000);
    chassis.moveToPoint(-50,120,2500,false,100);
    chassis.waitUntilDone();
    hangpiston.set_value(true);
    chassis.moveToPose(-83,55,-90,3000);
    chassis.moveToPoint(-100,55,1000,true,127);
    chassis.moveToPoint(-70,55,1000,false,127);
    chassis.moveToPoint(-100,55,1000,true,127);
    chassis.moveToPoint(-55,55,1000,false,100);
    hangpiston.set_value(false);
  
    */








    
    /* working far side auto 
    chassis.setPose(0,0,-34);//setting pose
    chassis.moveToPose(-36,56,-34,2000);//move to triball
    intake = 127;
    chassis.waitUntil(1);
    intake = -127;//intake
    chassis.waitUntilDone();
    chassis.moveToPose(-36,56,-40,500);//grasp ball
    intake = -50;
    chassis.moveToPoint(-1,52,2000,120);//push
    chassis.waitUntil(1);
    hangpiston.set_value(true);
    intake = 127;
    chassis.waitUntil(3);
    intake=0;
    chassis.waitUntilDone();
    hangpiston.set_value(false);
    chassis.moveToPoint(-12,52,3000,false,120);//pull
    chassis.waitUntilDone();
    chassis.moveToPose(-39.65,34.92,256,4000);//grab 3rd ball
    intake = -127;
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPose(-15,-5,-180,2500);//goes to other side triball position
    intake = -127;
    chassis.waitUntilDone();
    intake = 0;
    chassis.turnTo(-10,-5,2000,true,120);
    //chassis.turnTo(-5,0,2000,true,120);
    chassis.moveToPose(10,12,49,3000);
    chassis.waitUntil(5);
    hangpiston.set_value(true);
    chassis.waitUntilDone();
    hangpiston.set_value(false);
    chassis.moveToPoint(-1,0,500,false,120);
    chassis.waitUntilDone();
    chassis.moveToPose(14.5,20,3.33,2000);
    intake = 55;
    chassis.waitUntilDone();
    chassis.moveToPoint(14.5,8,1000,false,120);
    chassis.waitUntilDone();
    chassis.moveToPoint(14.5,27,1000,true,127);
    chassis.waitUntilDone();
    chassis.moveToPoint(14.5,8,1000,false,127);
    intake = 0;
    */
    
    
    /* working awp 
    chassis.setPose(0,0,10.4);//Sets the robot position as the new pose
    chassis.moveToPose(8, 57, 10, 3000); 
    intake = 127;
    chassis.waitUntil(1);
    intake = -127;
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPoint(0,0,1000,false);
    intake = -127;
    chassis.waitUntil(5);
    chassis.waitUntilDone();
    intake = 0;
    chassis.moveToPose(0,0,117,1000);
    chassis.moveToPoint(-23,14.579,2000,false,80);
    chassis.moveToPose(-23, 15.579, 179,2000);
    chassis.moveToPoint(-24,30,1200, false);
    chassis.moveToPoint(-24, 16.8,1200, true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-24,30,1000, false);
    chassis.moveToPose(-13,4.5,137, 2000);
    chassis.moveToPose(-17.5,4.58,86, 2000);
    hangpiston.set_value(true);
    chassis.moveToPose(-11.5,4.58,90, 2000);
    chassis.waitUntil(4);
    hangpiston.set_value(false);
    chassis.waitUntilDone();
    chassis.moveToPose(33,-2,86, 2000);
    intake = 127;
    chassis.waitUntilDone();
    intake = 0;
    */

   
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
    bool x = false;
    while (true) {
        Controller master(pros::E_CONTROLLER_MASTER);
        int yaxis = master.get_analog(ANALOG_LEFT_Y);
        int xaxis = master.get_analog(ANALOG_RIGHT_X);

        Powerdrive(yaxis,xaxis);
        
        
    
        

        if(master.get_digital(DIGITAL_Y)){
            catapult = 110;
        }
        else{
            if((rotation_sensor.get_angle() < 3900)){
            catapult = 115;
            }
            else{
                catapult = 0;
            }

        }
        if(master.get_digital_new_press(DIGITAL_L2)){
            hang = !hang;
            wings.set_value(hang);
        }
        

        if(master.get_digital(DIGITAL_L1) == true){
            intake = 117;
        }
        else if(master.get_digital(DIGITAL_R1) == true){
            intake = -127;
        }
        else{
            intake = 0;
        }
        if (master.get_digital_new_press(DIGITAL_A) == true){
            tog = !tog;
            hangpiston.set_value(tog);

        }
        if(master.get_digital_new_press(DIGITAL_R2)){
            x =!x;
            backwings.set_value(x);
        }
        
        
        
        delay(20);
        
    }
}