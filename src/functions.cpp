#include "main.h"
#include "math.h"
void resetSens(){
leftFrontMotor.tare_position();
leftMidMotor.tare_position();
leftBackMotor.tare_position();
catapult.tare_position();
rightFrontMotor.tare_position();
rightMidMotor.tare_position();
rightBackMotor.tare_position();
intake.tare_position();
inertial_sensor.tare_rotation();
}
void BrakeOn(){
	leftFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftMidMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	leftBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightFrontMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightMidMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	rightBackMotor.set_brake_mode(MOTOR_BRAKE_HOLD);
	
	
}
void BrakeOff(){
	leftFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	leftMidMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	leftBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	rightFrontMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	rightMidMotor.set_brake_mode(MOTOR_BRAKE_COAST);
	rightBackMotor.set_brake_mode(MOTOR_BRAKE_COAST);

}

void Powerdrive(int powerforward, int powerturning){

	leftFrontMotor = powerforward + powerturning;
	rightFrontMotor = powerforward - powerturning;
	leftMidMotor = powerforward + powerturning;
	rightMidMotor = powerforward - powerturning;
	leftBackMotor = powerforward + powerturning;
	rightBackMotor = powerforward - powerturning;
}

void PowerdriveSide(int leftPower, int rightPower){

	leftFrontMotor = leftPower;
	rightFrontMotor = rightPower;
	leftMidMotor = leftPower;
	rightMidMotor = rightPower;
	leftBackMotor = leftPower;
	rightBackMotor = rightPower;
}

void timedmove(int time){
	Powerdrive(60,0);
	delay(time);
	Powerdrive(0,0);
}

void millisdrive(int time, int power){
	int st = millis();
	while(time > millis() - st){
		Powerdrive(84*power,0 );
	}
	Powerdrive(SpeedCap(0),0);
}

void turning(int time, int turnp){
	int st = millis();
	while(time > millis() - st){
		Powerdrive(0,60*turnp);
	}
	Powerdrive(0,0);
}

void turn(int target){
    while(fabs(target - inertial_sensor.get_rotation()) > 0.5){
        if(target > inertial_sensor.get_rotation()){
            Powerdrive(0,-45);
        }else if(target < inertial_sensor.get_rotation()){
            Powerdrive(0,45);

        }
    }Powerdrive(0,0);
}
void drive(int target){
	leftFrontMotor.tare_position();
    while(abs(target - leftFrontMotor.get_position()) > 2 ){
        if(target > leftFrontMotor.get_position()){
            Powerdrive(30,0);
        }else if(target < leftFrontMotor.get_position()){
            Powerdrive(-30,0);

        }
    }Powerdrive(0,0);

}
double InchtoTicks(double distance){
	double external = (double)3/5; 
	double internal = (double)300;
	double diameter = 3.25; 
	double PI = 3.141;

	return  (double(distance/PI/diameter/external*internal)); 
}



void PIDturn (int degrees, double kP, double kI, double kD, int timer){
	resetSens();
	double difference = degrees-inertial_sensor.get_rotation();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();
	
	while((abs(degrees-inertial_sensor.get_rotation())>2.75) && timer > millis() - st){
			difference = degrees-inertial_sensor.get_rotation();
			if(fabs(degrees-inertial_sensor.get_rotation()) < 3){
			integral += difference;
			}
			derivative = difference - past_difference;
			past_difference = difference; 
			power = difference*kP + integral*kI + derivative*kD;
			Powerdrive(0,SpeedCap(power));

			pros::screen::print(TEXT_MEDIUM, 1, "Error: %lf \n", difference);
			pros::screen::print(TEXT_MEDIUM, 2, "Angle: %f \n", rotation_sensor.get_angle());
			pros::screen::print(TEXT_MEDIUM, 3, "exit timer: %d \n");
			pros::delay(20);
		}	
	Powerdrive(0,0);	
}


int SpeedCap(int speed){ 
	int limit = 127;
	
	if(abs(speed) <= limit ){
		return(speed);
	}else if(speed < -limit){
	    return -limit;
	}else {
		return limit;
	}

}
int SpeedCapLimit(int speed, int limit){ 
	
	if(abs(speed) <= limit ){
		return(speed);
	}else if(speed < -limit){
	    return -limit;
	}else {
		return limit;
	}

}
void timedintake(int time, int power){
	int seconds = millis();
	while(time > millis() - seconds){
		intake = power;
	}
	intake = 0;
}

void timedcata(int time, int power){
	int seconds = millis();
	while(time > millis() - seconds){
		catapult = power;
	}
	catapult = 0;
}
void resetcata(){

	while(rotation_sensor.get_angle() < 12150){
			catapult = -127;
	}
	catapult = 0;
		
}
	



void PIDdrive(int inches, double kP, double kI, double kD, int time){
	resetSens();
	double target = InchtoTicks(inches);
	double difference = target-leftFrontMotor.get_position();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();
	//turning
	double degrees = 0;

	int r_power;
	int r_integral;
	int r_past_difference;
	int r_derivative;
	double r_difference;
	double r_kP = 3.5;
	double r_kI = 0;
	double r_kD = 0;
	while((fabs(target-leftFrontMotor.get_position())>5 || (fabs(degrees-inertial_sensor.get_rotation())>0.75)) && time > millis() - st){
		pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
		pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);
		//derivative
		past_difference = difference; 
		difference = target-leftFrontMotor.get_position();
		derivative = difference-past_difference;

		if(fabs(target-leftFrontMotor.get_position())<10){
			integral += difference;
		}

		power = difference*kP + integral*kI + derivative*kD;
		//turning
		// r_past_difference = r_difference; 
		r_difference = degrees-inertial_sensor.get_rotation();
		// r_derivative = r_difference-r_past_difference; 

		// if(fabs(degrees-inertial_sensor.get_rotation())<2.5){
		// 	r_integral += r_difference;
		// }

		r_power = r_difference*r_kP;


		Powerdrive(SpeedCap(power),r_power);
	}
	
	
	Powerdrive(0,0);


}


void PIDdriverightArc(int inches, double kP, double kI, double kD, int time){
	resetSens();
	double target = InchtoTicks(inches);
	double difference = target-leftFrontMotor.get_position();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();
	//turning
	double degrees = 40;

	int r_power;
	int r_integral;
	int r_past_difference;
	int r_derivative;
	double r_difference;
	double r_kP = 3.5;
	double r_kI = 0;
	double r_kD = 0;
	while((fabs(target-leftFrontMotor.get_position())>5 || (fabs(degrees-inertial_sensor.get_rotation())>0.75)) && time > millis() - st){
		pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
		pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);
		//derivative
		past_difference = difference; 
		difference = target-leftFrontMotor.get_position();
		derivative = difference-past_difference;

		if(fabs(target-leftFrontMotor.get_position())<10){
			integral += difference;
		}

		power = difference*kP + integral*kI + derivative*kD;
		//turning
		// r_past_difference = r_difference;
		r_difference = degrees-inertial_sensor.get_rotation();
		// r_derivative = r_difference-r_past_difference; 

		// if(fabs(degrees-inertial_sensor.get_rotation())<2.5){
		// 	r_integral += r_difference;
		// }

		r_power = r_difference*r_kP;
		

		PowerdriveSide(SpeedCapLimit(power*2,60),SpeedCapLimit(power*2+r_power, 95));
	}
	
	
	Powerdrive(0,0);


}

void PIDdriveleftArc(int inches, double kP, double kI, double kD, int time){
	resetSens();
	double target = InchtoTicks(inches);
	double difference = target-leftFrontMotor.get_position();
	double power;
	double integral;
	double past_difference;
	double derivative;
	int st = millis();
	//turning
	double degrees = -60;

	int r_power;
	int r_integral;
	int r_past_difference;
	int r_derivative;
	double r_difference;
	double r_kP = 3.5;
	double r_kI = 0;
	double r_kD = 0;
	while((fabs(target-leftFrontMotor.get_position())>5 || (fabs(degrees-inertial_sensor.get_rotation())>0.75)) && time > millis() - st){
		pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
		pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);
		//derivative
		past_difference = difference; 
		difference = target-leftFrontMotor.get_position();
		derivative = difference-past_difference;

		if(fabs(target-leftFrontMotor.get_position())<10){
			integral += difference;
		}

		power = difference*kP + integral*kI + derivative*kD;
		//turning
		// r_past_difference = r_difference;
		r_difference = degrees-inertial_sensor.get_rotation();
		// r_derivative = r_difference-r_past_difference; 

		// if(fabs(degrees-inertial_sensor.get_rotation())<2.5){
		// 	r_integral += r_difference;
		// }

		r_power = r_difference*r_kP;
		

		PowerdriveSide(SpeedCapLimit(power*2+r_power, 95),SpeedCapLimit(power*2,60));
	}
	
	
	Powerdrive(0,0);


}

void PIDfdriverightArc(int inches, double kP, double kI, double kD, int time) {
    resetSens();
    double target = InchtoTicks(inches);
    double difference = target - leftFrontMotor.get_position();
    double power;
    double integral = 0.0;
    double past_difference = 0.0;
    double derivative;

    // Turning
    double degrees = 40;
    int r_power;
    double r_integral = 0.0;
    double r_past_difference = 0.0;
    double r_derivative;
    double r_difference;
    double r_kP = -3.5;  // Adjust the sign to turn in the negative direction
    double r_kI = 0;
    double r_kD = 0;

    int st = millis();

    while ((fabs(target - leftFrontMotor.get_position()) > 5 || (fabs(degrees - inertial_sensor.get_rotation()) > 0.75)) && time > millis() - st) {
        pros::screen::print(TEXT_MEDIUM, 1, "distance driven: %f", difference);
        pros::screen::print(TEXT_MEDIUM, 2, "rotation: %f", r_difference);

        // Forward movement PID
        past_difference = difference;
        difference = target - leftFrontMotor.get_position();
        derivative = difference - past_difference;

        if (fabs(target - leftFrontMotor.get_position()) < 10) {
            integral += difference;
        }

        power = difference * kP + integral * kI + derivative * kD;

        // Turning PID
        r_past_difference = r_difference;
        r_difference = degrees - inertial_sensor.get_rotation();
        r_derivative = r_difference - r_past_difference;

        if (fabs(degrees - inertial_sensor.get_rotation()) < 2.5) {
            r_integral += r_difference;
        }

        r_power = r_difference * r_kP;

        // Adjust the sign of the power for both forward movement and turning
        PowerdriveSide(SpeedCapLimit(-power * 2, 60), SpeedCapLimit(-power * 2 + r_power, 95));
    }
}