#ifndef _FUNCTIONS_HPP_
#define _FUNCTIONS_HPP_


void BrakeOn();
void BrakeOff();
void Powerdrive(int powerforward, int powerturning);
void timedmove(int time);
void millisdrive(int time, int power);
void turning(int time, int turnp);
void turn(int target);
void drive(int target);
void timedintake(int time, int power);
double InchtoTicks(double distance);
void PIDturn (int degrees, double kP, double kI, double kD, int timer);
int SpeedCap(int speed);
int SpeedCapLimit(int speed, int limit);
void PIDdrive(int inches, double kP, double kI, double kD, int time);
void PIDdriverightArc(int inches, double kP, double kI, double kD, int time);
void PIDdriveleftArc(int inches, double kP, double kI, double kD, int time);
void resetSens();
void timedcata(int time, int power);
void resetcata();
void PIDfdriverightArc(int inches, double kP, double kI, double kD, int time);
#endif