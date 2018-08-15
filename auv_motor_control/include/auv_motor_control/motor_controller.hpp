#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// Inputs heading given by other programs. set globalCoord to 1 to scale to the
// global coordinate plane; leave at 0 to give coordinates relative to the
// robot.
// outputs a heading in a more usable x/y coordinate plane
// This function should never be used directly by the user. It is called in
// future functions.
// float direction_strafe_fixed(float heading, bool globalCoord);

// Inputs direction_strafe_fixed and outputs magnitude in sway (y) and surge (x)
// directions respectively
// This function should never be used directly by the user. It is called in
// future functions.
float magnitude_sway(float heading);
float magnitude_surge(float heading);

// Inputs headings and output new magnitude in surge and sway directions
// respectively. Used to shift coordinate plane by 45 degrees
// This function should never be used directly by the user. It is called in
// future functions.
float magnitude_sway_travel(float heading);
float magnitude_surge_travel(float heading);

// Inputs header given DIRECTLY BY THE USER in previous programs
// calls all previous functions to fix plane, calculate magnitudes
// integrates PID values from PID controllers
// outputs a finalized heading
float postPIDHeading();

// Inputs preliminary thruster values from one set of thrusters (either on xy
// plane or vertical)
// Scales all values so greatest is no more than 100%
// Outputs a double which the thruster values are later divided by to scale down
double getProportionalDivider(double thrusterValue1, double thrusterValue2,
                              double thrusterValue3, double thrusterValue4);

double degreesToRadians(double degree_input); // converts degrees to radians.
double radiansToDegrees(double radian_input); // converts radians to degrees

double deadband(
    double input,
    double deadband); // sets up a simple deadband to ignore negligable amounts

bool convertToBool(int32_t input); // takes a one or zero value (int) and turns
                                   // into boolean true/false

double getDirection_strafe(float xAxis, float yAxis); // joystick input; takes x
                                                      // and y axis inputs and
                                                      // outputs angle in
                                                      // degrees

double max1(double input); // caps a value at 1
double removeNaN(double input); // Little trick to make sure thrusters can't
                                // output Nan... Checks if input!=input, which
                                // is only true if input=NaN
// If it is, return 0, otherwise return input
#endif
