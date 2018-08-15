#include "auv_motor_control/movement_values.h"
#include "auv_motor_control/thruster_int.h"
#include "auv_motor_control/thruster_values.h"
#include "math.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <auv_motor_control/motor_controller.hpp>
#include <cstdlib>
#include <sstream>
// MOVEMENT VALUES//
// THESE ARE SETPOINTS which are published by other nodes which tell the robot
// where to go

double magnitude_strafe = 100;
// double direction_strafe;
bool globalCoord_strafe = false;

double magnitude_yaw = 100;
double setpoint_yaw;
bool globalCoord_yaw = 0;

double magnitude_heave = 100;
double setpoint_heave;

double magnitude_roll = 0;
double setpoint_roll;

double magnitude_pitch = 0;
double setpoint_pitch;

/////////////////Used to log robot's current orientation///////////////////
double currentYaw = 100; // global coordinate calibrated so at start robot
                         // forward = global 0 degrees
double currentPitch;
double currentRoll;
double currentHeave;

// THRUSTER DECLARATIONS

double thruster_xy_frontRight;
double thruster_xy_frontLeft;
double thruster_xy_backRight;
double thruster_xy_backLeft;
double thruster_z_frontRight;
double thruster_z_frontLeft;
double thruster_z_backRight;
double thruster_z_backLeft;

// PID VARIABLE DECLARATIONS

float controlEffort_surge =
    0; // controlEfforts are INPUTS from the pid controller
float controlEffort_sway = 0;
float controlEffort_yaw = 0;
float controlEffort_heave = 0;
float controlEffort_pitch = 0;
float controlEffort_roll = 0;
// MOTOR CONTROL VARIABLES

////xy///
double strafe_backLeft;
double strafe_frontLeft;
double strafe_backRight;
double strafe_frontRight;
double yaw_frontLeft;
double yaw_backRight;
double yaw_backLeft;
double yaw_frontRight;
double thruster_xy_frontRight_merge;
double thruster_xy_frontLeft_merge;
double thruster_xy_backRight_merge;
double thruster_xy_backLeft_merge;
double proportionalDivider_xy;

////z////
double heave_allVertical;
double pitch_verticalFront;
double pitch_verticalBack;
double roll_verticalRight;
double roll_verticalLeft;
double thruster_z_frontRight_merge;
double thruster_z_frontLeft_merge;
double thruster_z_backRight_merge;
double thruster_z_backLeft_merge;
double proportionalDivider_vertical;

// bool enablePID = true;
// bool onlyPID = true; // used to distinguish if the movement_values message is
// being used or ignored
// it is ignored in most cases so the PID controllers can directly affect the
// movement without any interference

double xy_thruster_multiplier = .35;
double z_thruster_multiplier = 1.0;

double debug;

/*(void headingCallback(const auv_motor_control::movement_values::ConstPtr&
movement)
{
  //ROS_INFO("\033[2J\033[1;1H"); //clears ros_info
  //ROS_INFO("I heard: [%.10f]" , movement->direction_strafe);
  direction_strafe = movement->direction_strafe;
//  magnitude_strafe = movement->magnitude_strafe;
  globalCoord_strafe = movement->globalCoord_strafe;
  magnitude_yaw = movement->magnitude_yaw;
  setpoint_yaw = movement->setpoint_yaw;
  globalCoord_yaw = movement->globalCoord_yaw;
  setpoint_roll = movement->setpoint_roll;
  magnitude_roll = movement->magnitude_roll;
  setpoint_pitch = movement->setpoint_pitch;
  magnitude_pitch = movement->magnitude_pitch;
  magnitude_heave = movement->magnitude_heave;
  setpoint_heave = movement->setpoint_heave;
}
*/
void debugCallBack(const std_msgs::Float64::ConstPtr &msg) {
  //  magnitude_roll = 10;
  magnitude_pitch = msg->data;
}

void heaveCallBack(const std_msgs::Float64::ConstPtr &heave) {
  controlEffort_heave = heave->data;
  //  ROS_INFO("heave %f", controlEffort_heave);
}

void yawCallBack(const std_msgs::Float64::ConstPtr &yaw) {
  controlEffort_yaw = yaw->data;
  if (controlEffort_yaw >= 65)
    controlEffort_yaw = 65;
}

void surgeCallBack(const std_msgs::Float64::ConstPtr &surge) {
  controlEffort_surge = surge->data;
}

void swayCallBack(const std_msgs::Float64::ConstPtr &sway) {
  controlEffort_sway = sway->data;
  //  ROS_INFO("CE_S %f", controlEffort_sway);
}

void z_mult_callback(const std_msgs::Float64::ConstPtr &msg) {
  z_thruster_multiplier = msg->data;
}

void xy_mult_callback(const std_msgs::Float64::ConstPtr &msg) {
  xy_thruster_multiplier = msg->data;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle n;
  ros::Publisher thrusterValues_pub =
      n.advertise<auv_motor_control::thruster_values>("thruster_values", 100);
  //  ros::Subscriber sub_movement = n.subscribe("movement", 1,
  //  headingCallback);
  ros::Publisher thruster_int_pub =
      n.advertise<auv_motor_control::thruster_int>("thruster_values_int", 100);

  ros::Subscriber controlEffort_yaw_sub =
      n.subscribe("controlEffort_yaw", 100, yawCallBack);
  ros::Subscriber controlEffort_heave_sub =
      n.subscribe("controlEffort_heave", 100, heaveCallBack);
  ros::Subscriber controlEffort_surge_sub =
      n.subscribe("controlEffort_surge", 100, surgeCallBack);
  ros::Subscriber controlEffort_sway_sub =
      n.subscribe("controlEffort_sway", 100, swayCallBack);
  ros::Subscriber z_mult_sub = n.subscribe("z_mult", 100, z_mult_callback);
  ros::Subscriber xy_mult_sub = n.subscribe("xy_mult", 100, xy_mult_callback);

  ros::Rate loop_rate(10);
  int count = 0;

  thruster_xy_frontRight = 0;
  thruster_xy_frontLeft = 0;
  thruster_xy_backRight = 0;
  thruster_xy_backLeft = 0;

  thruster_z_frontRight = 0;
  thruster_z_frontLeft = 0;
  thruster_z_backRight = 0;
  thruster_z_backLeft = 0;

  while (ros::ok()) {
    auv_motor_control::thruster_values thruster_outputs;
    auv_motor_control::thruster_int thrusters_int;

    magnitude_strafe =
        sqrt(2) *
        sqrt((controlEffort_surge * controlEffort_surge) +
             (controlEffort_sway *
              controlEffort_sway)); // find magnitude of control efforts.
    // mult by srt(2) to scale to -100 to 100
    controlEffort_roll = 0;
    controlEffort_pitch = 0;

    ///////////////////////////xy plane
    /// thrusters/////////////////////////////////
    strafe_frontRight =
        (magnitude_surge_travel(postPIDHeading()) * magnitude_strafe);
    strafe_frontLeft =
        (magnitude_sway_travel(postPIDHeading()) * magnitude_strafe);
    strafe_backRight =
        (magnitude_sway_travel(postPIDHeading()) * magnitude_strafe);
    strafe_backLeft =
        (magnitude_surge_travel(postPIDHeading()) * magnitude_strafe);
    ROS_INFO("\033[2J\033[1;1H");
    ROS_INFO("magnitude_strafe %f", magnitude_strafe);
    ROS_INFO("pid head %f", postPIDHeading());
    ROS_INFO("mag_surg_travel pid head %f",
             magnitude_surge_travel(postPIDHeading()));

    ROS_INFO("STRAFE: FR: %f, FL: %f, BR: %f, BL: %f", strafe_frontRight,
             strafe_frontLeft, strafe_backRight, strafe_backLeft);

    // yaw_frontRight = -1 * magnitude_yaw * (controlEffort_yaw / -100);
    // yaw_frontLeft = magnitude_yaw * (controlEffort_yaw / -100);
    // yaw_backRight = -1 * magnitude_yaw * (controlEffort_yaw / -100);
    // yaw_backLeft = magnitude_yaw * (controlEffort_yaw / -100);

    yaw_frontRight = controlEffort_yaw;
    yaw_frontLeft = -1 * controlEffort_yaw;
    yaw_backRight = controlEffort_yaw;
    yaw_backLeft = -1 * controlEffort_yaw;

    ROS_INFO("YAW: FR: %f, FL: %f, BR: %f, BL: %f", yaw_frontRight,
             yaw_frontLeft, yaw_backRight, yaw_backLeft);

    thruster_xy_frontRight_merge = strafe_frontRight + yaw_frontRight;
    thruster_xy_frontLeft_merge = strafe_frontLeft + yaw_frontLeft;
    thruster_xy_backRight_merge = strafe_backRight + yaw_backRight;
    thruster_xy_backLeft_merge = strafe_backLeft + yaw_backLeft;

    ROS_INFO("COMBINED: FR: %f, FL: %f, BR: %f, BL: %f",
             thruster_xy_frontRight_merge, thruster_xy_frontLeft_merge,
             thruster_xy_backRight_merge, thruster_xy_backLeft_merge);

    proportionalDivider_xy = getProportionalDivider(
        thruster_xy_frontRight_merge, thruster_xy_frontLeft_merge,
        thruster_xy_backLeft_merge, thruster_xy_backRight_merge);

    thruster_xy_frontRight =
        removeNaN(thruster_xy_frontRight_merge / proportionalDivider_xy);
    ROS_INFO("xyfr %f", thruster_xy_frontRight);
    thruster_xy_frontLeft =
        removeNaN(thruster_xy_frontLeft_merge / proportionalDivider_xy);
    thruster_xy_backRight =
        removeNaN(thruster_xy_backRight_merge / proportionalDivider_xy);
    thruster_xy_backLeft =
        removeNaN(thruster_xy_backLeft_merge / proportionalDivider_xy);

    ///////////////////////////`Z` plane
    /// thrusters///////////////////////////////////

    heave_allVertical = magnitude_heave * (controlEffort_heave / 100);
    // ROS_INFO("control heave %f", controlEffort_heave);
    int count = 0;

    pitch_verticalFront = -1 * magnitude_pitch * (controlEffort_pitch / 100);
    pitch_verticalBack = magnitude_pitch * (controlEffort_pitch / 100);

    roll_verticalRight = -1 * magnitude_roll * controlEffort_roll / 100;
    roll_verticalLeft = magnitude_roll * controlEffort_roll / 100;

    thruster_z_frontRight_merge =
        heave_allVertical + pitch_verticalFront + roll_verticalRight;
    thruster_z_frontLeft_merge =
        heave_allVertical + pitch_verticalFront + roll_verticalLeft;
    thruster_z_backRight_merge =
        heave_allVertical + pitch_verticalBack + roll_verticalRight;
    thruster_z_backLeft_merge =
        heave_allVertical + pitch_verticalBack + roll_verticalLeft;

    proportionalDivider_vertical = getProportionalDivider(
        thruster_z_frontRight_merge, thruster_z_frontLeft_merge,
        thruster_z_backLeft_merge, thruster_z_backRight_merge);

    thruster_z_frontRight =
        removeNaN(thruster_z_frontRight_merge / proportionalDivider_vertical);
    thruster_z_frontLeft =
        removeNaN(thruster_z_frontLeft_merge / proportionalDivider_vertical);
    thruster_z_backRight =
        removeNaN(thruster_z_backRight_merge / proportionalDivider_vertical);
    thruster_z_backLeft =
        removeNaN(thruster_z_backLeft_merge / proportionalDivider_vertical);

    thruster_outputs.thruster_xy_frontRight =
        xy_thruster_multiplier *
        deadband(thruster_xy_frontRight, 0.5); /// sets all values in
                                               /// thruster_outputs topic to the
                                               /// thruster values. Useful for
                                               /// debugging
    thruster_outputs.thruster_xy_frontLeft =
        xy_thruster_multiplier * deadband(thruster_xy_frontLeft, 0.5);
    thruster_outputs.thruster_xy_backRight =
        xy_thruster_multiplier * deadband(thruster_xy_backRight, 0.5);
    thruster_outputs.thruster_xy_backLeft =
        xy_thruster_multiplier * deadband(thruster_xy_backLeft, 0.5);

    thruster_outputs.thruster_z_frontRight =
        removeNaN(z_thruster_multiplier * deadband(thruster_z_frontRight, 0.5));
    ROS_INFO("fr std::isnan: %u", std::isnan(thruster_outputs.thruster_z_frontRight));
    thruster_outputs.thruster_z_frontLeft =
        z_thruster_multiplier * deadband(thruster_z_frontLeft, 0.5);
    thruster_outputs.thruster_z_backRight =
        z_thruster_multiplier * deadband(thruster_z_backRight, 0.5);
    thruster_outputs.thruster_z_backLeft =
        z_thruster_multiplier * deadband(thruster_z_backLeft, 0.5);

    // publishes as ints, better for serial node
    thrusters_int.thruster_xy_frontRight =
        (int)thruster_outputs.thruster_xy_frontRight;
    thrusters_int.thruster_xy_frontLeft =
        (int)thruster_outputs.thruster_xy_frontLeft;
    thrusters_int.thruster_xy_backRight =
        (int)thruster_outputs.thruster_xy_backRight;
    thrusters_int.thruster_xy_backLeft =
        (int)thruster_outputs.thruster_xy_backLeft;
    // thrusters_int.thruster_xy_backRight = 0;
    // thrusters_int.thruster_xy_backLeft = 0;
    thrusters_int.thruster_z_frontRight =
        (int)thruster_outputs.thruster_z_frontRight;
    thrusters_int.thruster_z_frontLeft =
        (int)thruster_outputs.thruster_z_frontLeft;
    thrusters_int.thruster_z_backRight =
        (int)thruster_outputs.thruster_z_backRight;
    thrusters_int.thruster_z_backLeft =
        (int)thruster_outputs.thruster_z_backLeft;

    // thrusterValues_pub.publish(thruster_outputs); //published all thruster
    // values
    thruster_int_pub.publish(thrusters_int);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

float magnitude_surge(float heading) { // takes the desired angle of travel and
                                       // outputs magnitude in the y direction.
                                       // Outputs -1 to 1
  return cos(degreesToRadians(heading));
}

float magnitude_sway(float heading) { // takes the desired angle of travel and
                                      // outputs magnitude in the x direction
                                      // (surge) Outputs -1 to 1
  return sin(degreesToRadians(heading));
}

float magnitude_sway_travel(float heading) { // takes the desired angle of
                                             // travel and outputs magnitude in
                                             // the y direction after offseting
                                             // by 45 degrees
  heading += 45;
  if (heading >= 360) {
    heading = heading - 360;
  }
  return magnitude_sway(heading);
}

float magnitude_surge_travel(float heading) { // takes the desired angle of
                                              // travel and outputs magnitude in
                                              // the y direction
  heading += 45;
  if (heading >= 360) {
    heading -= 360;
  }

  return magnitude_surge(heading);
}

float postPIDHeading() { // integrates values from the surge and
                         // sway pid controllers and returns an
                         // updated heading
  // double magnitude_linear_surge;
  // double magnitude_linear_sway;
  // (most of the mission code)
  // magnitude_linear_surge = controlEffort_sway / 100; // deal with it
  // magnitude_linear_sway = controlEffort_surge / 100;
  // magnitude_linear_surge = controlEffort_sway;
  // magnitude_linear_sway = controlEffort_surge;

  ROS_INFO("surge %f, sway %f", controlEffort_surge, controlEffort_sway);
  int offset;

  // if (magnitude_linear_surge >= 0 && magnitude_linear_sway >= 0)
  //   offset = 0;
  // else if ((magnitude_linear_surge >= 0 && magnitude_linear_sway < 0) ||
  //          (magnitude_linear_surge < 0 && magnitude_linear_sway < 0))
  //   offset = 180;
  // else
  //   offset = 360;

  if (controlEffort_sway >= 0 && controlEffort_surge >= 0)
    offset = 0;
  else if ((controlEffort_sway >= 0 && controlEffort_surge < 0) ||
           (controlEffort_sway < 0 && controlEffort_surge < 0))
    offset = 180;
  else
    offset = 360;

  return radiansToDegrees(atan(controlEffort_sway / controlEffort_surge)) +
         offset;
}

double getProportionalDivider(double thrusterValue1, double thrusterValue2,
                              double thrusterValue3, double thrusterValue4) {

  double
      greatestAbsThrusterValue; // store the highestabs value of input thrusters

  if (fabs(thrusterValue1) >= fabs(thrusterValue2) &&
      fabs(thrusterValue1) >= fabs(thrusterValue3) &&
      fabs(thrusterValue1) >=
          fabs(thrusterValue4)) { // find greatestAbsThrusterValue
    greatestAbsThrusterValue = fabs(thrusterValue1);
    //  ROS_INFO("greatst thruster: [%f]", 1.0);

  }

  else if (fabs(thrusterValue2) >= fabs(thrusterValue1) &&
           fabs(thrusterValue2) >= fabs(thrusterValue3) &&
           fabs(thrusterValue2) >= fabs(thrusterValue4)) {
    greatestAbsThrusterValue = fabs(thrusterValue2);
    //  ROS_INFO("greatst thruster: [%f]", 2.0);

  }

  else if (fabs(thrusterValue3) >= fabs(thrusterValue1) &&
           fabs(thrusterValue3) >= fabs(thrusterValue2) &&
           fabs(thrusterValue3) >= fabs(thrusterValue4)) {
    greatestAbsThrusterValue = fabs(thrusterValue3);
    //    ROS_INFO("greatst thruster: [%f]", 3.0);

  }

  else {
    greatestAbsThrusterValue = fabs(thrusterValue4);
    //  ROS_INFO("greatst thruster: [%f]", 4.0);
  }

  if (fabs(greatestAbsThrusterValue) >
      100) { // checks if the greatest thruster value is actually greater than
             // 100, the max value. If it is, scale all down by a proportional
             // divider
    return greatestAbsThrusterValue / 100; // finds a multiplier that will scale
                                           // largest down to 100, everythign
                                           // else accordingly
  } else {
    return 1.0; // if not greater than 100, return 1; dividing by 1 changes
                // nothing
  }
}

double radiansToDegrees(double radian_input) {
  return radian_input * 180 / M_PI;
}
double degreesToRadians(double degree_input) {
  return degree_input * M_PI / 180;
}

double deadband(double input, double deadband) {
  if (fabs(input) < deadband) {
    return 0;
  } else {
    return input;
  }
}

double removeNaN(double input) {
  if (std::isnan(input))
    return 0;
  else
    return input;
}
