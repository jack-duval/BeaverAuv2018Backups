#include "auv_motor_control/movement_values.h"
#include "auv_motor_control/pid_enable.h"
#include "math.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <auv_motor_control/controller_to_heading.hpp>
#include <auv_motor_control/direction_strafe_macros.hpp>
#include <ros/console.h>
#include <sstream>
// #include <auv_mission_control/PidManager.hpp>
////Controller variables////
float leftJoy_x;
float leftJoy_y;
float rightJoy_x;
float rightJoy_y;
float rightTrigger;
float leftTrigger;
float dPad_x;
float dPad_y;
int32_t controllerButton_a;
int32_t controllerButton_b;
int32_t controllerButton_x;
int32_t controllerButton_y;
int32_t controllerButton_start;
int32_t controllerButton_back;
int32_t controllerButton_rightStick;
int32_t controllerButton_leftStick;
int32_t rightBumper;
int32_t leftBumper;

////pid variables///
bool pid_enable_heave = 0;
bool pid_enable_roll = 0;
bool pid_enable_pitch = 0;
bool pid_enable_yaw = 0;

double currentRoll;
double rollHold;

double currentPitch;
double pitchHold;

double currentDepth = 10;
double verticalHold;

double surge;
double sway;
double heave;
double roll;
double pitch;
double yaw;

double yawHold = 0;
double currentYaw;

////

void joyCallback(const sensor_msgs::Joy joy) {
  leftJoy_x = -1 * joy.axes[0];
  leftJoy_y = joy.axes[1];
  rightJoy_x = -1 * joy.axes[3];
  rightJoy_y = joy.axes[4];
  rightTrigger = -1 * (joy.axes[5] - 1) / 2;
  leftTrigger = -1 * (joy.axes[2] - 1) / 2;
  dPad_x = -1 * (joy.axes[6]);
  dPad_y = joy.axes[7];
  controllerButton_a = joy.buttons[0];
  controllerButton_b = joy.buttons[1];
  controllerButton_x = joy.buttons[2];
  controllerButton_y = joy.buttons[3];
  controllerButton_start = joy.buttons[7];
  controllerButton_back = joy.buttons[6];
  controllerButton_rightStick = joy.buttons[10];
  controllerButton_leftStick = joy.buttons[9];
  rightBumper = joy.buttons[4];
  leftBumper = joy.buttons[5];

  //   ROS_INFO("I heard: [%u]", joy.buttons[5]);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "controller_to_heading");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
  std_msgs::Float64 surge_msg;
  std_msgs::Float64 sway_msg;
  std_msgs::Float64 heave_msg;
  std_msgs::Float64 roll_msg;
  std_msgs::Float64 pitch_msg;
  std_msgs::Float64 yaw_msg;

  ros::Rate loop_rate(10);
  // ros::Publisher movement_pub =
  // n.advertise<auv_motor_control::movement_values>("movement", 1);
  ros::Publisher surge_pub =
      n.advertise<std_msgs::Float64>("controlEffort_surge", 10);
  ros::Publisher sway_pub =
      n.advertise<std_msgs::Float64>("controlEffort_sway", 10);
  ros::Publisher heave_pub =
      n.advertise<std_msgs::Float64>("controlEffort_heave", 10);
  ros::Publisher roll_pub =
      n.advertise<std_msgs::Float64>("controlEffort_roll", 10);
  ros::Publisher pitch_pub =
      n.advertise<std_msgs::Float64>("controlEffort_pitch", 10);
  ros::Publisher yaw_pub =
      n.advertise<std_msgs::Float64>("controlEffort_yaw", 10);

  int count = 0;
  while (ros::ok()) {

    surge_msg.data = leftJoy_y;
    sway_msg.data = leftJoy_x;
    heave_msg.data = dPad_y * 100;
    roll_msg.data = rightJoy_x;
    pitch_msg.data = rightJoy_y;
    yaw_msg.data = ((-1 * leftTrigger) + rightTrigger) * 100;

    surge_pub.publish(surge_msg);
    sway_pub.publish(sway_msg);
    heave_pub.publish(heave_msg);
    roll_pub.publish(roll_msg);
    pitch_pub.publish(pitch_msg);
    yaw_pub.publish(yaw_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

bool convertToBool(int32_t input) {
  if (input > 0) {
    return 1;
  } else {
    return 0;
  }
}

double getDirection_strafe(float xAxis, float yAxis) {
  if (xAxis > 0 && yAxis > 0) {
    return ((180 * atan(xAxis / yAxis)) / M_PI);
  } else if (xAxis < 0 && yAxis > 0) {
    return ((180 * atan(xAxis / yAxis)) / M_PI) + 360;
  } else if (xAxis < 0 && yAxis < 0) {
    return ((180 * atan(xAxis / yAxis)) / M_PI) + 180;
  } else if (xAxis > 0 && yAxis < 0) {
    return ((180 * atan(xAxis / yAxis)) / M_PI) + 180;
  }

  else if (xAxis == 0 && yAxis == 1) {
    return 0;
  } else if (xAxis == 1 && yAxis == 0) {
    return 90;
  } else if (xAxis == 0 && yAxis == -1) {
    return 180;
  } else if (xAxis == -1 && yAxis == 0) {
    return 270;
  }
}

double max1(double input) {
  if (input >= 1) {
    return 1;
  } else {
    return input;
  }
}

double deadband(double input, double deadband) {
  if (fabs(input) < deadband) {
    return 0;
  } else {
    return input;
  }
}
