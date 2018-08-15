#include "auv_motor_control/movement_values.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <auv_motor_control/pushValuesConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/console.h>
#include <sstream>

double setpoint_surge;
double setpoint_sway;
double setpoint_heave;
double setpoint_yaw;

double plantState_surge;
double plantState_sway;
double plantState_heave;
double plantState_yaw;

bool startSwitch;
bool killSwitch;

void configCb(auv_motor_control::pushValuesConfig &config, uint32_t level) {
  setpoint_surge = config.setpoint_surge;
  setpoint_sway = config.setpoint_sway;
  setpoint_heave = config.setpoint_heave;
  setpoint_yaw = config.setpoint_yaw;

  plantState_surge = config.plantState_surge;
  plantState_sway = config.plantState_sway;
  plantState_heave = config.plantState_heave;
  plantState_yaw = config.plantState_yaw;

  startSwitch = config.startSwitch;
  killSwitch = config.killSwitch;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "push_heading");
  ros::NodeHandle n;
  dynamic_reconfigure::Server<auv_motor_control::pushValuesConfig> server;
  dynamic_reconfigure::Server<auv_motor_control::pushValuesConfig>::CallbackType
      f;
  f = boost::bind(&configCb, _1, _2);
  server.setCallback(f);

  ros::Publisher setpoint_surge_pub =
      n.advertise<std_msgs::Float64>("setpoint_surge", 100);
  ros::Publisher setpoint_sway_pub =
      n.advertise<std_msgs::Float64>("setpoint_sway", 100);
  ros::Publisher setpoint_heave_pub =
      n.advertise<std_msgs::Float64>("detph", 100);
  ros::Publisher setpoint_yaw_pub =
      n.advertise<std_msgs::Float64>("setpoint_yaw", 100);

  ros::Publisher startSwitch_pub = n.advertise<std_msgs::Bool>("start", 1);
  ros::Publisher killSwitch_pub = n.advertise<std_msgs::Bool>("kill", 1);

  /*
    ros::Publisher plantState_surge_pub =
    n.advertise<std_msgs::Float64>("plantState_surge", 100);
    ros::Publisher plantState_sway_pub =
    n.advertise<std_msgs::Float64>("plantState_sway", 100);
    ros::Publisher plantState_heave_pub =
    n.advertise<std_msgs::Float64>("plantState_heave", 100);
    ros::Publisher plantState_yaw_pub =
    n.advertise<std_msgs::Float64>("markerAngle_vision", 100);
  */
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {

    std_msgs::Float64 set_surge;
    std_msgs::Float64 set_sway;
    std_msgs::Float64 set_heave;
    std_msgs::Float64 set_yaw;

    std_msgs::Float64 plant_surge;
    std_msgs::Float64 plant_sway;
    std_msgs::Float64 plant_heave;
    std_msgs::Float64 plant_yaw;

    std_msgs::Bool start;
    std_msgs::Bool kill;

    plant_surge.data = plantState_surge;
    plant_sway.data = plantState_sway;
    plant_heave.data = plantState_heave;
    plant_yaw.data = plantState_yaw;

    set_surge.data = setpoint_surge;
    set_sway.data = setpoint_sway;
    set_heave.data = setpoint_heave;
    set_yaw.data = setpoint_yaw;

    start.data = startSwitch;
    kill.data = killSwitch;

    setpoint_surge_pub.publish(set_surge);
    setpoint_sway_pub.publish(set_sway);
    setpoint_heave_pub.publish(set_heave);
    setpoint_yaw_pub.publish(set_yaw);
    startSwitch_pub.publish(start);
    killSwitch_pub.publish(kill);

    //    plantState_surge_pub.publish(plant_surge);
    //    plantState_sway_pub.publish(plant_sway);
    //    plantState_heave_pub.publish(plant_heave);
    //    plantState_yaw_pub.publish(plant_yaw);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  ros::spin();
  return 0;
}
