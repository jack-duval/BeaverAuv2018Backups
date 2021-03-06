#include <auv_mission_control/TaskGate.h>
//suitable depth for gate task, clears by a good amount, but is still visible from land.
double thisDepth = 1.5;
//int runOnce = 0;

TaskGate::TaskGate(){
}


TaskGate::TaskGate(PidManager* pm, Camera* cam, TaskVision* vision) : pm_(*pm), cam_(*cam), vision_(*vision){
  ROS_INFO("TASK GATE INIT");
}

TaskGate::~TaskGate(){

}

int TaskGate::execute(){

  ros::Rate gateRate(20.);

  //pm_.pidEnable("ALL", true);//turns on all 6 pid controllers
  //INFO we did not use this because we only semi-tuned one axis. This will be something to do next year.

  while(ros::ok){ // change so it's while keep running, some value that determines whether to keep running
  killSwitch = pm_.getKill();
  if(killSwitch){
    ROS_ERROR("Kill switch detected");
    return kill;
    break;
  }
  //  if(getTimeout()){ //checks 15 min timer, if activated signals to enter resurface state
  //    return timeout;
  //  }

    switch(action){
      case 0: { //first step, go to depth
        ros::spinOnce();

        pm_.setPidEnabled(AXIS_SWAY, false);
        pm_.setPidEnabled(AXIS_YAW, true);
        pm_.setPidEnabled(AXIS_HEAVE, true);
        pm_.setPidEnabled(AXIS_SURGE, false);

        //pm_.setControlEffort(AXIS_SWAY, 0);
        //pm_.setControlEffort(AXIS_SURGE, 0);
        //Changed To try and set 0 on start
	//if(runOnce == 1){
//Probably redundant, but if it ain't broke don't fix it.
        pm_.setZero(AXIS_YAW);
        pm_.setZero(AXIS_YAW);
        pm_.setZero(AXIS_YAW);
        pm_.setZero(AXIS_YAW);
        pm_.setZero(AXIS_YAW);
        pm_.setZero(AXIS_YAW);
        pm_.setZero(AXIS_YAW);
        pm_.setZero(AXIS_YAW);

	//runOnce = 1;
	//}

        killSwitch = pm_.getKill();
        if(killSwitch){
          ROS_ERROR("Kill switch detected");
          return kill;
          break;
        }

        ROS_INFO("Rummage Rummage Rummage, rummaging to depth");

	ROS_INFO("Buffalo Milk incoming, setting Yaw PlantState -> 0");
  // Changed at Competition from : ROS_INFO(pm.getYaw()
	      pm_.setPlantState(AXIS_YAW, 0);
        pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);
        pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);

        while(ros::ok){
          pm_.setPlantState(AXIS_HEAVE, pm_.getDepth());

          currentDepth = pm_.getDepth();
          double error = fabs(currentDepth - thisDepth);
          if(rosInfoCounter%20 == 0){
            ROS_INFO(startTimer ? "It do be like that sometimes, timer started" : "I did not make it to depth, I did not! Timer not started");
	    //ROS_INFO("error%f", error);
            //ROS_INFO("depth%f", pm_.getDepth());
	  }
          pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);
          //Changed on August 4th 2018 to try and head straight

	  pm_.setPlantState(AXIS_YAW, pm_.getYaw());
          pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);
          killSwitch = pm_.getKill();
          if(killSwitch){
            ROS_ERROR("Kill switch detected");
            return kill;
            break;
          }

          startTimer = false;
          rosInfoCounter++;
          if(error < .01)
            break;
          ros::spinOnce();
          gateRate.sleep();
        }

        ROS_INFO("Near depth setpoint of %f; currently at %f. Starting depth timer.", thisDepth, pm_.getDepth());
        pm_.setPlantState(AXIS_HEAVE, pm_.getDepth());
        pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);
        pm_.setPlantState(AXIS_YAW, pm_.getYaw());
        pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);

        startTimer = true;
        killSwitch = pm_.getKill();
        if(killSwitch){
          ROS_ERROR("Kill switch detected");
          return kill;
          break;
        }

        if(depthCounter < 1 && startTimer == true){
          pm_.setPlantState(AXIS_HEAVE, pm_.getDepth());
          pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);
//CHANGE
          pm_.setPlantState(AXIS_YAW, 0);
          pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);

          ROS_INFO("pm_.getDepth %f", pm_.getDepth());
          goToDepth_time.start();
          depthCounter ++;
          ROS_INFO("Depth timer started");
          killSwitch = pm_.getKill();
          if(killSwitch){
            ROS_ERROR("Kill switch detected");
            return kill;
            break;
          }
        }


        if(goToDepth_time.getTime() < 2){//just chill
	  ROS_INFO("Timer %f", pm_.getDepth());
          pm_.setPlantState(AXIS_HEAVE, pm_.getDepth());
          pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);
          pm_.setPlantState(AXIS_YAW, pm_.getYaw());
          pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);

          killSwitch = pm_.getKill();
          if(killSwitch){
            ROS_ERROR("Kill switch detected");
            return kill;
            break;
          }
          pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);
          ros::spinOnce();
          gateRate.sleep();
        }
	else
	{
        ROS_INFO("Done going to depth. At depth %f", pm_.getDepth());
        action = 1;
	}
        break;

      }

      case 1: {
        ROS_INFO("Vroom Vroom going forwards");
        if (forwardCounter < 1)
          driveForwards_time.start();
        forwardCounter++;
        pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);
        pm_.setPlantState(AXIS_HEAVE, pm_.getDepth());
        pm_.setPlantState(AXIS_YAW, pm_.getYaw());
        pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);

        killSwitch = pm_.getKill();
        if(killSwitch){
          ROS_ERROR("Kill switch detected");
          return kill;
          break;
        }

        while(driveForwards_time.getTime() < 520){
          pm_.setPlantState(AXIS_HEAVE, pm_.getDepth());
          pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);
          pm_.setPlantState(AXIS_YAW, pm_.getYaw());
          ROS_INFO("yaw%f", pm_.getYaw());
          pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);
	//changed this at 12:30 day of run, Worked on final day.
          pm_.setControlEffort(AXIS_SURGE, -75);
          killSwitch = pm_.getKill();
          if(killSwitch){
            ROS_ERROR("Kill switch detected");
            return kill;
            break;
          }
          ros::spinOnce();
          gateRate.sleep();
        }

        pm_.setControlEffort(AXIS_SURGE, 0);

        killSwitch = pm_.getKill();
        if(killSwitch){
          ROS_ERROR("Kill switch detected");
          return kill;
          break;
        }
        pm_.setSetpoint(AXIS_HEAVE, INPUT_DEPTH, thisDepth);

        pm_.setPlantState(AXIS_HEAVE, pm_.getDepth());
        pm_.setPlantState(AXIS_YAW, pm_.getYaw());
        pm_.setSetpoint(AXIS_YAW, INPUT_IMU_POS, 0);

        pm_.setControlEffort(AXIS_SURGE, 0);
        ROS_INFO("I THINK that I'm through the gate");
        action = 2;
        break;

      }

      case 2:{ //return succeeded, and please proceed to the nearest task as quickly and calmly as possible, keeping in mind that it may be behind you

   pm_.setControlEffort(AXIS_HEAVE, 0);
   return succeeded;
        break;

     }
    };

    ros::spinOnce();
    gateRate.sleep();

  }//while ros::ok
}//execute
