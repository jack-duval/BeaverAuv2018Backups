Header header

#Strafe Variables

float64 direction_strafe #What direction you move in - strafes
float64 magnitude_strafe #control speed. Default is 100. Only change if testing shows you need to go slower
bool globalCoord_strafe #If true, moves in relation to world. If false, forward is 0 degrees, right 90, back 180, left 270


#heave variables
float64 setpoint_heave #A specific depth to go to.
float64 magnitude_heave  #control speed. Default is 100. Only change if testing shows you need to go slower

#Yaw variables
float64 setpoint_yaw #Specific rotation point
float64 magnitude_yaw #control speed. Default is 100. Only change if testing shows you need to go slower
bool globalCoord_yaw #If true, moves in relation to world. If false, forward is 0 degrees, right 90, back 180, left 270

#roll variables
float64 magnitude_roll #control speed. Default is 100. Only change if testing shows you need to go slower
float64 setpoint_roll #rotation value. Set and robot will go to there. Default is 0. Right ascends, left goes from 360 down.

#pitch variables
float64 magnitude_pitch  #control speed. Default is 100. Only change if testing shows you need to go slower
float64 setpoint_pitch  #rotation value. Set and robot will go to there. Default is 0. Down ascends, Up goes from 360 down.
