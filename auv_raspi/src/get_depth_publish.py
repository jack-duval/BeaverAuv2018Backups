#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from ms5837 import *

def talker():
	rospy.init_node('Depth_Sensor', anonymous=True)
	pub = rospy.Publisher('state_heave', Float64, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	depthpub = rospy.Publisher('depth', Float64, queue_size=10)
	while not rospy.is_shutdown():
		#print sensor.temperature(UNITS_Farenheit)
		if sensor.read():
			pub.publish(sensor.depth())
			#pub.publish(0.2)
	        else:
                	print "Sensor read failed!"
		depthpub.publish(0.5)
		rate.sleep()

if __name__ == '__main__':
	try:
		sensor = MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)
		if not sensor.init():
        		print ("Sensor could not be initialized")
		sensor.setFluidDensity(DENSITY_FRESHWATER)
		talker()
	except rospy.ROSInterruptException:
		pass
