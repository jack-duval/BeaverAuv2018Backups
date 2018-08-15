#!/usr/bin/env python

#<node pkg ="auv_raspi" name = "StartSwitch" type = "StartSwitch.py" output="screen">
#</node>

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

def talker():
	pub = rospy.Publisher('start', Bool, queue_size=10)
	rospy.init_node('StartNode', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	runOnce = 0;
	while not rospy.is_shutdown():
		if GPIO.input(4):
			rospy.loginfo("Starting Mission")
			pub.publish(True)
		else:
			pub.publish(False)
			rospy.loginfo("Not starting Mission")
		rate.sleep()


if __name__ == '__main__':
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
	try:
		talker()
	except rospy.ROSInterruptException:
		rospy.logfatal("Runtime Exception")
