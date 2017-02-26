#!/usr/bin/env python

import rospy
import rosbag
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import numpy as np

f = open("bag_content.txt","w")
bag = rosbag.Bag('/home/first/catkin_ws/src/lab4/bag/grid.bag','r')
topics = []
topics.append('Motion')
topics.append('Observation')


try:
	for topic, msg, time_stamp in bag.read_messages(topics=['Movements', 'Observations']):
		
		f.write(str(topic) + "\n")
		if topic == 'Movements':
			print np.degrees(euler_from_quaternion((msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w))[2])
			f.write(str(msg) + "\n")
			f.write(str(time_stamp) + "\n")
		if topic == 'Observations': 			
			f.write(str(msg) + "\n")
			f.write(str(time_stamp) + "\n")

finally:
	bag.close()
	f.close()

