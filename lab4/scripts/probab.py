#!/usr/bin/env python
import roslib
import numpy
import sys
import rospy
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Point
import rosbag

cur_arraypos = numpy.zeros((35.0, 35.0, 4.0))
m = Marker()
tag_pos = numpy.array([[125, 525],[125, 325],[125, 125],[425, 125],[425, 325],[425, 525]])
def init(): 
	
	rate = rospy.Rate(10)
	bag = rosbag.Bag('/home/first/catkin_ws/src/lab4/bag/grid.bag','r')
	
	cur_arraypos[12-1,28-1,3-1] = 1
	pub = rospy.Publisher('tag_marker', Marker, queue_size=150)
	
	publish_tags(pub)
	try:
		for topic, msg, time_stamp in bag.read_messages(topics=['Movements', 'Observations']):
			if topic == 'Movements':
				rotation1 = numpy.degrees((euler_from_quaternion([msg.rotation1.x,msg.rotation1.y,msg.rotation1.z,msg.rotation1.w]))[2])
				rotation2 = numpy.degrees((euler_from_quaternion([msg.rotation2.x,msg.rotation2.y,msg.rotation2.z,msg.rotation2.w]))[2])
				new_pos(rotation1, msg.translation*100, rotation2)
			else: 
				r = msg.bearing
				r = numpy.degrees((euler_from_quaternion([r.x, r.y, r.z, r.w]))[2])
				distance = msg.range * 100				
				pos_obs(msg.tagNum, distance, r)
	finally:
		bag.close()

def publish_tags(pub):

	for i in numpy.arange(tag_pos.shape[0]):
		m = Marker()
		m.header.frame_id = "/lab4"
		m.header.stamp = rospy.Time.now()
		m.ns = "tag_rviz"
		m.id = (i+1)
		m.type = Marker.CUBE
		m.pose.position.x = tag_pos[i,0]/100.0 -4
		m.pose.position.y = tag_pos[i,1]/100.0 -4
		m.pose.position.z = 0
		m.scale.x = 0.1
		m.scale.y = 0.1
		m.scale.z = 0.1 
		m.color.r = 1.0
		m.color.g = 1.0
		m.color.b = 0.0
		m.color.a = 1.0
		m.action = Marker.ADD
		pub.publish(m)
		while (pub.get_num_connections() < 1):
			yoo = 1
		while (pub.get_num_connections() < 1):
			yoo = 1

		for i in numpy.arange(tag_pos.shape[0]):
			m = Marker()
			m.header.frame_id = "/lab4"
			m.header.stamp = rospy.Time.now()
			m.ns = "tag_rviz"
			m.type = Marker.CUBE			
			m.id = (i+1)
			m.pose.position.x = tag_pos[5-i,0]/100.0 -4
			m.pose.position.y = tag_pos[5-i,1]/100.0 -4
			print m.pose.position.x, m.pose.position.y
			m.pose.position.z = 0
			m.color.r = 0.0
			m.color.g = 0.0
			m.color.b = 1.0
			m.color.a = 1.0				
			m.scale.x = 0.1
			m.scale.y = 0.1
			m.scale.z = 0.1 
			m.action = Marker.ADD
			pub.publish(m)
			while (pub.get_num_connections() < 1):
				yoo = 1
			while (pub.get_num_connections() < 1):
				yoo = 1

def return_pos(i, j, k):
	r = -135 + k * 90
	x = i * 20 + 10
	y = j * 20 + 10
	return r, x, y


def new_pos(rot1, trans, rot2):
	global cur_arraypos, temp_pos
	temp_pos = cur_arraypos
	cur_arraypos = numpy.copy(temp_pos)
	final_p = 0
	for temp_i in numpy.arange(cur_arraypos.shape[0]):
		for temp_j in numpy.arange(cur_arraypos.shape[1]):
			for temp_k in numpy.arange(cur_arraypos.shape[2]):
				if temp_pos[temp_i, temp_j, temp_k] < 0.1:
					continue
				for i in numpy.arange(cur_arraypos.shape[0]):
					for j in numpy.arange(cur_arraypos.shape[1]):
						for k in numpy.arange(cur_arraypos.shape[2]):
							rot1_tmp, trans_tmp, rot2_tmp = Calculate(i, j, k, temp_i, temp_j, temp_k)
							trans_p = (1.0/(numpy.sqrt(2*numpy.pi)*10))*numpy.power(numpy.e,-1.0*(((trans_tmp-trans)**2)/(2.0*10**2)))
							rot1_p = (1.0/(numpy.sqrt(2*numpy.pi)*45))*numpy.power(numpy.e,-1.0*(((rot1_tmp-rot1)**2)/(2.0*45**2)))
							rot2_p = (1.0/(numpy.sqrt(2*numpy.pi)*45))*numpy.power(numpy.e,-1.0*(((rot2_tmp-rot2)**2)/(2.0*45**2)))

							value = temp_pos[temp_i, temp_j, temp_k] * trans_p * rot1_p * rot2_p
							cur_arraypos[i, j, k] = cur_arraypos[i, j, k] + value
							final_p = final_p + value
	cur_arraypos = cur_arraypos / final_p
	index = numpy.argmax(cur_arraypos)
	disp_angle = index % cur_arraypos.shape[2]
	index = index / cur_arraypos.shape[2]
	disp_y = index % cur_arraypos.shape[1]
	index = index / cur_arraypos.shape[1]
	disp_x = index % cur_arraypos.shape[0]
	rviz_display(disp_x, disp_y, disp_angle)


def pos_obs(tag_number, trans, rot):
	global cur_arraypos, temp_pos
	temp_pos = cur_arraypos
	cur_arraypos = numpy.copy(temp_pos)
	final_p = 0
	for i in numpy.arange(cur_arraypos.shape[0]):
		for j in numpy.arange(cur_arraypos.shape[1]):
			for k in numpy.arange(cur_arraypos.shape[2]):
				rot_tmp, trans_tmp = Observation(i, j, k, tag_number)
				rot_prb = (1.0/(numpy.sqrt(2*numpy.pi)*45))*numpy.power(numpy.e,-1.0*(((rot_tmp-rot)**2)/(2.0*45**2)))
				trans_prb = (1.0/(numpy.sqrt(2*numpy.pi)*10))*numpy.power(numpy.e,-1.0*(((trans_tmp-trans)**2)/(2.0*10**2)))
				value = temp_pos[i, j, k] * trans_prb * rot_prb
				cur_arraypos[i, j, k] = value
				final_p = final_p + value
	cur_arraypos = cur_arraypos / final_p
	index = numpy.argmax(cur_arraypos)
	index_angle = index % cur_arraypos.shape[2]
	index = index / cur_arraypos.shape[2]
	index_y = index % cur_arraypos.shape[1]
	index = index / cur_arraypos.shape[1]
	index_x = index % cur_arraypos.shape[0]
	rviz_display(index_x, index_y, index_angle)


def rviz_display(x, y, z):
	global cur_arraypos,m
	pub = rospy.Publisher('line_marker', Marker, queue_size=150)
	m.header.frame_id = "/lab4"
	m.header.stamp = rospy.Time.now()
	m.id = 0
	m.ns = "line_rviz"	
	m.type = Marker.LINE_STRIP
	m.color.r = 0.0
	m.color.g = 1.0
	m.color.b = 0.0
	m.color.a = 1.0	
	m.scale.x = 0.05
	m.scale.y = 0.0
	m.scale.z = 0.0 
	point = Point()
	display_angle, display_x, display_y = return_pos(x, y, z)
	point.x = display_x/100.0 -4.0
	point.y = display_y/100.0 -4.0
	print("X :", display_x/100.0)
	print("Y :", display_y/100.0)
	point.z = 0
	m.points.append(point)

	m.action = Marker.ADD
	pub.publish(m)
	while (pub.get_num_connections() < 1):
		yo = 1


def Calculate(x, y, z, temp_x, temp_y, temp_z):
	r1, t1_x, t1_y = return_pos(x, y, z)
	r2, t2_x, t2_y = return_pos(temp_x, temp_y, temp_z)
	trans = numpy.sqrt((t1_x - t2_x) ** 2 + (t1_y - t2_y) ** 2)
	angle = numpy.degrees(numpy.arctan2(t1_y-t2_y, t1_x - t2_x))
	rotate1 = r1 - angle	
	rotate2 = angle - r2
	if rotate2 > 180:
		rotate2 = rotate2 - 360
	elif rotate2 < -180:
		rotate2 = rotate2 + 360		
	if rotate1 > 180:
		rotate1 = rotate1 - 360
	elif rotate1 < -180:
		rotate1 = rotate1 + 360
	return rotate2, trans, rotate1


def Observation(x, y, z, tag_number):
	global tag_pos
	rotate, trans_x, trans_y = return_pos(x, y, z)
	angle = numpy.degrees(numpy.arctan2(tag_pos[tag_number,1]-trans_y, tag_pos[tag_number,0] - trans_x))	
	trans = numpy.sqrt((trans_x - tag_pos[tag_number,0]) ** 2 + (trans_y - tag_pos[tag_number,1]) ** 2)
	r = angle - rotate
	if r > 180:
		r = r - 360
	elif r < -180:
		r = r + 360
	return r, trans


if __name__ == '__main__':
	try:
		rospy.init_node('probab')
		init()
	except rospy.ROSInterruptException:
		pass
