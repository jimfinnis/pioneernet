#!/usr/bin/python

import rospy
from gazebo_msgs.msg import ModelStates

print "init"
rospy.init_node('logpos',anonymous=True)

def callback(data):
    idx=data.name.index('robot_description')
    pos = data.pose[idx].position
    print "%f,%f" % (pos.x,pos.y)
    
rospy.Subscriber('gazebo/model_states',ModelStates,callback)

rospy.spin()
