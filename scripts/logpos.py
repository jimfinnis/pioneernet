#!/usr/bin/python

import rospy
from gazebo_msgs.msg import ModelStates

rospy.init_node('logpos',anonymous=True)

# crude throttling
msgidx=0

def callback(data):
    global msgidx
    if(msgidx%100==0):
        idx=data.name.index('robot_description')
        pos = data.pose[idx].position
        print "%f,%f" % (pos.x,pos.y)
    msgidx=msgidx+1
    
rospy.Subscriber('gazebo/model_states',ModelStates,callback)

rospy.spin()
