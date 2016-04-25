/**
 * @file pioneernet_node.cpp
 * @brief  Brief description of file.
 *
 */
#include "ros/ros.h"
#include "backpropNoBiasHormone.h"

#include "lightsensor_gazebo/LightSensor.h"
#include "sensor_msgs/Range.h"


#define NUM_SONARS 8

void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,int i){
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"pioneernet_node");
    ros::NodeHandle n;
    
    ros::Subscriber s[NUM_SONARS];
    for(int i=0;i<NUM_SONARS;i++){
        char buf[32];
        sprintf(buf,"s%d",i);
        s[i] = n.subscribe<sensor_msgs::Range>(buf,1000,
                                 boost::bind(sonarCallback,_1,i));
    }
    
    
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

}
