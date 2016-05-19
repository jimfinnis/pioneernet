/**
 * @file pioneernet_node.cpp
 * @brief  Brief description of file.
 *
 */
#include "ros/ros.h"
#include "backpropNoBiasHormone.h"

#include "lightsensor_gazebo/LightSensor.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"


#define NUM_SONARS 8

double sonarRets[NUM_SONARS];

inline double sonarReturn(double d,double mnd,double mxd){
    if(d<0)return 0;
    d-=mnd;
    d/=(mxd-mnd);
    d=1.0-d;
    if(d<0)return 0;
    if(d>1)return 1;
    return d;
}

void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,int i){
    sonarRets[i] = sonarReturn(msg->range,100,400); // min/maxdist in mm!
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
        sonarRets[i]=0;
    }
    
    ros::Publisher leftmotor = n.advertise<std_msgs::Float64>("leftmotor",1000);
    ros::Publisher rightmotor = n.advertise<std_msgs::Float64>("rightmotor",1000);
    
    BackpropNet *net = BackpropNet::loadNet("net");
    printf("Net loaded (%d-%d-%d), running...\n",
           net->getLayerSize(0),
           net->getLayerSize(1),
           net->getLayerSize(2)
           );
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        
        // update the robot
        net->setH(0);
        net->setInputs(sonarRets);
        net->update();
        
        for(int i=0;i<NUM_SONARS;i++){
            printf("%2.2f ",sonarRets[i]);
        }printf("\n");
        
        double *outs = net->getOutputs();
        
        std_msgs::Float64 m;
        m.data = outs[0];
        leftmotor.publish(m);
        m.data = outs[1];
        rightmotor.publish(m);
    }

}
