/**
 * @file bridge_node.cpp
 * @brief A bridge from the Pioneer's ARIA system to the local
 * ROS system.
 * 1) reads IP packets coming from other side of the bridge, the Pioneer,
 *    and converts them to ROS topic publications.
 * 2) reads ROS topics, converts them to IP packets, sends them to the
 *    other side of the bridge.
 * This is started as a client, so needs a server on the Pioneer side
 * running.
 */

#include "ros/ros.h"

#include "lightsensor_gazebo/LightSensor.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "tcp.h"
#include "pioneerros.h"

#include <diamondapparatus/diamondapparatus.h>
#include <iostream>
#include <string>
#include <sstream>

#define ENABLEDIAMOND 0

// the port we connect to on the Pioneer
#define PORT 34312


ros::Publisher sonarPubs[NUM_SONARS];
ros::Publisher lightPub;

double sigWidth=1;
double sigCentre=0.5;
double thresh=0.3;
double sigAmount=0;




inline double gloveconv(double f){
    return f/1024.0;
}
inline double sigmoid(double x){
    x=(x-sigCentre)/(sigWidth+0.001);
    return 1.0/(1.0+exp(-x));
}

// 5-kernel, sigma=1, half of it, backwards ;)
float k[]={0.38774,0.24477,0.06136};

void blur(uint8_t *out,uint8_t *p,int ch,int n){
    uint8_t mx=0;
    for(int i=0;i<n;i++){
        double t = 0;
        for(int j=-2;j<=2;j++){
            int px = (i+j+n)%n;
            t+= (double)(p[px*3+ch])*k[j<0?-j:j];
        }
        t /= 5.0;
        uint8_t bv=(uint8_t)t;
        out[i*3+ch]=bv;
        if(bv>mx)mx=bv;
    }
    // and normalise
    double normfac=255.0/(double)mx;
    for(int i=0;i<n;i++){
        double px = (double)(out[i*3+ch]);
        px *= normfac;
        out[i*3+ch]=(uint8_t)px;
    }
}



int processpixel(int p){
    double x = p;
//    printf("In: %f ",x);
    x = x/255.0;
//    printf("Scale: %f ",x);
    double orig = x;
    x=sigmoid(x);
//    printf("Sig: %f ",x);
    
    x=(sigAmount*x)+(1-sigAmount)*orig;
    
    if(x<thresh)x=0;
//    printf("Thr: %f ",x);
    
    return (int)(x*255.0);
}

/// the client object

class BridgeClient : public TCPClient<MotorPacket,SensorPacket> {
    int seq;
    sensor_msgs::Range r; // a range message
public:
    BridgeClient(const char *addr) : TCPClient<MotorPacket,SensorPacket>(addr,PORT){
        seq=0;
        r.header.frame_id = 1; // global frame (0 is no frame)
        r.radiation_type = 0; // ultrasonic
        r.field_of_view = 0.2618f; // 15 degrees
        r.min_range = 0;
        r.max_range = 5;
    }
    
    virtual void process(){
        printf("Processing\n");
        // turn the sensor packet into ROS topic publications
        r.header.seq = seq++;
        r.header.stamp = ros::Time::now();
        
        for(int i=0;i<NUM_SONARS;i++){
            r.range = resp.sonars[i]*0.001f;
            sonarPubs[i].publish(r);
        }
        printf("Sonars OK\n");
        printf("response size %ld\n",sizeof(resp));
        
        printf("Sigamt: %f  Sig: c=%f/w=%f  Thr:%f\n",sigAmount,
               sigCentre,sigWidth,thresh);
        // Now the light sensor.
        
        // Blur the data with a gaussian (exp 280916)
        uint8_t blurred[NUM_PIXELS*3];
        blur(blurred,resp.pixels,0,NUM_PIXELS);
        blur(blurred,resp.pixels,1,NUM_PIXELS);
        blur(blurred,resp.pixels,2,NUM_PIXELS);
        
        
        
        // We convert the currently monochrome data into colour
        // for publications
        
        lightsensor_gazebo::LightSensor ls;
        for(int i=0;i<NUM_PIXELS;i++){
            // these are 0-255
            lightsensor_gazebo::Pixel pix;
            pix.r = blurred[i*3+0];
            pix.g = blurred[i*3+1];
            pix.b = blurred[i*3+2];
            
            pix.r = processpixel(pix.r);
            pix.g = processpixel(pix.g);
            pix.b = processpixel(pix.b);
//            printf("%d: %d %d %d\n",i,pix.r,pix.g,pix.b);
            
            
            ls.pixels.push_back(pix);
        }
        printf("Light sensors OK\n");
        lightPub.publish(ls);
    }
};

BridgeClient *client;

void motorCallback(const std_msgs::Float64::ConstPtr& msg, int i){
    // turn the ROS topic data into data to send to the pioneer -
    // it will be sent at regular intervals to ensure the robot
    // connection is safe - if the server notices no packets arriving
    // it will stop the motors.
    
    // Note that the Pioneer's ARIA library works in mm/sec, but we
    // receive commands in m/sec (to make us compatible with how Gazebo
    // works)
    
    client->req.motors[i]=msg->data*1000.0;
}

bool photoCallback(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& resp){
    client->req.command = COMMAND_PHOTO;
    return true;
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"bridge_node");
    ros::NodeHandle node;
    
#if(ENABLEDIAMOND)
    diamondapparatus::init();
#endif
    // publishers
    
    for(int i=0;i<NUM_SONARS;i++){
        std::ostringstream name;
        name << "s" << i;
        sonarPubs[i] = node.advertise<sensor_msgs::Range>(name.str(),100);
    }
    
    lightPub = node.advertise<lightsensor_gazebo::LightSensor>("light",100);
    
    // subscribers
    
    ros::Subscriber lsub = node.subscribe<std_msgs::Float64>("leftmotor",1000,
                                  boost::bind(motorCallback,_1,0));
    ros::Subscriber rsub = node.subscribe<std_msgs::Float64>("rightmotor",1000,
                                   boost::bind(motorCallback,_1,1));
    
    // services
    ros::ServiceServer serv = node.advertiseService("photo",photoCallback);
    
    // get parameters
    
    std::string addr;
    if(!node.getParam("pioneeraddr",addr)){
        ROS_FATAL("no 'pioneeraddr' parameter set");
        return 1;
    }
    
    // subscribe to diamond knob control stuff
#if(ENABLEDIAMOND)
    diamondapparatus::subscribe("/glove/knobs");
#endif    
    try {
        // start the client and loop away!
        client = new BridgeClient(addr.c_str());
    } catch(TCPException e) {
        ROS_FATAL("TCP failure - %s",e.what());
        exit(1);
    }
    // clear the outgoing motor packet, though..
    client->req.motors[0]=0;
    client->req.motors[1]=0;
    client->req.command=0;
    
    ros::Rate rate(5);
    while(ros::ok()){
        printf("Send\n");
        ros::spinOnce();
        client->update(); // check sensor data
        client->request(); // send motor data
        client->req.command=0; // clear any command
        rate.sleep();
        
#if(ENABLEDIAMOND)
        diamondapparatus::Topic gloveTopic =
              diamondapparatus::get("/glove/knobs",GET_WAITNONE);
        if(gloveTopic.isValid()){
            sigCentre = gloveconv(gloveTopic[0].f());
            sigWidth = gloveconv(gloveTopic[1].f());
            thresh = gloveconv(gloveTopic[2].f());
            sigAmount = gloveconv(gloveTopic[3].f());
        }
#endif
    }
}
