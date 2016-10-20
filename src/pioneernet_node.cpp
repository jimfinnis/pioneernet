/**
 * @file pioneernet_node.cpp
 * @brief  Brief description of file.
 *
 */

#include <getopt.h>
#include <angort/params.h>

#include "ros/ros.h"
#include "backpropNoBiasHormone.h"
#include "decimate.h"

#include "lightsensor_gazebo/LightSensor.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"

#include "power.h"
#include "diamondapparatus/diamondapparatus.h"

#define NUM_SONARS 8

// was 2.18
#define SIMSLOWFACTOR 2.033

#define FARAWAY 5

// Knuth, TAOCP vol2 p.122
double rand_gauss (double mean,double sigma) {
    double v1,v2,s;
    
    do {
        v1 = drand48()*2-1;
        v2 = drand48()*2-1;
        s = v1*v1 + v2*v2;
    } while ( s >= 1.0 );
    
    if (s == 0.0)
        s=0.0;
    else
        s=(v1*sqrt(-2.0 * log(s) / s));
    return (s*sigma)+mean;
}

// http://azzalini.stat.unipd.it/SN/
double skewnormal(double location,double scale, double shape){
    double a = rand_gauss(0,1);
    double b = rand_gauss(0,1);
    if(b>shape*a)
        a*=-1;
    return location+scale*a;
}

struct option opts[]={
    {"net", required_argument, NULL, 'n'},
    {"time", required_argument, NULL, 't'},
    {"vars", required_argument, NULL, 'V'},
    {"constfile", required_argument, NULL, 'c'},
    {"hormone", required_argument, NULL, 'h'},
    {"log",required_argument,NULL,'l'},
    {"slow",required_argument,NULL,'s'},
    {"sim",no_argument,NULL,'S'},
    {"zero",no_argument,NULL,'0'},
    {NULL,0,NULL,0},
};

double slowFactor=-1;

double kM,kPower,kBase;
Value *mapFunc; // function to map charge to hormone or hormone input
Value *resetFunc; // function to reset values between runs
Parameters *params;
bool simMode=false;

void getParams(const char *cf,char *varstr){
    params = new Parameters(cf,varstr);
    const char *err = params->check();
    if(err){
        fprintf(stderr,"Parameter fetch failed:%s\n",err);
        exit(1);
    }
    
    kBase = params->getFloat("kbase");
    kM = params->getFloat("km");
    kPower = params->getFloat("kpower");
    
    if(params->exists("hmapfunc"))
        mapFunc = params->getFunc("hmapfunc");
    else
        mapFunc = NULL;
    
    if(params->exists("resetfunc"))
        resetFunc = params->getFunc("resetfunc");
    else
        resetFunc = NULL;
    
    
}

int recvdflags=0;


double sonarDists[NUM_SONARS];
void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,int i){
    sonarDists[i] = msg->range;
    if(sonarDists[i]>FARAWAY)sonarDists[i]=FARAWAY;
    recvdflags|=1; // we have got sonar
}



uint8_t pix[1000][3];
double monoPix[1000];
int numPixels =0;
double totalLight=0;
void lightCallback(const lightsensor_gazebo::LightSensor::ConstPtr& msg){
    numPixels = msg->pixels.size();
    totalLight=0;
    for(int i=0;i<numPixels;i++){
        
        pix[i][0] = msg->pixels[i].r;
        pix[i][1] = msg->pixels[i].g;
        pix[i][2] = msg->pixels[i].b;
        
        double p = pix[i][0]+pix[i][1]+pix[i][2];
        p *= 1.0/(256.0*3.0);
        monoPix[i]=p;
        totalLight += p;
    }
    
                  
              
    
    recvdflags|=2; // we have got light
}


int main(int argc,char *argv[]){
    ros::init(argc,argv,"pioneernet_node");
    ros::NodeHandle n;
    ros::Subscriber s[NUM_SONARS];
    ros::Subscriber light;
    
    diamondapparatus::init();
    diamondapparatus::subscribe("/tracker/points");
    diamondapparatus::subscribe("/bright");
    
    float maxtime = 1000000;
    bool manual=false;
    double inithormone=0;
    char *varString = NULL;
    FILE *log=NULL;
    bool zeroOutputs=false;
    
    char constFile[1024];
    const char *cfe = getenv("WHEELYCONSTFILE");
    strcpy(constFile,cfe?cfe:"constants.ang");
    
    for(int i=0;i<NUM_SONARS;i++){
        char buf[32];
        sprintf(buf,"s%d",i);
        s[i] = n.subscribe<sensor_msgs::Range>(buf,1000,
                                 boost::bind(sonarCallback,_1,i));
        sonarDists[i]=FARAWAY;
    }
    
    light = n.subscribe<lightsensor_gazebo::LightSensor>
          ("light",1000,lightCallback);
    
    BackpropNet *net = NULL;
    int optidx=0,c;
    double setSlow=-1;
    while(c=getopt_long(argc,argv,"0Ss:l:n:t:V:c:h:",opts,&optidx)){
        if(c<0)break;
        switch(c){
            // sim mode triggers weird sonar noise
        case 'S':
            printf("SIM MODE\n");
            simMode = true;
            break;
        case 's':
            setSlow=atof(optarg);
            break;
        case 'l':
            log=fopen(optarg,"w");
            break;
        case 't':
            maxtime = atof(optarg);
            break;
        case 'n':
            printf("Loading network..\n");
            net = BackpropNet::loadNet(optarg);
            printf("Loaded..\n");
            break;
        case '0':
            printf("ZERO MODE\n");
            zeroOutputs=true;
            break;
        case 'V':
            varString = strdup(optarg);
            break;
        case 'c':
            strcpy(constFile,optarg);
            break;
        case 'h':
            manual=true;
            inithormone=atof(optarg);
            break;
        case '?':
            break;
        default:abort();
        }
    }
    getParams(constFile,varString);
    // handle default slow factors outside the loop so -s / -S order doesn't matter
    if(setSlow<0){
        if(simMode)
            slowFactor=SIMSLOWFACTOR;
        else
            slowFactor=20;
    }else
          slowFactor=setSlow;
        

    
    if(!net){
        fprintf(stderr,"Error: no network specified\n");
        abort();
    }
            
    
    ros::Publisher leftmotor = n.advertise<std_msgs::Float64>("leftmotor",1000);
    ros::Publisher rightmotor = n.advertise<std_msgs::Float64>("rightmotor",1000);
    
    printf("Net loaded (%d-%d-%d), running...\n",
           net->getLayerSize(0),
           net->getLayerSize(1),
           net->getLayerSize(2)
           );
    ros::Rate rate(10);
    
    double hormone = inithormone;
    
    int numins = net->getLayerSize(0);
    double *inp =new double [numins];
    for(int i=0;i<numins;i++)inp[i]=0;
    
    Power power(kBase,kM);
    
    if(resetFunc)
        params->runFunc(resetFunc);
    
    if(log){
        fprintf(log,"t,x,y,");
        for(int i=0;i<NUM_SONARS;i++)
            fprintf(log,"s%d,",i);
        //logging assumption - 8 light pixels
        for(int i=0;i<8;i++)
            fprintf(log,"l%d,",i);
        fprintf(log,"hormone,charge,powerin,l,r\n");
    }
    
    // spin for a bit to allow sim and ros time to sync
    for(int i=0;i<20;i++){
        ros::spinOnce();
        rate.sleep();
    }    
    
    ros::Time lastTick = ros::Time::now();
    
    ros::Time startTick = ros::Time::now();
    
    ros::Time lastData = ros::Time::now();
    
    while(ros::ok()){
        GaussianDecimator *decimator=NULL;
        
        ros::spinOnce();
        rate.sleep();
        
        // skip if we haven't got data on sensors yet
        if(recvdflags != 3)continue;
        
        // update the network, get the outputs and feed them
        // to the robot
        net->setH(hormone);
        
        double tnow = (lastTick-startTick).toSec();
        
        if(!simMode || (lastTick-lastData).toSec()>0.33){
            lastData = lastTick;
            int inpidx=0;
        
            for(int i=0;i<NUM_SONARS;i++){
                double d = sonarDists[i];
                if(simMode){
                    // skew-normal noise, location,scale,shape
//                    d+=skewnormal(0,0.2,3);
                }
                
                inp[inpidx++]=d;
            }
        
            // work out how many light inputs, and downsample to this,
            // creating the gaussian when we do this. If there is no
            // light input yet, set to zero.
            int numLightIns = numins - NUM_SONARS;
            if(numPixels){
                if(!decimator){
                    // kernelsize,sigma,pixels in
                    // was 41,4
                    decimator = new GaussianDecimator(5,1.0,numPixels);
                }
                if(numPixels!=decimator->getBufSize()){
                    ROS_FATAL("Number of input pixels has changed");
                    exit(1);
                }
            
                // the bloody gazebo sensor I wrote has w/2 as "front",
                // while in my network code zero is to the front. Dolt.
                double tmp[256]; // big enough..
                decimator->decimate(tmp,numLightIns,monoPix);
                for(int i=0;i<numLightIns;i++){
                    inp[inpidx++] = tmp[(i+numLightIns/2)%numLightIns];
                }
                //            printf("\nLIGHTS   ");//snark
                //            for(int i=0;i<numLightIns;i++)
                //                printf("%f ",inp[NUM_SONARS+i]);
                //            printf("\n");
            } else {
                for(int i=0;i<numLightIns;i++)
                    inp[inpidx++]=0;
            }
        }
        
        net->setInputs(inp);
        net->update();
        double *outs = net->getOutputs();
        
        double left = outs[0];
        if(left<-1)left=-1;
        if(left>1)left=1;
        double right = outs[1];
        if(right<-1)right=-1;
        if(right>1)right=1;
        
        // if both motors similar, make them the same in the sim.
        // This emulates the robot behaviour.
//        if(simMode && fabs(left-right)<0.2){
//            left = right = (left+right)*0.5;
//        }
        
        
        std_msgs::Float64 m;
        m.data = left/slowFactor;
        leftmotor.publish(m);
        
        m.data = right/slowFactor;
        rightmotor.publish(m);
        
        // update the power management
        
        double time = (ros::Time::now() - lastTick).toSec();
        lastTick = ros::Time::now();
        
        
        if(simMode){
            diamondapparatus::Topic t;
            t = diamondapparatus::get("/bright",GET_WAITANY);
            printf("Light fetch %d\n",t.state);
            if(t.isValid()){
                totalLight = t[0].f();
                printf("LIGHT%f\n",totalLight);
            }
        }
        
        double powerin = totalLight*kPower;
        
        
        power.update(time,powerin,outs[0],outs[1]);
        
        printf("time %f; step %f; motors: %f %f; charge: %f; hormone: %f\n",
               tnow,time,
               outs[0],outs[1],
               power.charge,hormone);
        
        // update the hormone
        if(!manual){
            if(mapFunc){
                params->pushFuncArg((float)power.charge);
                params->runFunc(mapFunc);
                hormone = params->popFuncResult();
            }
            else
                hormone = 1.0-power.charge;
        }
        else
            power.charge=1; // weird hormone, fix charge
        
        if(log){
            diamondapparatus::Topic tpos = 
                  diamondapparatus::get("/tracker/points",GET_WAITNONE);
            if(!tpos.isValid()){
                ROS_FATAL("No position data\n");
                break;
            }
            double px = tpos[0].f();
            double py = tpos[1].f();
            
            fprintf(log,"%f,%f,%f,",tnow,px,py);
            for(int i=0;i<16;i++) // assumption of 16 ins
                fprintf(log,"%f,",inp[i]);
            fprintf(log,"%f,%f,%f,%f,%f\n",hormone,power.charge,
                    powerin,outs[0],outs[1]);
        }
        
        diamondapparatus::Topic v;
        v.add(diamondapparatus::Datum(power.charge));
        v.add(diamondapparatus::Datum(hormone));
        v.add(diamondapparatus::Datum(totalLight));
        v.add(diamondapparatus::Datum(powerin));
        diamondapparatus::publish("/charge",v);
        
        // exit on zero power
        if(power.charge<0.000001)
            break;
    }
    std_msgs::Float64 m;
    m.data=0;
    leftmotor.publish(m);
    rightmotor.publish(m);
    ros::spinOnce();
    
    if(log)fclose(log);
    return 0;
}
