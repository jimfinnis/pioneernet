/**
 * @file power.h
 * @brief  Power management model for the robot
 * 
 *
 */

#ifndef __POWER_H
#define __POWER_H

struct Power {
    double charge;
    double basePowerUsage;
    double motorPowerUsage;
    
    Power(double base,double motor){
        charge=1;
        basePowerUsage=base;
        motorPowerUsage=motor;
              
    }
    
    double update(double timestep,double powerin,
                  double leftmotor,double rightmotor){
        charge += (powerin-(basePowerUsage+
                            (leftmotor+rightmotor)*motorPowerUsage))*timestep;
        if(charge<0)charge=0;
        if(charge>1)charge=1;
    }
};


#endif /* __POWER_H */
