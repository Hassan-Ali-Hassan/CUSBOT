#ifndef CUSBOT_H
#define CUSBOT_H

#include "motorHandler.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "wifi.h"

class CUSBOT{
  
  public:
  CUSBOT(int,int,int,int,int,int); //constructor
  void controlBot(float linearVelocity,float angularVelocity); //controls linear and angular speeds
  void controlBot();
  void IMU_init(); //or else, the code hangs. See implementation for more details
  void WIFI_init();
  
  protected:
  // ....................................VARIABLES...........................//
  // motor control variables
  int inA1; //A for left, B for right
  int inA2;
  int inB1;
  int inB2;
  int EA;
  int EB;
  
  // the objects on the rover
  motorHandler motorLeft;
  motorHandler motorRight;
  MPU6050 mpu;
  wifi esp;
  
  //variables
  float vLeftReq;
  float vRightReq;
  float RPMLeftReq;
  float RPMRightReq;
  float interWheelLength; //distance between two wheels in meters
  float wheelRadius; //Wheel radius in meters
  float currentYawRate;
  float currentVelocity;
  float vReq;
  float omegaReq;
  
  //variables for the velocity controller function
  float velocityErrorIntegral[2];
  float velocityErrorHistory[2];
  float velTimeOldv;
  float Kpv;
  float Kiv;
  
  //variables for the angular velocity controller function
  float omegaErrorIntegral[2];
  float omegaErrorHistory[2];
  float velTimeOldo;
  float Kpo;
  float Kio;
  
  //.....................................FUNCTIONS................................//
  void control1();
  float omegaController(float error);
  float velocityController(float error);
};


#endif
