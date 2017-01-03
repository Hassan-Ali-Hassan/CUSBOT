#ifndef CUSBOT_H
#define CUSBOT_H

#include "motorHandler.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "wifi.h"

#define ACTIVATE_SLAVE_MOTOR_CONTROL 1
#define ACTIVATE_ANGLE_CONTROL 1
class CUSBOT{
  
  public:
  CUSBOT(int,int,int,int,int,int); //constructor
  void controlBot(float linearVelocity,float angularVelocity); //controls linear and angular speeds
  void IMU_init(); //or else, the code hangs. See implementation for more details
  void WIFI_init();

  // functions for testing purposes
  void controlBot();
  void openLoop(float);
  void openLoopSlave(float);
  
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

  // some IMU related variables
  boolean IMU_ok_flag;
  int biasedHeading;
  
  //variables for the velocity controller function
  float velocityErrorIntegral[2];
  float velocityErrorHistory[2];
  float velocityErrorDifferential;
  float velTimeOldv;
  float Kpv;
  float Kiv;
  float Kdv;
  
  //variables for the angular velocity controller function
  float omegaErrorIntegral[2];
  float omegaErrorHistory[2];
  float omegaErrorDifferential;
  float velTimeOldo;
  float Kpo;
  float Kio;
  float Kdo;

  //variables for the heading angle controller function
  int old_yaw;
  int current_heading;
  float headingErrorIntegral[2];
  float headingErrorHistory[2];
  float headingErrorDifferential;
  float velTimeOldh;
  float Kph; //h for heading
  float Kih;
  float Kdh;
  
  //.....................................FUNCTIONS................................//
  void control1();
  void control2();
  float omegaController();
  float velocityController();
  float headingController(float);
  void sendDirectlyToMotors();
  void sendDirectlyToMotors(float,float);
  void sendToSlaveMotorController();
  void breakDownRPM(byte*,float);
  void IMU_settle();
};


#endif
