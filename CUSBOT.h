#ifndef CUSBOT_H
#define CUSBOT_H

#include "motorHandler.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "wifi.h"
#include "ADNS3080.h"

#define ACTIVATE_SLAVE_MOTOR_CONTROL 1
#define ACTIVATE_ANGLE_CONTROL 1
#define OPTICALFLOW_ENABLED 0

class CUSBOT{
  
  public:
  CUSBOT(int,int,int,int,int,int); //constructor
  void controlBot(float,float,char); //controls linear and angular speeds
  void IMU_init(); //or else, the code hangs. See implementation for more details
  void WIFI_init();
  #if OPTICALFLOW_ENABLED == 1
  void optical_init();
  #endif
  void setInitialPosition(float,float,float);
  void getPositions(float*);
  void getPositions2(float*);
  void espMqttTest(); //this function is for testing message received via esp. It updates the esp buffer and prints results
  void updateNeighboursPos();
  void updateNeighboursPos2();
  void updatePositions();
  void updatePositions2();
  void updatePositions3();
  void updatePositionsHTTP();
  void estimatePosition();
  void estimatePosition2();
  float estimateDistance();
  float getStartingTime();
  
  // functions for testing purposes
  void controlBot();
  void openLoop(float);
  void stopRover();
  void openLoopSlave(float);
  void goInCircle();
  void keepIMUBusy();
  void xbeeLikeOperation(); //this function makes the robot moves in different direction like it is controlled with conventional xbee or wireless module
  void parameterEstimationTest();
  void parameterEstimationTest2();
  
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
  #if OPTICALFLOW_ENABLED == 1
  ADNS3080 opticalFlow;
  #endif
  
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
  float headingReq;
  
  // some IMU related variables
  boolean IMU_ok_flag;
  int biasedHeading;
  float tStart;

  // some variables for wifi position update for other agents
  float oldTimeu;
  boolean batonFlag;
  float position[6];
  int startMotion;
  
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
  float initialHeading; // the angle between the local x and global x axes at the begining of motion (in radians). can be measured and given as input or can be supplied as input from motion capture system

  //variables for the distance estimation function
  float distanceCovered;
  float oldTimePos;
  float oldVelocity;
  float xPos;
  float yPos;
  int x_init;
  int y_init;
  
  //.....................................FUNCTIONS................................//
  void control1();
  void control2();
  float omegaController();
  float velocityController();
  float headingController();
  float headingController2();
  void sendDirectlyToMotors();
  void sendDirectlyToMotors(float,float);
  void sendToSlaveMotorController();
  void breakDownRPM(byte*,float);
  void IMU_settle();
};


#endif
