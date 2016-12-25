#include "CUSBOT.h"

CUSBOT::CUSBOT(int _inA1, int _inA2, int _EA, int _inB1, int _inB2, int _EB):motorLeft(6, _EA),motorRight(5, _EB) //for motorHandler, the first arguement is the address of the slave responsible for measuring speed, and the second is the port of PWM output
{
  // motor ports settings
  inA1 = _inA1;
  inA2 = _inA2;
  EA = _EA;
  inB1 = _inB1;
  inB2 = _inB2;
  EB = _EB;
  pinMode(inA1,OUTPUT);
  pinMode(inA2,OUTPUT);
  pinMode(EA,OUTPUT);
  pinMode(inB1,OUTPUT);
  pinMode(inB2,OUTPUT);
  pinMode(EB,OUTPUT);
  
  digitalWrite(inA1,HIGH);
  digitalWrite(inA2,LOW);
  digitalWrite(inB1,HIGH);
  digitalWrite(inB2,LOW);
  
  motorLeft.setPID(5,1,0);
  motorRight.setPID(5,1,0);
  motorLeft.setGainScheduling(true);
  motorRight.setGainScheduling(true);
  
  // variables initializations
  vLeftReq = 0;
  vRightReq = 0;
  RPMLeftReq = 0;
  RPMRightReq = 0;
  interWheelLength = 0.18;
  wheelRadius = 0.0692*0.5;
  vReq = 0;
  omegaReq = 0;
  
  // variables initialization for velocity control function
  int i = 0;
  for(i = 0; i < 2; i++)
  {
    velocityErrorIntegral[i] = 0;
    velocityErrorHistory[i] = 0;    
  }
  velTimeOldv = 0;
  Kpv = 2.5;
  Kiv = 6;
  
  //variables initialization for linear velocity control function
  for(i = 0; i < 2; i++)
  {
    omegaErrorIntegral[i] = 0;
    omegaErrorHistory[i] = 0;
  }
  velTimeOldo = 0;
  Kpo = 2.5;
  Kio = 6;
}

void CUSBOT::IMU_init()
{
  //this function is important because without it the constructor tries to 
  //connect to the IMU without enabling the wire library so it hangs and keeps
  //waiting indefinitely. So we start the wire library and then invoke this 
  //function to enable the IMU.
  mpu.initialize();
}

void CUSBOT::WIFI_init()
{
  //initializing the wifi module
  esp.init();
}

float CUSBOT::velocityController(float error)
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  
  //integrating velocity
  if(time - velTimeOldv > 0.02)
  {
    velocityErrorHistory[1] = error;
    velocityErrorIntegral[1] = velocityErrorIntegral[0] + (velocityErrorHistory[1]+velocityErrorHistory[0])*0.5*(time-velTimeOldv);
    velTimeOldv = time;
    velocityErrorHistory[0] = velocityErrorHistory[1];
    velocityErrorIntegral[0] = velocityErrorIntegral[1];
  }
  controlAction = error * Kpv + velocityErrorIntegral[1] * Kiv;
  return controlAction;
}

float CUSBOT::omegaController(float error)
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  
  //integrating omega
  if(time - velTimeOldo > 0.02)
  {
    omegaErrorHistory[1] = error;
    omegaErrorIntegral[1] = omegaErrorIntegral[0] + (omegaErrorHistory[1]+omegaErrorHistory[0])*0.5*(time-velTimeOldo);
    velTimeOldo = time;
    omegaErrorHistory[0] = omegaErrorHistory[1];
    omegaErrorIntegral[0] = omegaErrorIntegral[1];
  }
  controlAction = error * Kpo + omegaErrorIntegral[1] * Kio;
  return controlAction;
}

void CUSBOT::control1()
{
  float errorVelocity;
  float errorOmega;
  
  // collecting feedback data from sensors
  currentYawRate = -mpu.getRotationZ()/131 * PI/180.0; //the yaw rate in radians per seconds
  currentVelocity = (motorLeft.FilteredRPM + motorRight.FilteredRPM)*0.5 / 60.0 * 2 * PI * wheelRadius;
  
  // calculating errors
  errorOmega = (omegaReq - currentYawRate);
//  errorVelocity = vReq; //this option is for open loop operation of the velocity
  errorVelocity = (vReq - currentVelocity);
  
  // Applying controllers on errors
  errorOmega = omegaController(errorOmega);
  errorVelocity = velocityController(errorVelocity);
  
  // calculating required RPMs from processed errors
  vLeftReq = errorVelocity + errorOmega * interWheelLength * 0.5;
  vRightReq = errorVelocity - errorOmega * interWheelLength * 0.5;
//  vLeftReq = vReq + omegaReq * interWheelLength * 0.5; //this is for open loop operation
//  vRightReq = vReq - omegaReq * interWheelLength * 0.5;

  RPMLeftReq = vLeftReq * 30.0 / (PI*wheelRadius);
  RPMRightReq = vRightReq * 30.0 / (PI*wheelRadius);
  
  motorLeft.controlRPM(RPMLeftReq);
  motorRight.controlRPM(RPMRightReq);
}

void CUSBOT::controlBot(float linearVelocity,float angularVelocity)
{
  vReq = linearVelocity;
  omegaReq = angularVelocity;
  control1();
}

void CUSBOT::controlBot()
{
  esp.update();
  vReq = esp.messageI[1];
  omegaReq = esp.messageI[0];
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(vReq);
  Serial.print("\t");
  Serial.println(omegaReq);
//  control1();
}

