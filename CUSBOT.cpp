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
  
//  motorLeft.setPID(5,1,0);
//  motorRight.setPID(5,1,0);
  motorLeft.setPID(2,0.06,0.175);
  motorRight.setPID(2,0.06,0.175);
  motorLeft.setGainScheduling(false);
  motorRight.setGainScheduling(false);
  
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
  velocityErrorDifferential = 0;
  velTimeOldv = 0;
  Kpv = 0.06;
  Kiv = 2.6;
  Kdv = 0;
  //variables initialization for linear velocity control function
  for(i = 0; i < 2; i++)
  {
    omegaErrorIntegral[i] = 0;
    omegaErrorHistory[i] = 0;
  }
  velTimeOldo = 0;
//  Kpo = 0.056; //these are working values for proto 3.2
//  Kio = 2.3;
  Kpo = 1.056; //these are working values for proto 3.1
  Kio = 0;
//  Kpo = 0.1; //these are working values for proto 3.2
//  Kio = 5;
  Kdo = 0;
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
  float dt = time - velTimeOldv;
  //integrating velocity
  if( dt > 0.02)
  {
    velocityErrorHistory[1] = error;
    velocityErrorIntegral[1] = velocityErrorIntegral[0] + (velocityErrorHistory[1]+velocityErrorHistory[0])*0.5*(time-velTimeOldv);
    velTimeOldv = time;
    velocityErrorDifferential = (velocityErrorHistory[1] - velocityErrorHistory[0])/dt;
    velocityErrorHistory[0] = velocityErrorHistory[1];
    velocityErrorIntegral[0] = velocityErrorIntegral[1];
  }
  controlAction = error * Kpv + velocityErrorIntegral[1] * Kiv + velocityErrorDifferential*Kdv;
//  Serial.print(error);
//  Serial.print("\t");
//  Serial.println(controlAction);
//  controlAction = constrain(controlAction,0,1000);
  return controlAction;
}

float CUSBOT::omegaController(float error)
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  float dt = time - velTimeOldo;
  //integrating omega
  if(dt > 0.02)
  {
    omegaErrorHistory[1] = error;
    omegaErrorIntegral[1] = omegaErrorIntegral[0] + (omegaErrorHistory[1]+omegaErrorHistory[0])*0.5*(time-velTimeOldo);
    velTimeOldo = time;
    omegaErrorDifferential = (omegaErrorHistory[1] -omegaErrorHistory[0])/dt;
    omegaErrorHistory[0] = omegaErrorHistory[1];
    omegaErrorIntegral[0] = omegaErrorIntegral[1];
  }
  controlAction = error * Kpo + omegaErrorIntegral[1] * Kio + Kdo * omegaErrorDifferential;
  return controlAction;
}

void CUSBOT::control1()
{
  float errorVelocity;
  float errorOmega;
  if(ACTIVATE_SLAVE_MOTOR_CONTROL)
  {
    motorLeft.updateRPM_filtered();
    motorRight.updateRPM_filtered();
  }
  // collecting feedback data from sensors
  currentYawRate = -mpu.getRotationZ()/131 * PI/180.0; //the yaw rate in radians per seconds
//  if(abs(currentYawRate)<0.05)currentYawRate=0; //a condition to zero out some residues
  currentVelocity = (motorLeft.FilteredRPM + motorRight.FilteredRPM)*0.5 / 60.0 * 2 * PI * wheelRadius;
  
  // calculating errors
  errorOmega = (omegaReq - currentYawRate);
  errorVelocity = (vReq - currentVelocity);
  
  // Applying controllers on errors
  errorOmega = omegaController(errorOmega);
  errorVelocity = velocityController(errorVelocity);
  
  // calculating required RPMs from processed errors
  vLeftReq = errorVelocity + errorOmega * interWheelLength*0.5;
  vRightReq = errorVelocity - errorOmega * interWheelLength*0.5;

//  vLeftReq = vReq + omegaReq * interWheelLength * 0.5; //this is for open loop operation
//  vRightReq = vReq - omegaReq * interWheelLength * 0.5;

//  vLeftReq = vReq + errorOmega * interWheelLength * 0.5; //this is for open loop velocity and closed loop omega
//  vRightReq = vReq - errorOmega * interWheelLength * 0.5;

//  vLeftReq = errorVelocity + omegaReq * interWheelLength * 0.5; //applying closed loop velocity only
//  vRightReq = errorVelocity - omegaReq * interWheelLength * 0.5;

  RPMLeftReq = vLeftReq * 30.0 / (PI*wheelRadius);
  RPMRightReq = vRightReq * 30.0 / (PI*wheelRadius);
  
  RPMLeftReq = constrain(RPMLeftReq,0,1000);
  RPMRightReq = constrain(RPMRightReq,0,1000);
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(vReq);
  Serial.print("\t");
  Serial.println(omegaReq);
  
  if(ACTIVATE_SLAVE_MOTOR_CONTROL)
  {
    sendToSlaveMotorController();
  }
  else
  {
    sendDirectlyToMotors();
  } 
}

void CUSBOT::sendDirectlyToMotors()
{  
  motorLeft.controlRPM(RPMLeftReq);
  motorRight.controlRPM(RPMRightReq);
}

void CUSBOT::sendDirectlyToMotors(float a, float b)
{
  motorLeft.controlRPM(a);
  motorRight.controlRPM(b);
}

void CUSBOT::sendToSlaveMotorController()
{
  byte message[3] = {0,0,0};
  breakDownRPM(message,RPMLeftReq);
//  Serial.print("breaking message\t");
//  Serial.print(RPMLeftReq);
//  Serial.print("\t");
//  Serial.print(message[0]);
//  Serial.print("\t");
//  Serial.print(message[1]);
//  Serial.println("\t");
  Wire.beginTransmission(6);
  Wire.write(message,3);
  Wire.endTransmission();

  breakDownRPM(message,RPMRightReq);
  Wire.beginTransmission(5);
  Wire.write(message,3);
  Wire.endTransmission();
}

void CUSBOT::breakDownRPM(byte* message,float value)
{
  if(value < 0)
  {
    message[2] = 1;
  }
  else
  {
    message[2] = 0;
  }
  value = abs(value);
  message[1] = (int)value / 10;
  message[0] = (int)(value - message[1] * 10);
}

void CUSBOT::controlBot(float linearVelocity,float angularVelocity)
{
  vReq = linearVelocity;
  omegaReq = angularVelocity;
  control1();
}

void CUSBOT::openLoop(float rpm)
{
  sendDirectlyToMotors(rpm,rpm);
}

void CUSBOT::openLoopSlave(float rpm)
{
  RPMLeftReq = rpm;
  RPMRightReq = rpm;
  sendToSlaveMotorController();
  if(ACTIVATE_SLAVE_MOTOR_CONTROL)
  {
    motorLeft.updateRPM_filtered();
    motorRight.updateRPM_filtered();
  }
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(motorLeft.FilteredRPM);
  Serial.print("\t");
  Serial.print(motorRight.FilteredRPM);
  Serial.println("\t");
}

void CUSBOT::controlBot() 
{
  /*this function receives intructions sent through mqtt to move in 
  diferent directions and change its speed. Now if vReq = 0, this means
  that the robot is required to stop. In this case, there is no need to 
  call the control1 function, and instead all the integration values for 
  controllers are set to zero.*/
  esp.update();
  vReq = esp.messageI[1];
  omegaReq = esp.messageI[0];
//  Serial.print(millis());
//  Serial.print("\t");
//  Serial.print(vReq);
//  Serial.print("\t");
//  Serial.println(omegaReq);
//  if(vReq != 0)
//  {   
//    control1();
//  }
//  else
//  {
//    for(int i = 0; i < 2; i++)
//    {
//      velocityErrorIntegral[i] = 0;
//      velocityErrorHistory[i] = 0;    
//    }
//    for(int i = 0; i < 2; i++)
//    {
//      omegaErrorIntegral[i] = 0;
//      omegaErrorHistory[i] = 0;
//    }
//    motorRight.stop();
//    motorLeft.stop();
//  } 
  control1();
}

