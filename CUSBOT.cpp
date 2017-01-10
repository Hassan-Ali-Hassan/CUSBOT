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
  headingReq = 0;
  
  // variables initialization for velocity control function
  int i = 0;
  for(i = 0; i < 2; i++)
  {
    velocityErrorIntegral[i] = 0;
    velocityErrorHistory[i] = 0;    
  }
  velocityErrorDifferential = 0;
  velTimeOldv = 0;
  
  //variables initialization for angular velocity control function
  for(i = 0; i < 2; i++)
  {
    omegaErrorIntegral[i] = 0;
    omegaErrorHistory[i] = 0;
  }
  omegaErrorDifferential = 0;
  velTimeOldo = 0;

  //variables initialization for heading angle control function
  for(i = 0; i < 2; i++)
  {
    headingErrorIntegral[i] = 0;
    headingErrorHistory[i] = 0;
  }
  headingErrorDifferential = 0;
  velTimeOldh = 0;
}

void CUSBOT::IMU_init()
{
  //this function is important because without it the constructor tries to 
  //connect to the IMU without enabling the wire library so it hangs and keeps
  //waiting indefinitely. So we start the wire library and then invoke this 
  //function to enable the IMU.
  if(ACTIVATE_ANGLE_CONTROL)
  {
    mpu.start();
    IMU_settle();
  }
  else
  {
    mpu.initialize();
  }
}

void CUSBOT::IMU_settle()
{
  // mpu.start();
  int new_yaw = 0;
  int number_of_repeatings = 0;
  boolean yaw_settled = false;
  IMU_ok_flag = false;
  
  while(!yaw_settled && !IMU_ok_flag)
  {
    new_yaw = mpu.get_yaw();
    Serial.println(new_yaw);
    if(new_yaw == old_yaw)
    {
      number_of_repeatings++; //here, we count how many times a certain reading is repaeated in the transient state of the sensor till it settles 
    }
    else
    {
      number_of_repeatings = 0;
    }
    old_yaw = new_yaw; //handing the value of new_yaw to old_yaw so we can see if we converge in the next loop or not
  
          
    if (number_of_repeatings >= 60 && !yaw_settled) //this means that the readings are settled enough
    {
      yaw_settled = true;
      biasedHeading = new_yaw;
      digitalWrite(13,HIGH);
      Serial.println("OK");
    }
  }
  IMU_ok_flag = true;
//  return true;
}


void CUSBOT::WIFI_init()
{
  //initializing the wifi module
  esp.init();
}

float CUSBOT::velocityController()
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  float dt = time - velTimeOldv;
  float error = 0;
  
  //Setting controller gains
  Kpv = 0.06;
  Kiv = 2.6;
  Kdv = 0;
  
  //collecting sensor data
  if(ACTIVATE_SLAVE_MOTOR_CONTROL)
  {
    motorLeft.updateRPM_filtered();
    motorRight.updateRPM_filtered();
  }
  currentVelocity = (motorLeft.FilteredRPM + motorRight.FilteredRPM)*0.5 / 60.0 * 2 * PI * wheelRadius;

  //calculating error
  error = (vReq - currentVelocity);
  
  //integrating velocity
  if(dt > 0.02 && dt < 3)
  {
    velocityErrorHistory[1] = error;
    velocityErrorIntegral[1] = velocityErrorIntegral[0] + (velocityErrorHistory[1]+velocityErrorHistory[0])*0.5*(time-velTimeOldv);
    velocityErrorDifferential = (velocityErrorHistory[1] - velocityErrorHistory[0])/dt;
    velTimeOldv = time;
    velocityErrorHistory[0] = velocityErrorHistory[1];
    velocityErrorIntegral[0] = velocityErrorIntegral[1];
  }
  else
  {
    velTimeOldv = time;
  }
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(motorLeft.FilteredRPM);
  Serial.print("\t");
  Serial.println(motorRight.FilteredRPM);
  controlAction = error * Kpv + velocityErrorIntegral[1] * Kiv + velocityErrorDifferential*Kdv;
  return controlAction;
}

float CUSBOT::omegaController()
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  float dt = time - velTimeOldo;
  float error = 0;

  // ........Setting the controller gains.........//
//  Kpo = 0.056; //these are working values for proto 3.2
//  Kio = 2.3;
//  Kpo = 1.056; //these are working values for proto 3.1
//  Kio = 0;
  Kpo = 0.1; //these are working values for proto 3.2
  Kio = 5;
  Kdo = 0;
  
  // collecting feedback data from sensors
  currentYawRate = -mpu.getRotationZ()/131 * PI/180.0; //the yaw rate in radians per seconds
  // calculating errors
  error = (omegaReq - currentYawRate);
    
  //integrating omega
  //when the IMU initializes its DMP it takes time (about 15 seconds). Normally, the dt would 
  //be very large after initializing the IMU so the error integration value would be very large 
  //which will easily saturate the motors. 
  //This condition makes sure that dt is greater than the min sampling time, and also makes sure
  //it doesn't exceed a plausible limit, so the initial great dt after DMP initialization would be
  //detected. If dt is too big, velTimeOldo is set to be equal to current time without carrying out 
  //integration, so in the next loop dt will be plausible.
  if(dt > 0.02 && dt < 3)
  {
    omegaErrorHistory[1] = error;
    omegaErrorIntegral[1] = omegaErrorIntegral[0] + (omegaErrorHistory[1]+omegaErrorHistory[0])*0.5*(time-velTimeOldo);
    omegaErrorDifferential = (omegaErrorHistory[1] -omegaErrorHistory[0])/dt;
    velTimeOldo = time;
    Serial.println("hi man");
    omegaErrorHistory[0] = omegaErrorHistory[1];
    omegaErrorIntegral[0] = omegaErrorIntegral[1];
  }
  else
  {
    velTimeOldo = time;
  }
  controlAction = error * Kpo + omegaErrorIntegral[1] * Kio + Kdo * omegaErrorDifferential;
  return controlAction;
}

float CUSBOT::headingController()
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  float dt = time - velTimeOldh;
  float error = 0;
  int complementary_error = 0;
  int new_yaw = mpu.get_yaw();

  // making some operations on the obtained yaw reading to make sure it is useful and sound
  if(new_yaw < 0)
  {
    new_yaw += 360;
  }
  if(abs(new_yaw - old_yaw) < 20) //this is to make sure that when the new_yaw is rotated to be 360 for example, it flips to 1 and then we have 360 -1 added to current yaw for example
  {
    current_heading += (new_yaw - old_yaw);
    old_yaw = new_yaw; 
  }
  else
  {
    old_yaw = new_yaw; 
  }
  
  //this condition to make sure that the current heading is bounded between -180 and 180
  if(current_heading > 180)
  {
    current_heading -= 360;
  }
  else if(current_heading < -180)
  {
    current_heading += 360;
  }
  
  //this part is to make sure that the rover will rotate to the required heading from the shortest possible direction.
  error = (float)(headingReq - current_heading);
  if (error > 0)
  {
    complementary_error = error - 360; //calculating the complement of the error
  }
  else
  {
    complementary_error = error + 360;
  }
  
  if(abs(error) < abs(complementary_error))
  {
    error = error;
  }
  else
  {
    error = complementary_error;
  }

  // now that we have the error (and it is plausible and sound), we use it to generate the control action
  Kph = 5; //h for heading
  Kih = 0.1;
  Kdh = 0.5;
//  Kph = 10; //h for heading
//  Kih = 2;
//  Kdh = 0;
  error *= PI/180.0; //before this operation we had the error in deg, but now we need to be in rad, because the control action has to be per radians/sec, for units consistency when calculating left and right wheel velocities.
  if(dt > 0.02 && dt < 3)
  {
    headingErrorHistory[1] = error;
    headingErrorIntegral[1] = headingErrorIntegral[0] + (headingErrorHistory[1]+headingErrorHistory[0])*0.5*(time-velTimeOldh);
    headingErrorDifferential = (headingErrorHistory[1] -headingErrorHistory[0])/dt;
    velTimeOldh = time;
    headingErrorHistory[0] = headingErrorHistory[1];
    headingErrorIntegral[0] = headingErrorIntegral[1];
  }
  else
  {
    velTimeOldh = time;
  }
  controlAction = error * Kph + headingErrorIntegral[1] * Kih + Kdh * headingErrorDifferential;
  return controlAction;
}

float CUSBOT::headingController2()
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  float dt = time - velTimeOldh;
  float error = 0;
  float new_yaw = ((float)mpu.get_yaw()- biasedHeading)*PI/180.0 ;
  Serial2.print(biasedHeading);
  Serial2.print("\t");
  Serial2.println(new_yaw);
  float complementary = 0;
  float desiredLocal = 0;
  //******* calculating the required direction as seen by the robot
  float x = 1 * cos(-headingReq);
  float y = 1 * sin(-headingReq);
  float xn = cos(new_yaw) * x + sin(new_yaw) * y;
  float yn = cos(new_yaw) * y - sin(new_yaw) * x; 
  float phi = atan2(yn,xn);
  if(phi >= 0 && phi < PI)
  {
    desiredLocal = phi;
  }
  else if(phi > -PI && phi < 0)
  {
    desiredLocal = phi + 2 * (float)PI;
    complementary = 2 * (float)PI - desiredLocal;
    if(complementary < desiredLocal)
    {
      desiredLocal = -complementary;
    }
  }
  error = desiredLocal - 0; //because in the body axes the heading angle of the rover is always zero   
//  Serial2.print((float)millis()/1000.0,4);
//  Serial2.print("\t");
  // now that we have the error (and it is plausible and sound), we use it to generate the control action
  Kph = 5; //h for heading
  Kih = 0.5;
  Kdh = 0.1;
//  Kph = 10; //h for heading
//  Kih = 2;
//  Kdh = 0;
  if(dt > 0.02 && dt < 3)
  {
    headingErrorHistory[1] = error;
    headingErrorIntegral[1] = headingErrorIntegral[0] + (headingErrorHistory[1]+headingErrorHistory[0])*0.5*(time-velTimeOldh);
    headingErrorDifferential = (headingErrorHistory[1] -headingErrorHistory[0])/dt;
    velTimeOldh = time;
    headingErrorHistory[0] = headingErrorHistory[1];
    headingErrorIntegral[0] = headingErrorIntegral[1];
  }
  else
  {
    velTimeOldh = time;
  }
  controlAction = error * Kph + headingErrorIntegral[1] * Kih + Kdh * headingErrorDifferential;
  return controlAction;
}


void CUSBOT::control1() //this function invokes the neccessary functions to control linear and angular speeds
{
  float errorVelocity;
  float errorOmega;
  
  // Applying controllers on errors
  errorOmega = omegaController();
  errorVelocity = velocityController();
  
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
//  Serial.print(millis());
//  Serial.print("\t");
//  Serial.print(vReq);
//  Serial.print("\t");
//  Serial.println(omegaReq);
  
  if(ACTIVATE_SLAVE_MOTOR_CONTROL)
  {
    sendToSlaveMotorController();
  }
  else
  {
    sendDirectlyToMotors();
  } 
}

void CUSBOT::control2()
{
  float errorVelocity;
  float errorHeading;
  
  // Applying controllers on errors
  errorHeading = headingController2();
  
  // calculating required RPMs from processed errors
  vLeftReq = vReq + errorHeading * interWheelLength*0.5;
  vRightReq = vReq - errorHeading * interWheelLength*0.5;

  RPMLeftReq = vLeftReq * 30.0 / (PI*wheelRadius);
  RPMRightReq = vRightReq * 30.0 / (PI*wheelRadius);
  
  RPMLeftReq = constrain(RPMLeftReq,0,1000);
  RPMRightReq = constrain(RPMRightReq,0,1000);
//  Serial2.println(millis());
  
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

void CUSBOT::controlBot(float linearVelocity,float value, char MODE)
{
  vReq = linearVelocity;
  if(MODE == 'o') //which stands for omega
  {
    omegaReq = value;
    control1();
  }
  else if(MODE == 'h') //which stands for heading
  {
    headingReq = value;
    control2();
  }
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
//  Serial.print(millis());
//  Serial.print("\t");
//  Serial.print(motorLeft.FilteredRPM);
//  Serial.print("\t");
//  Serial.print(motorRight.FilteredRPM);
//  Serial.println("\t");
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
//  control1();
  control2();
}

