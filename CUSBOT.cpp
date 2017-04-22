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
//  wheelRadius = 0.0692*0.375;
  wheelRadius = 0.0633*0.5;
  vReq = 0;
  omegaReq = 0;
  headingReq = 0;
  oldTimeu = 0;
  batonFlag = false;
  
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
  initialHeading = 0;

  //variables for the distnace estimation function
  distanceCovered = 0;
  oldTimePos = 0;
  oldVelocity = 0;
}

CUSBOT::CUSBOT(int a):motorLeft(6, 2),motorRight(5, 7) //for motorHandler, the first arguement is the address of the slave responsible for measuring speed, and the second is the port of PWM output
{
  roverIndex = a;
  inA1 = 3;
  inA2 = 4;
  EA = 2;
  inB1 = 5;
  inB2 = 6;
  EB = 7;
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
  // variables initializations
  vLeftReq = 0;
  vRightReq = 0;
  RPMLeftReq = 0;
  RPMRightReq = 0;
  interWheelLength = 0.18;
//  wheelRadius = 0.0692*0.375;
  wheelRadius = 0.0633*0.5;
  vReq = 0;
  omegaReq = 0;
  headingReq = 0;
  oldTimeu = 0;
  batonFlag = false;
  
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
  initialHeading = 0;

  //variables for the distnace estimation function
  distanceCovered = 0;
  oldTimePos = 0;
  oldVelocity = 0;
}

void CUSBOT::keepIMUBusy()
{
  int new_yaw = mpu.get_yaw();
  Serial.println(new_yaw);
}

void CUSBOT::xbeeLikeOperation()
{
  char a = 0;
  static float s = 0;
  static float d = 0;
  #if OPTICALFLOW_ENABLED == 1
  float phi = ((float)mpu.get_yaw()- biasedHeading)*PI/180.0 - initialHeading;
  opticalFlow.update(phi);
  #endif
  if(Serial3.available())
  {
    a = Serial3.read();
    switch(a)
    {
      case 'f':
      s = 0.35;
      break;
      
      case 'w':
      s += 0.05;
      break;

      case 's':
      s -= 0.05;
      break;

      case 'd':
      d -= 10.0*PI/180.0;
      break;

      case 'a':
      d += 10.0*PI/180.0;
      break;

      default:
      s = 0;
      d = 0;
      break;
    }
//    Serial3.print(s);
//    Serial3.print("\t");
//    Serial3.println(d);
  }
  #if OPTICALFLOW_ENABLED == 1
  Serial3.print(opticalFlow.squal);
  Serial3.print("\t");
  Serial3.print(opticalFlow.X);
  Serial3.print("\t");
  Serial3.println(opticalFlow.Y);
  #endif
  controlBot(s,d,'h');
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

void CUSBOT::espMqttTest()
{
  if(esp.update())
  {
    analogWrite(13,(int)(esp.messageI[0]*10));
    analogWrite(12,(int)(esp.messageI[1]*10));
//    Serial.print(esp.messageI[0]*10);
//    Serial.print("\t");
    Serial.println(esp.messageI[2]*10);
  }
}
void CUSBOT::parameterEstimationTest()
{
  /*
   * Here we give the motors the order to run at 
   * full speed, then the master takes the speed
   * readings from the slaves and sends it to screen
   * with the electric current readings. This data 
   * is processed later to estimate motor parameters.
   */
  byte message[3] = {0,0,5};
  char a = 0;
  const int ttsize = 1;
  int timeTable[ttsize] = {5};
  int waitTime[ttsize] = {1};
  static float oldTimeParId = 0;
  static float timeBroadcast = 0;
  bool waitFlag = true;
  float input = 0;
  float t = (float)millis()/1000.0 - tStart;
  
  for(int i = 0; i < ttsize; i++)
  {
    while(waitFlag)
    {
      t = (float)millis()/1000.0 - tStart;
      if(t - oldTimeParId < waitTime[i] && waitFlag)
      {
        stopRover();
        float currentRight = (float)analogRead(A3)*5/1023.0/0.52;
        float currentLeft = (float)analogRead(A4)*5/1023.0/0.52;
        estimatePosition();
        input = 0;
        if(ACTIVATE_SLAVE_MOTOR_CONTROL)
        {
          motorLeft.updateRPM_filtered();
          motorRight.updateRPM_filtered();
        }
        if(t - timeBroadcast > 0.05)
        {
          Serial3.print(t);
          Serial3.print("\t");
          Serial3.print(xPos);
          Serial3.print("\t");
          Serial3.println(yPos);
          timeBroadcast = t;
        }
      }
      else
      {
        waitFlag = false;
        oldTimeParId = t;
      }
    }

    while(!waitFlag)
    {
      t = (float)millis()/1000.0 - tStart;
      if(t - oldTimeParId < timeTable[i] && !waitFlag)
      {
        Wire.beginTransmission(5);
        Wire.write(message,3);
        Wire.endTransmission();
  
        Wire.beginTransmission(6);
        Wire.write(message,3);
        Wire.endTransmission();
        
        float currentRight = (float)analogRead(A3)*5/1023.0/0.52;
        float currentLeft = (float)analogRead(A4)*5/1023.0/0.52;
        estimatePosition();
        input = (float)analogRead(A0)*5/1023.0 *2.97; //reading the battery voltage, as it is the input to the motors.
        if(ACTIVATE_SLAVE_MOTOR_CONTROL)
        {
          motorLeft.updateRPM_filtered();
          motorRight.updateRPM_filtered();
        }
        if(t - timeBroadcast > 0.05)
        {
          Serial3.print(t);
          Serial3.print("\t");
          Serial3.print(xPos);
          Serial3.print("\t");
          Serial3.println(yPos);
          timeBroadcast = t;
        }
      }
      else
      {
        waitFlag = true;
        oldTimeParId = t;
      }
    }
  }
  stopRover();  
}

void CUSBOT::parameterEstimationTest2()
{
  /*
   * Here we give the motors the order to run at 
   * full speed, then the master takes the speed
   * readings from the slaves and sends it to screen
   * with the electric current readings. This data 
   * is processed later to estimate motor parameters.
   */
  byte message[3] = {0,0,5};
  char a = 0;
  const int ttsize = 1;
  int timeTable[ttsize] = {5};
  int waitTime[ttsize] = {1};
  static float oldTimeParId = 0;
  static float timeBroadcast = 0;
  bool waitFlag = true;
  float input = 0;
  float t = (float)millis()/1000.0 - tStart;
  
  for(int i = 0; i < ttsize; i++)
  {
    while(waitFlag)
    {
      t = (float)millis()/1000.0 - tStart;
      if(t - oldTimeParId < waitTime[i] && waitFlag)
      {
        stopRover();
        float currentRight = (float)analogRead(A3)*5/1023.0/0.52;
        float currentLeft = (float)analogRead(A4)*5/1023.0/0.52;
        input = 0;
        if(ACTIVATE_SLAVE_MOTOR_CONTROL)
        {
          motorLeft.updateRPM_filtered();
          motorRight.updateRPM_filtered();
        }
        if(t - timeBroadcast > 0.05)
        {
          Serial3.print(t);
          Serial3.print("\t");
          Serial3.print(motorRight.FilteredRPM);
          Serial3.print("\t");
          Serial3.print(currentRight);
          Serial3.print("\t");
          Serial3.print(motorLeft.FilteredRPM);
          Serial3.print("\t");
          Serial3.print(currentLeft);
          Serial3.print("\t");
          Serial3.println(input); 
          timeBroadcast = t;
        }
      }
      else
      {
        waitFlag = false;
        oldTimeParId = t;
      }
    }

    while(!waitFlag)
    {
      t = (float)millis()/1000.0 - tStart;
      if(t - oldTimeParId < timeTable[i] && !waitFlag)
      {
        Wire.beginTransmission(5);
        Wire.write(message,3);
        Wire.endTransmission();
  
        Wire.beginTransmission(6);
        Wire.write(message,3);
        Wire.endTransmission();
        
        float currentRight = (float)analogRead(A3)*5/1023.0/0.52;
        float currentLeft = (float)analogRead(A4)*5/1023.0/0.52;
        input = (float)analogRead(A0)*5/1023.0 *2.97; //reading the battery voltage, as it is the input to the motors.
        if(ACTIVATE_SLAVE_MOTOR_CONTROL)
        {
          motorLeft.updateRPM_filtered();
          motorRight.updateRPM_filtered();
        }
        if(t - timeBroadcast > 0.05)
        {
          Serial3.print(t);
          Serial3.print("\t");
          Serial3.print(motorRight.FilteredRPM);
          Serial3.print("\t");
          Serial3.print(currentRight);
          Serial3.print("\t");
          Serial3.print(motorLeft.FilteredRPM);
          Serial3.print("\t");
          Serial3.print(currentLeft);
          Serial3.print("\t");
          Serial3.println(input); 
          timeBroadcast = t;
        }
      }
      else
      {
        waitFlag = true;
        oldTimeParId = t;
      }
    }
  }
  stopRover();  
}

void CUSBOT:: updateNeighboursPos()
{
  String m;
  String m2;
  int value = 0;
  int value2 = 0;
  float t = (float)millis()/1000.0 - tStart;
  if(t-oldTimeu > 0.5) //sending a new value for led intensity via mqtt every 0.5 seconds
  {
    value = (int)random(100,255);
    value2 = (int)random(0,90);
    if(batonFlag)
    {
      m2 = "(b:" + String(0)+")";
      Serial2.println(m2);
      batonFlag = !batonFlag;
    }
    else
    {
      m = "(p:"+String(value)+","+String(value2)+")";
      if(Serial2.available())
      {
        espMqttTest();
      }
      Serial2.println(m);
      batonFlag = !batonFlag;
    } 
    oldTimeu = t;
  }  
}

void CUSBOT:: updateNeighboursPos2()
{
  String m;
  String m2;
  int value = 0;
  int value2 = 0;
  float t = (float)millis()/1000.0 - tStart;
  
  if(t-oldTimeu > 0.5) //sending a new value for led intensity via mqtt every 0.5 seconds
  {
    espMqttTest();
    if(esp.messageI[2]*10 == 0)//robot with index = 1 has just received the baton
    {
      value = (int)random(100,255);
      value2 = (int)random(0,90);
      if(batonFlag)
      {
        value = 1;
        m2 = "(b:" + String(value)+","+String(value)+")";
        Serial2.print(m2);
        batonFlag = !batonFlag;
        esp.messageI[2] = -1;
      }
      else
      {
        m = "(p:"+String(value)+","+String(value2)+")";
        if(Serial2.available())
        {
          espMqttTest();
          Serial.println("we are checking stuff man");
        }
        Serial2.print(m);
        batonFlag = !batonFlag;
      }     
    }
    oldTimeu = t;
  }  
}

void CUSBOT:: updatePositions()
{
  String outS = String(16)+","+String(23);
  String in;
  char a;
  int index = 0;
  float t = (float)millis()/1000.0 - tStart;
  boolean convert = false;
  if(t-oldTimeu > 0.5)
  {
    Serial2.println(outS);
    delay(20);
    oldTimeu = t;
  }
//  Serial.println("#");
  while(Serial2.available())
  {
    a = Serial2.read();
//    Serial.println(a);
    if(a == ',')
    {
      position[index] = in.toInt();
      in="";
      index++;
      convert = true;
    }
    else
    {
      in += a;
    }    
  }
  if(convert)
  {
    position[index] = in.toInt();
    Serial.print(position[index-1]);
    Serial.print("\t");
    Serial.println(position[index]);
    Serial.println("\n");
    convert = false;
  }
}

void CUSBOT:: updatePositions2()
{
//  String outS = String(random(150,200))+","+String(random(100,150));
  String outS = String((int)(xPos*100))+","+String((int)(yPos*100)); //note that the ESP modules are currently accosumed to sending positions in cm (they are not prepared to handle dicimal points)
  String in;
  char a;
  int index = 0;
  float t = (float)millis()/1000.0 - tStart;
  boolean finish = false;
  boolean Listen = false;
  boolean start = false;
  boolean ledState = false;
  
  if(t-oldTimeu > 0.2)
  {
    Serial2.println(outS);
//    Serial.print(position[0]);
//    Serial.print("\t");
//    Serial.print(position[1]);
//    Serial.print("\t");
//    Serial.print(position[2]);
//    Serial.print("\t");
//    Serial.print(position[3]);
//    Serial.print("\t");
//    Serial.println(Serial2.available());
    Listen = true;
//    delay(5);
    oldTimeu = t;
  }
//  Serial.println("#");
  while(!finish && Listen)
  { 
    while(Serial2.available())
    {
      digitalWrite(13,LOW);
      a = Serial2.read();
      delay(2);
//      Serial.println(a);
      if(a == '$')
      {
        start = true;
      }
      if(start && a != '$') // we have to put the a != '$' condition because if we ommit it, the value of x will be equal to zero because its string will contain $
      {
//        digitalWrite(13,HIGH);
        if(a == ',')
        {
          digitalWrite(13,HIGH);
          position[index] = (float)in.toInt()/100.0;//note that the positions obtained are in cm. If used as is the required speed will be too high that will saturate the motors
//          Serial.println(in);
          in="";
          index++;
        }
        else if(a == '#')
        {
//          digitalWrite(13,HIGH);
//          Serial.println("HI");
          finish = true;
          Listen = false;
          start = false;
          while( Serial2.read() != -1 ); //flusing the serial buffer
          position[index] = (float)in.toInt()/100.0;  
          break;     
        }
        else
        {
          in += a;
        }
      }    
    } 
  }  
}

void CUSBOT:: updatePositions3()
{
//  String outS = String(random(150,200))+","+String(random(100,150));
  String outS = "a";
  String in;
  char a;
  int index = 0;
  float t = (float)millis()/1000.0 - tStart;
  boolean finish = false;
  boolean Listen = false;
  boolean start = false;
  boolean ledState = false;
  
  if(t-oldTimeu > 0.2)
  {
    Serial2.println(outS);
    Listen = true;
//    delay(5);
    oldTimeu = t;
  }
//  Serial.println("#");
  while(!finish && Listen)
  { 
    while(Serial2.available())
    {
      digitalWrite(13,LOW);
      a = Serial2.read();
      delay(2);
//      Serial3.println(a);
      if(a == '$')
      {
        start = true;
      }
      if(start && a != '$') // we have to put the a != '$' condition because if we ommit it, the value of x will be equal to zero because its string will contain $
      {
//        digitalWrite(13,HIGH);
        if(a == ',')
        {
          digitalWrite(13,HIGH);
          position[index] = (float)in.toInt()/100.0;//note that the positions obtained are in cm. If used as is the required speed will be too high that will saturate the motors
//          Serial.println(in);
          in="";
          index++;
        }
        else if(a == '#')
        {
//          digitalWrite(13,HIGH);
//          Serial.println("HI");
          position[index] = (float)in.toInt()/100.0;  
          xPos = position[0];
          yPos = position[1];
          in="";
          break;     
        }
        else if(a == '^')
        {
          finish = true;
          Listen = false;
          start = false;
          while( Serial2.read() != -1 ); //flusing the serial buffer
          startMotion = in.toInt();
        }
        else
        {
          in += a;
        }
      }    
    } 
  } 
//  for(int i = 0; i < 6; i++)
//  {
//    Serial.print(position[i]);
//    Serial.print("\t"); 
//  }
  Serial.println();
}

void CUSBOT:: manipulatePosition(float a,float b)
{
  xPosToTarget = xPos - a;
  yPosToTarget = yPos - b;
}

void CUSBOT::updatePositionsHTTP(char c)//the character c is used as a selector character to select among several functionalities; if c == 'f', the position values are modified so the difference between current and desired psoition is broadcasted, while if c=='r', then the normal position is sent to other agents.
{
  String outS;
  if (c == 'f') //formation specific
  {
    outS = String((int)(xPosToTarget*100))+","+String((int)(yPosToTarget*100)); //note that the ESP modules are currently accosumed to sending positions in cm (they are not prepared to handle dicimal points)
  }
  else //normal
  {
    outS = String((int)(xPos*100))+","+String((int)(yPos*100)); //note that the ESP modules are currently accosumed to sending positions in cm (they are not prepared to handle dicimal points)
  }
  String in;
  char a;
  int index = 0;
  float t = (float)millis()/1000.0 - tStart;
  float yaw = 0;
  boolean finish = false;
  boolean Listen = false;
  boolean start = false;
  boolean ledState = false;
  
  if(t-oldTimeu > 0.3)
  {
    Serial2.println(outS);
    Listen = true;
    oldTimeu = t;
  }

  while(!finish && Listen)
  {  
    yaw = mpu.get_yaw();
    while(Serial2.available())
    {
      digitalWrite(13,LOW);
      a = Serial2.read();
      delay(1);
      if(a == '$')
      {
        start = true;
      }
      if(start && a != '$') // we have to put the a != '$' condition because if we ommit it, the value of x will be equal to zero because its string will contain $
      {
        if(a == ',')
        {
          digitalWrite(13,HIGH);
          position[index] = (float)in.toInt()/100.0;//note that the positions obtained are in cm. If used as is the required speed will be too high that will saturate the motors
          in="";
          index++;
        }
        else if(a == '@')
        {
          position[index] = (float)in.toInt()/100.0;
          in="";
        }
        else if(a == '#')
        {
          finish = true;
          Listen = false;
          start = false;
          while( Serial2.read() != -1 ); //flusing the serial buffer
          startMotion = in.toInt();
          break;     
        }
        else
        {
          in += a;
        }
      }    
    } 
  }
//  for(int j = 0; j < 4; j++){Serial.print(position[j]);Serial.print("\t");}
//  Serial.println(startMotion);
}

void CUSBOT::getPositions(float* a)
{
  float yaw = mpu.get_yaw(); //we invoke this function to make sure the IMU is always invokes so as not to veer into chaos state.
  a[0] = xPos;
  a[1] = yPos;
  a[2] = position[0];
  a[3] = position[1];
  a[4] = position[2];
  a[5] = position[3];
  a[6] = startMotion;
}

void CUSBOT::getPositions2(float* a)
{
  float yaw = mpu.get_yaw(); //we invoke this function to make sure the IMU is always invokes so as not to veer into chaos state.
  a[0] = position[0];
  a[1] = position[1];
  a[2] = position[2];
  a[3] = position[3];
  a[4] = position[4];
  a[5] = position[5];
  a[6] = startMotion;
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
//    Serial3.println(new_yaw);
    if(new_yaw == old_yaw)
    {
      number_of_repeatings++; //here, we count how many times a certain reading is repaeated in the transient state of the sensor till it settles 
    }
    else
    {
      number_of_repeatings = 0;
    }
    old_yaw = new_yaw; //handing the value of new_yaw to old_yaw so we can see if we converge in the next loop or not
  
          
    if (number_of_repeatings >= 120 && !yaw_settled) //this means that the readings are settled enough
    {
      yaw_settled = true;
      biasedHeading = new_yaw;
      tStart = (float)millis()/1000.0;
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

#if OPTICALFLOW_ENABLED == 1 /*using optical flow sensor*/
void CUSBOT::optical_init()
{
  if(opticalFlow.mousecam_init()==-1)
  {
    Serial.println("Mouse cam failed to init");
    Serial2.println("Mouse cam failed to init");
    while(1);
  }  
}
#endif

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
//  Serial3.println(new_yaw);

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

/*the core of the heading control is to estimate the current euler angles of the rover wrt the 
 global axes. In case of using dead reconning, the sensors onboard can can estimate the change 
 from the initial state. However, they don't have enough information to estimate the heading wrt
 the global axes. For this reason, we supply the initial heading wrt global axes, and the onboard
 sensors do the task of estimating the change from these initial values, thus obtaining an estimate
 of the euler angles wrt global axes.*/
void CUSBOT::setInitialPosition(float x,float y,float theta)
{
  x_init = x;
  y_init = y;
  xPos = x / 100.0;
  yPos = y / 100.0;
  initialHeading = theta;
}

float CUSBOT::headingController2()
{
  float time = (float)millis()/1000.0;
  float controlAction = 0;
  float dt = time - velTimeOldh;
  float error = 0;
  float new_yaw = ((float)mpu.get_yaw()- biasedHeading)*PI/180.0 - initialHeading ;
//  Serial.print(new_yaw);
//  Serial.print("\t");
//  Serial2.print(biasedHeading);
//  Serial2.print("\t");
//  Serial3.println(new_yaw);
  float complementary = 0;
  float desiredLocal = 0;
  //******* calculating the required direction as seen by the robot
  float x = 1 * cos(-headingReq); //this negative is because the world axes are defined so that the z axis points upwards, so positive heading will be when rotating ccw, while the local axes of the imu are defined so that the z axis points downwards so the positive angle will be when rotating cw. So a negative should be put, synanomous to making a rotation of axes from the local (with z down) to global (with z down)
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

void CUSBOT::stopRover()
{
  /*
   * Now notice that the codes in the master and slave units are designed 
   * in way that it has a certain convention: when messsge[2] = 0; this means
   * normal operation, =1 means reverse rotation direction while =2 means 
   * stop completely.
   */
  byte message[3] = {0,0,2};
  
  Wire.beginTransmission(6);
  Wire.write(message,3);
  Wire.endTransmission();

  Wire.beginTransmission(5);
  Wire.write(message,3);
  Wire.endTransmission();
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
  if(linearVelocity == 0)
  {
    stopRover();
  }
  else if(MODE == 'o') //which stands for omega
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

void CUSBOT::goInCircle()
{
  float v = 1;
  float f = 0.15;
  float t = (float)millis()/1000.0 - tStart;
  
  float xd = v*cos(2*PI*f*t);
  float yd = v*sin(2*PI*f*t);
  float heading = atan2(yd,xd);
 
//  float dt = t - oldTime;
//  if(dt > 0 && dt < 1.5)heading=0;
//  else if(dt > 1.5 && dt < 3)heading = -PI/2;
//  else if(dt >3 && dt < 4.5)heading = -PI;
//  else if(dt > 4.5 && dt <6)heading = PI/2;
//  else
//  {
//    heading = 0;
//    oldTime = t;
//  }
  
  // translating the x and y velocity components into velocity and directions
  controlBot(v,heading,'h');
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
//  omegaReq = esp.messageI[0];
  headingReq = -esp.messageI[0];
  Serial.println(vReq);
  /*note: there used to be a code chunk right here that controls the motors
   * from the main arduino. It has been erased in newer versions as the 
   * control is being done from the slave units. To obtain this code chunk 
   * back please refer to older versions code on github.
   */
  control2();
}

float CUSBOT::estimateDistance()
{
  /*
   * This function basically integrates the rover's speed to estimate the distance 
   * covered with time.
   */
   float t = (float)millis()/1000.0-tStart;
   float dt = t - oldTimePos;
   if(ACTIVATE_SLAVE_MOTOR_CONTROL)
  {
    motorLeft.updateRPM_filtered();
    motorRight.updateRPM_filtered();
  }
  currentVelocity = (motorLeft.FilteredRPM + motorRight.FilteredRPM)*0.5 / 60.0 * 2 * PI * wheelRadius;
  if(currentVelocity < 1.5 && currentVelocity > 0.05)
  {
    currentVelocity *= 0.75;
    distanceCovered += (currentVelocity + oldVelocity)*0.5*dt;
    oldVelocity = currentVelocity;
//    Serial2.print(t);
//    Serial2.print("\t");
//    Serial2.print(currentVelocity);
//    Serial2.print("\t");
//    Serial2.println(distanceCovered);
  }
  oldTimePos = t;
}

void CUSBOT::estimatePosition()
{
   float t = (float)millis()/1000.0-tStart;
   float dt = t - oldTimePos;
   float phi = 0;
   float dDistance = 0;
   if(ACTIVATE_SLAVE_MOTOR_CONTROL)
  {
    motorLeft.updateRPM_filtered();
    motorRight.updateRPM_filtered();
  }
  currentVelocity = (motorLeft.FilteredRPM + motorRight.FilteredRPM)*0.5 / 60.0 * 2 * PI * wheelRadius;
  if(currentVelocity < 1.5 && currentVelocity > 0.05)
  {
    currentVelocity *= 1;
    phi = ((float)mpu.get_yaw()- biasedHeading)*PI/180.0 - initialHeading ;
    dDistance = (currentVelocity + oldVelocity)*0.5*dt;
    distanceCovered += dDistance;
    xPos += dDistance * cos(-phi);
    yPos += dDistance * sin(-phi);
    oldVelocity = currentVelocity;
//    Serial2.print(t);
//    Serial2.print("\t");
//    Serial2.print(xPos);
//    Serial2.print("\t");
//    Serial2.println(yPos);
  }
  oldTimePos = t;
}

void CUSBOT::estimatePosition2()
{
  static bool slaveSet = false;
  int i = 0;
  float px,py;
  float phi = ((float)mpu.get_yaw()- biasedHeading) - initialHeading *180.0 /PI;
  if(phi < 0) phi += 360;
//  Serial2.print(phi);
//  Serial2.print("\t");
  byte inMessage[4];
  byte outMessage[7];//contains the heading 
//  phi = 0;
  //sending heading angle to the optical flow
  outMessage[1] = (int)phi / 10;
  outMessage[0] = (int)(phi - outMessage[1] * 10);
  outMessage[3] = (int)x_init / 10;
  outMessage[2] = (int)(x_init - outMessage[3] * 10);
  outMessage[5] = (int)y_init / 10;
  outMessage[4] = (int)(y_init - outMessage[5] * 10);
  if(!slaveSet)
  {  
    outMessage[6] = 1;
    slaveSet = true;
  }
  else
  {
    outMessage[6] = 0;
  }
  Wire.beginTransmission(8);
  Wire.write(outMessage,7);
  Wire.endTransmission();
  
  //receiving the current position from optical flow
  Wire.requestFrom(8,6);
  while(Wire.available())
  {
    inMessage[i] = Wire.read();
//    Serial2.print(inMessage[i]);
//    Serial2.print("\t");
    i++;
  }
//  Serial2.println();
  px = inMessage[0] + inMessage[1] * 10;
  if(inMessage[2] == 1) px *= -1;
  py = inMessage[3] + inMessage[4] * 10;
  if(inMessage[5] == 1) py *= -1;
  px /= 100.0; //converting to m
  py /= 100.0;
  xPos = px;
  yPos = py;
//  Serial2.print(xPos);
//  Serial2.print("\t");
//  Serial2.println(yPos);
}

float CUSBOT::getStartingTime()
{
  return tStart;
}

