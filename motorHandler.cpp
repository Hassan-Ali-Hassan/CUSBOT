#include "motorHandler.h"

motorHandler :: motorHandler(int encAdd, int port)
{
  int i = 0;
  encoderAddress = encAdd;
  motorPort = port;
  RPM = 0;
  FilteredRPM = 0;
  motorON = false;
  motorOFF = true;
  
  // variables for mit function
  for (i = 0; i < 2; i++)
  {
    ym[i]=0;
    theta[i] = 0;
    E[i] = 0;
  }
  tOld_ym = 0;
  tOld = 0;
  
  // variables for controlRPM function
  oldTime = 0;
  
  // variables for PID function
  for(i=0; i<2; i++)
  {
    errorIntegral[i]= 0;
  }
  errorOld = 0;
  oldTimePID = 0;
  Kp = 5;
  Ki = 1;
  Kd = 0;
  gainSchedulingOn = false;
  
  //variables for IIR filter
  for(i = 0; i < 3; i++)
  {
    oldFiltered[i] = 0;
    rpmOldIIR[i] = 0;
  }
  
  //variables for rejectOutlier function
  counter = 0;
  auxCounter = 0;
  allow = false;
  sorted = false;
  
  //variables for checkStop function
  rpm_old = 0;
  lastRPM = 0;
  repetitionCounter = 0;
}

void motorHandler::stop()
{
  analogWrite(motorPort,0);
}

void motorHandler::setPID(float kp, float ki, float kd)
{
  KpMin = kp;
  KiMin = ki;
  Kd = kd;
  
  KpMax = KpMin * 1.5;
  KiMax = KiMin * 1.5;
  
  Kp = KpMin;
  Ki = KiMin;
}

void motorHandler::setGainScheduling(boolean a)
{
  gainSchedulingOn = a;
}

void motorHandler::checkStop(float rpm)
{
  if(rpm == rpm_old)
  {
    repetitionCounter++;
    if(repetitionCounter > 10) //this means the motor has stopped
    {
      motorON = false;
      motorOFF = true;
      RPM = 0;
      lastRPM = rpm;
      repetitionCounter = 0;
      RPM = 0;
//      Serial.println("the motor has stopped");
    }  
  }
  else
  {
    repetitionCounter--;
  }
  if(repetitionCounter > 15){repetitionCounter = 15;}
  else if(repetitionCounter < 0){repetitionCounter = 0;}
  rpm_old = rpm;
  
  if(motorOFF)
  {
    if(rpm != lastRPM) //motion happens
    {
      motorON = true;
      motorOFF = false;
//      Serial.println("the motor has started again!");
    }
  }
}

void motorHandler::updateRPM()
{
  int i = 0;
  byte message[2];
  
  Wire.requestFrom(encoderAddress,2);
  while(Wire.available())
  {
    message[i] = Wire.read();
    delay(3);
    i++;
  }
//    Serial.print(message[0]);
//    Serial.print("\t");
//    Serial.print(message[1]);
//    Serial.print("\t");
  
  RPM = (float)(message[0] + message[1] * 10);
}

void motorHandler::updateRPM_filtered()
{
  int i = 0;
  byte message[2];
  
  Wire.requestFrom(encoderAddress,2);
  while(Wire.available())
  {
    message[i] = Wire.read();
    i++;
  }  
  RPM = (float)(message[0] + message[1] * 10);
//  FilteredRPM = rejectOutlier(RPM);
  FilteredRPM = RPM;
}


float motorHandler::mean(float* a,int n)
{
  int i = 0;
  float sum = 0;
  for(i = 0; i < n; i++)
  {
    sum += a[i];
  }
  sum /= (float)n;
  return sum;
}

void motorHandler::swap(float&a, float&b)
{
  float temp = a;
  a = b;
  b = temp;
}

void motorHandler::bubble_sort(float* a)
{
  int j = 0;
  int i = 0;
  for(j = 0; j < SIZE; j++)
  {
    for(i = 0; i < SIZE-j-1; i++)
    {
      if(a[i]>a[i+1])
      {
        swap(a[i],a[i+1]);
      }
    }
  }
}

float motorHandler::rejectOutlier(float rpm)
{
  float sortedReadings[SIZE];
  float Q1 = 0;
  float Q2 = 0;
  float upperBound = 0;
  float lowerBound = 0;
  int i = 0;
  float k = 1.5;
  checkStop(rpm);// always invoke this function or otherwise the MIT function doesn't work properly and the control action saturates
  if(motorON)
  {
    if(counter < SIZE)
    {
      readings[counter] = rpm;
      counter++;
      return rpm;
    }
    else
    {
      // adding the new RPM to the queue of numbers we have
      for(i = 0; i < SIZE - 1; i++)
      {
        readings[i] = readings[i+1];
        sortedReadings[i] = readings[i];
      }
      readings[SIZE-1] = rpm;
      sortedReadings[SIZE-1] = rpm;
      
      // Now sorting the readings
      bubble_sort(sortedReadings);
      
      // Checking the interquartile difference
      Q1 = sortedReadings[1];
      Q2 = sortedReadings[6];
      upperBound = Q2 + (Q2 - Q1) * 1.5;
      lowerBound = Q1 - (Q2 - Q1) * 1.5;
      if((rpm > upperBound || rpm < lowerBound) && !allow) //in this case the rpm value  is an outlier
      {        
        readings[SIZE-1] = readings[0]; // overwritting the last rpm value and make it equal to another acceptable value
        auxReadings[auxCounter++] = rpm;
      }
      else
      {
        auxCounter = 0;
        allow = false;
      }
      if(auxCounter >= SIZEAUX) //which means we have a sequence of other readings out of box and consistent
      {
        for(i = 0; i < SIZEAUX; i++)
        {
          readings[SIZE-SIZEAUX+i] = auxReadings[i];
        }
        auxCounter = 0;
        allow = true;
      } 
     return readings[SIZE-1]; 
    }
  }
  
}

float motorHandler::filterButter(float rpm) //the value supplied to this function should be the rpm value after ommitting the erronous values 
{
  int i = 0;
  float filteredRPM = 0;
  rpm = rejectOutlier(rpm);
  if(motorOFF)
  {
  }
  else
  {
     filteredRPM = 0.1659*(rpm + rpmOldIIR[1]) + 0.6682 * oldFiltered[1]; //7-8 hz
//    filteredRPM = 0.1871*(rpm + rpmOldIIR[1]) + 0.6258* oldFiltered[1]; //9hz
//    filteredRPM = 0.2841*(rpm + rpmOldIIR[1]) + 0.4318* oldFiltered[1]; //15hz
//    filteredRPM = 0.09163*(rpm + rpmOldIIR[1]) + 0.8167 * oldFiltered[1]; //4hz
    rpmOldIIR[1] = rpm;
    oldFiltered[1] = filteredRPM;  
    return filteredRPM;
  }
}

float motorHandler::filterFIR(float rpm) //the value supplied to this function should be the rpm value after ommitting the erronous values 
{
  int i = 0;
  orderFIR = 4;
//  float alpha[order] = {0.00051555,0.0035786,0.01305,0.033127,0.063933,0.10029,0.13303,0.15248,0.15248,0.13303,0.10029,0.063933,0.033127,0.01305,0.0035786,0.00051555}/*{0.0154,0.0584,0.1659,0.2603,0.2603,0.1659,0.0584,0.0154}*/;
//  float alpha[order] = {-0.00042079,-0.0012406,-0.0025434,-0.0044093,-0.0064192,-0.0075779,-0.0064454,-0.0014695,0.0085542,0.024022,0.044207,0.067194,0.090126,0.10972,0.12292,0.12758,0.12292,0.10972,0.090126,0.067194,0.044207,0.024022,0.0085542,-0.0014695,-0.0064454,-0.0075779,-0.0064192,-0.0044093,-0.0025434,-0.0012406,-0.00042079};
  float alpha[4] = {0.04478,0.45522,0.45522,0.04478};
  float filteredRPM = 0;
  rpm = rejectOutlier(rpm);
  
  if(motorOFF)
  {
  }
  else
  {
    // add the new reading to the array
    for(i = 0; i < orderFIR-1; i++)
    {
      rpmOldFIR[i] = rpmOldFIR[i+1];
    }
    rpmOldFIR[orderFIR-1] = rpm;
    
    //applying convolution to evaluate the filtered value
    filteredRPM = 0;
    for(i = 0; i < orderFIR; i++)
    {
      filteredRPM += rpmOldFIR[i]*alpha[orderFIR-i-1];
    }
  }
  return filteredRPM;
}

void motorHandler::PID(float rpm, float requiredRPM)
{
  float voltage = 0;
  float error = 0;
  float errorDifferential = 0; 
  int PWMvalue = 0;
  float timePID = (float)millis()/1000;  
  float dt = timePID - oldTimePID;
  
  // P part
  error = (requiredRPM - rpm);
  
  // I part
  if(motorON)
  {
    errorIntegral[1] = errorIntegral[0] + (error + errorOld)*0.5 * dt; //watch out!! when this term increases too much, it becomes negative and the motor suddenly stops and becomes sluggish
    errorIntegral[0] = errorIntegral[1];
  }
  else
  {
    errorIntegral[1] += 0;
  }
  
  // D part
  errorDifferential = (error - errorOld)/dt; 
  errorOld = error;
  
  // Using feedback from battery level to adjust the controller gains
  if(gainSchedulingOn)
  {
    voltage = (float)analogRead(A0) * 5.0 / 1023.0 * 3;
    Kp = 15.0882 - 1.2*voltage;
    Ki = 10.4117 - 1.176*voltage;
  }
  else
  {
    Kp = KpMin;
    Ki = KiMin;
  }
  
  // PID part
  if(abs(error) < 40)
  {
    PWMvalue = Kp * error * 0.3 + Ki * errorIntegral[1] * 0.8 + Kd*0 * errorDifferential;
  }
  else
  {
    PWMvalue = Kp * error + Ki * errorIntegral[1] + Kd * errorDifferential;
  }
  
  Serial.print(PWMvalue);
  Serial.print("\t");
  Serial.println(rpm);
  if(PWMvalue > 255){PWMvalue = 255;}
  else if(PWMvalue < 0){PWMvalue = 0;}
  analogWrite(motorPort,PWMvalue);
  oldTimePID = timePID;
}

void motorHandler::MIT(float rpm, float requiredRPM)
{
  float t = (float)millis()/1000.0;
  float dti = 0;
  float dtm = 0;
  float gamma = 0.01;
  float u = 0;
  float e = 0;
  // calculating the reference output
  dtm = t - tOld_ym;
  if(dtm > 0.05) //update the reference output every 50ms
  {
    ym[1] = 0.213*ym[0] + 2*0.3935*requiredRPM; //the transfer function for this is 1/(0.1s+1) converted to z domain
    ym[0] = ym[1];
    tOld_ym = t;
  }
  
  // while ym[1] is constant, calculate the error between actual and required outputs
  e = rpm - ym[1];
  E[1] = e * ym[1];
  dti = (t - tOld);
  if(dti > 0.03)
  {
    motorON = true;
    if(motorON)
    {
      theta[1] = theta[0] + (E[1]+E[0]) * 0.5 * dti;
      E[0] = E[1];
      theta[0] = theta[1];
    }
    tOld = t;
  }
  // calculating the control action going to the motor
  
  if(e > 20)
  {
    u = -gamma*(requiredRPM - rpm) * theta[1] /*+ 150*exp(-t*8)*/;
  }
  else
  {
    u = -0.001*gamma*(requiredRPM - rpm) * theta[1] /*+ 150*exp(-t*8)*/;
  }
  Serial.print(u); //don't remove this serial print command because when removed the integration blows up and the control action saturates indefenitely
  Serial.print("\t");
  if(u > 255){u = 255;}
  else if(u < 0){u = 0;}
  analogWrite(motorPort,u);  
}

void motorHandler::controlRPM(float speedRequired)
{
  int Ts = 0;
  unsigned long time = millis();
  
  if(/*(time - oldTime) >= Ts*/ true)
  {
    updateRPM();

    // the next two lines are just for testing
//    checkStop(RPM);
    FilteredRPM = RPM;

//    FilteredRPM = rejectOutlier(RPM);
//    FilteredRPM = filterButter(RPM);

    oldTime = time;
//    MIT(FilteredRPM,speedRequired);
    PID(FilteredRPM,speedRequired);
  }
}

void motorHandler::EF()
{
  int Ts = 10;
  unsigned long time = millis();
  if((time - oldTime) >= Ts)
  {
    updateRPM();
    FilteredRPM = rejectOutlier(RPM);
    oldTime = time;
  }
}

float motorHandler::getRPM()
{
 return RPM;  
}

float motorHandler::getFilteredRPM()
{
 return FilteredRPM;  
}
