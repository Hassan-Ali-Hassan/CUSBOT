#ifndef MOTORHANDLER_H
#define MOTORHANDLER_H

#include "Arduino.h"
#include<Wire.h>
#define SIZE 8
#define SIZEAUX 3
class motorHandler{
  
  public:
  
  motorHandler(int encAdd, int port);
  void controlRPM(float spreq);
  void updateRPM();
  void updateRPM_filtered();
  void stop();
  void EF(); //stands for elementary filteration
  float getRPM();
  float getFilteredRPM();
  float RPM;
  float FilteredRPM;
  void setPID(float,float,float);
  void setGainScheduling(boolean);
  
  protected:
  
  int encoderAddress;
  int motorPort;
  bool motorON;
  bool motorOFF;
  
  // Some useful functions
  void checkStop(float rpm); //makes sure the motor hasn't stopped for its operation
  float mean(float* a,int n);
  void bubble_sort(float* a);
  void swap(float& a,float& b);
  
  // Filtering function
  float rejectOutlier(float rpm); // rejecting outliers in data
  float filterFIR(float rpm); //using FIR filters
  float filterButter(float rpm); //using Butterworth filters 
  
  // Controller functions
  void MIT(float rpm, float requiredRPM); // adaptive controller using MIT rule
  void PID(float rpm, float requiredRPM); // PID controller
  
  // Variables for MIT function
  float ym[2]; // for the reference output from the reference model
  float theta[2]; //for signal integration with time using trapezoidal rule
  float E[2]; //This is the error multiplied by ym, the quantity to be integrated
  float tOld_ym;
  float tOld;
  
  //variables for controlRPM function
  unsigned long oldTime;
  
  //variables for pid function
  float errorIntegral[2];
  float errorOld;
  float oldTimePID;
  float Kp;
  float Ki;
  float Kd;
  float KpMax;
  float KpMin;
  float KiMax;
  float KiMin;
  boolean gainSchedulingOn;
  
  //variables for FIR filter
  int orderFIR;
  float rpmOldFIR[4];
  
  //variables for IIR filter
  float oldFiltered[3]; //we are using only oldFiltered[1] and oldFiltered[2], with 2 being the older value
  float rpmOldIIR[3];
  
  //variables for rejectOutlier function
  float readings[SIZE];
  float auxReadings[SIZEAUX];
  int counter;
  int auxCounter;
  bool allow;
  bool sorted;
  
  //variables for the checkStop function
  float rpm_old;
  float lastRPM;
  int repetitionCounter;
   
};

#endif
