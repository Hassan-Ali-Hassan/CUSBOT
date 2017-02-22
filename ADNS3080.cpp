#include "ADNS3080.h"
ADNS3080::ADNS3080()
{
  X = Y = 0;
  convFactorX = (1.0 / (float)(ADNS3080_PIXELS_X * SCALAR)) * 2.0 * tan(FOV / 2.0) * 1.2 ;/*I added this last 2.6 after some clibration*/;
  convFactorY = (1.0 / (float)(ADNS3080_PIXELS_X * SCALAR)) * 2.0 * tan(FOV / 2.0) * 1.2 ;
  altitude = 1.64 / 100.0; //altitude of lens of ground in meters, so x and y will be in meters
//  pinMode(PIN_SS,OUTPUT);
//  pinMode(PIN_MISO,INPUT);
//  pinMode(PIN_MOSI,OUTPUT);
//  pinMode(PIN_SCK,OUTPUT);
//  mousecam_reset();
}

void ADNS3080::mousecam_reset()
{
  digitalWrite(PIN_MOUSECAM_RESET,HIGH);
  delay(1); // reset pulse >10us
  digitalWrite(PIN_MOUSECAM_RESET,LOW);
  delay(35); // 35ms from reset to functional
}

int ADNS3080::mousecam_init()
{
  pinMode(PIN_MOUSECAM_RESET,OUTPUT);
  pinMode(PIN_MOUSECAM_CS,OUTPUT);
  
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  
  mousecam_reset();
  
  int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;

  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);

  return 0;
}

void ADNS3080::mousecam_write_reg(int reg, int val)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_MOUSECAM_CS,HIGH);
  delayMicroseconds(50);
}

int ADNS3080::mousecam_read_reg(int reg)
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(reg);
  delayMicroseconds(75);
  int ret = SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(1);
  return ret;
}

void ADNS3080::mousecam_read_motion()
{
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  SPI.transfer(ADNS3080_MOTION_BURST);
  delayMicroseconds(75);
  motion =  SPI.transfer(0xff);
  dx =  SPI.transfer(0xff);
  dy =  SPI.transfer(0xff);
  squal =  SPI.transfer(0xff);
  shutter =  SPI.transfer(0xff)<<8;
  shutter |=  SPI.transfer(0xff);
  max_pix =  SPI.transfer(0xff);
  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(5);
}

void ADNS3080::update(float a)
{
  byte motion_reg;
  squal = (unsigned int)mousecam_read_reg(ADNS3080_SQUAL);
  delayMicroseconds(50);  // small delay

    // check for movement, update x,y values
  motion_reg = mousecam_read_reg(ADNS3080_MOTION);
  overflowFlag = ((motion_reg & 0x10) != 0);  // check if we've had an overflow
  if( (motion_reg & 0x80) != 0 ) 
  {
    raw_dx = ((char)mousecam_read_reg(ADNS3080_DELTA_X));
    delayMicroseconds(50);  // small delay
    raw_dy = ((char)mousecam_read_reg(ADNS3080_DELTA_Y));
    motionFlag = true;
  }
  else
  {
    motionFlag = false;
    raw_dx = 0;
    raw_dy = 0;
  }

  rotate(a);
  if(squal > 10) //if the surface is well textured enough
  {
    dx_cm = -Dx * altitude * convFactorX;
    dy_cm = -Dy * altitude * convFactorY;
    X += dx_cm;
    Y += dy_cm;
  }
}

void ADNS3080::rotate(float a) //the given angle should be in radians
{
  Dx = raw_dx * cos(a) - raw_dy * sin(a);
  Dy = raw_dx * sin(a) + raw_dy * cos(a);
}

int ADNS3080::mousecam_frame_capture(byte *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);
  
  digitalWrite(PIN_MOUSECAM_CS, LOW);
  
  SPI.transfer(ADNS3080_PIXEL_BURST);
  delayMicroseconds(50);
  
  int pix;
  byte started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI.transfer(0xff);
    delayMicroseconds(10);
    if(started==0)
    {
      if(pix&0x40)
        started = 1;
      else
      {
        timeout++;
        if(timeout==100)
        {
          ret = -1;
          break;
        }
      }
    }
    if(started==1)
    {
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }

  digitalWrite(PIN_MOUSECAM_CS,HIGH); 
  delayMicroseconds(14);
  
  return ret;
}

void ADNS3080::disp()
{
  int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
  for(int i=0; i<squal/4; i++)
    Serial.print('*');
  Serial.print(' ');
  Serial.print((val*100)/351);
  Serial.print(' ');
  Serial.print(shutter); Serial.print(" (");
  Serial.print((int)dx); Serial.print(',');
  Serial.print((int)dy); Serial.println(')');
  delay(100);
}

void ADNS3080::disp2()
{
  update(0);
//  Serial.print(squal);
//  Serial.print("\t");
//  Serial.print(X);
//  Serial.print("\t");
//  Serial.println(Y);
  Serial2.print(squal);
  Serial2.print("\t");
  Serial2.print(X);
  Serial2.print("\t");
  Serial2.println(Y);
}

