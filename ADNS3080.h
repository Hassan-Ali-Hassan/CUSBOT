#ifndef ADNS3080_H
#define ADNS3080_H

#include "SPI.h"

// these pins may be different on different boards
#define PIN_SS        7
#define PIN_MISO      50
#define PIN_MOSI      51
#define PIN_SCK       52

#define PIN_MOUSECAM_RESET     53
#define PIN_MOUSECAM_CS        48

#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17

#define FOV 0.202458 
#define SCALAR 1.1


class ADNS3080
{
  public:
  ADNS3080();
  void disp();
  void disp2();
  int mousecam_init();
  void mousecam_read_motion();
  void update(float angle);
  float X,Y;
  byte squal;
  
  protected:
  void mousecam_reset();
  void mousecam_write_reg(int reg, int val);
  int mousecam_read_reg(int reg);
  int mousecam_frame_capture(byte *pdata);
  void rotate(float angle);
  
  /*****some variables****/
  byte frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];
  byte motion;
  char dx, dy;
  word shutter;
  byte max_pix;
  bool motionFlag;
  bool overflowFlag;
  /*these variables are adapted from ArduPilot library*/
  int raw_dx,raw_dy; /*the type of this variable needs checking*/
  float Dx,Dy;
  float dx_cm,dy_cm;
  float convFactorX,convFactorY;
  float altitude;
};

#endif
