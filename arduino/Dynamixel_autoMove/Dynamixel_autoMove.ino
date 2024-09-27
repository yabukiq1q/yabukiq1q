#include <Dynamixel2Arduino.h>
#include "odometry.h"

#if defined(ARDUINO_OpenCM904)
#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 22;
#elif defined(ARDUINO_OpenCR)
#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;
#elif defined(ARDUINO_OpenRB)
#define DXL_SERIAL Serial1
const int DXL_DIR_PIN = -1;
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const unsigned long SERIAL_BAUDRATE = 115200;
const unsigned long DXL_BAUDRATE = 1000000;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t dxl_left_id   = 1;
const uint8_t dxl_right_id  = 2;
const uint8_t dxl_left_id2  = 3;
const uint8_t dxl_right_id2 = 4;

/*
front
1    2
  □
3    4
*/

const uint8_t dxl_id[4]=
{
  1,
  2,
  3,
  4
};

float targetVel_x = 0.0;
float targetRot_z = 0.0;

byte buffer[9];

//モータを認識するまで待機
void waitUntilScanDXL()
{
  while(!dxl.scan())
  {
    Serial.println("NotFound");   
  }
  Serial.println("Found");
}

//============================================================
double data[4 + 1] =
{
  0.0,
  0.0,
  0.0,
  0.0,
  0.0
};
//============================================================

Odometry odom;

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  waitUntilScanDXL();

  for(int i = 0; i < sizeof(dxl_id)/sizeof(dxl_id[0]); i++)
  {
    dxl.torqueOff(dxl_id[i]);
    dxl.setOperatingMode(dxl_id[i], OP_VELOCITY);
    dxl.torqueOn(dxl_id[i]);
  }

  // for(uint8_t id = 1; id <= 2; id++)
  // {
  //   Serial.println(id);
  //   Serial.println(dxl.getPresentPosition(id, UNIT_DEGREE));
  //   data[id] = dxl.getPresentPosition(id, UNIT_DEGREE);
  // }
  // odom.initializePresentPos(data);
}

void loop()
{
  if(Serial.available() > 0)
  {
    if(Serial.readBytesUntil(0xFE, buffer, sizeof(buffer)) == 9)
    {
      if(buffer[0] == 0xFF)
      {
        memcpy(&targetVel_x, &buffer[1], sizeof(float));
        memcpy(&targetRot_z, &buffer[5], sizeof(float));
        dxl.setGoalVelocity(1, targetVel_x - targetRot_z);
        dxl.setGoalVelocity(3, targetVel_x - targetRot_z);
        dxl.setGoalVelocity(2, targetVel_x + targetRot_z);
        dxl.setGoalVelocity(4, targetVel_x + targetRot_z);
      }
    }
  }

  // dxl.setGoalVelocity(1, 40.0);
  // dxl.setGoalVelocity(2, 40.0);

  for(uint8_t id = 1; id <= 2; id++)
  {
    // data[id] = dxl.getPresentVelocity(id);
    //Serial.println(id);
    //Serial.println(data[id]);
    //Serial.println(dxl.getPresentPosition(id, UNIT_DEGREE));
    data[id] = dxl.getPresentPosition(id, UNIT_DEGREE);
  }

  odom.estimateOdometry(data);
}
