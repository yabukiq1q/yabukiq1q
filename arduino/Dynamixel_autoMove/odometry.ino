#include "odometry.h"

#define PI 3.14159265359

Odometry::Odometry()
{
  r =  62.0 / 2.0;
  d = 140.0 / 2.0;
}

Odometry::Odometry(double R, double D)
{
  r = R;
  d = D;
}


void Odometry::estimateOdometry(double currentPos[])
{
  double deltaL_L = ((currentPos[1] - presentPos[1]) * PI / 180.0) * r;
  double deltaL_R = ((currentPos[2] - presentPos[2]) * PI / 180.0) * r;

  double deltaL     = (deltaL_R + deltaL_L) / 2.0;
  double deltaTheta = (deltaL_R - deltaL_L) / 2.0 / d;

  if(abs(deltaTheta) > 0.01)
  {
    double rho = deltaL / deltaTheta;

    double deltaL2 = 2.0 * rho * sin(deltaTheta / 2.0);

    estimated.x += deltaL2 * cos(estimated.theta + deltaTheta / 2.0);
    estimated.y += deltaL2 * sin(estimated.theta + deltaTheta / 2.0);
    estimated.theta += deltaTheta;
  }
  else
  {
    estimated.x += deltaL * cos(estimated.theta + deltaTheta / 2.0);
    estimated.y += deltaL * sin(estimated.theta + deltaTheta / 2.0);
    estimated.theta += deltaTheta;
  }

  for(int i = 0; i < sizeof(presentPos) / sizeof(presentPos[0]); i++)
  {
    presentPos[i] = currentPos[i];
  }

  Serial.print("x: ");
  Serial.println(estimated.x);
  Serial.print("y: ");
  Serial.println(estimated.y);
  Serial.print("theta: ");
  Serial.println(estimated.theta);
}

void Odometry::initializePresentPos(double data[])
{
  for(int i = 0; i < sizeof(presentPos) / sizeof(presentPos[0]); i++)
  {
    presentPos[i] = data[i];
  }
}

void Odometry::sendOdom()
{
  byte* bytePointer = (byte*)(void*)&estimated.x;
  for (int i = 0; i < sizeof(double); i++) {
    Serial.write(bytePointer[i]);
  }
}