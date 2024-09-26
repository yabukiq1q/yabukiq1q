#ifndef ODOMETRY_H
#define ODOMETRY_H

class Odometry
{
public:
  Odometry();
  Odometry(double R, double D);
  void estimateOdometry(double currentPos[]);
  void initializePresentPos(double data[]);
  void sendOdom();

private:
  double r; //車輪の半径
  double d; //中心から車輪までの距離

  double presentPos[2 + 1] = {0.0}; //[deg]

  struct Estimated
  {
    double x;
    double y;
    double theta;
  };

  Estimated estimated; 
};

#endif