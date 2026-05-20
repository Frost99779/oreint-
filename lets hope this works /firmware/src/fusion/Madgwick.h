#ifndef ORIENTATION_MADGWICK_H
#define ORIENTATION_MADGWICK_H

class Madgwick {
 public:
  void begin();
  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my,
              float mz, float beta, float dt);
  void eulerDeg(float &roll, float &pitch, float &yaw) const;
  void copyQuat(float out[4]) const;

 private:
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float beta, float dt);
  float q0{1.0f}, q1{0.0f}, q2{0.0f}, q3{0.0f};
};

float betaFromConf(bool still, bool useMag, float confA, float confM);

#endif  // ORIENTATION_MADGWICK_H
