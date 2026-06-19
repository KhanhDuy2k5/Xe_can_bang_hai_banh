#ifndef MPU6050_KALMAN_H
#define MPU6050_KALMAN_H

#include "Wire.h"
#define   RAD2DEG   57.295779

class SMPU6050 {
  public:
    SMPU6050() {
    };

    void init(int address) {
      this->i2cAddress = address;

      this->gyroXOffset = 0;
      this->gyroYOffset = 0;
      this->gyroZOffset = 0;

      this->accX = 0;
      this->accY = 0;

      this->xAngle = 0;
      this->yAngle = 0;
      this->zAngle = 0;

      // Khoi tao Kalman filter cho truc Y
      this->angleY = 0;
      this->biasY  = 0;
      this->P_Y[0][0] = 0; this->P_Y[0][1] = 0;
      this->P_Y[1][0] = 0; this->P_Y[1][1] = 0;

      // Khoi tao Kalman filter cho truc X
      this->angleX = 0;
      this->biasX  = 0;
      this->P_X[0][0] = 0; this->P_X[0][1] = 0;
      this->P_X[1][0] = 0; this->P_X[1][1] = 0;

      this->prevMillis = millis();

      // reset va cau hinh MPU6050
      Wire.begin();
      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x6B); // PWR_MGMT_1
      Wire.write(0);    // reset MPU6050
      Wire.endTransmission(true);

      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x19); // SMPLRT_DIV
      Wire.write(0);
      Wire.endTransmission(true);

      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x1B); // GYRO_CONFIG
      Wire.write(0);    // +-250 deg/s
      Wire.endTransmission(true);

      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x1C); // ACCEL_CONFIG
      Wire.write(0);    // +-2g
      Wire.endTransmission(true);
    }

    // ham hieu chinh gyro offset
    void calibrate(int times) {
      long gyroXTotal = 0, gyroYTotal = 0, gyroZTotal = 0;
      int count = 0;
      int gyroRawX, gyroRawY, gyroRawZ;
      for (int i = 0; i < times; i++) {
        Wire.beginTransmission(this->i2cAddress);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(this->i2cAddress, 6, true);

        gyroRawX = Wire.read() << 8 | Wire.read();
        gyroRawY = Wire.read() << 8 | Wire.read();
        gyroRawZ = Wire.read() << 8 | Wire.read();

        gyroXTotal += gyroRawX;
        gyroYTotal += gyroRawY;
        gyroZTotal += gyroRawZ;
        count += 1;
      }
      gyroXOffset = -gyroXTotal * 1.0 / count;
      gyroYOffset = -gyroYTotal * 1.0 / count;
      gyroZOffset = -gyroZTotal * 1.0 / count;
    }

    double getXAngle() {
      this->readAngles();
      return this->xAngle;
    };

    double getYAngle() {
      this->readAngles();
      return this->yAngle;
    };

    double getZAngle() {
      this->readAngles();
      return this->zAngle;
    };

    double getXAcc() {
      this->readAngles();
      return this->accX;
    };

    double getYAcc() {
      this->readAngles();
      return this->accY;
    };

    void getXYZAngles(double &x, double &y, double &z) {
      this->readAngles();
      x = xAngle;
      y = yAngle;
      z = zAngle;
    }

    // tham so Kalman co the chinh tu ben ngoai neu can
    void setKalmanQangle(double q) { Q_angle = q; }
    void setKalmanQbias(double q)  { Q_bias  = q; }
    void setKalmanRmeasure(double r) { R_measure = r; }

  private:
    int i2cAddress;
    double accX, accY, gyroX, gyroY, gyroZ;
    double gyroXOffset, gyroYOffset, gyroZOffset;
    double xAngle, yAngle, zAngle;
    unsigned long prevMillis;

    // ===== Kalman filter state =====
    double Q_angle   = 0.001;
    double Q_bias    = 0.003;
    double R_measure = 0.03;

    double angleY, biasY, P_Y[2][2];
    double angleX, biasX, P_X[2][2];

    // Kalman 1D: newAngle = goc do tu accel (deg), newRate = toc do goc tu gyro (deg/s)
    double kalman1D(double newAngle, double newRate, double dt,
                     double &angle, double &bias, double P[2][2]) {
      // Predict
      angle += dt * (newRate - bias);

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      // Update
      double S = P[0][0] + R_measure;
      double K0 = P[0][0] / S;
      double K1 = P[1][0] / S;

      double y = newAngle - angle;
      angle += K0 * y;
      bias  += K1 * y;

      double P00_temp = P[0][0];
      double P01_temp = P[0][1];

      P[0][0] -= K0 * P00_temp;
      P[0][1] -= K0 * P01_temp;
      P[1][0] -= K1 * P00_temp;
      P[1][1] -= K1 * P01_temp;

      return angle;
    }

    void readAngles() {
      if (millis() - this->prevMillis < 3)
        return;

      int accRawX, accRawY, accRawZ, gyroRawX, gyroRawY, gyroRawZ;

      Wire.beginTransmission(this->i2cAddress);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(this->i2cAddress, 14, true);

      accRawX = Wire.read() << 8 | Wire.read();
      accRawY = Wire.read() << 8 | Wire.read();
      accRawZ = Wire.read() << 8 | Wire.read();
      Wire.read(); Wire.read();
      gyroRawX = Wire.read() << 8 | Wire.read();
      gyroRawY = Wire.read() << 8 | Wire.read();
      gyroRawZ = Wire.read() << 8 | Wire.read();

      // Goc tinh tu accelerometer (deg)
      accX = atan((accRawY / 16384.0) / sqrt(pow((accRawX / 16384.0), 2) + pow((accRawZ / 16384.0), 2))) * RAD2DEG;
      accY = atan(-1 * (accRawX / 16384.0) / sqrt(pow((accRawY / 16384.0), 2) + pow((accRawZ / 16384.0), 2))) * RAD2DEG;

      // Toc do goc tu gyro (deg/s)
      gyroX = (gyroRawX + gyroXOffset) / 131.0;
      gyroY = (gyroRawY + gyroYOffset) / 131.0;
      gyroZ = (gyroRawZ + gyroZOffset) / 131.0;

      unsigned long curMillis = millis();
      double duration = (curMillis - prevMillis) * 1e-3;
      if (duration <= 0) duration = 0.001;
      prevMillis = curMillis;

      // Kalman filter cho X va Y, complementary cho Z (khong co goc tham chieu accel cho yaw)
      xAngle = kalman1D(accX, gyroX, duration, angleX, biasX, P_X);
      yAngle = kalman1D(accY, gyroY, duration, angleY, biasY, P_Y);
      zAngle = zAngle + gyroZ * duration;
    }
};

void mpu6050Init(SMPU6050 &smpu, int address) {
  smpu.init(address);
}

void mpu6050Calibrate(SMPU6050 &smpu, int times) {
  smpu.calibrate(times);
}

double mpu6050GetXAngle(SMPU6050 &smpu) {
  return smpu.getXAngle();
}

double mpu6050GetYAngle(SMPU6050 &smpu) {
  return smpu.getYAngle();
}

double mpu6050GetZAngle(SMPU6050 &smpu) {
  return smpu.getZAngle();
}

void mpu6050GetXYZAngles(SMPU6050 &smpu, double &xAngle, double &yAngle, double &zAngle) {
  smpu.getXYZAngles(xAngle, yAngle, zAngle);
}

#endif  /*MPU6050_KALMAN_H*/
