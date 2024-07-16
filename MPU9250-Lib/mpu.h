#ifndef MPU_H
#define MPU_H

#include <Wire.h>
#include <Arduino.h>

class MPU9250 {
  public:
    float accx,accy,accz;
    float gyrox,gyroy,gyroz;
    float mx,my,mz;
    float mxh = 0.0;
    float myh = 0.0; 
    float mzh = 0.0;
    float mxs = 1.0;
    float mys = 1.0;
    float mzs = 1.0;
    float mxh1 = 0.0;
    float myh1 = 0.0; 
    float mzh1 = 0.0;
    float mxs1 = 1.0;
    float mys1 = 1.0;
    float mzs1 = 1.0;
    float mxh2 = 0.0;
    float myh2 = 0.0; 
    float mzh2 = 0.0;
    float mxs2 = 1.0;
    float mys2 = 1.0;
    float mzs2 = 1.0;
    float eInt[3] = {0.0, 0.0, 0.0};
    float q[4] = {1.0, 0.0, 0.0, 0.0};

    MPU9250(TwoWire &w) : fir(w) { }

    void setmpu();
    void getAccelData();
    void getGyroData();
    void calibrateGyro();
    void getGyroDataCalibrated();
    void getMagData();
    void calibrateMag();
    void getMagDataCalibrated();
    void updatempumahoni(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
    void updatempumadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
    void set_cal_mag_data(float maghx ,float maghy, float maghz, float magsx, float magsy, float magsz, float maghx1 ,float maghy1, float maghz1, float maghx2, float maghy2, float maghz2);

  private:
    float asax = 180.0, asay = 182.0, asaz = 169.0;
    float mxadj,myadj,mzadj;
    float beta = sqrt(3.0f / 4.0f) *  PI * (60.0f / 180.0f); //gyro drift pentru madwick
    float gyroxadj,gyroyadj,gyrozadj;
    float gyroxb,gyroyb,gyrozb;
    static constexpr float Kp = 15 ;//42
    static constexpr float Ki = 1.2 ;//0
     
    TwoWire& fir;

    void writeinregister(int disp, int reg, int val);

};

#endif // MPU_H