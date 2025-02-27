#include "mpu.h"

void MPU9250:: setmpu() { // function to set the MPU9250
    delay(100);
    
    writeinregister(0x69,0x6B,0x00); // wakeup mpu6050 selectare 20mhz oscilator 
    writeinregister(0x69,0x37,0x02); // bypass active
    writeinregister(0x69,0x1A,0b00000011); // set dlpf gyro : 3 41HZ for delay de 5.9ms 
    writeinregister(0x69,0x1B,0b00001000); // set fchoice 00 
    writeinregister(0x69,0x1C,0b00011000); // set accel range 2G , 00 for 2g , 01 for 4g , 10 for 8g , 11 for 16g  
    writeinregister(0x69,0x1D,0b00001100); // set filter accel 21.2 hz 
    writeinregister(0x69,0x19,0b00001001); // set smpldiv

    writeinregister(0x0C,0x0B,0b01); // reset
    writeinregister(0x0C,0x0A,0b00010110); // set continum mode
}

void MPU9250:: writeinregister(int disp, int reg, int val) { // function to write in register
    fir.beginTransmission(disp);
    fir.write(reg);
    fir.write(val);
    fir.endTransmission();
}

void MPU9250:: getAccelData() { // functiom to get accelerometer data
    fir.beginTransmission(0x69);
    fir.write(0x3B);
    fir.endTransmission();
    fir.requestFrom(0x69,6);

    int16_t accelX = fir.read() << 8 | fir.read();
    int16_t accelY = fir.read() << 8 | fir.read();
    int16_t accelZ = fir.read() << 8 | fir.read();

    accx = (float)accelX / 2048.0;
    accy = (float)accelY / 2048.0;
    accz = (float)accelZ / 2048.0;

    //divide to 16384 to obtain value for 2g
    //divide la 8192 to obtain value for 4g
    //divide la 4096 to obtain value for 8g
    //divide la 2048 to obtain value for 16g
}

void MPU9250::getGyroData() {  // function to get gyro data
    fir.beginTransmission(0x69);
    fir.write(0x43);
    fir.endTransmission();
    fir.requestFrom(0x69,6);

    int16_t gyroX = fir.read() << 8 | fir.read(); 
    int16_t gyroY = fir.read() << 8 | fir.read();
    int16_t gyroZ = fir.read() << 8 | fir.read();

    gyroxadj = (float)gyroX / 65.5;
    gyroyadj = (float)gyroY / 65.5;
    gyrozadj = (float)gyroZ / 65.5;
}

void MPU9250::calibrateGyro() { // function to calibrate gyro
    int numReadings1 = 0;
    float gxsum = 0, gysum = 0, gzsum = 0;

    while(numReadings1 < 1000) {
        numReadings1++;
        getGyroData();
        gxsum += gyroxadj;
        gysum += gyroyadj;
        gzsum += gyrozadj;
        delay(10);
    }

    gyroxb = gxsum / numReadings1;
    gyroyb = gysum / numReadings1;
    gyrozb = gzsum / numReadings1;
}

void MPU9250::getGyroDataCalibrated() { // function to get calibratede data from gyro
    getGyroData();
    gyrox = gyroxadj - gyroxb;
    gyroy = gyroyadj - gyroyb;
    gyroz = gyrozadj - gyrozb;
}

void MPU9250::getMagData(){ // function to get magnetometer data
    int16_t mxraw, myraw, mzraw;

    fir.beginTransmission(0x0C);
    fir.write(0x03); // Start de la HXL
    fir.endTransmission();
    fir.requestFrom(0x0C, 6); // citesc 6 bytes
    
    mxraw = fir.read() | fir.read() << 8;
    myraw = fir.read() | fir.read() << 8 ;
    mzraw = fir.read() | fir.read() << 8 ;
    
    fir.beginTransmission(0x0C); // need to read from this register accordin to datasheet
    fir.write(0x09);
    fir.endTransmission(); 
    fir.requestFrom(0x0C, 1); 

    mxadj = (((((asax - 128) * 0.5) / 128) + 1)* mxraw) * 0.15; // datasheet formula (((asax - 128) * 0.5) / 128 + 1) 
    myadj = (((((asay - 128) * 0.5) / 128) + 1)* myraw) * 0.15; // 0.15 scaling factor is 4912/32768
    mzadj = (((((asaz - 128) * 0.5) / 128) + 1)* mzraw) * 0.15;
}

void MPU9250::calibrateMag(){ // calibrate the magnetometer
    int numReadings = 0;
    float mxmax = -10000, mymax = -10000, mzmax = -10000;
    float mxmin = 10000, mymin = 10000, mzmin = 10000;

    while(numReadings < 100000) { 
        numReadings++;
        getMagData();
        if (mxadj > mxmax) mxmax = mxadj;
        if (myadj > mymax) mymax = myadj;
        if (mzadj > mzmax) mzmax = mzadj;
        if (mxadj < mxmin) mxmin = mxadj;
        if (myadj < mymin) mymin = myadj;
        if (mzadj < mzmin) mzmin = mzadj;
    }

    //hard iron losses
    mxh = (mxmax + mxmin) / 2.0;
    myh = (mymax + mymin) / 2.0;
    mzh = (mzmax + mzmin) / 2.0;

    // soft iron losses
    float xcoord = (mxmax - mxmin) / 2.0;
    float ycoord = (mymax - mymin) / 2.0;
    float zcoord = (mzmax - mzmin) / 2.0;
    float avgcoord = (xcoord + ycoord + zcoord) / 3.0;

    mxs = avgcoord / xcoord;
    mys = avgcoord / ycoord;
    mzs = avgcoord / zcoord;

    numReadings = 0;
}

void MPU9250::getMagDataCalibrated(){ // function to get calibrated data from the magnetometer
    getMagData();
    mx = ((((mxadj - mxh) * mxs)));
    my = ((((myadj - myh) * mys)));
    mz = ((((mzadj - mzh) * mzs)));
}

void MPU9250::set_cal_mag_data(float maghx ,float maghy, float maghz, float magsx, float magsy, float magsz){ // function to set the calibrated values
    mxh = maghx;
    myh = maghy;
    mzh = maghz;
    mxs = magsx;
    mys = magsy;
    mzs = magsz;
}

void MPU9250::updatempumahoni(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat) // mahoni filter function
{ 
    // local variables 
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // auxiliar variables 
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // acceleration normalization
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // magnetometer normalization
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // reference direction for Earth magnetic Field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // estimated direction of gravity
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // errors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
    eInt[0] += ex;      
    eInt[1] += ey;
    eInt[2] += ez;
    }
    else
    {
    eInt[0] = 0.0f;    
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
    }

    // feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // integrate the rate of change
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // quaternion normalization
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void MPU9250::updatempumadgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat) // madwick filter function
{   
    // local variables
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // auxiliar variables
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // acceleration normalization
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // magnetometer normalization
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // reference direction for Earth magnetic Field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // calculate rate of change
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // integrate the quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}



