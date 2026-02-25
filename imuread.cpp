#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstring>
#include <unistd.h>
#include "I2CDevice.h"

using namespace std;
using namespace exploringRPi; // Adjust namespace if your copy of Molloy's code uses a different one

// ==========================================
// SENSOR REGISTER DEFINITIONS
// ==========================================

// LSM6DS33 (Gyro + Accel) - Default I2C Address 0x6B
#define LSM6DS33_ADDR 0x6B
#define LSM6DS33_CTRL1_XL 0x10  // Linear Acceleration Sensor Control
#define LSM6DS33_CTRL2_G  0x11  // Angular Rate Sensor Control
#define LSM6DS33_CTRL3_C  0x12  // Control Register 3
#define LSM6DS33_OUTX_L_G 0x22  // Output Start (Gyro X Low)

// LIS3MDL (Magnetometer) - Default I2C Address 0x1E
#define LIS3MDL_ADDR  0x1E
#define LIS3MDL_CTRL1 0x20
#define LIS3MDL_CTRL2 0x21
#define LIS3MDL_CTRL3 0x22
#define LIS3MDL_OUTX_L 0x28     // Output Start (Mag X Low)

// ==========================================
// MADGWICK FILTER IMPLEMENTATION
// ==========================================
class Madgwick {
private:
    float beta; // algorithm gain
    float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame

public:
    Madgwick() : beta(0.1f), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            // Call IMU update (no mag) here if needed, but for now we return
            return;
        }

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = 1.0f / sqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;
            
            // Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            
            recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        // Normalise quaternion
        recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    void getEuler(float &roll, float &pitch, float &yaw) {
        // Convert Quaternion to Euler angles (Roll, Pitch, Yaw)
        roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
        pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
        yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        
        // Convert to degrees
        roll *= 180.0f / M_PI;
        pitch *= 180.0f / M_PI;
        yaw *= 180.0f / M_PI;
    }
    void getQuaternion(float &w, float &x, float &y, float &z){
		w = q0;
		x = q1;
		y = q2;
		z = q3;
	}
};

// ==========================================
// HELPER FUNCTIONS
// ==========================================

// Helper to combine two bytes into a signed 16-bit integer
int16_t combineBytes(unsigned char msb, unsigned char lsb) {
    return (int16_t)((msb << 8) | lsb);
}

// Global clock for dt calculation
auto last_time = std::chrono::steady_clock::now();

float get_dt() {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> diff = current_time - last_time;
    last_time = current_time;
    return diff.count();
}

int main(int argc, char *argv[]) {
    // 1. Initialize I2C Devices
    // Bus 1 is standard for Raspberry Pi
    I2CDevice *imuDev = new I2CDevice(1, LSM6DS33_ADDR);
    I2CDevice *magDev = new I2CDevice(1, LIS3MDL_ADDR);

    // 2. Configure LSM6DS33 (Gyro/Accel)
    // CTRL1_XL: 0x60 = 416 Hz ODR, 2g scale, anti-aliasing 400Hz
    imuDev->writeRegister(LSM6DS33_CTRL1_XL, 0x20); 
    // CTRL2_G: 0x60 = 416 Hz ODR, 2000 dps scale
    imuDev->writeRegister(LSM6DS33_CTRL2_G, 0x24); 
    // CTRL3_C: 0x44 = Block Data Update (BDU) | Auto-increment
    imuDev->writeRegister(LSM6DS33_CTRL3_C, 0x44);
    // 3. Configure LIS3MDL (Magnetometer)
    // CTRL_REG1: 0x70 = Temp sensor en, Ultra-high perform, 80 Hz ODR, FAST_ODR off
    magDev->writeRegister(LIS3MDL_CTRL1, 0x6c); 
    // CTRL_REG2: 0x00 = +/- 4 gauss scale
    magDev->writeRegister(LIS3MDL_CTRL2, 0x00); 
    // CTRL_REG3: 0x00 = Continuous-conversion mode
    magDev->writeRegister(LIS3MDL_CTRL3, 0x00); 

    //std::cout << "Sensors initialized. Starting capture..." << std::endl;
    //std::cout << "Note: Ensure sensor is flat and stationary for first few seconds." << std::endl;

    Madgwick filter;
    
    // Buffers
    unsigned char imuBuf[12]; // 6 bytes Gyro, 6 bytes Accel
    unsigned char magBuf[6];  // 6 bytes Mag
    
    // Sensitivities (Based on 2g and 2000dps and 4gauss settings)
    // Accel 2g: 0.061 mg/LSB -> 0.000061 g/LSB
    float accel_sen = 0.000061f; 
    // Gyro 2000dps: 70 mdps/LSB -> 0.07 dps/LSB
    float gyro_sen = 0.0175f;
    // Mag 4gauss: 6842 LSB/gauss -> 1/6842 gauss/LSB
    float mag_sen = 1.0f / 6842.0f;

    // Main Loop
    while (1) {
        // --- READ GYRO & ACCEL (LSM6DS33) ---
        // We read 12 bytes starting from OUTX_L_G (0x22).
        // Order is usually: GxL, GxH, GyL, GyH, GzL, GzH, AxL, AxH, AyL, AyH, AzL, AzH
        unsigned char* imuBuf = imuDev->readRegisters(12,LSM6DS33_OUTX_L_G);
        
        int16_t raw_gx = combineBytes(imuBuf[1], imuBuf[0]);
        int16_t raw_gy = combineBytes(imuBuf[3], imuBuf[2]);
        int16_t raw_gz = combineBytes(imuBuf[5], imuBuf[4]);
        
        int16_t raw_ax = combineBytes(imuBuf[7], imuBuf[6]);
        int16_t raw_ay = combineBytes(imuBuf[9], imuBuf[8]);
        int16_t raw_az = combineBytes(imuBuf[11], imuBuf[10]);
        
        delete[] imuBuf;

        // --- READ MAGNETOMETER (LIS3MDL) ---
        // Read 6 bytes starting from OUTX_L (0x28)
        unsigned char* magBuf = imuDev->readRegisters(6,LIS3MDL_OUTX_L);
        
        int16_t raw_mx = combineBytes(magBuf[1], magBuf[0]);
        int16_t raw_my = combineBytes(magBuf[3], magBuf[2]);
        int16_t raw_mz = combineBytes(magBuf[5], magBuf[4]);

		delete[] magBuf;
		
        // --- CONVERT UNITS ---
        // 1. Accel in Gs
        float ax = raw_ax * accel_sen;
        float ay = raw_ay * accel_sen;
        float az = raw_az * accel_sen;

        // 2. Gyro in Radians/Sec (Required by Madgwick)
        // raw * sensitivity * (PI / 180)
        float gx = raw_gx * gyro_sen * (M_PI / 180.0f);
        float gy = raw_gy * gyro_sen * (M_PI / 180.0f);
        float gz = raw_gz * gyro_sen * (M_PI / 180.0f);

        // 3. Mag in Gauss
        float mx = raw_mx * mag_sen;
        float my = raw_my * mag_sen;
        float mz = raw_mz * mag_sen;

        // --- DATA FUSION ---
        float dt = get_dt();
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

        float w, x, y, z;
        filter.getQuaternion(w, x, y, z);

        // --- TERMINAL OUTPUT ---
        // Using \r to overwrite the line for a clean dashboard look
        printf("%f %f %f %f\n", w, x, y, z);
        fflush(stdout);

        // Small sleep to prevent CPU hogging (approx 100Hz loop)
        usleep(10000); 
    }

    return 0;
}
