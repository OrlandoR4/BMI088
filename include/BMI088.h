#ifndef BMI088_H
#define BMI088_H

#include <Arduino.h>
#include <Wire.h>

//ADDRESS
#define BMI088_ACC_ADDR_LOW 0x18
#define BMI088_ACC_ADDR_HIGH 0x19
#define BMI088_GYRO_ADDR_LOW 0x68
#define BMI088_GYRO_ADDR_HIGH 0x69
//ACC_BANDWITH
#define ACC_OSR_4 0x08
#define ACC_OSR_2 0x09
#define ACC_OSR_NORMAL 0x0A
//ACC_ODR
#define ACC_ODR_12hz 0x05
#define ACC_ODR_25hz 0x06
#define ACC_ODR_50hz 0x07
#define ACC_ODR_100hz 0x08
#define ACC_ODR_200hz 0x09
#define ACC_ODR_400hz 0x0A
#define ACC_ODR_800hz 0x0B
#define ACC_ODR_1600hz 0x0C
//ACC_RANGE
#define ACC_RNG_3G 0x00
#define ACC_RNG_6G 0x01
#define ACC_RNG_12G 0x02
#define ACC_RNG_24G 0x03
//GYRO_ODR
#define GYRO_ODR_2000hz_532hz 0x00
#define GYRO_ODR_2000hz 0x01
#define GYRO_ODR_1000hz 0x02
#define GYRO_ODR_400hz 0x03
#define GYRO_ODR_200hz 0x04
#define GYRO_ODR_100hz 0x05
#define GYRO_ODR_200hz_64hz 0x06
#define GYRO_ODR_100hz_32hz 0x07
//GYRO_RANGE
#define GYRO_RNG_2000 0x00
#define GYRO_RNG_1000 0x01
#define GYRO_RNG_500 0x02
#define GYRO_RNG_250 0x03
#define GYRO_RNG_125 0x04

class BMI088_ACC{
    private:
        uint8_t writeToAcc(uint8_t reg, uint8_t data);
        uint8_t requestFromAcc(uint8_t reg, uint8_t count);

        float acc_mss_conversion;
    public:
        uint8_t acc_address;

        float x, y, z;

        BMI088_ACC(uint8_t anAddress){
            acc_address = anAddress;
        }
        
        bool begin();

        void accConfig(uint8_t acc_osr, uint8_t acc_odr, uint8_t acc_range);
        
        void doAcc();
};

class BMI088_GYRO{
    private:
        uint8_t writeToGyro(uint8_t reg, uint8_t data);
        uint8_t requestFromGyro(uint8_t reg, uint8_t count);

        float gyro_rad_conversion;

        //Calibration
        uint32_t sampleCount;
        float pre_x;
        float pre_y;
        float pre_z;     
    public:
        uint8_t gyro_address;

        float x, y, z;
        float x_off, y_off, z_off;

        bool gyroCal;

        BMI088_GYRO(uint8_t anAddress){
            gyro_address = anAddress;
        }

        bool begin();

        void gyroConfig(uint8_t gyro_odr, uint8_t gyro_range);
        
        void doGyro();

        bool calibrateGyro(uint32_t calculate_samples);

        void resetCalibration();
};

#endif