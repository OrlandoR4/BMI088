#pragma once

#include <Arduino.h>
#include <Wire.h>

//ADDRESS
#define BMI088_ACC_ADDR_LOW 0x18
#define BMI088_ACC_ADDR_HIGH 0x19
#define BMI088_GYRO_ADDR_LOW 0x68
#define BMI088_GYRO_ADDR_HIGH 0x69
//ACC_BANDWITH
#define BMI088_ACC_OSR_4 0x08
#define BMI088_ACC_OSR_2 0x09
#define BMI088_ACC_OSR_NORMAL 0x0A
//ACC_ODR
#define BMI088_ACC_ODR_12hz 0x05
#define BMI088_ACC_ODR_25hz 0x06
#define BMI088_ACC_ODR_50hz 0x07
#define BMI088_ACC_ODR_100hz 0x08
#define BMI088_ACC_ODR_200hz 0x09
#define BMI088_ACC_ODR_400hz 0x0A
#define BMI088_ACC_ODR_800hz 0x0B
#define BMI088_ACC_ODR_1600hz 0x0C
//ACC_RANGE
#define BMI088_ACC_RNG_3G 0x00
#define BMI088_ACC_RNG_6G 0x01
#define BMI088_ACC_RNG_12G 0x02
#define BMI088_ACC_RNG_24G 0x03
//GYRO_ODR
#define BMI088_GYRO_ODR_2000hz_532hz 0x00
#define BMI088_GYRO_ODR_2000hz 0x01
#define BMI088_GYRO_ODR_1000hz 0x02
#define BMI088_GYRO_ODR_400hz 0x03
#define BMI088_GYRO_ODR_200hz 0x04
#define BMI088_GYRO_ODR_100hz 0x05
#define BMI088_GYRO_ODR_200hz_64hz 0x06
#define BMI088_GYRO_ODR_100hz_32hz 0x07
//GYRO_RANGE
#define BMI088_GYRO_RNG_2000 0x00
#define BMI088_GYRO_RNG_1000 0x01
#define BMI088_GYRO_RNG_500 0x02
#define BMI088_GYRO_RNG_250 0x03
#define BMI088_GYRO_RNG_125 0x04

class BMI088_ACCELEROMETER{
    private:
        uint8_t writeToAcc(uint8_t reg, uint8_t data){
            Wire.beginTransmission(acc_address);
            Wire.write(reg);
            Wire.write(data);
            return Wire.endTransmission();
        }
        uint8_t requestFromAcc(uint8_t reg, uint8_t count){
            Wire.beginTransmission(acc_address); 
            Wire.write(reg); 
            uint8_t err = Wire.endTransmission(); 
            Wire.requestFrom(acc_address, count);
            return err;
        }

        uint8_t acc_address;
        int8_t x_sign = 1, y_sign = 1, z_sign = 1;
        float acc_mss_conversion;
    public:
        BMI088_ACCELEROMETER(uint8_t acc_address_ = BMI088_ACC_ADDR_LOW){
            acc_address = acc_address_;
        }

        void set_axes_signs(int8_t _x_sign = 1, int8_t _y_sign = 1, int8_t _z_sign = 1){
            x_sign = _x_sign;
            y_sign = _y_sign;
            z_sign = _z_sign;
        }

        uint8_t setAccConfig(uint8_t acc_osr, uint8_t acc_odr, uint8_t acc_range){
            uint8_t configByte = (acc_osr << 4) | (acc_odr);

            uint8_t err = 0;

            err = writeToAcc(0x40, configByte); //Write to ACC_CONF address
            if(err > 0) return err;

            err = writeToAcc(0x41, acc_range); //Write to ACC_RANGE address
            if(err > 0) return err;

            acc_mss_conversion = powf(2, acc_range + 1) * 1.5f * 9.807f * (1.0f / 32768.0f);

            return 0;
        }

        uint8_t read_accelerometer(float *ax, float *ay, float *az){
            uint8_t err = requestFromAcc(0x12, 6); //Read from ACC_X_LSB address
            if(err == 0){
                *ax = int16_t(Wire.read() | (Wire.read() << 8)) * int16_t(x_sign) * acc_mss_conversion;
                *ay = int16_t(Wire.read() | (Wire.read() << 8)) * int16_t(y_sign) * acc_mss_conversion;
                *az = int16_t(Wire.read() | (Wire.read() << 8)) * int16_t(z_sign) * acc_mss_conversion;
            }
            return err;
        }

        bool begin(){
            Wire.begin();

            delay(10); //Wait after POR or reset
            
            Wire.beginTransmission(acc_address);
            uint8_t error = Wire.endTransmission();

            if(error != 0){ return false; }

            requestFromAcc(0x02, 1); //Read 1 byte from the ACC_ERR_REG register  
            uint8_t readError = Wire.read();

            bool fatalError = readError & 0b00000001;
            bool configError = (readError & 0b00011100) >> 2;

            writeToAcc(0x7D, 0x4); //Set ACC_PWR_CTRL Register power mode to on
            writeToAcc(0x7C, 0x00); //Set ACC_PWR_CONF Register power mode to active mode

            delay(50); //Wait for power mode change

            setAccConfig(BMI088_ACC_OSR_2, BMI088_ACC_ODR_400hz, BMI088_ACC_RNG_24G);

            requestFromAcc(0x00, 1); 
            uint8_t readID = Wire.read();

            bool chipID_ERR = !(readID == 0x1E); //Check if read ID is correct (0x1E : ACC_CHIP_ID)

            if(fatalError || configError || chipID_ERR){ return false; } //Error checking

            return true;
        }
};

class BMI088_GYROSCOPE{
    private:
        uint8_t writeToGyro(uint8_t reg, uint8_t data){
            Wire.beginTransmission(gyro_address);
            Wire.write(reg);
            Wire.write(data);
            return Wire.endTransmission();
        }
        uint8_t requestFromGyro(uint8_t reg, uint8_t count){
            Wire.beginTransmission(gyro_address); 
            Wire.write(reg); 
            uint8_t err = Wire.endTransmission(); 
            Wire.requestFrom(gyro_address, count);
            return err;
        }

        uint8_t gyro_address;
        int8_t x_sign = 1, y_sign = 1, z_sign = 1;
        float gyro_rad_conversion;
    public: 
        BMI088_GYROSCOPE(uint8_t gyro_address_ = BMI088_GYRO_ADDR_LOW){
            gyro_address = gyro_address_;
        }

        void set_axes_signs(int8_t _x_sign = 1, int8_t _y_sign = 1, int8_t _z_sign = 1){
            x_sign = _x_sign;
            y_sign = _y_sign;
            z_sign = _z_sign;
        }

        uint8_t setGyroConfig(uint8_t gyro_odr, uint8_t gyro_range){
            uint8_t err = 0;

            err = writeToGyro(0x10, gyro_odr); //Write to GYRO_BANDWIDTH address
            if(err > 0) return err;

            err = writeToGyro(0x0F, gyro_range); //Write to GYRO_RANGE address
            if(err > 0) return err;

            switch(gyro_range){
                case BMI088_GYRO_RNG_2000:
                    gyro_rad_conversion = 1.0f/16.384f * (3.14159f / 180.0f);
                    break;
                case BMI088_GYRO_RNG_1000:
                    gyro_rad_conversion = 1.0f/32.768f * (3.14159f / 180.0f);
                    break;
                case BMI088_GYRO_RNG_500:
                    gyro_rad_conversion = 1.0f/65.536f * (3.14159f / 180.0f);
                    break;
                case BMI088_GYRO_RNG_250:
                    gyro_rad_conversion = 1.0f/131.072f * (3.14159f / 180.0f);
                    break;
                case BMI088_GYRO_RNG_125:
                    gyro_rad_conversion = 1.0f/262.144f * (3.14159f / 180.0f);
                    break;
            }

            return 0;
        }

        uint8_t read_gyroscope(float *gx, float *gy, float *gz){
            uint8_t err = requestFromGyro(0x02, 6); //Read from GYRO_X_LSB address
            if(err == 0){
                *gx = int16_t(Wire.read() | (Wire.read() << 8)) * int16_t(x_sign) * gyro_rad_conversion;
                *gy = int16_t(Wire.read() | (Wire.read() << 8)) * int16_t(y_sign) * gyro_rad_conversion;
                *gz = int16_t(Wire.read() | (Wire.read() << 8)) * int16_t(z_sign) * gyro_rad_conversion;
            }
            return err;
        }

        bool begin(){
            Wire.begin();

            delay(10);

            Wire.beginTransmission(gyro_address);
            uint8_t error = Wire.endTransmission();

            if(error != 0){ return false; }

            requestFromGyro(0x00, 1); //Read from GYRO_CHIP_ID
            uint8_t readID = Wire.read();

            bool chipID_ERR = !(readID == 0x0F); //Check if read ID is correct (0x0F : GYRO_CHIP_ID)

            if(chipID_ERR){ return false; } //Error checking

            setGyroConfig(BMI088_GYRO_ODR_400hz, BMI088_GYRO_RNG_1000);

            return true;
        }
};