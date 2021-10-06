#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"

//BMI088_ACC
uint8_t BMI088_ACC::writeToAcc(uint8_t reg, uint8_t data){
    Wire.beginTransmission(acc_address);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission();
}
uint8_t BMI088_ACC::requestFromAcc(uint8_t reg, uint8_t count){
    Wire.beginTransmission(acc_address); 
    Wire.write(reg); 
    uint8_t err = Wire.endTransmission(); 
    Wire.requestFrom(acc_address, count);
    return err;
}

bool BMI088_ACC::begin(){
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

    accConfig(ACC_OSR_2, ACC_ODR_400hz, ACC_RNG_24G);

    requestFromAcc(0x00, 1); 
    uint8_t readID = Wire.read();

    bool chipID_ERR = !(readID == 0x1E); //Check if read ID is correct (0x1E : ACC_CHIP_ID)

    if(fatalError || configError || chipID_ERR){ return false; } //Error checking

    return true;
}

void BMI088_ACC::accConfig(uint8_t acc_osr, uint8_t acc_odr, uint8_t acc_range){
    uint8_t configByte = (acc_osr << 4) | (acc_odr);
    writeToAcc(0x40, configByte); //Write to ACC_CONF address

    writeToAcc(0x41, acc_range); //Write to ACC_RANGE address

    acc_mss_conversion = pow(2, acc_range + 1) * 1.5f * 9.807f * (1.0f / 32768.0f);
}

void BMI088_ACC::doAcc(){
    requestFromAcc(0x12, 6); //Read from ACC_X_LSB address
    int16_t x_read = (Wire.read() | (Wire.read() << 8));
    int16_t y_read = (Wire.read() | (Wire.read() << 8));
    int16_t z_read = (Wire.read() | (Wire.read() << 8));

    x = float(x_read) * acc_mss_conversion;
    y = float(y_read) * acc_mss_conversion;
    z = float(z_read) * acc_mss_conversion;
}

//BMI088_GYRO
uint8_t BMI088_GYRO::writeToGyro(uint8_t reg, uint8_t data){
    Wire.beginTransmission(gyro_address);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission();
}
uint8_t BMI088_GYRO::requestFromGyro(uint8_t reg, uint8_t count){
    Wire.beginTransmission(gyro_address); 
    Wire.write(reg); 
    uint8_t err = Wire.endTransmission(); 
    Wire.requestFrom(gyro_address, count);
    return err;
}

bool BMI088_GYRO::begin(){
    Wire.begin();

    delay(10);

    Wire.beginTransmission(gyro_address);
    uint8_t error = Wire.endTransmission();

    if(error != 0){ return false; }

    requestFromGyro(0x00, 1); //Read from GYRO_CHIP_ID
    uint8_t readID = Wire.read();

    bool chipID_ERR = !(readID == 0x0F); //Check if read ID is correct (0x0F : GYRO_CHIP_ID)

    if(chipID_ERR){ return false; } //Error checking

    gyroConfig(GYRO_ODR_400hz, GYRO_RNG_1000);

    return true;
}

void BMI088_GYRO::gyroConfig(uint8_t gyro_odr, uint8_t gyro_range){
    writeToGyro(0x10, gyro_odr); //Write to GYRO_BANDWIDTH address

    writeToGyro(0x0F, gyro_range); //Write to GYRO_RANGE address

    const float DEGTORAD = 3.14159f / 180.0f;

    switch(gyro_range){
        case GYRO_RNG_2000:
            gyro_rad_conversion = 1.0f/16.384f * DEGTORAD;
            break;
        case GYRO_RNG_1000:
            gyro_rad_conversion = 1.0f/32.768f * DEGTORAD;
            break;
        case GYRO_RNG_500:
            gyro_rad_conversion = 1.0f/65.536f * DEGTORAD;
            break;
        case GYRO_RNG_250:
            gyro_rad_conversion = 1.0f/131.072f * DEGTORAD;
            break;
        case GYRO_RNG_125:
            gyro_rad_conversion = 1.0f/262.144f * DEGTORAD;
            break;
    }   
}

void BMI088_GYRO::doGyro(){
    requestFromGyro(0x02, 6); //Read from GYRO_X_LSB address
    int16_t x_read = (Wire.read() | (Wire.read() << 8));
    int16_t y_read = (Wire.read() | (Wire.read() << 8));
    int16_t z_read = (Wire.read() | (Wire.read() << 8));

    x = float(x_read) * gyro_rad_conversion - x_off;
    y = float(y_read) * gyro_rad_conversion - y_off;
    z = float(z_read) * gyro_rad_conversion - z_off;
}

bool BMI088_GYRO::calibrateGyro(uint32_t calculate_samples){
    if(sampleCount < calculate_samples){
        if(sqrtf(x*x + y*y + z*z) < 0.2f){
            sampleCount++;
            pre_x += x; pre_y += y; pre_z += z;
        }
        return (gyroCal = false);
    }else{
        x_off = pre_x/float(sampleCount); y_off = pre_y/float(sampleCount); z_off = pre_z/float(sampleCount);
        return (gyroCal = true);
    }
}

void BMI088_GYRO::resetCalibration(){
    sampleCount = 0;
    x_off = 0.0f; y_off = 0.0f; z_off = 0.0f;
    pre_x = 0.0f; pre_y = 0.0f; pre_z = 0.0f;
}