#include <Arduino.h>
#include "BMI088.h"
#include <Wire.h>

#define Serial SerialUSB
#define BAUD_RATE 115200

BMI088_ACC bmi_acc(BMI088_ACC_ADDR_LOW);
BMI088_GYRO bmi_gyro(BMI088_GYRO_ADDR_LOW);

void setup(){
  delay(2000);
  Serial.begin(BAUD_RATE);

  while(!bmi_acc.begin()){
    Serial.println("FAILED_ACC");
    delay(500);
  }
  while(!bmi_gyro.begin()){
    Serial.println("FAILED_GYRO");
    delay(500);
  }
}

void loop(){
  bmi_acc.doAcc();
  bmi_gyro.doGyro();

  Serial.print(bmi_acc.x); Serial.print(", ");
  Serial.print(bmi_acc.y); Serial.print(", ");
  Serial.println(bmi_acc.z);

  delay(1);
}