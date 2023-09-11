/*
   @file    LSM6DSO16IS_ISPU_Sensor_Fusion.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSO16IS ISPU Sensor Fusion of the accelerometer and gyroscope, configured in high-performance mode at 104 Hz.
 *******************************************************************************
   Copyright (c) 2023, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/
// Includes
#include "LSM6DSO16ISSensor.h"
#include "sensor_fusion.h"

#define INT_1 A5

//Interrupts.
volatile int sf_event = 0;

LSM6DSO16ISSensor sensor(&Wire);
ucf_line_ext_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;
void INT1Event_cb();

union data {
    uint8_t bytes[4];
    float_t values;
};

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  Serial.begin(115200);

  // Initlialize i2c.
  Wire.begin();

  // Initlialize components.
  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();

  // Feed the program to ISPU 
  ProgramPointer = (ucf_line_ext_t *)ispu_conf;
  TotalNumberOfLine = sizeof(ispu_conf) / sizeof(ucf_line_ext_t);
  Serial.println("LSM6DSO16IS ISPU Sensor Fusion");
  Serial.print("UCF Number Line=");
  Serial.println(TotalNumberOfLine);

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
    if(ProgramPointer[LineCounter].op == MEMS_UCF_OP_WRITE){
  		if (sensor.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
  		  Serial.print("Error loading the Program to LSM6DSO16ISSensor at line: ");
  		  Serial.println(LineCounter);
  		  while (1) {
    			// Led blinking.
    			digitalWrite(LED_BUILTIN, HIGH);
    			delay(250);
    			digitalWrite(LED_BUILTIN, LOW);
    			delay(250);
  		  }
		  }
	  }
    else if(ProgramPointer[LineCounter].op == MEMS_UCF_OP_DELAY){
      delay(ProgramPointer[LineCounter].data);
    }
  }

  Serial.println("Program loaded inside the LSM6DSO16IS ISPU");

  //Interrupts.
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);
  
  //Enable the access to ISPU interaction registers
  sensor.Write_Reg(LSM6DSO16IS_FUNC_CFG_ACCESS,0x80);

}

void loop()
{
  union data quaternions;
  //When the quaternion for the new sample is computed and available in the output registers an interrupt is generated
  if (sf_event) {
    sf_event = 0;
    Serial.print("Quaternion: ");
    //Get quaternion scalar component as float mapped starting from ISPU_DOUT_06_L (10h) and print it
    sensor.Read_Multi(LSM6DSO16IS_ISPU_DOUT_06_L ,&quaternions.bytes[0],4);
    Serial.print(quaternions.values, 4);
    Serial.print(", ");
    //Get quaternion y-axis as float mapped starting from ISPU_DOUT_02_L (10h) and print it
    sensor.Read_Multi(LSM6DSO16IS_ISPU_DOUT_02_L ,&quaternions.bytes[0],4);
    Serial.print(-quaternions.values, 4);
    Serial.print(", ");
    //Get quaternion x-axis as float mapped starting from ISPU_DOUT_00_L (10h) and print it
    sensor.Read_Multi(LSM6DSO16IS_ISPU_DOUT_00_L ,&quaternions.bytes[0],4);
    Serial.print(quaternions.values, 4);
    Serial.print(", ");
    //Get quaternion z-axis as float mapped starting from ISPU_DOUT_04_L (10h) and print it
    sensor.Read_Multi(LSM6DSO16IS_ISPU_DOUT_04_L ,&quaternions.bytes[0],4);
    Serial.println(quaternions.values, 4);
  }
}

void INT1Event_cb()
{
  sf_event = 1;
}
