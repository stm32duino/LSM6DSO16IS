/*
   @file    LSM6DSO16IS_ISPU_Tap.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSO16IS ISPU Tap Detection based on the accelerometer data.
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
#include "tap_detection.h"

#define INT_1 A5

//Interrupts.
volatile int tap_event = 0;

LSM6DSO16ISSensor sensor(&Wire);

ucf_line_ext_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;
void INT1Event_cb();
void printTapStatus(uint8_t status);

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

  // Feed the program to ISPU 
  ProgramPointer = (ucf_line_ext_t *)ispu_conf;
  TotalNumberOfLine = sizeof(ispu_conf) / sizeof(ucf_line_ext_t);
  Serial.println("LSM6DSO16IS ISPU Tap");
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
  //Enable the access to the ISPU interaction registers
  sensor.Write_Reg(LSM6DSO16IS_FUNC_CFG_ACCESS,0x80);

}

void loop()
{
  uint8_t data;
  //When the tap evet for the new sample is computed and available in the output registers an interrupt is generated
  if (tap_event) {
    tap_event = 0;
    //Get the tap event as uint8_t mapped starting from ISPU_DOUT_06_L and print it.
    sensor.Read_Reg(LSM6DSO16IS_ISPU_DOUT_06_L,&data);
    printTapStatus(data);
  }
}

void INT1Event_cb()
{
  tap_event = 1;
}

void printTapStatus(uint8_t status)
{
  switch (status) {
    case 1:
      Serial.println("Single Tap Detected");
      break;
    case 2:
      Serial.println("Double Tap Detected");
      break;
    case 3:
      Serial.println("Triple Tap Detected");
      break;
    default:
      break;
  }
}
