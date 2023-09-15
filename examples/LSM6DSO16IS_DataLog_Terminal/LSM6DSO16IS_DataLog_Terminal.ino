/*
   @file    LSM6DSO16IS_DataLog_Terminal.ino
   @author  STMicroelectornics
   @brief   Example to use the LSM6DSO16IS inertial measurement sensor
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/

#include <LSM6DSO16ISSensor.h>

LSM6DSO16ISSensor sensor(&Wire);
int32_t accel[3], angrate[3];

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();
}

void loop()
{
  sensor.Get_X_Axes(accel);
  sensor.Get_G_Axes(angrate);

  Serial.print("Accel-X[mg]:");
  Serial.print(accel[0], 2);
  Serial.print(",Accel-Y[mg]:");
  Serial.print(accel[0], 2);
  Serial.print(",Accel-Z[mg]:");
  Serial.print(accel[0], 2);

  Serial.print(",AngRate-X[mdps]:");
  Serial.println(angrate[0], 2);
  Serial.print(",AngRate-Y[mdps]:");
  Serial.println(angrate[1], 2);
  Serial.print(",AngRate-Z[mdps]:");
  Serial.println(angrate[2], 2);

  blink(LED_BUILTIN);
}

inline void blink(int pin)
{
  digitalWrite(pin, HIGH);
  delay(25);
  digitalWrite(pin, LOW);
  delay(975);
}
