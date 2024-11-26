#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <iostream>
#include <cmath>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.H>



// Put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  int result = myFunction(2, 4);
  Serial.print("this is the value of myFunction: ");
  Serial.println(result);  // Print the result of myFunction
  delay(1000);  // Delay for 1 second to avoid flooding the serial output
}

// Put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
