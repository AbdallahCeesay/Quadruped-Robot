#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <cmath>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.H>
#include <ComplementaryFilter.h>


/*remember to always add some sort of delay to allow the IMU to settle*/
Adafruit_MPU6050 mpu;
ComplementaryFilter filter;

/* PID gains*/
const float Kp = 25;
const float Ki = 10;
const float Kd = 0.8;
 

/* PID variables*/
float rollError = 0, pitchError = 0;                  /*these are the variable for the Proportional term*/
float rollErrorSum = 0, pitchErrorSum = 0;             /* these are the variables for the Integral term*/
float previousRollError = 0, previousPitchError = 0;  /* these are the variable for the Derivative term*/


/* desired set points (robot upright)*/
float rollSetPoint = 0;
float pitchSetPoint = 0;


unsigned long lastTime = 0;

const float maxIntegral = 10.0;  // Maximum value for the integral term
const float maxOutput = 200;   // Maximum pulse width.this will be the output of the servo angles in degrees. so that means that we ask the question, what output from the PID controller will correspond to the maximum and minimum angles of the servo
const float minOutput = -200;  // Minimum pulse width

void setup() {
  Serial.begin(115200);
  while (!Serial) 
  delay(10);  

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  filter.begin(mpu);

  // Allow MPU6050 to stabilize
  Serial.println("MPU6050 initialized. Stabilizing...");
  delay(500); 

  filter.calibrateGyro(mpu);

  lastTime = millis(); /* this initializes the start time of the program*/
}

void loop() {

  /* time step*/
  unsigned long currentTime = millis();         /* gets the current time in microseconds*/
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime; /*lastTime stores the previous currentTime*/

  /* this is to make sure that there's no division by 0 error*/
  if (dt <= 0.0001)
  {
    return; /*ignore this iteration of the loop*/
  }

  /* get roll and pitch angles from the comp filter*/
  float roll, pitch;
  filter.update(mpu, roll, pitch);


  /* Calculating roll and pitch error*/
  /* error = setpoint (desired output of the plant) - feedback (sensor readings (current roll and pitch))*/
  rollError = (rollSetPoint - roll);
  pitchError = (pitchSetPoint - pitch);


  /* Integral Calculation - keeps running sum of the error overtime (area under the curve)*/
  rollErrorSum += (rollError * dt);
  pitchErrorSum += (pitchError * dt);


  /* Accounting for Integral windup (clamping the integral term)*/
  rollErrorSum = constrain(rollErrorSum, -maxIntegral, maxIntegral);
  pitchErrorSum = constrain(pitchErrorSum, -maxIntegral, maxIntegral);


  /* Derivative Calculation - rate of change of the error with time*/
  /* Current error (roll and pitch error) - previous error / dt*/
  float rollErrorRate = (rollError - previousRollError) / dt;
  float pitchErrorRate = (pitchError - previousPitchError) / dt;


  /* PID outputs*/
  float rollOutput = ((Kp * rollError) + (Ki * rollErrorSum) + (Kd * rollErrorRate));
  float pitchOutput = ((Kp * pitchError) + (Ki * pitchErrorSum) + (Kd * pitchErrorRate));


  /* Account for saturation - Disable Integral accumulation if the system is in saturation*/
  if (rollOutput >= maxOutput || rollOutput <= minOutput) {
    rollErrorSum = 0;  // Reset the integral term if the output is saturated
  }

  if (pitchOutput >= maxOutput || pitchOutput <= minOutput) {
    pitchErrorSum = 0;  // Reset the integral term if the output is saturated
  }


  /* the current rollError will then be used as previousRollError for the next cycle
  this is important and used for the calculation of the Derivative term*/
  previousRollError = rollError; 
  previousPitchError = pitchError;

  
  Serial.print("roll: ");
  Serial.print(rollOutput);
  Serial.print(" pitch: ");
  Serial.println(pitchOutput); 
}