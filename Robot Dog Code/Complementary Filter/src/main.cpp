#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <iostream>
#include <cmath>

using namespace std;

Adafruit_MPU6050 mpu;
const float Alpha = 0.1;

/* Time tracking variables */
unsigned long previous_time = 0; // previous timestamp in microseconds
float dt = 0.0;                  // time step in seconds

/* Bias variables (to be calibrated manually or computed) */
float gyro_x_bias = 0.0;
float gyro_y_bias = 0.0;
float gyro_z_bias = 0.0;

/* Filtered roll and pitch estimates */
float filteredRoll = 0.0; // phi_hat
float filteredPitch = 0.0; // theta_hat

// Function prototype for calibrateGyro
void calibrateGyro();

void setup() {
    Serial.begin(9600);

    while (!Serial)
        delay(10);
    Serial.println("Adafruit MPU6050 test!");

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 Found!");

    /* MPU6050 configurations */
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setCycleRate(MPU6050_CYCLE_20_HZ);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    /* Time tracking initialization */
    previous_time = micros();

    /* Calibrate gyro biases (stationary IMU required for this step) */
    calibrateGyro();
}

void loop() {
    /* Get the current time */
    unsigned long current_time = micros();

    /* Calculate the time step in seconds */
    dt = (current_time - previous_time) / 1000000.0;

    /* Update the previous time */
    previous_time = current_time;

    /* Get sensor readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Calculate angles from accelerometer (phi_hat_acc, theta_hat_acc) */
    float rollAccel = atan2(a.acceleration.y, a.acceleration.z); // Roll (phi_hat_acc)
    float pitchAccel = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)); // Pitch (theta_hat_acc)

    /* Correct gyro data for biases */
    float p = g.gyro.x - gyro_x_bias; // Roll rate
    float q = g.gyro.y - gyro_y_bias; // Pitch rate
    float r = g.gyro.z - gyro_z_bias; // Yaw rate (not used in this example)

    /* Calculate roll and pitch rate (phi_dot, theta_dot) */
    float rollRate = p + sin(filteredRoll) * tan(filteredPitch) * q + cos(filteredRoll) * tan(filteredPitch) * r; // phi_dot
    float pitchRate = cos(filteredRoll) * q - sin(filteredRoll) * r; // theta_dot

    /* Update roll and pitch using complementary filter */
    filteredRoll = (1 - Alpha) * (filteredRoll + rollRate * dt) + Alpha * rollAccel;
    filteredPitch = (1 - Alpha) * (filteredPitch + pitchRate * dt) + Alpha * pitchAccel;

    /* Convert to degrees for output */
    float rollDegrees = filteredRoll * RAD_TO_DEG;
    float pitchDegrees = filteredPitch * RAD_TO_DEG;


    Serial.print("Roll: ");
    Serial.print(rollDegrees);
    Serial.print(" Pitch: ");
    Serial.print(pitchDegrees);
    Serial.println("");
}

/* Function to calibrate gyroscope biases */
void calibrateGyro() {
    Serial.println("Calibrating gyro biases...");
    int numSamples = 200;
    float xSum = 0, ySum = 0, zSum = 0;

    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        xSum += g.gyro.x;
        ySum += g.gyro.y;
        zSum += g.gyro.z;

        delay(10); // Small delay between samples
    }

    gyro_x_bias = xSum / numSamples;
    gyro_y_bias = ySum / numSamples;
    gyro_z_bias = zSum / numSamples;

    Serial.println("Gyro biases calibrated:");
    Serial.print("X: ");
    Serial.println(gyro_x_bias);
    Serial.print("Y: ");
    Serial.println(gyro_y_bias);
    Serial.print("Z: ");
    Serial.println(gyro_z_bias);
}
