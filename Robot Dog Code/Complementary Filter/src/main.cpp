#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <ComplementaryFilter.h>
/*this is the comp filter code*/

Adafruit_MPU6050 mpu;
ComplementaryFilter filter;

void setup() {
    Serial.begin(9600);
    while (!Serial) 
    delay(10);

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }

    filter.begin(mpu);
    filter.calibrateGyro(mpu);
}

void loop() {
    float rollDegrees, pitchDegrees;
    filter.update(mpu, rollDegrees, pitchDegrees);

    Serial.print("Roll: ");
    Serial.print(rollDegrees);
    Serial.print(" Pitch: ");
    Serial.println(pitchDegrees);
}
