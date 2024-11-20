#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <iostream>
#include <cmath>

using namespace std;

Adafruit_MPU6050 mpu;
float gravity = 9.80665; // in m/s^2
const float Alpha = 0.05;

/*time tracking variables*/
unsigned long previous_time = 0; //previous timestamp in microseconds
float dt = 0.0;             // time step in seconds

/*initalizing roll and pitch*/
float gyroRoll_rad_s = 0.0;
float gyroPitch_rad_s = 0.0;
float roll_radians = 0.0;       // from acc
float pitch_radians = 0.0;      // from acc

float rollFromGyro_degrees = 0.0;
float pitchFromGyro_degrees = 0.0;

/*complimentary filter estimates*/
float filteredRoll = 0.0;
float filteredPitch = 0.0;


void setup() {
    Serial.begin(9600);

    while (!Serial)
    delay(10); 
    Serial.println("Adafruit MPU6050 test!");

    if (!mpu.begin()){
        Serial.println ("Failed to find MPU6050 chip");
    }
    Serial.println("MPU6050 Found!");

    /*configure accelerometer sensitivity*/
    mpu6050_accel_range_t accSensitivity = MPU6050_RANGE_2_G;
    mpu.setAccelerometerRange(accSensitivity); 

    Serial.print("Accelerometer Sensitivity is: ");
    switch (accSensitivity)
    {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2g");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4g");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16");
        break;
    default:
        Serial.println("UNKNOWN");
    }

    /*configure accelerometer sampling rate and low power mode*/
    mpu.enableCycle(false);         // setting this to true does not allow the gyro to take measurement
    mpu6050_cycle_rate_t sampleRate = MPU6050_CYCLE_20_HZ;
    mpu.setCycleRate(sampleRate);

    Serial.print("The sample rate is: ");
    switch (sampleRate)
    {
    case MPU6050_CYCLE_1_25_HZ:
        Serial.println("1.25Hz");
        break;
    case MPU6050_CYCLE_5_HZ:
        Serial.println("5Hz");
        break;
    case MPU6050_CYCLE_20_HZ:
        Serial.println("20Hz");
        break;
    case MPU6050_CYCLE_40_HZ:
        Serial.println("40Hz");
        break;
    default:
        Serial.println("UNKNOWN");
    }

    /*configure LPF filter bandwith*/
    mpu6050_bandwidth_t LPF_cutoff = MPU6050_BAND_5_HZ; //filters anything above 5.This value seems to be much more stable than 10Hz
    mpu.setFilterBandwidth(LPF_cutoff);

    Serial.print("LowpPass Filter cutoff set to: ");
    switch (LPF_cutoff)
    {
    case MPU6050_BAND_5_HZ:
        Serial.println("5Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94Hz");
        break;

    default:
        Serial.println("UNKNOWN");
    }

    /*configure the gyro range*/
    mpu6050_gyro_range_t gyroRange = MPU6050_RANGE_250_DEG;
    mpu.setGyroRange(gyroRange); 
    Serial.print("Gyro Range set to: ");
    switch (gyroRange) {
        case MPU6050_RANGE_250_DEG: Serial.println("±250°/s"); break;
        case MPU6050_RANGE_500_DEG: Serial.println("±500°/s"); break;
        case MPU6050_RANGE_1000_DEG: Serial.println("±1000°/s"); break;
        case MPU6050_RANGE_2000_DEG: Serial.println("±2000°/s"); break;
        default: Serial.println("UNKNOWN");
    }

    /*configure the gyro band pass filter bandwidth*/
    mpu6050_highpass_t HPF_cutoff = MPU6050_HIGHPASS_5_HZ;
    mpu.setHighPassFilter(HPF_cutoff);

    Serial.print("High-Pass Filter cutoff set to: ");
    switch (HPF_cutoff) {
        case MPU6050_HIGHPASS_HOLD: Serial.println("HOLD (No HPF)"); break;
        case MPU6050_HIGHPASS_5_HZ: Serial.println("5 Hz"); break;
        case MPU6050_HIGHPASS_2_5_HZ: Serial.println("2.5 Hz"); break;
        case MPU6050_HIGHPASS_1_25_HZ: Serial.println("1.25 Hz"); break;
        case MPU6050_HIGHPASS_0_63_HZ: Serial.println("0.63 Hz"); break;
        default: Serial.println("UNKNOWN");
    }

    /*there is no need to configure the DLPF for the gyro as this has been
    done for the acc as well and they both share the same DLPF*/

    /*Initialize time tracking*/
    previous_time = micros();
}

void loop() {

    /*get the current time*/
    unsigned long current_time = micros();

    /*calculate the time step in seconds*/
    dt = (current_time - previous_time) / 1000000.0;

    /*update the previous time*/
    previous_time = current_time;

    /*getting sensor reading*/
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /*accelerometer roll calculation w / division by zero check*/
    if (a.acceleration.z == 0) {
        Serial.println ("Error: Division by zero in roll calculation");
        return; // no calculations done
    }
    roll_radians = atan(a.acceleration.y / a.acceleration.z);
    float rollFromAccel = roll_radians * RAD_TO_DEG;

    /*accelerometer pitch calculation with division by zero check*/
    if (a.acceleration.y == 0 && a.acceleration.z == 0) {
        Serial.println("Error: Invalid denominator in pitch calculation.");
        return;
    }
    pitch_radians = atan (-a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2)));
    float pitchFromAceel = pitch_radians * RAD_TO_DEG;

    /*transform body rates to Euler rates*/
    float eulerRateRoll = g.gyro.x + sin(filteredRoll) * tan(filteredPitch) * g.gyro.y + cos(filteredRoll) * tan(filteredPitch) * g.gyro.z;
    float eulerRatePitch = cos(filteredRoll) * g.gyro.y - sin(filteredRoll) * g.gyro.z;



    /*gyro roll calculations*/
    gyroRoll_rad_s += (eulerRateRoll * dt);

    rollFromGyro_degrees = gyroRoll_rad_s * RAD_TO_DEG;         // this is now the new gyro estimate integrate

    /*gyro pitch calculations*/
    gyroPitch_rad_s += (eulerRatePitch * dt);      
    
    pitchFromGyro_degrees = gyroPitch_rad_s * RAD_TO_DEG;

    /*complementary filter for roll and pitch*/
    filteredRoll = (1 - Alpha) * (filteredRoll + rollFromGyro_degrees) + Alpha * rollFromAccel;
    filteredPitch = (1 - Alpha) * (filteredPitch + pitchFromGyro_degrees) + Alpha * pitchFromGyro_degrees;

    Serial.print("Roll: ");
    Serial.print(filteredRoll);
    Serial.print(" Pitch: ");
    Serial.print(filteredPitch);
    Serial.println("");

    /*Serial.print("Delta t: ");
    Serial.print(dt, 6); // Print dt with 6 decimal places
    Serial.println(" seconds");*/
}


// function definitions