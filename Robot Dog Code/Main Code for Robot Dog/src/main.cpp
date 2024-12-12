#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <iostream>
#include <cmath>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.H>
#include <ComplementaryFilter.h>
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

using namespace std;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// our servo # counter
uint8_t servonum = 0;



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
    Serial.println("Adafruit MPU6050 test!");

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 Found!");

    /* MPU6050 configurations */
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setCycleRate(MPU6050_CYCLE_20_HZ);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);  /* digital low pass filter */
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    /* Time tracking initialization */
    float previous_time;
    previous_time = micros();

    /* Calibrate gyro biases (stationary IMU required for this step) */
    filter.calibrateGyro();

    pwm.begin();
    pwm.setOscillatorFrequency(25300000); // Analog servos run at ~50 Hz updates
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    delay(10);
}
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  


  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~50 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

const int pmw = 300;

int convertRange(int originalValue, int originalMin, int originalMax, int newMin, int newMax) { // funtion to convert ranges of numbers
    // Make sure the originalValue is within the original range
    originalValue = std::max(originalMin, std::min(originalMax, originalValue));

    // Scale the original value to the new range
    double scaleFactor = static_cast<double>(newMax - newMin) / (originalMax - originalMin);
    return static_cast<int>((originalValue - originalMin) * scaleFactor + newMin);
}




float BL_1[100] = {138.051860 ,138.752449 ,139.631657 ,140.486941 ,141.304297 ,142.069988 ,142.770454 ,143.392458 
,143.923188 ,144.350336 ,144.662169 ,144.847619 ,144.896388 ,144.799084 ,144.547392 ,144.134268 ,143.554160 ,142.803244 
,141.879647 ,140.783666 ,139.517926 ,138.087486 ,136.499857 ,134.764920 ,132.894741 ,130.903286 ,128.806053 ,126.619645 
,124.361315 ,122.048518 ,119.698503 ,117.327969 ,114.952797 ,112.587877 ,110.247013 ,107.942912 ,105.687236 ,103.490706 
,101.363246 ,99.314145 ,97.352238 ,95.486083 ,93.724139 ,92.074923 ,90.547145 ,89.149811 ,87.892270 ,86.784198 ,85.835491 
,85.056036 ,84.455356 ,84.042117 ,83.823517 ,83.804607 ,83.987651 ,84.371624 ,84.951957 ,85.720596 ,86.666358 ,87.775525 
,89.032543 ,90.420737 ,91.922926 ,93.521908 ,95.200804 ,96.943268 ,98.733608 ,100.556838 ,102.398695 ,104.245646 ,106.084895 
,107.904407 ,109.692948 ,111.440140 ,113.136542 ,114.773738 ,116.344435 ,117.842573 ,119.263425 ,120.603699 ,121.861632 ,
123.037070 ,124.131539 ,125.148290 ,126.092320 ,126.970364 ,127.790825 ,128.563633 ,129.300014 ,130.012116 ,130.712499 ,
131.413476 ,132.126329 ,132.860484 ,133.622747 ,134.416722 ,135.242508 ,136.096726 ,136.972840 ,137.861692 };

float BR_1[100] = {84.814601 ,84.224964 ,83.908114 ,83.788887 ,83.870868 ,84.154723 ,84.637670 ,85.313370 ,86.172143 
,87.201454 ,88.386576 ,89.711312 ,91.158661 ,92.711383 ,94.352402 ,96.065087 ,97.833407 ,99.642021 ,101.476301 ,
103.322351 ,105.167003 ,106.997840 ,108.803219 ,110.572326 ,112.295241 ,113.963024 ,115.567808 ,117.102902 ,118.562901 
,119.943781 ,121.243006 ,122.459607 ,123.594264 ,124.649358 ,125.629014 ,126.539103 ,127.387205 ,128.182511 ,128.935635 
,129.658317 ,130.362988 ,131.062175 ,131.767776 ,132.490249 ,133.237797 ,134.015681 ,134.825771 ,135.666403 ,136.532564 
,137.416332 ,138.307471 ,139.194080 ,140.063180 ,140.901220 ,141.694454 ,142.429211 ,143.092076 ,143.670014 ,144.150457 
,144.521375 ,144.771355 ,144.889699 ,144.866542 ,144.693009 ,144.361393 ,143.865369 ,143.200218 ,142.363065 ,141.353097 
,140.171757 ,138.822881 ,137.312756 ,135.650096 ,133.845903 ,131.913235 ,129.866875 ,127.722928 ,125.498379 ,123.210637 ,
120.877102 ,118.514786 ,116.140007 ,113.768167 ,111.413619 ,109.089615 ,106.808330 ,104.580941 ,102.417749 ,100.328340 ,
98.321753 ,96.406662 ,94.591556 ,92.884902 ,91.295299 ,89.831596 ,88.502968 ,87.318935 ,86.289303 ,85.423998 ,84.732780 };

 float FL_1[100] = {113.628200 ,115.173049 ,116.725981 ,118.205186 ,119.606143 ,120.925936 ,122.163195 ,123.318181 ,124.392848 
 ,125.390881 ,126.317717 ,127.180515 ,127.988076 ,128.750682 ,129.479828 ,130.187819 ,130.887223 ,131.590170 ,132.307554 ,133.048215 
 ,133.818203 ,134.620260 ,135.453602 ,136.314019 ,137.194265 ,138.084624 ,138.973549 ,139.848281 ,140.695373 ,141.501097 ,142.251743 
 ,142.933819 ,143.534187 ,144.040154 ,144.439549 ,144.720797 ,144.873009 ,144.886101 ,144.750933 ,144.459489 ,144.005076 ,143.382553 
 ,142.588560 ,141.621751 ,140.482990 ,139.175505 ,137.704969 ,136.079497 ,134.309538 ,132.407664 ,130.388261 ,128.267145 ,126.061124 
 ,123.787542 ,121.463839 ,119.107157 ,116.734011 ,114.360048 ,111.999891 ,109.667069 ,107.374020 ,105.132158 ,102.951993 ,100.843273 
 ,98.815156 ,96.876391 ,95.035497 ,93.300931 ,91.681248 ,90.185223 ,88.821947 ,87.600855 ,86.531693 ,85.624378 ,84.888748 ,84.334162 
 ,83.968981 ,83.799928 ,83.831416 ,84.064924 ,84.498555 ,85.126850 ,85.940929 ,86.928913 ,88.076556 ,89.367961 ,90.786272 ,92.314264 
 ,93.934789 ,95.631085 ,97.386962 ,99.186902 ,101.016098 ,102.860468 ,104.706663 ,106.542076 ,108.354867 ,110.134008 ,111.869349 ,113.551697 };

   float FR_1[100] = {131.391705 ,129.339342 ,127.173671 ,124.931685 ,122.630873 ,120.288570 ,117.921641 ,115.546192 ,113.177372 ,110.829261 
   ,108.514836 ,106.246014 ,104.033739 ,101.888118 ,99.818585 ,97.834069 ,95.943182 ,94.154392 ,92.476187 ,90.917218 ,89.486410 ,88.193025 ,
   87.046664 ,86.057184 ,85.234501 ,84.588271 ,84.127429 ,83.859601 ,83.790437 ,83.922953 ,84.256982 ,84.788848 ,85.511345 ,86.414026 ,
   87.483740 ,88.705327 ,90.062327 ,91.537636 ,93.114019 ,94.774492 ,96.502562 ,98.282367 ,100.098740 ,101.937237 ,103.784143 ,105.626476
    ,107.452008 ,109.249299 ,111.007752 ,112.717682 ,114.370408 ,115.958349 ,117.475124 ,118.915663 ,120.276308 ,121.554904 ,122.750890 
    ,123.865365 ,124.901145 ,125.862791 ,126.756607 ,127.590589 ,128.374304 ,129.118681 ,129.835681 ,130.537817 ,131.237534 ,131.946447 
    ,132.674525 ,133.429290 ,134.215189 ,135.033207 ,135.880814 ,136.752221 ,137.638867 ,138.530041 ,139.413519 ,140.276138 ,141.104266 
    ,141.884155 ,142.602183 ,143.245021 ,143.799745 ,144.253912 ,144.595639 ,144.813683 ,144.897540 ,144.837579 ,144.625198 ,144.253014 
    ,143.715083 ,143.007124 ,142.126758 ,141.073720 ,139.850041 ,138.460167 ,136.910999 ,135.211839 ,133.374230 ,131.411694 };





  float BL_2[100] = {46.892634 ,45.820415 ,44.077030 ,42.192670 ,40.191800 ,38.097966 ,35.934251 ,33.723067 ,31.486075 ,
  29.244183 ,27.017592 ,24.825857 ,22.687952 ,20.622305 ,18.646813 ,16.778821 ,15.035053 ,13.431506 ,11.983298 ,10.704484 
  ,9.607837 ,8.704613 ,8.004311 ,7.514450 ,7.240372 ,7.185099 ,7.349241 ,7.730985 ,8.326143 ,9.128269 ,10.128834 ,11.317438 
  ,12.682041 ,14.209207 ,15.884325 ,17.691821 ,19.615320 ,21.637779 ,23.741575 ,25.908544 ,28.119992 ,30.356663 ,32.598690 
  ,34.825527 ,37.015897 ,39.147761 ,41.198356 ,43.144336 ,44.962059 ,46.628085 ,48.119911 ,49.416961 ,50.501799 ,51.361441 
  ,51.988599 ,52.382622 ,52.549935 ,52.503844 ,52.263725 ,51.853747 ,51.301359 ,50.635763 ,49.886572 ,49.082729 ,48.251735 
  ,47.419130 ,46.608192 ,45.839786 ,45.132303 ,44.501667 ,43.961356 ,43.522443 ,43.193632 ,42.981282 ,42.889432 ,42.919813 
  ,43.071854 ,43.342679 ,43.727098 ,44.217592 ,44.804284 ,45.474913 ,46.214789 ,47.006766 ,47.831222 ,48.666083 ,49.486919 
  ,50.267168 ,50.978534 ,51.591623 ,52.076877 ,52.405780 ,52.552316 ,52.494517 ,52.215884 ,51.706464 ,50.963361 ,49.990612 ,48.798474 ,47.402308 };

  float BR_2[100] = {47.208489 ,48.793619 ,49.986837 ,50.960372 ,51.704285 ,52.214517 ,52.493940 ,52.552483 ,52.406622 ,52.078311 ,51.593553 
  ,50.980857 ,50.269782 ,49.489722 ,48.668979 ,47.834123 ,47.009591 ,46.217465 ,45.477376 ,44.806480 ,44.219472 ,43.728626 ,43.343823 ,43.072593
   ,42.920132 ,42.889325 ,42.980752 ,43.192688 ,43.521104 ,43.959647 ,44.499623 ,45.129967 ,45.837208 ,46.605433 ,47.416257 ,48.248825 ,49.079868
    ,49.883851 ,50.633282 ,51.299219 ,51.852052 ,52.262575 ,52.503330 ,52.550133 ,52.383589 ,51.990370 ,51.364027 ,50.505185 ,49.421111 ,48.124771
     ,46.633589 ,44.968130 ,43.150894 ,41.205320 ,39.155047 ,37.023425 ,34.833220 ,32.606473 ,30.364463 ,28.127738 ,25.916169 ,23.749012 ,21.644965
      ,19.622192 ,17.698319 ,15.890391 ,14.214784 ,12.687079 ,11.321886 ,10.132648 ,9.131410 ,8.328578 ,7.732689 ,7.350194 ,7.185290 ,7.239800 ,7.513120 
      ,8.002239 ,8.701821 ,9.604355 ,10.700348 ,11.978549 ,13.426192 ,15.029225 ,16.772532 ,18.640121 ,20.615268 ,22.680633 ,24.818318 ,27.009898 ,29.236401 
      ,31.478275 ,33.715319 ,35.926630 ,38.090549 ,40.184665 ,42.185898 ,44.070707 ,45.815473 ,47.397117 };
  
  float FL_2[100] = {42.642928 ,42.946054 ,43.128586 ,43.428388 ,43.840122 ,44.355684 ,44.964604 ,45.654017 ,46.408623 ,47.210656 ,48.039877 ,
  48.873610 ,49.686866 ,50.452603 ,51.142180 ,51.726064 ,52.174831 ,52.460450 ,52.557771 ,52.446068 ,52.110400 ,51.542553 ,50.741401 ,49.712623 
  ,48.467867 ,47.023568 ,45.399626 ,43.618159 ,41.702450 ,39.676145 ,37.562701 ,35.385046 ,33.165409 ,30.925260 ,28.685326 ,26.465639 ,24.285606 
  ,22.164058 ,20.119292 ,18.169073 ,16.330595 ,14.620414 ,13.054322 ,11.647191 ,10.412777 ,9.363495 ,8.510185 ,7.861870 ,7.425542 ,7.205975 ,7.205594 
  ,7.424401 ,7.859982 ,8.507570 ,9.360182 ,10.408800 ,11.642591 ,13.049145 ,14.614709 ,16.324416 ,18.162476 ,20.112336 ,22.156804 ,24.278116 ,26.457978 
  ,28.677560 ,30.917458 ,33.157641 ,35.377386 ,37.555225 ,39.668932 ,41.695580 ,43.611715 ,45.393689 ,47.018218 ,48.463178 ,49.708659 ,50.738213 ,51.540170
   ,52.108832 ,52.445298 ,52.557758 ,52.461131 ,52.176126 ,51.727879 ,51.144414 ,50.455153 ,49.689631 ,48.876492 ,48.042785 ,47.213507 ,46.411342 ,45.656539 
   ,44.966872 ,44.357647 ,43.841741 ,43.429630 ,43.129429 ,42.946915 ,42.885551 };
  
  float FR_2[100] = {7.086734 ,7.287545 ,7.615338 ,8.157661 ,8.908767 ,9.860628 ,11.003291 ,12.325103 ,13.812956 ,15.452512 ,17.228420 ,19.124492 ,21.123842 ,
  23.208985 ,25.361892 ,27.564003 ,29.796206 ,32.038793 ,34.271392 ,36.472915 ,38.621513 ,40.694595 ,42.668940 ,44.520944 ,46.227055 ,47.764454 ,49.111978 ,
  50.251283 ,51.168149 ,51.853760 ,52.305742 ,52.528745 ,52.534407 ,52.340691 ,51.970706 ,51.451226 ,50.811152 ,50.080104 ,49.287279 ,48.460595 ,47.626118 ,
  46.807712 ,46.026852 ,45.302549 ,44.651343 ,44.087319 ,43.622149 ,43.265125 ,43.023191 ,42.900963 ,42.900750 ,43.022556 ,43.264081 ,43.620715 ,44.085523 ,
  44.649222 ,45.300148 ,46.024223 ,46.804918 ,47.623229 ,48.457689 ,49.284444 ,50.077434 ,50.808746 ,51.449188 ,51.969138 ,52.339692 ,52.534064 ,52.529131 ,
  52.306908 ,51.855735 ,51.170937 ,50.254864 ,49.116312 ,47.769482 ,46.232708 ,44.527144 ,42.675608 ,40.701647 ,38.628867 ,36.480493 ,34.279115 ,32.046586 ,
  29.803999 ,27.571725 ,25.369476 ,23.216365 ,21.130954 ,19.131276 ,17.234815 ,15.458461 ,13.818403 ,12.329998 ,11.007584 ,9.864277 ,8.911734 ,8.159916 ,
  7.616855 ,7.288456 ,7.178329};

void loop() {

   for (int i = 0; i < 100; i++) {

  int pwm13 =  convertRange((BL_1[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet
  int pwm9 =  convertRange((180-BR_1[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet
  int pwm5 =  convertRange((FL_1[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet
  int pwm1 =  convertRange((180-FR_1[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet
  
  int pwm14 =  convertRange((180-BL_2[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet
  int pwm10 =  convertRange((BR_2[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet
  int pwm6 =  convertRange((180-FL_2[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet
  int pwm2 =  convertRange((FR_2[i]), 0, 180, 150, 450);       // convert the new converted potentiometer1 reading to a x cordanet


    pwm.setPWM(0, 0, pmw-20); 
    pwm.setPWM(1, 0, pwm1); 
    pwm.setPWM(2, 0, pwm2); 

    pwm.setPWM(4, 0, pmw+20); 
    pwm.setPWM(5, 0, pwm5); 
    pwm.setPWM(6, 0, pwm6 +10);

    pwm.setPWM(8, 0,  pmw+20); 
    pwm.setPWM(9, 0, pwm9); 
    pwm.setPWM(10, 0, pwm10); 

    pwm.setPWM(12, 0, pmw-20); 
    pwm.setPWM(13, 0, pwm13); 
    pwm.setPWM(14, 0, pwm14); 
    delay(100);
    

    /*this section is for the PID controller. Note that the code for the complimentary filter is embedded within 
    the code for the PID controller because the PID controller usese the complimentary filter as the feedback.
    I made this into it's now calss and made it a library.*/
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
  };
}


