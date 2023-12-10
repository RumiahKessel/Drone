#include "Wire.h"
#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>
#include <esp_timer.h>
#include <math.h>

#define G 9.81
/* ================================================================*/

/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low accelerometer and
magnetometer data registers. They can be converted to units of g and
gauss using the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).

Example: An LSM303D gives a magnetometer X axis reading of 1982 with
its default full scale setting of +/- 4 gauss. The M_GN specification
in the LSM303D datasheet (page 10) states a conversion factor of 0.160
mgauss/LSB (least significant bit) at this FS setting, so the raw
reading of -1982 corresponds to 1982 * 0.160 = 317.1 mgauss =
0.3171 gauss.

In the LSM303DLHC, LSM303DLM, and LSM303DLH, the acceleration data
registers actually contain a left-aligned 12-bit number, so the lowest
4 bits are always 0, and the values should be shifted right by 4 bits
(divided by 16) to be consistent with the conversion factors specified
in the datasheets.

Example: An LSM303DLH gives an accelerometer Z axis reading of -16144
with its default full scale setting of +/- 2 g. Dropping the lowest 4
bits gives a 12-bit raw value of -1009. The LA_So specification in the
LSM303DLH datasheet (page 11) states a conversion factor of 1 mg/digit
at this FS setting, so the value of -1009 corresponds to -1009 * 1 =
1009 mg = 1.009 g.
*/
#define LSM303D_SDA_PIN 21
#define LSM303D_SCL_PIN 22
#define SENS_A_REF      4096
#define TEMP_REF.       25

LSM303 compass;
vector* prev_a;

//Accelerometer
float SENS_A;


//Magnetometer
float SENS_M;

/*=====================================================================*/

/*=====================================================================*/
/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/

#define DEG_TO_RAD (M_PI/180.0)
#define RAD_TO_DEG (180.0/M_PI)
#define CONVERSION_G 8.75 // correlational to sensitivity -> refer to datasheet
L3G gyro;

vector* prev_g;

/*=====================================================================*/

LPS barometer;


float base_alt;


typedef struct vector
{
  float x, y, z;
} vector;


//Our calibration of magnetometer
float B[3] = { 9955.15, -7948.26, 8511.80};
float Ainv[3][3] = {{  0.25060,  0.02942, -0.02955},
  {  0.02942,  0.31692,  0.00789},
  { -0.02955,  0.00789,  0.30592}
};

//Low-pass filter coefficients that need to be tested
float lpfGyr = 0.7f; 
float lpfAcc = 0.9;
float lpfMag = 0.4;
float lpfVa = 0.7;

float x[7]; //state estimate vector

float p, q, r;

float P[4];
float Q[2];

char report[200];

void setup() {
  Serial.begin(115200);
  Wire.begin(LSM303D_SDA_PIN, LSM303D_SCL_PIN);
  if (!compass.init()) {
    Serial.println("failed to detect compass");
    while(1);
  }
  compass.enableDefault();
  // compass.writeReg(LSM303::CTRL2, 0x18); //change to +-8g check if we need higher
  compass.writeReg(LSM303::CTRL2, 0x20); //+/16g

  //initialize accel variables
  prev_a->x = 0.0f; prev_a->y = 0.0f; prev_a->z = 0.0f;

  // compass.writeReg(LSM303::CTRL6, 0x40); 
  compass.writeReg(LSM303::CTRL6, 0x60); 


  if (!gyro.init()) {
    Serial.println("failed to detect gyro");
    while(1);
  }

  gyro.enableDefault();
  // gyro.writeReg(L3G::CTRL4, 0x20); //change the sensitivity of gyro to 2000dps

  //initialize gyro variables
  prev_g->x = 0.0f; prev_g->y = 0.0f; prev_g->z = 0.0f;

  if (!barometer.init()) {
    Serial.println("failed to detect barometer");
    while(1);
  }
  barometer.enableDefault();

  calibrate();
}

void initializeEKF(float Pinit, float* Q, float* R) {
  
  p = 0.0f; q = 0.0f; r = 0.0f;


  //Initialize roll and pitch with sitting measurements
  vector* a;
  read_accel(a);
  roll = atan2(a->y, a->z);
  pitch = atan2(a->x, sqrt(pow(a->x, 2), pow(a->y, 2), pow(a->z, 2)));

  //initialize covariance matrix: P
  P[0] = Pinit; P[1] = 0.0f;
  P[2] = 0.0f;  P[3] = Pinit;
  
  
  //initialize noise matrix: Q
  Q[0] = 0; Q[1] = 0;

  //initialize measurement noise matrix: r
  R[0] = 0; R[1] = 1; R[2] = 0;
  
}

void predictEKF() {
  vector* g;
  read_gyro(g);

  p = g->x;
  q = g->y;
  r = g->z;

  vector* a;
  read_accel(a);
  float ax = a->x;
  float ay = a->y;
  float az = a->z;


  /* Compute common trig terms*/
  float sin_phi = sin(roll); float cos_phi = cos(roll);

  /* tan(theta) is undefined for theta=90deg */
  if (fabs(pitch) > 1.57952297305f || fabs(pitch) < 1.56206968053f) {
		return;
	}

  /* Integegrate to get new state estimates (Euler Method)
      x+= x + T*f(x,u)*/
  roll = roll + T* (p + tan(pitch) * (q * sin_phi + r * cos_phi));
  pitch = pitch + T * (q * cos_phi - r * sin_phi);
  

  if (fabs(pitch) > 1.57952297305f || fabs(pitch) < 1.56206968053f) {
		return;
	}

  /* Compute common trig terms*/
  sin_phi = sin(roll); cos_phi = cos(roll);
  float sin_theta = sin(pitch); 
  float cos_theta = cos(pitch); 
  float tan_theta = sin_theta / cos_theta;

  /* Jacobian of f(x,u)*/
  A[4] = {tan_theta * (q * cos_phi - r * sin_phi), (r * cos_phi + q * sin_phi) * (tan_theta * tan_theta + 1.0f),
                     -(r * cos_phi + q * sin_phi),  0.0f};

  /*Update covariance matrix P[n + 1] = P[n] + T * (A * P[n] + P[n]*A^T + Q) */
  float Ptmp[4] = { T*(Q[0]      + 2.0f*A[0]*P[0] + A[1]*P[1] + A[1]*P[2]), T*(A[0]*P[1] + A[2]*P[0] + A[1]*P[3] + A[3]*P[1]),
					  T*(A[0]*P[2] + A[2]*P[0]   + A[1]*P[3] + A[3]*P[2]),    T*(Q[1]      + A[2]*P[1] + A[2]*P[2] + 2.0f*A[3]*P[3]) };

  
  P[0] = P[0] + Ptmp[0]; P[1] = P[1] + Ptmp[1];
	P[2] = P[2] + Ptmp[2]; P[3] = P[3] + Ptmp[3];
} 

void updateEKF() {
  /* Output function h(x,u)*/
  float h[3] = { q * Va * sin_theta              + g * sin_theta, 
				   r * Va * cos_theta - p * Va * sin_theta - g * cos_theta * sin_phi,
				  -q * Va * cos_theta               - g * cos_theta * cos_phi };

/* Jacobian of h(x,u)*/
float C[6] = { 0.0f,         q * Va * cos_theta + g * cos_theta,
				  -g * cos_phi * cos_theta, -r * Va * sin_theta - p * Va * cos_theta + g * sin_phi * sin_theta,
				   g * sin_phi * cos_theta, (q * Va + g * cos_phi) * sin_theta };


/* Kalman gain K = P * C' / (C * P * C' + R) */
float G[9] = { P[3]*C[1]*C[1] + R[0],        C[1]*C[2]*P[2] + C[1]*C[3]*P[3],                                                   C[1]*C[4]*P[2] + C[1]*C[5]*P[3],
          C[1]*(C[2]*P[1] + C[3]*P[3]), R[1] + C[2]*(C[2]*P[0] + C[3]*P[2]) + C[3]*(C[2]*P[1] + C[3]*P[3]), C[4]*(C[2]*P[0] + C[3]*P[2]) + C[5]*(C[2]*P[1] + C[3]*P[3]),
                C[1]*(C[4]*P[1] + C[5]*P[3]), C[2]*(C[4]*P[0] + C[5]*P[2]) + C[3]*(C[4]*P[1] + C[5]*P[3]),             R[2] + C[4]*(C[4]*P[0] + C[5]*P[2]) + C[5]*(C[4]*P[1] + C[5]*P[3]) };

float Gdetinv = 1.0f / (G[0]*G[4]*G[8] - G[0]*G[5]*G[7] - G[1]*G[3]*G[8] + G[1]*G[5]*G[6] + G[2]*G[3]*G[7] - G[2]*G[4]*G[6]);

float Ginv[9] = { Gdetinv * (G[4]*G[8] - G[5]*G[7]), -Gdetinv * (G[1]*G[8] - G[2]*G[7]),  Gdetinv * (G[1]*G[5] - G[2]*G[4]), 
            -Gdetinv * (G[3]*G[8] - G[5]*G[6]),  Gdetinv * (G[0]*G[8] - G[2]*G[6]), -Gdetinv * (G[0]*G[5] - G[2]*G[3]),
                  Gdetinv * (G[3]*G[7] - G[4]*G[6]), -Gdetinv * (G[0]*G[7] - G[1]*G[6]),  Gdetinv * (G[0]*G[4] - G[1]*G[3]) };

float K[6] = { Ginv[3]*(C[2]*P[0] + C[3]*P[1]) + Ginv[6]*(C[4]*P[0] + C[5]*P[1]) + C[1]*Ginv[0]*P[1], Ginv[4]*(C[2]*P[0] + C[3]*P[1]) + Ginv[7]*(C[4]*P[0] + C[5]*P[1]) + C[1]*Ginv[1]*P[1], Ginv[5]*(C[2]*P[0] + C[3]*P[1]) + Ginv[8]*(C[4]*P[0] + C[5]*P[1]) + C[1]*Ginv[2]*P[1],
          Ginv[3]*(C[2]*P[2] + C[3]*P[3]) + Ginv[6]*(C[4]*P[2] + C[5]*P[3]) + C[1]*Ginv[0]*P[3], Ginv[4]*(C[2]*P[2] + C[3]*P[3]) + Ginv[7]*(C[4]*P[2] + C[5]*P[3]) + C[1]*Ginv[1]*P[3], Ginv[5]*(C[2]*P[2] + C[3]*P[3]) + Ginv[8]*(C[4]*P[2] + C[5]*P[3]) + C[1]*Ginv[2]*P[3] };

/* Update covariance matrix P++ = (I - K * C) * P+ */
Ptmp[0] = -P[2]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - P[0]*(C[2]*K[1] + C[4]*K[2] - 1.0f); Ptmp[1] = -P[3]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - P[1]*(C[2]*K[1] + C[4]*K[2] - 1.0f);
Ptmp[2] = -P[2]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f) - P[0]*(C[2]*K[4] + C[4]*K[5]); Ptmp[3] = -P[3]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f) - P[1]*(C[2]*K[4] + C[4]*K[5]);

P[0] = P[0] + Ptmp[0]; P[1] = P[1] + Ptmp[1];
P[2] = P[2] + Ptmp[2]; P[3] = P[3] + Ptmp[3];

/* Update state estimate x++ = x+ + K * (y - h) */
roll  = roll   + K[0] * (ax - h[0]) + K[1] * (ay - h[1]) + K[2] * (az - h[2]);
pitch = pitch + K[3] * (ax - h[0]) + K[4] * (ay - h[1]) + K[5] * (az - h[2]);  

}
void calibrate() {
  const int numReadings = 100;
  float sumPressure = 0.0;
  Serial.println("Calibrating...");
  delay(5000); 
  for (int i = 0; i < numReadings; ++i) {
    sumPressure += barometer.readPressureMillibars();
    delay(10); 
  }
  float base_pressure = sumPressure / numReadings;
  base_alt = barometer.pressureToAltitudeMeters(base_pressure);
  Serial.println("base alt: ");
  Serial.println(base_alt);
}

void calibrate_mag(vector* m) {
  static float x, y, z;
  compass.read();
  x = (float) compass.m.x - B_mag[0];
  y = (float) compass.m.y - B_mag[1];
  z = (float) compass.m.z - B_mag[2];
  m->x = Ainv_mag[0][0] * x + Ainv_mag[0][1] * y + Ainv_mag[0][2] * z;
  m->y = Ainv_mag[1][0] * x + Ainv_mag[1][1] * y + Ainv_mag[1][2] * z;
  m->z = Ainv_mag[2][0] * x + Ainv_mag[2][1] * y + Ainv_mag[2][2] * z;

  //maybe Axis remapping need to double check
  m->x = -1*m->y;
  m->y = -1*m->x;
  m->z = -1*m->z;

  //low pass filter mag
  low_pass_filter(m, prev_m, lpf_m);

}

/* Calibrate values, remap axis, put through low pass filter and convert */
void read_accel(vector* a) {
  static float x, y, z;
  x = (float) compass.a.x - B_accel[0];
  y = (float) compass.a.y - B_accel[1];
  z = (float) compass.a.z - B_accel[2];
  a->x = Ainv_accel[0][0] * x + Ainv_accel[0][1] * y + Ainv_accel[0][2] * z;
  a->y = Ainv_accel[1][0] * x + Ainv_accel[1][1] * y + Ainv_accel[1][2] * z;
  a->z = Ainv_accel[2][0] * x + Ainv_accel[2][1] * y + Ainv_accel[2][2] * z;

  //Axis remapping 
  a->x = -1*a->y;
  a->y = -1*a->x;
  a->z = -1*a->z;

  low_pass_filter(a, prev_a, lpf_a);

  //convert accel values from mg to m/s^2
  a->x = convert_accel(a->x);
  a->y = convert_accel(a->y);
  a->z = convert_accel(a->z);
}

/* Outputs gyro rad/sec */
void read_gyro(vector* g) {
  gyro.read();
  
  //calibrate step is missing

  // axis remapping
  g->x = -1 * gyro.g.y;
  g->y = -1 * gyro.g.x;
  g->z = -1 * gyro.g.z;

  low_pass_filter(g, prev_g, lpf_g);

  g->x = convert_gyro(g->x);
  g->y = convert_gyro(g->y);
  g->z = convert_gyro(g->z);
}

// raw value -> rad/sec
float convert_gyro(float num) {
  float dps = num * CONVERSION_G / 1000;
  return dps * DEG_TO_RAD; //returns rad/s
}


float convert_accel(float num) {
  float accel= num / SENS_A; //convert from mg to g
  accel *= 9.81; //Convert to m/s^2
  return accel;
}

void low_pass_filter(vector* v, vector* prev_v, float lpf_v) {
  //low pass filter accel (first-order low-pass filter)
  v->x = lf_alpha*prev_v->x + (1.0f - lpf_v) * v->x;
  v->y = lf_alpha*prev_v->y + (1.0f - lpf_v) * v->y;
  v->z = lf_alpha*prev_v->z + (1.0f - lpf_v) * v->z;

  //update prev data
  prev_v->x = v->x;
  prev_v->y = v->y;
  prev_v->z = v->z;
}



float true_heading(vector* m, float declination) {
  atan2(-m->y*cos())
}

void kalman_



// void read_gyro(vector* g) {
//   static float x, y, z;
//   gyro.read();
//   x = 
// }


// void calc_sensitivity(float alpha, float temp) {
//   SENS_A = SENS_A_REF * (1+ alpha/100 * temp - TEMP_A_REF);
// }

void loop() {
  gyro.read();

  vector m;
  int heading;
  read_data(&m);

  float pressure = barometer.readPressureMillibars();
  float altitude = barometer.pressureToAltitudeMeters(pressure);
  altitude -= base_alt;
  float temperature = barometer.readTemperatureF();
  
  Serial.println();

  heading = 180*atan2(m.y, m.x) / PI;
  snprintf(report, sizeof(report), "Accel: %6d %6d %6d \nMag: %6d %6d %6d \nGyro: %6d %6d %6d \nBaro: %2f %2f %2f",
    compass.a.x, compass.a.y, compass.a.z,
    compass.m.x, compass.m.y, compass.m.z,
    gyro.g.x, gyro.g.y, gyro.g.z,
    pressure, altitude, temperature);
  Serial.println(report);
  
  delay(1000);
}

