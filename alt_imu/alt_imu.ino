
#include "Wire.h"
#include <LSM303.h>
#include <L3G.h>
#include <LPS.h>
#include <esp_timer.h>
#include <math.h>

extern "C" {
#include <KalmanRollPitchYaw.h>
  // #include <KalmanRollPitch.h>
}


typedef struct vector {
  float x, y, z;
} vector;

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
#define SENS_A_REF 4096
#define TEMP_REF 25
#define EKF_N_ACC 0.000067666
#define EKF_N_PSI 0.00005
#define SAMPLE_HZ_ACCEL 1600
#define SAMPLE_TIME_ACCEL_MS 1 / SAMPLE_HZ_ACCEL * 1000  //in ms
#define SAMPLE_HZ_MAG 100
#define SAMPLE_TIME_MAG_MS 1 / SAMPLE_HZ_MAG * 1000  //in ms
#define NUM_READINGS_ACCEL 10
#define NUM_REPORT_ACCEL 20
#define NUM_READINGS_MAG 10
#define NUM_REPORT_MAG 20  //how many reports we keep track of mag to avg

LSM303 compass;
vector prev_a;

float B_accel[3] = { -154.590426, -219.816113, -141.307594 };
float Ainv_accel[3][3] = { { 0.608190, -0.001143, 0.009962 },
                           { -0.001143, 0.628152, -0.000687 },
                           { 0.009962, -0.000687, 0.606519 } };


vector prev_m;

//Our calibration of magnetometer
float B_mag[3] = { 9955.15, -7948.26, 8511.80 };
float Ainv_mag[3][3] = { { 0.25060, 0.02942, -0.02955 },
                         { 0.02942, 0.31692, 0.00789 },
                         { -0.02955, 0.00789, 0.30592 } };


//Accelerometer
float SENS_A;
float accel_report[3][20];
float accel_sum[3];
uint8_t index_accel;


//Magnetometer
float SENS_M;
float psiMag;
float mag_report[3][NUM_REPORT_MAG];  //keep track of 20
float mag_sum[3];
uint8_t index_mag;

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

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define EKF_N_GYR 0.00000191056
#define CONVERSION_G 8.75  // correlational to sensitivity -> refer to datasheet
#define SAMPLE_HZ_GYRO 757.6
#define SAMPLE_TIME_GYRO_MS 1 / SAMPLE_HZ_GYRO * 1000  //1 divide by hz
#define NUM_READINGS_GYRO 10

L3G gyro;

float gyro_avg[3];
float gyro_avg_count;

vector prev_g;

/*=====================================================================*/

LPS barometer;


float base_alt;


#define CF_ALPHA 0.01  //need to test this value

uint32_t curr;

//Low-pass filter coefficients that need to be tested
float LPF_GYR_CUTOFF_HZ = 0.7;
float LPF_ACC_CUTOFF_HZ = 0.9;
float LPF_MAG_CUTOFF_HZ = 0.4;
float LPF_BAR_CUTOFF_HZ = 0.7;

float x[7];  //state estimate vector

float p, q, r;

float roll, pitch, psi;

float P[4];
float Q[2];
float R[3];  //noise matrix

KalmanRollPitchYaw kalmanData;

// KalmanRollPitch* kalmanData;

float prev;

void setup() {
  Serial.begin(115200);
  Wire.begin(LSM303D_SDA_PIN, LSM303D_SCL_PIN);

  if (!compass.init()) {
    Serial.println("failed to detect compass");
    while (1)
      ;
  }
  compass.enableDefault();
  // /* change sensitivity and sample frequency of accel */
  // // compass.writeReg(LSM303::CTRL2, 0x18); //change to +-8g check if we need higher
  compass.writeReg(LSM303::CTRL2, 0x20);  //+/16g
  compass.writeReg(LSM303::CTRL1, 0xA7);  //change to hz=1600

  //initialize accel variables
  prev_a.x = 0.0f;
  prev_a.y = 0.0f;
  prev_a.z = 0.0f;
  index_accel = 0;

  /* change sensitivity and sample frequency of mag */
  compass.writeReg(LSM303::CTRL6, 0x60);  //+/- 12 gauss
  compass.writeReg(LSM303::CTRL5, 0x14);  //change to hz=100

  //initialize mag variables
  prev_m.x = 0.0f;
  prev_m.y = 0.0f;
  prev_m.z = 0.0f;
  index_mag = 0;


  if (!gyro.init()) {
    Serial.println("failed to detect gyro");
    while (1)
      ;
  }
  gyro.enableDefault();
  gyro.writeReg(L3G::CTRL1, 0xCF);  //change to hz=800
  gyro.writeReg(L3G::CTRL4, 0x20);  //change the sensitivity of gyro to 2000dps
  prev_g.x = 0.0f;
  prev_g.y = 0.0f;
  prev_g.z = 0.0f;
  for (int i = 0; i < 3; i++) {
    gyro_avg[i] = 0;  //initialize
  }
  gyro_avg_count = 0;

  if (!barometer.init()) {
    Serial.println("failed to detect barometer");
    while (1)
      ;
  }
  barometer.enableDefault();

  for (int j = 0; j < NUM_REPORT_MAG; j++) {
    mag_report[0][j] = 0;
    mag_report[1][j] = 0;
    mag_report[2][j] = 0;
  }

  for (int j = 0; j < NUM_REPORT_ACCEL; j++) {
    accel_report[0][j] = 0;
    accel_report[1][j] = 0;
    accel_report[2][j] = 0;
    Serial.print("Setup "); Serial.println(accel_report[0][j]);
  }


  // Serial.println("almost end of setup");
  calibrate();
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

  //initialize ekf with yaw
  float Q[] = { 3.0f * EKF_N_GYR, 2.0f * EKF_N_GYR, 1.0f * EKF_N_GYR };
  float Racc = EKF_N_ACC;
  float Rpsi = EKF_N_PSI;
  KalmanRollPitchYaw_Init(&kalmanData, 10.0f, Q, Racc, Rpsi);

  /* Initialize EKF with no yaw */
  // float Q[] = {3.0f * EKF_N_GYR, 2.0f * EKF_N_GYR};
  // float R[] = {EKF_N_ACC, EKF_N_ACC, EKF_N_ACC};
  // KalmanRollPitch_Init(&kalmanData, 10.0f, Q, R);

  // for (int j = 0; j < NUM_REPORT_MAG; j++) {
  //   compass.read();
  //   mag_report[0][index_mag] += compass.m.x;
  //   mag_report[1][index_mag] += compass.m.y;
  //   mag_report[2][index_mag] += compass.m.y;
  //   index_mag += (index_mag + 1) % NUM_REPORT_MAG;
  // }

  // Serial.print
  // for (int j = 0; j < NUM_REPORT_ACCEL; j++) {
  //   compass.read();
  //   accel_report[0][index_accel] += compass.a.x;
  //   accel_report[1][index_accel] += compass.a.y;
  //   accel_report[2][index_accel] += compass.a.y;
  //   Serial.print("y "); Serial.println(compass.a.y);
  //   Serial.print("accel_report x"); Serial.println(accel_report[0][index_accel]);
  //   Serial.print("accel_report y"); Serial.println(accel_report[1][index_accel]);
  //   Serial.print("accel_report z"); Serial.println(accel_report[2][index_accel]);
  //   index_accel += (index_accel + 1) % NUM_REPORT_ACCEL;
  // }


  vector m;
  init_mag(&m);
  psiMag = -atan2(m.y, m.x);

  vector a;
  init_accel(&a);
  roll = atan2(a.y, a.z);
  pitch = atan2(a.x, sqrt(pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2)));
  psi = atan2(-m.y * cos(roll) + m.z * sin(roll),
              m.x * cos(pitch) + m.y * sin(roll) * sin(pitch) + m.z * cos(roll) * sin(pitch));
  Serial.print("cali: psi"); Serial.println(psi);


  Serial.print("check kalmanData init"); Serial.println(kalmanData.phi); 
  Serial.print("next"); Serial.println(kalmanData.theta);
  prev = millis();
}

void read_mag(vector* m) {
  cali_mag(m);
  //maybe Axis remapping need to double check
  axis_remap(m);

  //low pass filter mag
  low_pass_filter(m, &prev_m, LPF_MAG_CUTOFF_HZ);
}

void init_mag(vector* m) {
  cali_mag(m);

  float m_norm = 1.0f / sqrt(pow(m->x, 2) + pow(m->y, 2) + pow(m->z, 2));
  m->x *= m_norm;
  m->y *= m_norm;
  m->z *= m_norm;
  //maybe Axis remapping need to double check
  axis_remap(m);
}

void avg_mag(vector* m) {
  for (int j = 0; j < NUM_READINGS_MAG; j++) {
    mag_sum[0] -= mag_report[0][index_mag];
    mag_sum[1] -= mag_report[1][index_mag];
    mag_sum[2] -= mag_report[2][index_mag];
    compass.read();
    mag_report[0][index_mag] += compass.m.x;
    mag_report[1][index_mag] += compass.m.y;
    mag_report[2][index_mag] += compass.m.y;

    mag_sum[0] += mag_report[0][index_mag];
    mag_sum[1] += mag_report[1][index_mag];
    mag_sum[2] += mag_report[2][index_mag];
    index_mag += (index_mag + 1) % NUM_REPORT_MAG;
  }

  m->x = mag_sum[0] / NUM_REPORT_MAG;
  m->y = mag_sum[1] / NUM_REPORT_MAG;
  m->z = mag_sum[2] / NUM_REPORT_MAG;
}

void cali_mag(vector* m) {
  // avg_mag(m);
  static float x, y, z;
  x = (float)m->x - B_mag[0];
  y = (float)m->y - B_mag[1];
  z = (float)m->z - B_mag[2];
  m->x = Ainv_mag[0][0] * x + Ainv_mag[0][1] * y + Ainv_mag[0][2] * z;
  m->y = Ainv_mag[1][0] * x + Ainv_mag[1][1] * y + Ainv_mag[1][2] * z;
  m->z = Ainv_mag[2][0] * x + Ainv_mag[2][1] * y + Ainv_mag[2][2] * z;
}

/* Calibrate values, remap axis, put through low pass filter and convert */
void read_accel(vector* a) {
  cali_accel(a);

  //Axis remapping
  a->x = -1 * a->y;
  a->y = -1 * a->x;
  a->z = -1 * a->z;

  low_pass_filter(a, &prev_a, LPF_ACC_CUTOFF_HZ);

  //convert accel values from mg to m/s^2
  convert_accel(a);
}

void avg_accel(vector* a) {
  for (int j = 0; j < NUM_READINGS_ACCEL; j++) {
    accel_sum[0] -= accel_report[0][index_accel];
    accel_sum[1] -= accel_report[1][index_accel];
    accel_sum[2] -= accel_report[2][index_accel];
    compass.read();
    accel_report[0][index_accel] += compass.a.x;
    accel_report[1][index_accel] += compass.a.y;
    accel_report[2][index_accel] += compass.a.y;

    accel_sum[0] += accel_report[0][index_accel];
    accel_sum[1] += accel_report[1][index_accel];
    accel_sum[2] += accel_report[2][index_accel];
    index_accel += (index_accel + 1) % NUM_REPORT_ACCEL;
    Serial.print("avg accel y "); Serial.println(compass.a.y);
  }

  a->x = accel_sum[0] / NUM_REPORT_ACCEL;
  Serial.print("accel_sum "); Serial.println(accel_sum[0]);
  a->y = accel_sum[1] / NUM_REPORT_ACCEL;
  a->z = accel_sum[2] / NUM_REPORT_ACCEL;


  Serial.print("compass read avg_accel"); Serial.println(a->y);
}

void cali_accel(vector* a) {
  // avg_accel(a);
  // Serial.print("cali_accel "); Serial.println(a->x);
  float x, y, z;
  x = compass.a.x - B_accel[0];
  // Serial.print("B_accel[0] "); Serial.println(B_accel[0]);
  // Serial.print("B_accel[1] "); Serial.println(B_accel[1]);
  // Serial.print("B_accel[2] "); Serial.println(B_accel[2]);
  y = compass.a.y - B_accel[1];
  z = compass.a.z - B_accel[2];

  // Serial.print("x "); Serial.println(x);
  // Serial.print("y "); Serial.println(y);
  // Serial.print("z "); Serial.println(z);
  a->x = Ainv_accel[0][0] * x + Ainv_accel[0][1] * y + Ainv_accel[0][2] * z;
  float test = Ainv_accel[0][0] * x + Ainv_accel[0][1] * y + Ainv_accel[0][2] * z;
  Serial.print("test "); Serial.println(test);
  a->y = Ainv_accel[1][0] * x + Ainv_accel[1][1] * y + Ainv_accel[1][2] * z;
  a->z = Ainv_accel[2][0] * x + Ainv_accel[2][1] * y + Ainv_accel[2][2] * z;

  // Serial.print("Ainv_accel[0][0] "); Serial.println(Ainv_accel[0][0]);
  // Serial.print("Ainv_accel[0][1] "); Serial.println(Ainv_accel[0][1]);
  // Serial.print("Ainv_accel[0][2] "); Serial.println(Ainv_accel[0][2]);
  // Serial.print("cali_end a->x "); Serial.println(a->x);
}
void init_accel(vector* a) {

  cali_accel(a);
  // Serial.print("init_accel "); Serial.println(a->x);
  //Axis remapping
  axis_remap(a);
  convert_accel(a);
}

/* Outputs gyro rad/sec */
void read_gyro(vector* g) {
  gyro.read();

  //calibrate step is missing
  g->x = gyro.g.x;
  g->y = gyro.g.y;
  g->z = gyro.g.z;

  // axis remapping
  axis_remap(g);

  low_pass_filter(g, &prev_g, LPF_GYR_CUTOFF_HZ);

  convert_gyro(g);
}

void axis_remap(vector* v) {
  v->x = -1 * v->y;
  v->y = -1 * v->x;
  v->z = -1 * v->z;
}

void initial_g(vector* g) {
  gyro.read();

  //calibrate step is missing
  g->x = gyro.g.x;
  g->y = gyro.g.y;
  g->z = gyro.g.z;

  // axis remapping
  axis_remap(g);

  convert_gyro(g);
}

// raw value -> rad/sec
float raw_to_rad_sec(float num) {
  float dps = num * CONVERSION_G / 1000;
  return dps * DEG_TO_RAD;  //returns rad/s
}

void convert_gyro(vector* g) {
  g->x = raw_to_rad_sec(g->x);
  g->y = raw_to_rad_sec(g->y);
  g->z = raw_to_rad_sec(g->z);
}

float mg_to_m_sec2(float num) {
  float accel = num / SENS_A;  //convert from mg to g
  accel *= GRAVITY;               //Convert to m/s^2
  return accel;
}

void convert_accel(vector* a) {
  Serial.print("convert_accel "); Serial.println(a->x);
  a->x = mg_to_m_sec2(a->x);
  a->y = mg_to_m_sec2(a->y);
  a->z = mg_to_m_sec2(a->z);
}

void low_pass_filter(vector* v, vector* prev_v, float lpf_v) {
  //low pass filter accel (first-order low-pass filter)
  v->x = lpf_v * prev_v->x + (1.0f - lpf_v) * v->x;
  v->y = lpf_v * prev_v->y + (1.0f - lpf_v) * v->y;
  v->z = lpf_v * prev_v->z + (1.0f - lpf_v) * v->z;

  //update prev data
  prev_v->x = v->x;
  prev_v->y = v->y;
  prev_v->z = v->z;
}



// float true_heading(vector* m, float declination) {
//   return atan2(-m->y*cos())
// }

void loop() {
  // Serial.println("hello");
  float pressure = barometer.readPressureMillibars();
  float altitude = barometer.pressureToAltitudeMeters(pressure);
  altitude -= base_alt;
  float temperature = barometer.readTemperatureF();


  curr = millis();
  if (curr - prev >= SAMPLE_TIME_GYRO_MS) {  //start gyro task
    vector g_fitted;
    read_gyro(&g_fitted);
    float gyro_k[3];
    gyro_k[0] = g_fitted.x;
    gyro_k[1] = g_fitted.y;
    gyro_k[2] = g_fitted.z;
    Serial.print("gyro_k[0] "); Serial.println(gyro_k[0]);
    Serial.print("gyro_k[1] "); Serial.println(gyro_k[1]);
    Serial.print("gyro_k[2] "); Serial.println(gyro_k[2]);
    KalmanRollPitchYaw_Predict(&kalmanData, gyro_k, SAMPLE_TIME_GYRO_MS*1000); //time in seconds
    Serial.print("gyro phi "); Serial.println(kalmanData.phi);
    Serial.print("gyro theta "); Serial.println(kalmanData.theta);
    roll = kalmanData.phi;
    pitch = kalmanData.theta;
    psi = CF_ALPHA * psiMag + (1.0f - CF_ALPHA) * (psi + SAMPLE_TIME_GYRO_MS * 1000 *
      (sin(kalmanData.phi) * g_fitted.y + cos(kalmanData.phi) * g_fitted.z) / cos(kalmanData.theta));
    
  }

  if (curr-prev >= SAMPLE_TIME_ACCEL_MS) { //start accel task
    vector a_fitted;
    read_accel(&a_fitted);
    float accel_k[3];
    accel_k[0] = a_fitted.x; accel_k[1] = a_fitted.y; accel_k[2] = a_fitted.z;
    KalmanRollPitchYaw_UpdateAcc(&kalmanData, accel_k, SAMPLE_TIME_ACCEL_MS*1000);
    roll = kalmanData.phi;
    pitch = kalmanData.theta;
    Serial.print("accel phi "); Serial.println(kalmanData.phi);
    Serial.print("accel theta "); Serial.println(kalmanData.theta);
  }



  if (curr-prev >= SAMPLE_TIME_MAG_MS) { //start mag task
    vector m_reading;
    read_mag(&m_reading);

    psiMag = -atan2(m_reading.y, m_reading.x);
    // float mag_k[3];
    // mag_k[0] = m_reading->x; mag_k[1] = m_reading->y; mag_k[2] = m_reading->z;
    KalmanRollPitchYaw_UpdatePsi(&kalmanData, psiMag);
    roll = kalmanData.phi;
    pitch = kalmanData.theta;
    Serial.print("mag phi"); Serial.print(kalmanData->phi);
    Serial.print("mag theta"); Serial.print(kalmanData->theta);
  }

  Serial.println("Roll" ); 
  Serial.println(roll);
  Serial.println("Pitch "); 
  Serial.println(pitch);
  Serial.println("Psi "); 
  Serial.println(psi);

  prev = curr;

  delay(1000);

}
