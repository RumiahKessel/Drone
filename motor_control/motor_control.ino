#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "PID.h"
#include "ESC.h" // RC_ESP library installed by Library Manager

//RTIMULIB
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>

//BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PITCH_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define ROLL_UUID "beb5483e-36e2-4688-b7f5-ea07361b26a8"
#define YAW_UUID "beb5483e-36e3-4688-b7f5-ea07361b26a8"
#define THROTTLE_UUID "beb5483e-36e4-4688-b7f5-ea07361b26a8"
#define SPEED_UUID "beb5483e-36e5-4688-b7f5-ea07361b26a8"

#define LED_BUILTIN (2) // not defaulted properly for ESP32s/you must define it
#define BATTERY_SENS (13) // pin for battery sense

// Note: the following speeds may need to be modified for your particular hardware.
#define MIN_SPEED 1050 // speed just slow enough to turn motor off
#define MAX_SPEED 2000 // speed where my motor drew 3.6 amps at 12v.

#define DISPLAY_INTERVAL  20   
#define YAW_INTERVAL 20

enum motors{
  FL = 33,
  FR = 25,
  RL = 26,
  RR = 27
};

unsigned long lastDisplay;
unsigned long lastRate;

float roll_trim = 0;
float pitch_trim = 0;
float yaw_trim = 0;

// ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)
ESC fl_esc (FL, 1000, 2000, 500);
ESC fr_esc (FR, 1000, 2000, 500);
ESC rl_esc (RL, 1000, 2000, 500);
ESC rr_esc (RR, 1000, 2000, 500);

// User inputs
int throttle;
int pitch;
int roll;
int yaw;
int yaw_change;
int max_speed;
bool yaw_zeroed;

// Attitude estimation
RTIMU *imu;
RTFusionRTQF fusion;
RTIMUSettings settings;

//PID for roll control
PIDController roll_pid;
PIDController pitch_pid;
PIDController yaw_pid;

int parse_ble(std::__cxx11::string value){
  int negative = 1;
  int buff = 0;
  for (int i = 0; i < value.length(); i++){
    int x = value[i] - 48;
    if(x == -3){
      negative = -1;
    }else if(x >= 0 && x <= 9){
      buff = buff*10 + x;
    }
  }
  buff *= negative;
  return buff;
}


class PITCHCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        pitch = parse_ble(value)/100;
//        Serial.printf("Setting pitch = %d\n", pitch);
      }
    }
};

class ROLLCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        roll = parse_ble(value)/100;
//        Serial.printf("Setting roll = %d\n", roll);
      }
    }
};

class YAWCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        yaw_change = parse_ble(value)/100;
//        Serial.printf("Setting yaw = %d\n", yaw);
      }
    }
};

class THROTTLECallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        throttle = parse_ble(value)/10;
//        Serial.printf("Setting throttle = %d\n", throttle);
      }
    }
};

class SPEEDCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        max_speed = parse_ble(value);
//        Serial.printf("Setting max speed = %d\n", max_speed);
      }
    }
};

bool initialize_motors();
void set_motor(int fl_spd, int fr_spd, int rl_spd, int rr_spd);
void angle_stabilization(double curr_x, double curr_y, double curr_z, double desired_x, double desired_y, double desired_z, int desired_speed);

float roll_v_p = 0.2;
float pitch_v_p = 0.2;
float yaw_v_p = 2;

void initialize_pid(){
  roll_pid.Kp = 0.28;
  roll_pid.Ki = 0;
  roll_pid.Kd = 0;
  roll_pid.tau = 1;
  roll_pid.limMin = -150;
  roll_pid.limMax = 150;
  roll_pid.limMaxInt = -10;
  roll_pid.limMaxInt = 10;
  
  pitch_pid.Kp = 0.28;
  pitch_pid.Ki = 0;
  pitch_pid.Kd = 0;
  pitch_pid.tau = 1;
  pitch_pid.limMin = -150;
  pitch_pid.limMax = 150;
  pitch_pid.limMaxInt = -10;
  pitch_pid.limMaxInt = 10;

  yaw_pid.Kp = 4;
  yaw_pid.Ki = 0;
  yaw_pid.Kd = 0;
  yaw_pid.tau = 0.001;
  yaw_pid.limMin = -200;
  yaw_pid.limMax = 200;
  yaw_pid.limMaxInt = -10;
  yaw_pid.limMaxInt = 10;

  PIDController_Init(&roll_pid);
  PIDController_Init(&pitch_pid);
  PIDController_Init(&yaw_pid);
}

void initialize_ble(){
  BLEDevice::init("Ultra Drone");
  BLEDevice::setPower(ESP_PWR_LVL_P7, ESP_BLE_PWR_TYPE_ADV);
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pitchCharacteristic = pService->createCharacteristic(
                                         PITCH_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pitchCharacteristic->setCallbacks(new PITCHCallbacks());
  pitchCharacteristic->setValue("I am PITCH");
  BLECharacteristic *rollCharacteristic = pService->createCharacteristic(
                                         ROLL_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  rollCharacteristic->setCallbacks(new ROLLCallbacks());
  rollCharacteristic->setValue("I am ROLL");
  BLECharacteristic *yawCharacteristic = pService->createCharacteristic(
                                         YAW_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  yawCharacteristic->setCallbacks(new YAWCallbacks());
  yawCharacteristic->setValue("I am YAW");
  BLECharacteristic *throttleCharacteristic = pService->createCharacteristic(
                                         THROTTLE_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  throttleCharacteristic->setCallbacks(new THROTTLECallbacks());
  throttleCharacteristic->setValue("I am THROTTLE");
  BLECharacteristic *speedCharacteristic = pService->createCharacteristic(
                                         SPEED_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  speedCharacteristic->setCallbacks(new SPEEDCallbacks());
  speedCharacteristic->setValue("I am SPEED");

  
  
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void initialize_rtimu(){
  int errcode;
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object

  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
      Serial.println("Using compass calibration");
  else {
      Serial.println("No valid compass calibration data");
    while(1);
  }

  yaw_zeroed = false;
  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.025);
  
  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BATTERY_SENS, INPUT);
  initialize_ble();
  
  initialize_pid();
  initialize_motors();
  initialize_rtimu();
  
  pitch, roll, yaw, throttle, max_speed = 0;
//  delay(1000); // let yaw settle
  lastDisplay = lastRate = millis();
  int sum = 0;
  for (int i = 0; i < 100; i++){
    while (imu->IMURead()) {                                // get the latest data if ready yet
      sum++;
      fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
      RTVector3& vec = (RTVector3&)fusion.getFusionPose();
      float cur_roll = vec.x() * RTMATH_RAD_TO_DEGREE;
      float cur_pitch = vec.y() * RTMATH_RAD_TO_DEGREE;
      float cur_yaw = vec.z() * RTMATH_RAD_TO_DEGREE;
      roll_trim += cur_roll;
      pitch_trim += cur_pitch;
      yaw_trim += cur_yaw;
    }
    delay(10);
  }

  roll_trim /= sum;
  pitch_trim /= sum;
  yaw_trim /= sum;

  Serial.printf("Initialization successful\n");
} // speed will now jump to pot setting

void loop() {
  unsigned long now = millis();
  float bat = analogReadMilliVolts(BATTERY_SENS) * .0047;
  int loopCount = 1;
  while (imu->IMURead()) {                                // get the latest data if ready yet
    // this flushes remaining data in case we are falling behind
    if (++loopCount >= 10)
        continue;

    RTVector3& gyro = (RTVector3&)imu->getGyro();
    float cur_roll_rate = gyro.x() * RTMATH_RAD_TO_DEGREE;
    float cur_pitch_rate = gyro.y() * RTMATH_RAD_TO_DEGREE;
    float cur_yaw_rate = gyro.z() * RTMATH_RAD_TO_DEGREE;

    Serial.printf("roll_rate:%.2f pitch_rate:%.2f", cur_roll_rate, cur_pitch_rate);

    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    if (now - lastRate >= YAW_INTERVAL) {
      lastRate = now;
      yaw += yaw_change;
    }
      RTVector3& vec = (RTVector3&)fusion.getFusionPose();
      float cur_roll = (vec.x() * RTMATH_RAD_TO_DEGREE) - roll_trim;
      float cur_pitch = (vec.y() * RTMATH_RAD_TO_DEGREE) - pitch_trim;
      float cur_yaw = (vec.z() * RTMATH_RAD_TO_DEGREE) - yaw_trim;
//      if(!yaw_zeroed){
//        yaw = cur_yaw;
//        yaw_zeroed = true;
//      }
//      Serial.printf("roll_rate:%.2f pitch_rate:%.2f yaw_rate:%.2f", cur_roll_rate, cur_pitch_rate, cur_yaw_rate);
////      if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
        Serial.printf(" roll:%.2f pitch:%.2f", cur_roll, cur_pitch);
////      }
//     
      angle_stabilization(cur_roll, cur_yaw, cur_pitch, cur_roll_rate, cur_yaw_rate, cur_pitch_rate, roll, yaw, pitch, max_speed, throttle);
//    }
  }
}

// x is roll (positive is right), y is yaw (positive is right), z is pitch (positive is up)
void angle_stabilization(float curr_roll, float curr_yaw, float curr_pitch, float cur_roll_rate, float cur_yaw_rate, float cur_pitch_rate, float desired_roll, float desired_yaw, float desired_pitch, int desired_speed, int throttle){
  /*
    Logic here is as follows. I have 3 pid controllers one for each angle. 
    The yaw gets to influence the ratio between the diagonals. 
    The roll gets to influence the ratio between left and right.
    The pitch gets to influence the ratio between front and back.
    FL = desired_speed + pitch_pid + roll_pid - Yaw_pid
    FR = desired_speed + pitch_pid - roll_pid + Yaw_pid
    RL = desired_speed - pitch_pid + roll_pid + Yaw_pid
    RR = desired_speed - pitch_pid - roll_pid - Yaw_pid
  */

  float desired_roll_v = roll_v_p*(desired_roll - curr_roll);
  float desired_pitch_v = pitch_v_p*(desired_pitch - curr_pitch);
  float desired_yaw_v = yaw_v_p*(desired_yaw- curr_yaw);

  Serial.printf(" Pitch_v_pid:%0.2f Roll_v_pid:%0.2f", desired_pitch_v, desired_roll_v);

//  float yaw_out = PIDController_Update(&yaw_pid, desired_yaw_v, cur_yaw_rate);
  float yaw_out = 0;
  float roll_out = PIDController_Update(&roll_pid, desired_roll_v, cur_roll_rate);
  float pitch_out = PIDController_Update(&pitch_pid, desired_pitch_v, cur_pitch_rate);
  
  int fl_spd = desired_speed + throttle + pitch_out + roll_out + yaw_out;
  int fr_spd = desired_speed + throttle + pitch_out - roll_out - yaw_out;
  int rl_spd = desired_speed + throttle - pitch_out + roll_out - yaw_out;
  int rr_spd = desired_speed + throttle - pitch_out - roll_out + yaw_out;

  Serial.printf(" Pitch_pid:%0.2f Roll_pid:%0.2f\n", pitch_out, roll_out);

  if (desired_speed == 0){
    set_motors(0, 0, 0, 0);
    return;
  }
  set_motors(fl_spd, fr_spd, rl_spd, rr_spd);
}

bool initialize_motors(){
  digitalWrite(LED_BUILTIN, HIGH); // set led to on to indicate arming
  fl_esc.arm(); // Send the Arm command to ESC
  fr_esc.arm(); // Send the Arm command to ESC
  rl_esc.arm(); // Send the Arm command to ESC
  rr_esc.arm(); // Send the Arm command to ESC
  delay(5000); // Wait a while
  digitalWrite(LED_BUILTIN, LOW); // led off to indicate arming completed
  set_motors(0,0,0,0);
  delay(1000); // Wait a while
}

void set_motors(int fl_spd, int fr_spd, int rl_spd, int rr_spd) {
  fl_spd = map(fl_spd, 0, 1000, MIN_SPEED, MAX_SPEED);
  fl_esc.speed(fl_spd);
  fr_spd = map(fr_spd, 0, 1000, MIN_SPEED, MAX_SPEED);
  fr_esc.speed(fr_spd);
  rl_spd = map(rl_spd, 0, 1000, MIN_SPEED, MAX_SPEED);
  rl_esc.speed(rl_spd);
  rr_spd = map(rr_spd, 0, 1000, MIN_SPEED, MAX_SPEED);
  rr_esc.speed(rr_spd);
}
