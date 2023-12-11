# 1 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino"
# 2 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 3 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 4 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2

//RTIMULIB
# 7 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 8 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 9 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 10 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 11 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 12 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 13 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2

//BLE
# 16 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 17 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
# 18 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino" 2
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PITCH_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define ROLL_UUID "beb5483e-36e2-4688-b7f5-ea07361b26a8"
#define YAW_UUID "beb5483e-36e3-4688-b7f5-ea07361b26a8"
#define THROTTLE_UUID "beb5483e-36e4-4688-b7f5-ea07361b26a8"

#define LED_BUILTIN (2) /* not defaulted properly for ESP32s/you must define it*/
#define BATTERY_SENS (13) /* pin for battery sense*/

// Note: the following speeds may need to be modified for your particular hardware.
#define MIN_SPEED 1050 /* speed just slow enough to turn motor off*/
#define MAX_SPEED 2000 /* speed where my motor drew 3.6 amps at 12v.*/

enum motors{
  FL = 16,
  FR = 17,
  RL = 18,
  RR = 19
};

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
        pitch = parse_ble(value);
        Serial.printf("Setting pitch = %d", pitch);
      }
    }
};

class ROLLCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        roll = parse_ble(value);
        Serial.printf("Setting roll = %d", roll);
      }
    }
};

class YAWCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        yaw = parse_ble(value);
        Serial.printf("Setting yaw = %d", yaw);
      }
    }
};

class THROTTLECallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        throttle = parse_ble(value);
        Serial.printf("Setting throttle = %d", throttle);
      }
    }
};

bool initialize_motors();
void set_motor(int fl_spd, int fr_spd, int rl_spd, int rr_spd);
void angle_stabilization(double curr_x, double curr_y, double curr_z, double desired_x, double desired_y, double desired_z, int desired_speed);

void initialize_pid(){
  roll_pid.Kp = 0.1;
  roll_pid.Ki = 0;
  roll_pid.Kd = 0;
  roll_pid.tau = 0.001;
  roll_pid.limMin = -20;
  roll_pid.limMax = 20;
  roll_pid.limMaxInt = -10;
  roll_pid.limMaxInt = 10;

  pitch_pid = roll_pid;

  yaw_pid.Kp = 0.02;
  yaw_pid.Ki = 0;
  yaw_pid.Kd = 0;
  yaw_pid.tau = 0.001;
  yaw_pid.limMin = -20;
  yaw_pid.limMax = 20;
  yaw_pid.limMaxInt = -10;
  yaw_pid.limMaxInt = 10;

  PIDController_Init(&roll_pid);
  PIDController_Init(&pitch_pid);
  PIDController_Init(&yaw_pid);
}

void initialize_ble(){
  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

  BLECharacteristic *pitchCharacteristic = pService->createCharacteristic(
                                         "beb5483e-36e1-4688-b7f5-ea07361b26a8",
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pitchCharacteristic->setCallbacks(new PITCHCallbacks());
  pitchCharacteristic->setValue("I am PITCH");
  BLECharacteristic *rollCharacteristic = pService->createCharacteristic(
                                         "beb5483e-36e2-4688-b7f5-ea07361b26a8",
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  rollCharacteristic->setCallbacks(new ROLLCallbacks());
  rollCharacteristic->setValue("I am ROLL");
  BLECharacteristic *yawCharacteristic = pService->createCharacteristic(
                                         "beb5483e-36e3-4688-b7f5-ea07361b26a8",
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  yawCharacteristic->setCallbacks(new YAWCallbacks());
  yawCharacteristic->setValue("I am YAW");
  BLECharacteristic *throttleCharacteristic = pService->createCharacteristic(
                                         "beb5483e-36e4-4688-b7f5-ea07361b26a8",
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  throttleCharacteristic->setCallbacks(new THROTTLECallbacks());
  throttleCharacteristic->setValue("I am THROTTLE");

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void initialize_rtimu(){
  int errcode;
  Wire.begin();
  imu = RTIMU::createIMU(&settings); // create the imu object

  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
      Serial.println("Using compass calibration");
  else
      Serial.println("No valid compass calibration data");

  // Slerp power controls the fusion and can be between 0 and 1
  // 0 means that only gyros are used, 1 means that only accels/compass are used
  // In-between gives the fusion mix.
  fusion.setSlerpPower(0.04);

  // use of sensors in the fusion algorithm can be controlled here
  // change any of these to false to disable that sensor
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode((2) /* not defaulted properly for ESP32s/you must define it*/, 0x02);
  pinMode((13) /* pin for battery sense*/, 0x01);
  initialize_rtimu();
  initialize_ble();
  initialize_pid();
  initialize_motors();
  pitch, roll, yaw, throttle = 0;

  Serial.printf("Initialization successful\n");
} // speed will now jump to pot setting

void loop() {
  float bat = analogReadMilliVolts((13) /* pin for battery sense*/) * .0047;
  int loopCount = 1;
  while (imu->IMURead()) { // get the latest data if ready yet
    // this flushes remaining data in case we are falling behind
    if (++loopCount >= 10)
        continue;
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    RTVector3& vec = (RTVector3&)fusion.getFusionPose();
    float cur_roll = vec.x() * (180.0 / 3.14159265358979323846);
    float cur_pitch = vec.y() * (180.0 / 3.14159265358979323846);
    float cur_yaw = vec.z() * (180.0 / 3.14159265358979323846);
    Serial.print(" roll:"); Serial.print(cur_roll);
    Serial.print(" pitch:"); Serial.print(cur_pitch);
    Serial.print(" yaw:"); Serial.println(cur_yaw);
    angle_stabilization(cur_roll, cur_yaw, cur_pitch, roll, pitch, yaw, throttle);
  }
}

// x is roll (positive is left), y is yaw (positive is right), z is pitch (positive is down)
void angle_stabilization(double curr_roll, double curr_yaw, double curr_pitch, double desired_roll, double desired_yaw, double desired_pitch, int desired_speed){
  /*

    Logic here is as follows. I have 3 pid controllers one for each angle. 

    The yaw gets to influence the ratio between the diagonals. 

    The roll gets to influence the ratio between left and right.

    The pitch gets to influence the ratio between front and back.

    FL = desired_speed - pitch_pid + roll_pid - Yaw_pid

    FR = desired_speed - pitch_pid - roll_pid + Yaw_pid

    RL = desired_speed + pitch_pid - roll_pid - Yaw_pid

    RR = desired_speed + pitch_pid + roll_pid + Yaw_pid

  */
# 259 "C:\\Users\\rukes\\Documents\\UCB\\FA23\\CS149\\Drone\\motor_control\\motor_control.ino"
  if (desired_speed == 0){
    set_motors(0, 0, 0, 0);
    return;
  }

  float yaw_out = PIDController_Update(&yaw_pid, desired_yaw, curr_yaw);
  float roll_out = PIDController_Update(&roll_pid, desired_roll, curr_roll);
  float pitch_out = PIDController_Update(&pitch_pid, desired_pitch, curr_pitch);

  int fl_spd = desired_speed - pitch_out + roll_out - yaw_out;
  int fr_spd = desired_speed - pitch_out - roll_out + yaw_out;
  int rl_spd = desired_speed + pitch_out - roll_out - yaw_out;
  int rr_spd = desired_speed + pitch_out + roll_out + yaw_out;

  Serial.printf("FL: %d, FR: %d, RL: %d, RR: %d\n", fl_spd, fr_spd, rl_spd, rr_spd);

  set_motors(fl_spd, fr_spd, rl_spd, rr_spd);
}

bool initialize_motors(){
  digitalWrite((2) /* not defaulted properly for ESP32s/you must define it*/, 0x1); // set led to on to indicate arming
  fl_esc.arm(); // Send the Arm command to ESC
  fr_esc.arm(); // Send the Arm command to ESC
  rl_esc.arm(); // Send the Arm command to ESC
  rr_esc.arm(); // Send the Arm command to ESC
  delay(5000); // Wait a while
  digitalWrite((2) /* not defaulted properly for ESP32s/you must define it*/, 0x0); // led off to indicate arming completed
  set_motors(0,0,0,0);
  delay(1000); // Wait a while
}

void set_motors(int fl_spd, int fr_spd, int rl_spd, int rr_spd) {
  fl_spd = map(fl_spd, 0, 1000, 1050 /* speed just slow enough to turn motor off*/, 2000 /* speed where my motor drew 3.6 amps at 12v.*/);
  fl_esc.speed(fl_spd);
  fr_spd = map(fr_spd, 0, 1000, 1050 /* speed just slow enough to turn motor off*/, 2000 /* speed where my motor drew 3.6 amps at 12v.*/);
  fr_esc.speed(fr_spd);
  rl_spd = map(rl_spd, 0, 1000, 1050 /* speed just slow enough to turn motor off*/, 2000 /* speed where my motor drew 3.6 amps at 12v.*/);
  rl_esc.speed(rl_spd);
  rr_spd = map(rr_spd, 0, 1000, 1050 /* speed just slow enough to turn motor off*/, 2000 /* speed where my motor drew 3.6 amps at 12v.*/);
  rr_esc.speed(rr_spd);
}