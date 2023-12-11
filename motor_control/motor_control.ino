#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "PID.h"
#include "ESC.h" // RC_ESP library installed by Library Manager

#define LED_BUILTIN (2) // not defaulted properly for ESP32s/you must define it
#define BATTERY_SENS (13) // pin for battery sense

// Note: the following speeds may need to be modified for your particular hardware.
#define MIN_SPEED 1050 // speed just slow enough to turn motor off
#define MAX_SPEED 2000 // speed where my motor drew 3.6 amps at 12v.

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

int val;

//PID for roll control
PIDController roll_pid;
PIDController pitch_pid;
PIDController yaw_pid;

bool initialize_motors();
void set_motor(int fl_spd, int fr_spd, int rl_spd, int rr_spd);
void angle_stabilization(double curr_x, double curr_y, double curr_z, double desired_x, double desired_y, double desired_z, int desired_speed);

void initialize_pid(){
  roll_pid.Kp = 1;
  roll_pid.Ki = 0;
  roll_pid.Kd = 0;
  roll_pid.tau = 0.001;
  roll_pid.limMin = -100;
  roll_pid.limMax = 100;
  roll_pid.limMaxInt = -10;
  roll_pid.limMaxInt = 10;

  pitch_pid = roll_pid;

  yaw_pid.Kp = 1;
  yaw_pid.Ki = 0;
  yaw_pid.Kd = 0;
  yaw_pid.tau = 0.001;
  yaw_pid.limMin = -100;
  yaw_pid.limMax = 100;
  yaw_pid.limMaxInt = -10;
  yaw_pid.limMaxInt = 10;

  PIDController_Init(&roll_pid);
  PIDController_Init(&pitch_pid);
  PIDController_Init(&yaw_pid);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BATTERY_SENS, INPUT);
  initialize_pid();
  initialize_motors();
  Serial.printf("Initialization successful\n");
} // speed will now jump to pot setting

void loop() {

  //retrieve data from serial
  bool set = false;
  int buff = 0;
  int negative = 1;
  while (Serial.available() > 0){
    int x = Serial.read() - 48;
    if(x == -3){
      negative = -1;
    }else if(x >= 0 && x <= 9){
      buff = buff*10 + x;
      set = true;
    }
    delay(10);
  }
  if (set){
    Serial.print("You send: ");
    if(negative == -1) Serial.print("-");
    Serial.print(buff);
    Serial.println();
    val = negative * buff;
    buff = 0;
  }
  angle_stabilization(0, 0, val, 0, 0, 0, 15);
  float bat = analogReadMilliVolts(BATTERY_SENS) * .0047;
  Serial.printf("Battery Voltage: %0.2f\n", bat);
  delay(200);
}

// x is roll (positive is right), y is yaw (positive is right), z is pitch (positive is down)
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

  float yaw_out = PIDController_Update(&yaw_pid, desired_yaw, curr_yaw);
  float roll_out = PIDController_Update(&roll_pid, desired_roll, curr_roll);
  float pitch_out = PIDController_Update(&pitch_pid, desired_pitch, curr_pitch);
  
  int fl_spd = desired_speed - pitch_out + roll_out - yaw_out;
  int fr_spd = desired_speed - pitch_out - roll_out + yaw_out;
  int rl_spd = desired_speed + pitch_out - roll_out - yaw_out;
  int rr_spd = desired_speed + pitch_out + roll_out + yaw_out;

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
