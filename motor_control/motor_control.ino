#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "ESC.h" // RC_ESP library installed by Library Manager

#define LED_BUILTIN (2) // not defaulted properly for ESP32s/you must define it

// Note: the following speeds may need to be modified for your particular hardware.
#define MIN_SPEED 1040 // speed just slow enough to turn motor off
#define MAX_SPEED 1240 // speed where my motor drew 3.6 amps at 12v.

enum motors{
  FL = 12
};

ESC fl_esc (FL, 1000, 2000, 500); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)

bool initialize_motors();
void set_motor(int fl_spd, int fr_spd, int rl_spd, int rr_spd);

void setup() {
  Serial.begin(9600);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  initialize_motors();
  Serial.printf("Initialization successful\n");
} // speed will now jump to pot setting

void loop() {

  //retrieve data from serial
  int buff = 0;
  bool set = false;
  while (Serial.available() > 0){
    int x = Serial.read() - 48;
    buff = buff*10 + x;
    set = true;
    delay(5);
  }
  if(set){
    Serial.printf("Setting motor to: %d\n", buff);
    set_motors(buff,0,0,0);
    delay(10); // Wait for a while
  }
  
}

bool initialize_motors(){
  digitalWrite(LED_BUILTIN, HIGH); // set led to on to indicate arming
  fl_esc.arm(); // Send the Arm command to ESC
  delay(5000); // Wait a while
  digitalWrite(LED_BUILTIN, LOW); // led off to indicate arming completed
  set_motors(0,0,0,0);
}

void set_motors(int fl_spd, int fr_spd, int rl_spd, int rr_spd) {
  fl_spd = map(fl_spd, 0, 100, MIN_SPEED, MAX_SPEED);
  fl_esc.speed(fl_spd);
}
