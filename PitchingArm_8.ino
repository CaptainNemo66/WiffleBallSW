// Connect to Arduino, Upload/Verfiy, runs automatically, and hit the reset button to run again
// Robert Lim
#include <Servo.h>

// --- HARDWARE PIN DEFINITIONS ---
const int PIN_JOYSTICK = 2;  // Joystick Switch (SW pin)
const int PIN_SERVO    = 11;  // Servo for Arm Angle

// Motor Controller Pins (HW-095 / L298N)
const int PIN_IN1      = 3;  // Direction Pin A
const int PIN_IN2      = 4;  // Direction Pin B
const int PIN_ENA      = 5;  // Speed Control (PWM)

const int PIN_LED      = 13; // Onboard LED for status

// --- USER CONFIGURATION (ANGLES) ---
const int ANGLE_EASY_MODE = 120; 
const int ANGLE_HARD_MODE = 90;  

// --- PRESET DEFINITIONS ---
struct PitchProfile {
  int servoAngle;     // Trajectory
  double targetSpeed; // Setpoint
  double P_val;       
  double I_val;       
  double D_val;       
  int holdTime;       
};

// MODIFY THESE PRESETS FOR TUNNING!!!!
PitchProfile easyPitch = {
  ANGLE_EASY_MODE, 
  140.0,  // Target Speed
  0.5,    // Kp 
  0.001,  // Ki
  0.1,    // Kd
  1500    // holdTime (ms)
};

PitchProfile hardPitch = {
  ANGLE_HARD_MODE, 
  255.0,  // Target Speed
  1.2,    // Kp 
  0.005,  // Ki
  0.5,    // Kd
  1200    // holdTime (ms)
};

// --- USER PID VARIABLES ---
double sensed_output = 0; 
double control_signal = 0; 
double setpoint = 0;
double Kp = 0;
double Ki = 0;
double Kd = 0;
int T = 10; 
unsigned long last_time = 0;
double total_error = 0;
double last_error = 0;
int max_control = 255;
int min_control = 0;

Servo aimServo;

void setup() {
  Serial.begin(19200);
  
  // Pin Setup
  pinMode(PIN_JOYSTICK, INPUT_PULLUP); // Assumes switch connects to GND when pressed
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  
  aimServo.attach(PIN_SERVO);
  aimServo.write(60);
  
  // Set Motor Direction
  digitalWrite(PIN_IN1, HIGH); 
  digitalWrite(PIN_IN2, LOW);
  
  Serial.println("--- SYSTEM READY ---");
  Serial.println("Waiting for Input: 1 Click = Easy, 2 Clicks = Hard");
}

void loop() {
  // 1. DETECT CLICKS (Blocking function until user inputs)
  int mode = getClickMode();
  
  PitchProfile selectedProfile;
  
  if (mode == 1) {
    Serial.println(">> EASY MODE SELECTED (1 Click)");
    selectedProfile = easyPitch;
    indicateMode(1);
  } else {
    Serial.println(">> HARD MODE SELECTED (2 Clicks)");
    selectedProfile = hardPitch;
    indicateMode(2);
  }
  
  // 2. SET SERVO ANGLE
  Serial.print("Setting Angle: ");
  Serial.println(selectedProfile.servoAngle);
  aimServo.write(selectedProfile.servoAngle);
  delay(2000); // Wait for servo to move
  
  // 3. EXECUTE THROW
  performPIDThrow(selectedProfile);
  
  Serial.println("Throw Complete. Press RESET button to throw again.");
}

// --- INPUT DETECTION LOGIC ---
int getClickMode() {
  // Wait for the first press (Idle state)
  while (digitalRead(PIN_JOYSTICK) == HIGH) {
    // Do nothing, just wait
  }
  
  // First Press Detected!
  delay(50); // Debounce
  while (digitalRead(PIN_JOYSTICK) == LOW) {}; // Wait for release
  
  unsigned long firstClickTime = millis();
  
  // Check for Double Click Window (e.g. 500ms)
  while (millis() - firstClickTime < 500) {
    if (digitalRead(PIN_JOYSTICK) == LOW) {
      // Second Press Detected!
      delay(50); // Debounce
      while (digitalRead(PIN_JOYSTICK) == LOW) {}; // Wait for release
      return 2; // Double Click
    }
  }
  
  return 1; // Timeout reached -> Single Click
}

// --- PID LOGIC ---
void PID_Control(){
  unsigned long current_time = millis(); 
  int delta_time = current_time - last_time; 

  if (delta_time >= T){
    double error = setpoint - sensed_output;
    total_error += error; 
    
    if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;

    double delta_error = error - last_error; 

    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; 

    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;

    last_error = error;
    last_time = current_time;
  } 
}

void performPIDThrow(PitchProfile profile) {
  setpoint    = profile.targetSpeed;
  Kp          = profile.P_val;
  Ki          = profile.I_val;
  Kd          = profile.D_val;
  
  total_error = 0;
  last_error  = 0;
  sensed_output = 0; 
  last_time = millis();
  unsigned long lastPrint = 0;

  Serial.print("Pitching... Target: "); Serial.println(setpoint);

  // RAMP UP
  while (sensed_output < (setpoint - 5)) {
    PID_Control();
    analogWrite(PIN_ENA, (int)control_signal);
    double inertia = 0.05; 
    sensed_output += (control_signal - sensed_output) * inertia;
    delay(1); 
  }

  // HOLD (THROW DURATION)
  unsigned long holdStart = millis();
  while (millis() - holdStart < profile.holdTime) {
    PID_Control();
    analogWrite(PIN_ENA, (int)control_signal);
    double inertia = 0.05;
    sensed_output += (control_signal - sensed_output) * inertia;
    delay(1);
  }

  // STOP
  analogWrite(PIN_ENA, 0);
  Serial.println("Motor Stopped.");
}

void indicateMode(int blinks) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(PIN_LED, HIGH); delay(200);
    digitalWrite(PIN_LED, LOW);  delay(200);
  }
}