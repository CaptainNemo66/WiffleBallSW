// Connect to Arduino, Upload/Verfiy, runs automatically, and hit the reset button to run again
// Robert Lim

#include <Servo.h>

// --- HARDWARE PIN DEFINITIONS ---
const int PIN_SERVO   = 9;   // Servo for Arm Angle

// Motor Controller Pins (HW-095 / L298N)
const int PIN_IN1     = 3;   // Direction Pin A
const int PIN_IN2     = 4;   // Direction Pin B
const int PIN_ENA     = 5;   // Speed Control (PWM)

const int PIN_LED     = 13;  // Onboard LED for status

// --- USER CONFIGURATION  ---

// CHANGE THIS VARIABLE TO SWITCH MODES:
// 1 = EASY MODE (Slower, Higher Angle)
// 2 = HARD MODE (Faster, Flatter Angle)
const int SELECTED_MODE = 1; 

// Configure angles:
const int ANGLE_EASY_MODE = 120; 
const int ANGLE_HARD_MODE = 90;  

// --- PRESET DEFINITIONS ---
struct PitchProfile {
  int servoAngle;     // Trajectory
  double targetSpeed; // Setpoint
  double P_val;       
  double I_val;       
  double D_val;       
  int holdTime;       // TIME TUNING: Adjust this if it pitches more than once!
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
  Serial.begin(115200);
  
  // Pin Setup
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  
  aimServo.attach(PIN_SERVO);
  
  // Set Motor Direction
  digitalWrite(PIN_IN1, HIGH); 
  digitalWrite(PIN_IN2, LOW);
  
  Serial.println("--- SYSTEM START ---");
  Serial.println("SAFETY DELAY: 2 Seconds");
  delay(2000); // Gives you time to move hand after pressing Reset

  // 1. SELECT MODE (MANUAL VARIABLE SWAP)
  PitchProfile selectedProfile;
  
  if (SELECTED_MODE == 1) {
    selectedProfile = easyPitch;
    indicateMode(1); // Blink once
    Serial.println("Mode Selected: EASY (via Code Variable)");
  } else {
    selectedProfile = hardPitch;
    indicateMode(2); // Blink twice
    Serial.println("Mode Selected: HARD (via Code Variable)");
  }

  // 2. SET SERVO ANGLE
  Serial.print("Setting Angle: "); Serial.println(selectedProfile.servoAngle);
  aimServo.write(selectedProfile.servoAngle);
  delay(800); // Wait for servo to move

  // 3. EXECUTE THROW
  performPIDThrow(selectedProfile);

  Serial.println("Throw Complete. Press RESET button to throw again.");
}

void loop() {
  // Do nothing. Waiting for user to press Reset button to restart setup().
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