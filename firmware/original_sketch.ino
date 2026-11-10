// Original project sketch, transcribed from the screenshot in
// docs/4_further_developments.pdf ("For PID tuning" slide). Kept unchanged for reference.
// The improved version is in ball_on_beam/ball_on_beam.ino; the README explains the differences.

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// TOF Sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Stepper Motor Pins
#define DIR_PIN 25 // Direction pin
#define STEP_PIN 26 // Step pin

// PID Constants
float Kp = 1.2;  // Proportional gain
float Ki = 0.5;  // Integral gain
float Kd = 0.1;  // Derivative gain

// PID Variables
float setpoint = 100; // Desired position in mm
float input = 0;      // Current position from TOF sensor
float output = 0;     // PID output
float error = 0;
float lastError = 0;
float integral = 0;

// Timing Variables
unsigned long lastTime = 0;
float sampleTime = 50; // Sample time in ms


void setup() {
  Serial.begin(115200);

  // Initialize TOF sensor
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1);
  }
  Serial.println("VL53L0X started");

  // Initialize Stepper Motor Pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
}

void loop() {
  // Get ball position from TOF sensor
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    input = measure.RangeMilliMeter; // Current position in mm
  } else {
    Serial.println("Out of range");
    return;
  }

  // PID Calculation
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= sampleTime) {
    error = setpoint - input;
    integral += error * (currentTime - lastTime);
    float derivative = (error - lastError) / (currentTime - lastTime);

    output = Kp * error + Ki * integral + Kd * derivative;

    lastError = error;
    lastTime = currentTime;

    // Map output to motor control
    controlMotor(output);
  }

  // Debugging
  Serial.print("Input: ");
  Serial.print(input);
  Serial.print(" Output: ");
  Serial.println(output);
}

void controlMotor(float pidOutput) {
  // Convert PID output to stepper motor steps and direction
  int steps = abs(pidOutput);
  bool direction = pidOutput > 0;

  digitalWrite(DIR_PIN, direction); // Set direction
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500); // Adjust for motor speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
}
