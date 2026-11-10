/*
 * Ball balancing on a beam: ESP32 + VL53L0X time-of-flight sensor + stepper motor.
 *
 * Improved version of the original project sketch (see firmware/original_sketch.ino).
 * The control logic is the same as ImprovedController in simulation/ball_beam.py,
 * and the default gains are the ones tuned in that simulation.
 *
 * NOTE: this version has been checked in simulation only, not yet on the hardware.
 * Gains and the motor direction will most likely need adjusting on the real rig.
 *
 * Libraries (Arduino Library Manager): "Adafruit_VL53L0X", "AccelStepper"
 *
 * Wiring (as in the project presentations):
 *   VL53L0X  SDA -> GPIO21, SCL -> GPIO22, VIN -> 3V3, GND -> GND
 *   Driver   PUL+ -> GPIO26 (STEP), DIR+ -> GPIO25 (DIR), PUL-/DIR-/ENA- -> GND
 *   All grounds connected together.
 *   These pins are for the classic ESP32. The ESP32-S2 (e.g. Saola-1) has no GPIO22-25:
 *   there, use for example SDA 8, SCL 9, STEP 17, DIR 18 and change the constants below.
 *
 * Start-up: hold the beam level when powering on. That position becomes angle 0.
 *
 * Serial (115200 baud): prints "distance_mm,setpoint_mm,angle_deg" every cycle,
 * which the Arduino Serial Plotter can draw. Send "kp 2.0", "ki 0.3", "kd 1.5" or
 * "sp 220" to change gains or the setpoint while it runs, and "stop"/"go" to pause.
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <AccelStepper.h>

// ---------------- Hardware settings ----------------
const int STEP_PIN = 26;
const int DIR_PIN = 25;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const bool INVERT_DIRECTION = false;     // flip if the beam tilts the wrong way

const float STEPS_PER_REV = 200 * 8;     // 1.8° motor at 1/8 microstepping: match the driver DIP switches
const float GEAR_RATIO = 1.0;            // beam angle per motor angle through the linkage
const float MAX_ANGLE_DEG = 12.0;        // keep below the mechanical limit of the linkage
const float MAX_STEP_RATE = 4000;        // steps/s
const float STEP_ACCEL = 40000;          // steps/s²

// ---------------- Control settings ----------------
const float BEAM_CENTRE_MM = 240;        // half of the 48 cm beam
const float BALL_RADIUS_MM = 20;         // the sensor sees the near surface of the ball
float setpointMm = BEAM_CENTRE_MM - BALL_RADIUS_MM;

float Kp = 2.2;   // rad of tilt per metre of error
float Ki = 0.2;   // rad per (metre · second)
float Kd = 0.6;   // rad per (metre / second), acts on the measurement
const float INTEGRAL_LIMIT = 0.05;       // m·s, anti-windup clamp

const uint16_t SAMPLE_MS = 20;           // control period = sensor measurement period
const int FILTER_LEN = 8;                // moving-average length
const uint16_t MIN_VALID_MM = 20;        // readings outside this range are ignored
const uint16_t MAX_VALID_MM = 480;

// ---------------- Objects and state ----------------
Adafruit_VL53L0X tof;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

float window[FILTER_LEN];
int windowCount = 0, windowIndex = 0;
float integral = 0;
float lastX = NAN;
bool running = true;
const float RAD_PER_STEP = 2 * PI / STEPS_PER_REV * GEAR_RATIO;

float filtered(float x) {
  window[windowIndex] = x;
  windowIndex = (windowIndex + 1) % FILTER_LEN;
  if (windowCount < FILTER_LEN) windowCount++;
  float sum = 0;
  for (int i = 0; i < windowCount; i++) sum += window[i];
  return sum / windowCount;
}

void controlStep(uint16_t distanceMm) {
  const float dt = SAMPLE_MS / 1000.0;
  float x = filtered(distanceMm / 1000.0);          // metres
  float error = x - setpointMm / 1000.0;             // > 0: ball too far from the sensor

  integral = constrain(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float dx = isnan(lastX) ? 0 : (x - lastX) / dt;   // derivative on measurement
  lastX = x;

  float angle = -(Kp * error + Ki * integral + Kd * dx);   // target beam angle, rad
  float maxAngle = radians(MAX_ANGLE_DEG);
  angle = constrain(angle, -maxAngle, maxAngle);

  long targetSteps = lround(angle / RAD_PER_STEP);
  stepper.moveTo(INVERT_DIRECTION ? -targetSteps : targetSteps);  // absolute position, non-blocking

  Serial.print(distanceMm);
  Serial.print(',');
  Serial.print(setpointMm);
  Serial.print(',');
  Serial.println(degrees(angle), 2);
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  float value = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
  if (cmd.startsWith("kp ")) Kp = value;
  else if (cmd.startsWith("ki ")) { Ki = value; integral = 0; }
  else if (cmd.startsWith("kd ")) Kd = value;
  else if (cmd.startsWith("sp ")) setpointMm = value;
  else if (cmd == "stop") { running = false; stepper.moveTo(0); }
  else if (cmd == "go") { running = true; integral = 0; lastX = NAN; }
  Serial.printf("# Kp=%.3f Ki=%.3f Kd=%.3f setpoint=%.0f mm %s\n",
                Kp, Ki, Kd, setpointMm, running ? "running" : "stopped");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!tof.begin()) {
    Serial.println("# VL53L0X not found: check wiring");
    while (true) delay(1000);
  }
  tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
  tof.startRangeContinuous(SAMPLE_MS);

  stepper.setMaxSpeed(MAX_STEP_RATE);
  stepper.setAcceleration(STEP_ACCEL);
  stepper.setCurrentPosition(0);           // beam is level at power-up

  Serial.println("# distance_mm,setpoint_mm,angle_deg");
}

void loop() {
  stepper.run();                           // must be called as often as possible

  if (tof.isRangeComplete()) {             // a new reading every SAMPLE_MS
    uint16_t d = tof.readRange();
    if (running && d >= MIN_VALID_MM && d <= MAX_VALID_MM) controlStep(d);
  }

  handleSerial();
}
