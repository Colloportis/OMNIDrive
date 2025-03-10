#include <SPI.h> // Required for NRF24L01
#include <RF24.h>  // Required for NRF24L01
#include <AccelStepper.h>

// --- NRF24L01 Configuration (Receiver - Robot) ---
#define CE_PIN   9 // Match Transmitter Code (D9) - Using D9 on Arduino
#define CSN_PIN  10 // Match Transmitter Code (D10) - Using D10 on Arduino
RF24 radio(CE_PIN, CSN_PIN);
const byte chan[6] = "00007"; // MUST match transmitter address

// --- Data structure to receive - MUST match transmitter structure ---
byte data[10]; // Byte array to receive

// --- Define BAUD_RATE for Serial Communication (Debugging) ---
#define BAUD_RATE 115200 // Keep same for debug serial

// --- Motor and Driver Configuration ---
#define X_STEP_PIN 2  // ORIGINAL VALUE
#define X_DIR_PIN 5   // ORIGINAL VALUE
#define Y_STEP_PIN 3  // ORIGINAL VALUE
#define Y_DIR_PIN 6   // ORIGINAL VALUE
#define Z_STEP_PIN 4  // ORIGINAL VALUE
#define Z_DIR_PIN 7   // ORIGINAL VALUE

#define MOTOR_INTERFACE_TYPE 1 // ORIGINAL VALUE

// --- Robot Geometry and Kinematics ---
const float WHEEL_RADIUS = 0.037; // ORIGINAL VALUE
const float ROBOT_RADIUS = 0.133;  // ORIGINAL VALUE
const int STEPS_PER_REVOLUTION = 200; // ORIGINAL VALUE
const int MICROSTEPPING = 1;        // ORIGINAL VALUE
const float WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
const float STEPS_PER_METER = (STEPS_PER_REVOLUTION * MICROSTEPPING) / (2 * PI * WHEEL_RADIUS);

// --- Motor Wheel Angles ---
const float WHEEL_1_ANGLE = 0.0;         // Radians
const float WHEEL_2_ANGLE = 2.0 * PI / 3.0; // Radians
const float WHEEL_3_ANGLE = 4.0 * PI / 3.0; // Radians

// --- AccelStepper Objects ---
AccelStepper stepper1(MOTOR_INTERFACE_TYPE, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, Z_STEP_PIN, Z_DIR_PIN);

// --- Robot State and Control Variables ---
float target_vx = 0;
float target_vy = 0;
float target_vz_degrees = 0;
float target_vz_radians = 0;

// --- Function Prototypes ---
void setMotorSpeeds();
void processCommands();
void updateMotorSpeeds();
void readNrfData(); // New function to read NRF24L01 data

void setup() {
  Serial.begin(BAUD_RATE); // For debugging
  Serial.println("3 Omni Wheel Holonomic Robot - NRF24L01 Control");
  Serial.print("STEPS_PER_METER: "); Serial.println(STEPS_PER_METER);

  // --- NRF24L01 Initialization (Receiver) ---
  radio.begin();
  radio.setAutoAck(true);
  radio.setRetries(3, 10);
  radio.setChannel(110); // Match transmitter channel
  radio.openReadingPipe(0, chan);
  radio.startListening();
  radio.setDataRate(RF24_1MBPS); // Match transmitter data rate
  radio.setPALevel(RF24_PA_LOW);  // Match transmitter power level
  Serial.println("NRF24L01 Receiver Initialized");

  // --- Motor Setup ---
  stepper1.setMaxSpeed(1000.0 * STEPS_PER_METER);
  stepper2.setMaxSpeed(1000.0 * STEPS_PER_METER);
  stepper3.setMaxSpeed(1000.0 * STEPS_PER_METER);

  stepper1.setAcceleration(500.0 * STEPS_PER_METER);
  stepper2.setAcceleration(500.0 * STEPS_PER_METER);
  stepper3.setAcceleration(500.0 * STEPS_PER_METER);

  Serial.println("Serial and Stepper Motors Initialized");
}

void loop() {
  readNrfData();     // Read data from NRF24L01
  processCommands(); // Process joystick commands (convert degrees to radians)
  updateMotorSpeeds(); // Update motor speeds based on target velocities
/*
  // --- DEBUG PRINT - INPUT VELOCITIES ---
  Serial.print("Target VX: "); Serial.print(target_vx);
  Serial.print(", VY: "); Serial.print(target_vy);
  Serial.print(", VZ_rad: "); Serial.println(target_vz_radians);
*/
  stepper1.run();
  stepper2.run();
  stepper3.run();

  delay(1); // Keep loop responsive, adjust if needed.
}

// --- NEW FUNCTION: readNrfData() - Reads Joystick Data from NRF24L01 ---
void readNrfData() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));

    // --- Example Mapping - Adjust these scaling factors as needed for your robot! ---
    float max_vel_linear = 0.4;  // Max linear velocity in m/s (adjust!)
    float max_vel_angular = 120.0; // Max angular velocity in degrees/sec (adjust!)

    // Map joystick X (LX - Forward/Backward) to VX (-max_vel_linear to +max_vel_linear)
    target_vx = map(data[0], 1, 255, max_vel_linear * 100, -max_vel_linear * 100) / 100.0; // Divide by 100.0 to get m/s;

    // Map joystick Y (LY - Left/Right) to VY (-max_vel_linear to +max_vel_linear)
    target_vy = map(data[1], 1, 255, max_vel_linear * 100, -max_vel_linear * 100) / 100.0; // Divide by 100.0 to get m/s;

    // Map joystick RX (Rotation) to VZ_degrees (INVERTED AXIS - +max_vel_angular to -max_vel_angular)
    target_vz_degrees = map(data[4], 1, 255, -max_vel_angular, max_vel_angular); // Inverted range!

    // --- STRAIGHT SIDEWAYS TRANSLATION ASSIST ---
    int straightSidewaysThreshold = 20; // ADJUST THIS VALUE - Experiment with 20, 30, 40, etc.

    // Check if the forward/backward command (LX, data[0]) is near center
    if (abs(data[0] - 127) < straightSidewaysThreshold) {
      target_vx = 0; // If LX is close to center, force VX to zero for straight sideways
    }


    // --- Deadzone handling ---
    if (data[0] == 0) target_vx = 0; // If LX is in deadzone (mapped to 0), stop VX
    if (data[1] == 0) target_vy = 0; // If LY is in deadzone (mapped to 0), stop VY
    if (data[4] == 0) target_vz_degrees = 0; // If RX is in deadzone (mapped to 0)
    // --- Optional: Serial Print for Receiver Debugging ---
    
    Serial.print("Received NRF - LX: "); Serial.print(data[0]);
    Serial.print(", LY: "); Serial.print(data[1]);
    Serial.print(", RX: "); Serial.print(data[4]);
    Serial.print(" | VX: "); Serial.print(target_vx);
    Serial.print(", VY: "); Serial.print(target_vy);
    Serial.print(", VZ_deg: "); Serial.println(target_vz_degrees);
    
  }
}


void processCommands() { // Remains the same - converts degrees to radians
  target_vz_radians = -target_vz_degrees * PI / 180.0; // Robot rotates opposite to joystick direction
}

void updateMotorSpeeds() { // Remains the same - Holonomic Kinematics
  float linearVelocity = target_vx;
  float sidewaysVelocity = target_vy;
  float rotationalVelocity = target_vz_radians;

 /* // --- DEBUG PRINT - VELOCITIES BEFORE KINEMATICS ---
  Serial.print("linVel: "); Serial.print(linearVelocity);
  Serial.print(", sideVel: "); Serial.print(sidewaysVelocity);
  Serial.print(", rotVel_rad: "); Serial.println(rotationalVelocity);
*/
  float wheel1_speed_mps = linearVelocity + rotationalVelocity * ROBOT_RADIUS;
  float wheel2_speed_mps = linearVelocity * cos(WHEEL_2_ANGLE) + sidewaysVelocity * sin(WHEEL_2_ANGLE) + rotationalVelocity * ROBOT_RADIUS;
  float wheel3_speed_mps = linearVelocity * cos(WHEEL_3_ANGLE) + sidewaysVelocity * sin(WHEEL_3_ANGLE) + rotationalVelocity * ROBOT_RADIUS;
/*
  // --- DEBUG PRINT - WHEEL SPEEDS in mps ---
  Serial.print("Wheel 1 mps: "); Serial.print(wheel1_speed_mps);
  Serial.print(", Wheel 2 mps: "); Serial.print(wheel2_speed_mps);
  Serial.print(", Wheel 3 mps: "); Serial.println(wheel3_speed_mps);
*/
  float wheel1_speed_steps_sec = wheel1_speed_mps * STEPS_PER_METER;
  float wheel2_speed_steps_sec = wheel2_speed_mps * STEPS_PER_METER;
  float wheel3_speed_steps_sec = wheel3_speed_mps * STEPS_PER_METER;
/*
  // --- DEBUG PRINT - WHEEL SPEEDS in steps/sec ---
  Serial.print("Wheel 1 steps/sec: "); Serial.print(wheel1_speed_steps_sec);
  Serial.print(", Wheel 2 steps/sec: "); Serial.print(wheel2_speed_steps_sec);
  Serial.print(", Wheel 3 steps/sec: "); Serial.println(wheel3_speed_steps_sec);
*/
  stepper1.setSpeed(wheel1_speed_steps_sec);
  stepper2.setSpeed(wheel2_speed_steps_sec);
  stepper3.setSpeed(wheel3_speed_steps_sec);
}