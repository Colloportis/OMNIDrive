#include "SPI.h"
#include "RF24.h"

// --- NRF24L01 Configuration (Transmitter) ---
#define CE_PIN   9 // Match Robot Code (D9)
#define CSN_PIN  10 // Match Robot Code (D10)
RF24 radio(CE_PIN, CSN_PIN);
const byte chan[6] = "00007"; // MUST match receiver address - Match Robot Code

// --- Joystick Input Pins (Transmitter) - As per your provided code ---
int l_xJoy = A5;
int l_yJoy = A6;
int l_joy = A4; // Unused in robot control for now
int r_xJoy = A2;
int r_yJoy = A3; // Unused in robot control for now
int r_joy = A1; // Unused in robot control for now
int l_bumper = 3; // Unused in robot control for now
int r_bumper = 2; // Unused in robot control for now
int l_pot = A7; // Unused in robot control for now
int r_pot = A0; // Unused in robot control for now

// --- Joystick Calibration ---
const int JOYSTICK_CENTER = 512; // Approximate center reading (adjust if needed)
const int JOYSTICK_DEADZONE = 50; // Dead zone around center

// --- Data structure to send to receiver - MUST match receiver expectations ---
byte data[10]; // Byte array to transmit

void setup() {
  Serial.begin(115200); // Match Robot Code Serial Baud Rate for potential debugging
  Serial.println("NRF24L01 Transmitter - Corrected Joystick Mapping + Power/Rate Fixes");

  pinMode(l_xJoy, INPUT);
  pinMode(l_yJoy, INPUT);
  pinMode(r_xJoy, INPUT);
  pinMode(r_yJoy, INPUT);
  pinMode(l_joy, INPUT_PULLUP);
  pinMode(r_joy, INPUT_PULLUP);
  pinMode(l_bumper, INPUT_PULLUP);
  pinMode(r_bumper, INPUT_PULLUP);
  pinMode(l_pot, INPUT);
  pinMode(r_pot, INPUT);

  radio.begin();
  radio.setAutoAck(true);       // Ensure auto-acknowledgement is enabled
radio.setRetries(3, 10);     // Up to 3 retries, delay of 10*250us between retries
  radio.setChannel(110); // Try channel 80 (example - experiment with other high channel numbers)
  radio.openWritingPipe(chan);
  radio.stopListening();

  // --- Data Rate and Power Level Settings (for stability) ---
  radio.setDataRate(RF24_1MBPS); // Set data rate to 1Mbps for better reliability
  radio.setPALevel(RF24_PA_LOW);  // Set power level to minimum for short-range, can try PA_LOW if needed

  Serial.println("Transmitter Initialized with Rate/Power Settings");
}

void loop() {
  // --- Read Joystick Values and Map to 1-255 range with Center at ~127 ---
  int raw_l_x = analogRead(l_xJoy);
  int raw_l_y = analogRead(l_yJoy);
  int raw_r_x = analogRead(r_xJoy);
  int raw_r_y = analogRead(r_yJoy);

  // Map left joystick X (for robot VX - Forward/Backward)
  data[0] = mapJoystickValue(raw_l_x); // data[0] for left joystick X (VX control)

  // Map left joystick Y (for robot VY - Left/Right)
  data[1] = mapJoystickValue(raw_l_y); // data[1] for left joystick Y (VY control)

  // Map right joystick X (for robot VZ - Rotation)
  data[4] = mapJoystickValue(raw_r_x); // data[4] for right joystick X (VZ control)

  // --- Buttons and Potentiometers - Ignored for now (set to 0) ---
  data[2] = 0; // Left joystick button
  data[3] = 0; // Left bumper
  data[5] = 0; // Right joystick Y (unused for robot control in current code)
  data[6] = 0; // Right joystick button
  data[7] = 0; // Right bumper
  data[8] = 0; // Left potentiometer
  data[9] = 0; // Right potentiometer


  // --- Transmit Data ---
  radio.write(&data, sizeof(data));

  // --- Optional: Serial Print for Transmitter Debugging ---
  
  Serial.print("LX: "); Serial.print(data[0]);
  Serial.print(", LY: "); Serial.print(data[1]);
  Serial.print(", RX: "); Serial.print(data[4]);
  Serial.println("");
  

  delay(50); // Small delay
}

// --- Function to map joystick analog reading to 1-255 range centered at ~127 ---
byte mapJoystickValue(int rawValue) {
  int mappedValue = map(rawValue, 0, 1023, 1, 255); // Map 0-1023 to 1-255 range
  // Apply dead zone around center (optional in transmitter, can be handled in receiver too)
  if (abs(rawValue - JOYSTICK_CENTER) <= JOYSTICK_DEADZONE) {
    mappedValue = 0; // Set to 0 in dead zone -  Receiver interprets 0 as center.
  } else {
    if (mappedValue == 0) mappedValue = 1; // Ensure mapped value is not exactly 0 outside deadzone, keep in 1-255 range. Original map produced 1-255, but just in case.
  }
  return (byte)mappedValue; // Cast to byte for transmission
}