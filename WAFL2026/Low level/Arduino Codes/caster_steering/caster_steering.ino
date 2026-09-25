/*
===============================================================
  ARDUINO ENCODER-DRIVEN DC MOTOR POSITION CONTROL
---------------------------------------------------------------
  • Reads user-entered angle from Serial Monitor
  • Converts target angle → encoder pulses
  • Rotates a DC motor to target position using encoder feedback
  • Stops precisely with overshoot protection

  Author: Mohamed Montasser & Omar Emad
  Date: 9/12/2025
===============================================================
*/

// ========================
// USER SETTINGS
// ========================

// Encoder pulses per full revolution (PPR)
const float PPR = 23700.0;

// Quadrature encoder input pins
const int pinA = 15;     // Encoder A (interrupt pin)
const int pinB = 2;     // Encoder B

// Motor control pins
const int motorPWM = 13; // PWM speed control pin
const int motorDIR = 14; // Direction pin (HIGH=CW, LOW=CCW)

// Global encoder position counter (updated in ISR)
volatile long encoderCount = 0;


// ========================
// ENCODER INTERRUPT SERVICE ROUTINE
// ========================
/*
  Called on rising edge of encoder A.
  Determines direction by reading encoder B.
*/
void encoderA_ISR() {
  int b = digitalRead(pinB);

  // Quadrature decoding: A leads B (CW), B leads A (CCW)
  if (b == HIGH) encoderCount++;
  else encoderCount--;
}


// ========================
// SETUP
// ========================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Hello ESP");
  // Setup encoder pins
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  // Attach interrupt on rising edge of Encoder A
  attachInterrupt(digitalPinToInterrupt(pinA), encoderA_ISR, RISING);

  // Motor output pins
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);

  Serial.println("Enter target angle (degrees):");
}


// ========================
// MOTOR CONTROL FUNCTIONS
// ========================

// Spin motor clockwise (CW)
void motorForward(int speed) {
  digitalWrite(motorDIR, HIGH);
  analogWrite(motorPWM, speed);
}

// Spin motor counter-clockwise (CCW)
void motorBackward(int speed) {
  digitalWrite(motorDIR, LOW);
  analogWrite(motorPWM, speed);
}

// Stop motor
void motorStop() {
  analogWrite(motorPWM, 0);
}


// ========================
// MAIN LOOP
// ========================
void loop() {

  static bool moving = false;        // Whether a motion command is active
  static long targetPulse = 0;       // Target encoder count for angle
  static bool directionSet = false;  // Only set motor direction once

  // -----------------------------------------------------------
  // SERIAL INPUT: Read user angle and convert to target pulses
  // -----------------------------------------------------------
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Ignore noise or accidental empty input
    if (input.length() == 0) return;

    // Check if input was parsed as zero incorrectly
    if (!input.equals("0") && input.toFloat() == 0) {
      Serial.println("Ignored accidental zero (noise).");
      return;
    }

    // Convert angle → pulses
    float angle = input.toFloat();
    targetPulse = (long)(angle / 360.0 * PPR);

    Serial.print("User angle = ");
    Serial.println(angle);
    Serial.print("Target pulses = ");
    Serial.println(targetPulse);

    moving = true;
    directionSet = false;
  }


  // -----------------------------------------------------------
  // PRINT CURRENT POSITION
  // -----------------------------------------------------------
  long pulses = encoderCount;
  float currentAngle = (pulses * 360.0) / PPR;

  Serial.print("Pulses: ");
  Serial.print(pulses);
  Serial.print("   Angle: ");
  Serial.println(currentAngle);

  delay(50);  // Slow output refresh rate


  // -----------------------------------------------------------
  // MOVEMENT CONTROL LOGIC
  // -----------------------------------------------------------
  if (moving) {

    long error = targetPulse - encoderCount;

    // ---- Set motor direction ONCE at the start of motion ----
    if (!directionSet) {
      if (error > 0) {
        motorForward(120);
        Serial.println("Moving CW...");
      } else {
        motorBackward(120);
        Serial.println("Moving CCW...");
      }
      directionSet = true;
    }

    // ---- Normal stop condition ----
    if (abs(error) < 80) {
      motorStop();
      moving = false;

      Serial.println("==== Reached Target ====");
      Serial.print("Final Pulses: ");
      Serial.println(encoderCount);
      Serial.print("Final Angle: ");
      Serial.println((encoderCount * 360.0) / PPR);
      Serial.println("========================");

      return;
    }

    // ---- Overshoot protection ----
    if (directionSet) {
      
      // If moving CW but error indicates we passed target
      if (error < 0 && digitalRead(motorDIR) == HIGH) {
        motorStop();
        moving = false;
        Serial.println("Overshoot CW — Stopped (No Bounce).");
      }

      // If moving CCW but error indicates we passed target
      if (error > 0 && digitalRead(motorDIR) == LOW) {
        motorStop();
        moving = false;
        Serial.println("Overshoot CCW — Stopped (No Bounce).");
      }
    }
  }
}
