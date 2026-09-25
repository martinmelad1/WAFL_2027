//*******************************************************************************//
//*     TITLE: Distance-based movement control                                  *//
//*     AUTHORS: Omar Emad & Mohamed Montasser                                  *//
//*     DATE: 02/MAY/2025                                                       *//
//*     DESCRIPTION:                                                             *
//*     This program drives a differential robot a specific distance entered     *
//*     through the Serial Monitor. Distance is converted into wheel revolutions *
//*     → encoder pulses → motor commands. Direction is decoded using a          *
//*     quadrature encoder (A+B channels).                                       *
//*     M1:C1:Left     M2:C2:Right        Enc:C2:[A:blue:16 - B:green:17]       *
//*******************************************************************************//


//**************************************//
//* USER CONFIG                        *//
//**************************************//

const float WHEEL_RADIUS = 0.1;     // Wheel radius in meters
const int PULSES_PER_REV = 600;       // Encoder pulses per full encoder shaft rotation

// Gear ratio: wheel_rev = encoder_rev / GEAR_RATIO
// This converts encoder revolutions to actual wheel revolutions
float GEAR_RATIO = (1885.0 / 600.0);

const int MOTOR_SPEED = 50;          // PWM speed for both motors


//**************************************//
//* PINS CONFIG                        *//
//**************************************//

// Motor 1 control pins
const int M1_PWM = 32;
const int M1_DIR = 14;

// Encoder pins
const int ENCODER_B = 17; // Interrupt pin (INT0)
const int ENCODER_A = 16; // Interrupt pin (INT1)

// Motor 2 control pins
const int M2_PWM = 22;
const int M2_DIR = 2;


//**************************************//
//* VARIABLES                          *//
//**************************************//

volatile long encoderCount = 0;   // Signed pulse count (increases forward, decreases backward)
long targetPulses = 0;            // Number of pulses needed to reach target distance
bool motorRunning = false;        // Indicates whether the robot is currently moving

// RPM calculation variables
unsigned long lastTime = 0;
long lastCount = 0;


//**************************************//
//* ENCODER INTERRUPT (Quadrature)     *//
//* Detects direction using A + B      *//
//**************************************//

void encoderISR()
{
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  // If A == B → forward step, else backward
  if (a == b) encoderCount++;
  else encoderCount--;
}


//**************************************//
//* MOTOR CONTROL FUNCTIONS            *//
//**************************************//

// Stops both motors
void motorsStop()
{
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

// Moves both motors forward
void motorsForward()
{
  digitalWrite(M1_DIR, LOW);
  analogWrite(M1_PWM, MOTOR_SPEED);

  digitalWrite(M2_DIR, HIGH);
  analogWrite(M2_PWM, MOTOR_SPEED);
}

// Moves both motors backward
void motorsBackward()
{
  digitalWrite(M1_DIR, HIGH);
  analogWrite(M1_PWM, MOTOR_SPEED);

  digitalWrite(M2_DIR, LOW);
  analogWrite(M2_PWM, MOTOR_SPEED);
}


//**************************************//
//* SETUP                              *//
//**************************************//

void setup()
{
  Serial.begin(115200);

  // Motor pins
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  // Encoder inputs with pull-ups
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Attach interrupt for channel A (RISING only)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  Serial.println("Enter distance in meters (positive = forward, negative = backward):");
}


//**************************************//
//* RPM CALCULATION FUNCTION           *//
//**************************************//

float calculateRPM()
{
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastTime;

  // Update every 200 ms
  if (dt >= 200)
  {
    long diff = encoderCount - lastCount;   // pulses since last check
    lastCount = encoderCount;
    lastTime = currentTime;

    // Convert pulses → encoder rev/sec
    float encoder_rps = (float)diff / PULSES_PER_REV;

    // Convert encoder revs → wheel revs using gear ratio
    float wheel_rps = encoder_rps * (1.0 / GEAR_RATIO);

    return wheel_rps * 60.0; // return RPM
  }
  return -1; // Not enough time passed to update
}


//**************************************//
//* MAIN LOOP                          *//
//**************************************//

void loop()
{
  // ----- READ NEW USER COMMAND -----
  if (Serial.available() && !motorRunning)
  {
    // User inputs distance in meters 
    float distanceMeters = Serial.parseFloat();
    //,2distanceMeters = distanceMeters - (0.15*(distanceMeters/abs(distanceMeters)));
    //float distanceMeters = distanceMeters_ip - (0.15*distanceMeters_ip);

    if (distanceMeters != 0)
    {
      encoderCount = 0; // reset pulse counter

      // ---- Convert distance → wheel revolutions ----
      // circumference = 2πr → revs = distance / circumference
      float wheelRevs = distanceMeters / (2.0 * PI * WHEEL_RADIUS);

      // ---- Convert wheel revs → encoder pulses ----
      float encoderRevsNeeded = wheelRevs * GEAR_RATIO;
      targetPulses = abs(encoderRevsNeeded * PULSES_PER_REV);

      // Determine direction
      if (distanceMeters > 0) motorsForward();
      else motorsBackward();

      motorRunning = true;

      // Debug print
      Serial.print("Target distance (m): ");
      Serial.println(distanceMeters);

      Serial.print("Equivalent wheel revolutions: ");
      Serial.println(wheelRevs, 6);

      Serial.print("Target encoder pulses: ");
      Serial.println(targetPulses);

      lastTime = millis();
      lastCount = 0;
    }

    // Clear Serial buffer
    while (Serial.available()) Serial.read();
  }


  // ----- MOVEMENT MONITORING -----
  if (motorRunning)
  {
    float rpm = calculateRPM();

    // Live feedback every 200ms
    if (rpm >= 0)
    {
      Serial.print("Encoder Count = "); Serial.print(encoderCount);
      Serial.print(" | Wheel RPM = "); Serial.println(rpm);
    }

    // ----- STOP CONDITION -----
    if (abs(encoderCount) >= targetPulses)
    {
      motorsStop();
      motorRunning = false;

      // Compute final actual distance
      float wheel_revs_real =
        (float)encoderCount / (PULSES_PER_REV * GEAR_RATIO);

      float distance = wheel_revs_real * (2 * PI * WHEEL_RADIUS);

      // Print movement results
      Serial.println("---- Movement Completed ----");
      Serial.print("Final encoder pulses: "); Serial.println(encoderCount);
      Serial.print("Actual wheel revs: "); Serial.println(wheel_revs_real, 4);
      Serial.print("Distance moved (m): "); Serial.println(distance, 4);
      Serial.println("--------------------------------");
      Serial.println("Enter next command:");
    }
  }
}
