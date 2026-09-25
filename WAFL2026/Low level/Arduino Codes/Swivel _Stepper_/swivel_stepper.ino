//*******************************************************************************//
//*     TITLE: Stepper Motor Angle Control                                      *//
//*     AUTHORS: Omar Emad & Mohamed Montasser                                  *//
//*     DATE: 01/DEC/2025                                                       *//
//*     DESCRIPTION:                                                             *
//*     This program drives a NEMA17 stepper motor (with planetary gearbox)      *
//*     to a specific angle entered through the Serial Monitor.                  *
//*     Angle is converted into driver pulses based on a calibrated PPR value.   *
//*     Supports bidirectional movement and absolute angle tracking.             *
//*     RED-> A+          BLACK-> B+            BLUE-> A-           GREEN->B-    *
//*     DIR-> 32                  PULS-> 27                  EN-> 33 (GND)       * 
//*******************************************************************************//


//**************************************//
//* USER CONFIG                        *//
//**************************************//

/**
 * Pulses Per Revolution (PPR) for the output shaft
 * Calibrated for 17HS4401S-PG518 (5.18:1 gear ratio)
 */
#define PULSES_PER_REV 8250

/**
 * Step pulse delay in microseconds
 * Controls motor speed: lower value = faster rotation
 * Recommended: 800-1000 µs for smooth high-torque motion
 */
unsigned int step_delay_us = 800;


//**************************************//
//* PINS CONFIG                        *//
//**************************************//

// TB6600 Driver Pins
const int STEP_PIN = 27;  // PUL (Pulse/Step)
const int DIR_PIN = 32;   // DIR (Direction)
const int ENA_PIN = 33;   // DIR (Direction)

// Note: ENA (Enable) is left unconnected (or tied to 5V)


//**************************************//
//* VARIABLES                          *//
//**************************************//

unsigned long pulse_count = 0;      // Pulses sent in current movement
unsigned long pulses_needed = 0;    // Pulses required for target angle
boolean moving = false;             // Indicates if motor is currently rotating

float absolute_angle = 0.0;         // Track total angle moved since start


//**************************************//
//* SETUP                              *//
//**************************************//

void setup()
{
  // Initialize Serial communication
  Serial.begin(9600);
  delay(2000);
  Serial.println("HELLO ESP32");

  // Configure GPIO pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  
  // Set initial state
  digitalWrite(DIR_PIN, HIGH);    // Default direction
  digitalWrite(STEP_PIN, LOW);    // Default step state
  digitalWrite(ENA_PIN, LOW);    // Default ENA state

  // Print welcome message
  Serial.println("=== Stepper Motor Angle Control ===");
  Serial.println("Enter angle (positive or negative):");
  Serial.println("Examples: 90, -45, 180, -720, 360");
  Serial.println("Absolute angle: 0.0°");
}


//**************************************//
//* MAIN LOOP                          *//
//**************************************//

void loop()
{
  // ----- READ NEW USER COMMAND -----
  if (Serial.available() > 0)
  {
    // User inputs angle in degrees
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    float angle = input.toFloat();
    
    // ---- Convert angle → stepper pulses ----
    long pulses_signed = (long)((angle / 360.0) * PULSES_PER_REV);
    pulses_needed = abs(pulses_signed);
    pulse_count = 0;
    moving = true;

    // ---- Set Direction ----
    if (angle >= 0) {
      digitalWrite(DIR_PIN, HIGH);  // Clockwise
      Serial.print("Moving CW +");
    } else {
      digitalWrite(DIR_PIN, LOW);   // Counter-clockwise
      Serial.print("Moving CCW ");
    }
    
    // Debug print
    Serial.print(angle);
    Serial.print(" deg (");
    Serial.print(pulses_needed);
    Serial.println(" pulses)...");
    
    // Update absolute tracking
    absolute_angle += angle;
    Serial.print("Total angle from start: ");
    Serial.print(absolute_angle);
    Serial.println(" deg");
  }
  
  
  // ----- MOVEMENT EXECUTION -----
  if (moving && pulse_count < pulses_needed)
  {
    // Generate Step Pulse (TB6600 requires >2.2us pulse width)
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);      
    digitalWrite(STEP_PIN, LOW);
    
    // Speed control delay
    delayMicroseconds(step_delay_us);
    
    pulse_count++;
  }
  
  
  // ----- STOP CONDITION -----
  if (moving && pulse_count >= pulses_needed)
  {
    moving = false;
    
    Serial.println("Movement complete!");
    Serial.print("Absolute angle: ");
    Serial.print(absolute_angle);
    Serial.println(" deg");
    Serial.println("Enter next angle:");
  }
}
