//*****************************************************************************************//
//*  TITLE: Unified Relay & Motor Control System                                          *//
//*  AUTHORS: Mohamed Montasser & Omar Emad                                               *//
//*  DATE: 02 MAY 2026                                                                    *//
//*  DESCRIPTION: Combined lighting relay control and Lifting motor direction control.    *//
//*****************************************************************************************//

// --- Relay System Definitions ---
#define RELAY1 17   // FRONT
#define RELAY2 16   // RIGHT
#define RELAY3 2    // LEFT
#define RELAY4 15   // BACK
int relays[4] = {RELAY1, RELAY2, RELAY3, RELAY4};

// --- Motor System Definitions ---
#define sig1 14
#define sig2 25
#define led 13
#define sw0 4           // Emergency stop (active LOW)
#define LimitUp 12
#define LimitDown 33

// Motor State Variables
bool cmdUp  = true;  // true = not pressed (HIGH)
bool cmdDwn = true;  // true = not pressed (HIGH)

String command = "";

void setup() {
  // Common Serial Initialization (Using 115200 from Code 1)
  Serial.begin(115200);
  
  // Setup Relay Pins
  for (int i = 0; i < 4; i++) {
    pinMode(relays[i], OUTPUT);
    digitalWrite(relays[i], HIGH);  // All OFF initially (Active Low)
  }
  pinMode(12, OUTPUT); // Pin 12 from Code 1 setup
  digitalWrite(12, HIGH);

  // Setup Motor Pins
  pinMode(sig1, OUTPUT);
  pinMode(sig2, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(sw0, INPUT_PULLUP);
  pinMode(LimitUp, INPUT_PULLUP);
  pinMode(LimitDown, INPUT_PULLUP);

  Serial.println("System Ready.");
  Serial.println("Relay Commands: 1 1, 1 0, ..., all on, all off");
  Serial.println("Motor Commands: UP, DOWN, STOP");
}

void loop() {
  // ===============================
  // SERIAL COMMAND PROCESSING
  // ===============================
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    
    Serial.print("Received: ");
    Serial.println(command);

    // --- Motor Control Commands ---
    String motorCmd = command;
    motorCmd.toUpperCase();
    if (motorCmd == "UP") {
      cmdUp  = false;
      cmdDwn = true;
    }
    else if (motorCmd == "DOWN") {
      cmdUp  = true;
      cmdDwn = false;
    }
    else if (motorCmd == "STOP") {
      cmdUp  = true;
      cmdDwn = true;
    }

    // --- Relay Control Commands ---
    // ALL ON
    if (command.equalsIgnoreCase("all on")) {
      for (int i = 0; i < 4; i++) digitalWrite(relays[i], LOW);
      Serial.println("All relays ON");
    }
    // ALL OFF
    else if (command.equalsIgnoreCase("all off")) {
      for (int i = 0; i < 4; i++) digitalWrite(relays[i], HIGH);
      Serial.println("All relays OFF");
    }
    // Individual relay control (e.g., "1 1" or "2 0")
    else if (command.length() >= 3 && isDigit(command.charAt(0))) {
      int channel = command.charAt(0) - '0';
      if (channel >= 1 && channel <= 4) {
        if (command.endsWith("1")) {
          digitalWrite(relays[channel - 1], LOW);
          Serial.print("Relay "); Serial.print(channel); Serial.println(" ON");
        }
        else if (command.endsWith("0")) {
          digitalWrite(relays[channel - 1], HIGH);
          Serial.print("Relay "); Serial.print(channel); Serial.println(" OFF");
        }
      }
    }
  }

  // ===============================
  // MOTOR ALGORITHM (UNCHANGED)
  // ===============================
  if (digitalRead(sw0) == 0) {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, LOW);
  } 
  else if (digitalRead(LimitUp) == 0 && cmdDwn == false && cmdUp == true) {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, HIGH);
  } 
  else if (digitalRead(LimitDown) == 0 && cmdDwn == true && cmdUp == false) {
    digitalWrite(sig1, HIGH);
    digitalWrite(sig2, LOW);
  } 
  else if (digitalRead(LimitUp) == 0 || digitalRead(LimitDown) == 0) {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, LOW);
  }
  else if (cmdUp == false && cmdDwn == true) {
    digitalWrite(sig1, HIGH);
    digitalWrite(sig2, LOW);
  } 
  else if (cmdDwn == false && cmdUp == true) {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, HIGH);
  } 
  else {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, LOW);
  }
}
