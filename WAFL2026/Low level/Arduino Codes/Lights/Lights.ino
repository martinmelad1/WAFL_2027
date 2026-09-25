//*******************************************************************************//
//*     TITLE: Serial-Based Relay Control of Lights                              *//
//*     AUTHORS: Mohamed Montasser & Omar Emad                                   *//
//*     DATE: 26/JAN/2026                                                        *//
//*     DESCRIPTION:                                                             *
//*     This program controls four relays connected to an ESP32 using commands   *
//*     sent through the Serial Monitor. Each relay corresponds to a motion      *
//*     direction of the robot (Front, Back, Left, Right).                       *
//*                                                                              *
//*     The relay module is active-low, where:                                   *
//*         LOW  -> Relay ON                                                     *
//*         HIGH -> Relay OFF                                                    *
//*                                                                              *
//*     The program supports:                                                    *
//*       - Individual direction control using commands:                         *
//*           "1 1", "2 0", etc.                                                 *
//*       - Global control using commands:                                       *
//*           "all on", "all off"                                                *
//*                                                                              *
//*******************************************************************************//

#define RELAY1 17   // FRONT
#define RELAY2 16   // RIGHT
#define RELAY3 2    // LEFT
#define RELAY4 15   // BACK

int relays[4] = {RELAY1, RELAY2, RELAY3, RELAY4};

String command = "";

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(relays[i], OUTPUT);
    digitalWrite(relays[i], HIGH);  // All OFF initially
  }
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  Serial.println("Relay Control Ready.");
  Serial.println("Commands: 1 on, 1 off, 2 on, 2 off, ... , all on, all off");
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();   // Remove spaces and \r

    Serial.print("Received: ");
    Serial.println(command);

    // === ALL ON ===
    if (command.equalsIgnoreCase("all on")) {
      for (int i = 0; i < 4; i++) digitalWrite(relays[i], LOW);
      Serial.println("All relays ON");
    }

    // === ALL OFF ===
    else if (command.equalsIgnoreCase("all off")) {
      for (int i = 0; i < 4; i++) digitalWrite(relays[i], HIGH);
      Serial.println("All relays OFF");
    }

    // === Individual control ===
    else {
      int channel = command.charAt(0) - '0';  // '1' -> 1, '2' -> 2, etc.

      if (channel >= 1 && channel <= 4) {
        if (command.endsWith("1")) {
          digitalWrite(relays[channel - 1], LOW);
          Serial.print("Relay ");
          Serial.print(channel);
          Serial.println(" ON");        }
        else if (command.endsWith("0")) {
          digitalWrite(relays[channel - 1], HIGH);
          Serial.print("Relay ");
          Serial.print(channel);
          Serial.println(" OFF");        }
        else {
          Serial.println("Invalid command format.");
        }
      }
      else {
        Serial.println("Invalid relay number. Use 1 to 4.");
      }
    }
  }
}
