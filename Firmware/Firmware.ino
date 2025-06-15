// Pin configuration
const int PIN_UP = 34;
const int PIN_DOWN = 35;
const int RELAY_POWER = 23;
const int RELAY_DIRECTION = 2;
const int ANEMO_PIN = 13;

const int GREEN_LED = 27;
const int RED_LED = 26;//25

// Wind sensor parameters
volatile unsigned int tickCount = 256;
unsigned long lastWindCheck = 0;
const unsigned long WIND_CHECK_INTERVAL = 3000;
const int WIND_TICK_THRESHOLD = 5;

// Debounce
unsigned long lastDebounceTimeUP = 0;
unsigned long lastDebounceTimeDOWN = 0;
const unsigned long DEBOUNCE_DELAY = 50;
bool lastButtonStateUP = HIGH;
bool lastButtonStateDOWN = HIGH;
bool buttonStateUP = HIGH;
bool buttonStateDOWN = HIGH;

// State
bool windAlarmActive = false;
unsigned long alarmStartTime = 0;
bool raisingInProgress = false;
unsigned long raiseStartTime = 0;
unsigned short countergreen = 0;

const unsigned long ALARM_RESET_TIMEOUT = 60000; 

// LED blink
unsigned long lastBlinkTime = 0;
bool ledState = false;

void IRAM_ATTR windSensorISR() {
  tickCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);
  pinMode(RELAY_POWER, OUTPUT);
  pinMode(RELAY_DIRECTION, OUTPUT);
  pinMode(ANEMO_PIN, INPUT_PULLUP);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ANEMO_PIN), windSensorISR, FALLING);

  digitalWrite(RELAY_POWER, LOW);
  digitalWrite(RELAY_DIRECTION, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  // --- LED blinking ---
  if (windAlarmActive) {
    if (currentMillis - lastBlinkTime >= 100) {
      ledState = !ledState;
      digitalWrite(RED_LED, ledState);
      digitalWrite(GREEN_LED, LOW);
      lastBlinkTime = currentMillis;
    }
  } else {
    if (currentMillis - lastBlinkTime >= 100) {
      countergreen++;
      ledState = countergreen % 50 == 0;
      digitalWrite(GREEN_LED, ledState);
      digitalWrite(RED_LED, LOW);
      lastBlinkTime = currentMillis;
    }
  }

  // --- Wind check ---
  if (currentMillis - lastWindCheck >= WIND_CHECK_INTERVAL) {
    lastWindCheck = currentMillis;

    Serial.print("Wind tick/s: ");
    Serial.println(tickCount);

    if (tickCount >= WIND_TICK_THRESHOLD && !windAlarmActive) {
      windAlarmActive = true;
      alarmStartTime = currentMillis;
      retractAwningDueToAlarm();
    }

    tickCount = 0;
  }

  // --- Wind alarm reset ---
  if (windAlarmActive)
  { 
    if(currentMillis - alarmStartTime >= ALARM_RESET_TIMEOUT) {
      Serial.println("Wind alarm reset.");
      raisingInProgress = false;
      windAlarmActive = false;
    }
    else
    {
      raiseAwning();
    }
  }

  // --- Button debounce logic ---
  bool readingUP = digitalRead(PIN_UP);
  if (readingUP != lastButtonStateUP) {
    lastDebounceTimeUP = currentMillis;
  }
  if ((currentMillis - lastDebounceTimeUP) > DEBOUNCE_DELAY) {
    if (readingUP != buttonStateUP) {
      buttonStateUP = readingUP;
      if (buttonStateUP == LOW && !windAlarmActive) {
        raiseAwning();
      }
    }
  }
  lastButtonStateUP = readingUP;

  bool readingDOWN = digitalRead(PIN_DOWN);
  if (readingDOWN != lastButtonStateDOWN) {
    lastDebounceTimeDOWN = currentMillis;
  }
  if ((currentMillis - lastDebounceTimeDOWN) > DEBOUNCE_DELAY) {
    if (readingDOWN != buttonStateDOWN) {
      buttonStateDOWN = readingDOWN;
      if (buttonStateDOWN == LOW && !windAlarmActive) {
        lowerAwning();
      }
    }
  }
  lastButtonStateDOWN = readingDOWN;

  // Stop motor if no button pressed and no automatic raise
  if (buttonStateUP == HIGH && buttonStateDOWN == HIGH && !raisingInProgress) {
    stopAwning();
  }
}

// --- Relay logic updated to support double-deviator wiring ---

void raiseAwning() {
  Serial.println("UP.");
  digitalWrite(RELAY_DIRECTION, LOW);  // Direction: UP
  delay(100);                          // Small delay to settle relays
  digitalWrite(RELAY_POWER, HIGH);    // Power ON
}

void lowerAwning() {
  Serial.println("DOWN.");
  digitalWrite(RELAY_DIRECTION, HIGH); // Direction: DOWN
  delay(100);
  digitalWrite(RELAY_POWER, HIGH);     // Power ON
}

void stopAwning() {
  digitalWrite(RELAY_POWER, LOW); // Power OFF
  delay(100);
  digitalWrite(RELAY_DIRECTION, LOW); // Direction: DOWN
  Serial.println("stopAwning");
}

void retractAwningDueToAlarm() {
  Serial.println("Wind alarm! Retracting awning...");
  raisingInProgress = true;
  raiseStartTime = millis();
}
