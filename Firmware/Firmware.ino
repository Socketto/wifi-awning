// Pin configuration
const int PIN_UP = 12;
const int PIN_DOWN = 14;
const int RELAY_POWER = 26;
const int RELAY_DIRECTION = 27;
const int ANEMO_PIN = 25;

const int GREEN_LED = 32;
const int RED_LED = 33;

// Wind sensor parameters
volatile unsigned int tickCount = 0;
unsigned long lastWindCheck = 0;
const unsigned long WIND_CHECK_INTERVAL = 1000;
const int WIND_TICK_THRESHOLD = 10;

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

const unsigned long RAISE_TIMEOUT = 60000;
const unsigned long ALARM_RESET_TIMEOUT = 300000;

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
    if (currentMillis - lastBlinkTime >= 250) {
      ledState = !ledState;
      digitalWrite(RED_LED, ledState);
      digitalWrite(GREEN_LED, LOW);
      lastBlinkTime = currentMillis;
    }
  } else {
    if (currentMillis - lastBlinkTime >= 1000) {
      ledState = !ledState;
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
  if (windAlarmActive && currentMillis - alarmStartTime >= ALARM_RESET_TIMEOUT) {
    Serial.println("Wind alarm reset.");
    windAlarmActive = false;
  }

  // --- Raise timeout ---
  if (raisingInProgress && currentMillis - raiseStartTime >= RAISE_TIMEOUT) {
    Serial.println("Raise timeout. Stopping awning.");
    stopAwning();
    raisingInProgress = false;
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
  Serial.println("Manual raise.");
  digitalWrite(RELAY_DIRECTION, LOW);  // Direction: UP
  delay(100);                          // Small delay to settle relays
  digitalWrite(RELAY_POWER, HIGH);    // Power ON
  raisingInProgress = true;
  raiseStartTime = millis();
}

void lowerAwning() {
  Serial.println("Manual lower.");
  digitalWrite(RELAY_DIRECTION, HIGH); // Direction: DOWN
  delay(100);
  digitalWrite(RELAY_POWER, HIGH);     // Power ON
  raisingInProgress = false;
}

void stopAwning() {
  digitalWrite(RELAY_POWER, LOW); // Power OFF
}

void retractAwningDueToAlarm() {
  Serial.println("⚠️ Wind alarm! Retracting awning...");
  digitalWrite(RELAY_DIRECTION, LOW); // Direction: UP
  delay(100);
  digitalWrite(RELAY_POWER, HIGH);    // Power ON
  raisingInProgress = true;
  raiseStartTime = millis();
}
