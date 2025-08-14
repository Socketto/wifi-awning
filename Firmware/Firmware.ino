#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "Credentials.h"
#include "time.h"

//#define TestBoard
const char* timezone = "CET-1CEST,M3.5.0/2,M10.5.0/3";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

#ifndef TestBoard
// --- WiFi & MQTT Configuration ---
const char *mqtt_topic_command = "awning/command";
const char *mqtt_topic_state = "awning/state";
const char *mqtt_topic_alarm = "awning/alarm";
const char *mqtt_topic_info = "awning/info";
const char *mqtt_topic_learn = "awning/learn";
#else
// --- WiFi & MQTT Configuration ---
const char *mqtt_topic_command = "awning1/command";
const char *mqtt_topic_state = "awning1/state";
const char *mqtt_topic_alarm = "awning1/alarm";
const char *mqtt_topic_info = "awning1/info";
const char *mqtt_topic_learn = "awning1/learn";
#endif
const char *mqtt_topic_log = "bot_log";
char TempString[2048];

bool isMovingUp = false;
bool isMovingDown = false;
bool LCDPresent = false;
bool FirstStart = true;
bool ReconnectionSuccess = true;

WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 4);

// Pin configuration
const int PIN_UP = 34;
const int PIN_DOWN = 35;
const int RELAY_POWER = 23;
const int RELAY_DIRECTION = 2;
const int CURRENT_SENSOR_PIN = 32;
const int ANEMO_PIN = 13;
unsigned long currentNormal = 0;
unsigned long currentRun = 0;
unsigned long useCurrent = 0;
int CounterWifiLost = 0;
unsigned long CURRENT_THRESHOLD = 820;
const unsigned long CURRENT_ZERO_DELAY = 500;
volatile unsigned long ActualCurrent;
volatile bool UpdateState = false;
bool movingToTarget = false;
const int GREEN_LED = 27;
const int RED_LED = 26;  // 25
enum LearningState {
  LEARNING_IDLE = 0,
  LEARNING_INIT,
  LEARNING_CURRENT_CONSUPTION,
  LEARNING_WAIT_OPEN_INIT,
  LEARNING_WAIT_CLOSE,
  LEARNING_WAIT_OPEN,
  LEARNING_DONE = 255
};
LearningState LearningStateMachine = LEARNING_IDLE;

// Wind sensor parameters
#ifndef TestBoard
volatile unsigned int tickCount = 256;
#else
volatile unsigned int tickCount = 0;
#endif
volatile unsigned int MAXtickCount = 0;
unsigned long lastWindCheck = 0;
const unsigned long WIND_CHECK_INTERVAL = 3000;
unsigned long WIND_TICK_THRESHOLD = 40;


bool AlarmConsuption = false;
bool PrecAlarmConsuption = false;
unsigned long lastConsuptionCheck = 0;
bool wifiConnected = false;
unsigned long lastWifiCheck = 0;
unsigned long lastMQTTCheck = 0;
unsigned long lastwindMQTTCheck = 0;
unsigned long learnTimer = 0;
unsigned long millisStop = 0;
// Debounce
unsigned long lastDebounceTimeUP = 0;
unsigned long lastDebounceTimeDOWN = 0;
const unsigned long DEBOUNCE_DELAY = 50;
bool lastButtonStateUP = HIGH;
bool lastButtonStateDOWN = HIGH;
// --- Preferences ---
Preferences prefs;
// State

unsigned long openDuration = 0;
unsigned long closeDuration = 0;
unsigned long LASTopenDuration = 0;
unsigned long LASTcloseDuration = 0;
unsigned long targetMoveDuration = 0;
unsigned long actualPositionTime = 0;
volatile unsigned long DoNotRestart = 0;

volatile unsigned long currentMillis = millis();
volatile bool windAlarmActive = false;
volatile unsigned long alarmStartTime = 0;
volatile bool raisingInProgress = false;
unsigned short countergreen = 0;

const unsigned long ALARM_RESET_TIMEOUT = 60000 * 7;  // 7 minutes
const unsigned long ALARM_STOP_UP_TIMEOUT = 60000;

// LED blink
unsigned long lastBlinkTime = 0;
bool ledState = false;
volatile unsigned long lastInterruptTime = 0;

void IRAM_ATTR windSensorISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > 10) {  // 10ms
    tickCount++;
    lastInterruptTime = currentTime;
  }
}

bool first_read_time = true;
void CheckLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  } else {
    if (timeinfo.tm_hour == 19 && timeinfo.tm_min == 30 && windAlarmActive == false)
    {
        windAlarmActive = true;
        alarmStartTime = currentMillis;
        retractAwningDueToAlarm();
        sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Auto close awning for the end of the day 19.30\"}");
        client.publish(mqtt_topic_log, TempString);
    }
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

bool devicePresent(byte address) {
  Wire.begin(21, 22);     // SDA, SCL
  Wire.setClock(100000);  // Imposta frequenza a 100 kHz
  delay(2000);
  Serial.println("Scansione I2C...");
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

int rssi_to_percentage(int rssi) {
  const int RSSI_MIN = -100;
  const int RSSI_MAX = -30;

  if (rssi < RSSI_MIN) {
    rssi = RSSI_MIN;
  } else if (rssi > RSSI_MAX) {
    rssi = RSSI_MAX;
  }

  int percentage = ((rssi - RSSI_MIN) * 100) / (RSSI_MAX - RSSI_MIN);
  return percentage;
}

void setup_wifi() {
  Serial.println("WiFi...");
  Serial.println(personalSSID);
  Serial.println(personalSSIDPWD);
#ifndef TestBoard
  WiFi.hostname("Esp32Awning1");
#else
  WiFi.hostname("Esp32AwningTest");
#endif
  WiFi.begin(personalSSID, personalSSIDPWD);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.println("Connecting to WiFi...");
  delay(1000);
}

void callback(char *topic, byte *message, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++)
    msg += (char)message[i];
  msg.trim();

  if (msg == "learn") {
    prefs.begin("awn", false);
    prefs.putULong("useCurrent", 1);
    useCurrent = 1;
    prefs.end();
    startLearning();
  } else if (msg == "endday") {
    windAlarmActive = true;
    alarmStartTime = currentMillis;
    retractAwningDueToAlarm();
    sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Close awning for the end of the day from BOT\"}");
    client.publish(mqtt_topic_log, TempString);
  } else if (msg == "reboot") {
    ESP.restart();
  } else if (msg == "state") {
    UpdateState = true;
  } else if (msg == "current") {
    sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"ActualCurrent: %u\" }", ActualCurrent);
    client.publish(mqtt_topic_log, TempString);
    sprintf(TempString, "{\"ActualCurrent\": %u }", ActualCurrent);
    client.publish(mqtt_topic_info, TempString);
  } else if (msg == "wind") {
    sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Wind: %u\" }", tickCount);
    client.publish(mqtt_topic_log, TempString);
    sprintf(TempString, "{\"Wind\": %u }", tickCount);
    client.publish(mqtt_topic_info, TempString);
  } else if (msg == "info") {
    sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Request info: openDuration: %u, closeDuration: %u, actualPositionTime: %u, targetMoveDuration: %u, WIND_TICK_THRESHOLD: %u, currentNormal: %u, currentRun: %u, CURRENT_THRESHOLD: %u, WiFi_signal : %u, useCurrent : %u\"}", openDuration, closeDuration, actualPositionTime, targetMoveDuration, WIND_TICK_THRESHOLD, currentNormal, currentRun, CURRENT_THRESHOLD, rssi_to_percentage(WiFi.RSSI()), useCurrent);
    client.publish(mqtt_topic_log, TempString);
    sprintf(TempString, "{\"openDuration\": %u , \"closeDuration\": %u , \"actualPositionTime\": %u , \"targetMoveDuration\": %u, \"WIND_TICK_THRESHOLD\": %u, \"currentNormal\": %u, \"currentRun\": %u, \"CURRENT_THRESHOLD\": %u, \"WiFi_signal\" : %u}", openDuration, closeDuration, actualPositionTime, targetMoveDuration, WIND_TICK_THRESHOLD, currentNormal, currentRun, CURRENT_THRESHOLD, rssi_to_percentage(WiFi.RSSI()));
    client.publish(mqtt_topic_info, TempString);
  } else if (msg.startsWith("set ")) {
    float percent = msg.substring(4).toFloat();
    moveToPercentage(percent);
  } else if (msg.startsWith("setDuration ")) {
    closeDuration = msg.substring(12).toInt();
    openDuration = msg.substring(12).toInt();
    prefs.begin("awn", false);
    prefs.putULong("openTime", openDuration);
    prefs.putULong("closeTime", closeDuration);
    prefs.putULong("useCurrent", 0);
    useCurrent = 0;
    prefs.end();
  } else if (msg.startsWith("setTick ")) {
    WIND_TICK_THRESHOLD = msg.substring(8).toInt();
    prefs.begin("awn", false);
    prefs.putULong("tickThreshold", WIND_TICK_THRESHOLD);
    prefs.end();
    sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"setTick %ld\"}", WIND_TICK_THRESHOLD);
    client.publish(mqtt_topic_log, TempString);
  }
}

void reconnect() {
  if (!client.connected()) {
    client.setServer(mqtt_server, 1883);
    client.setBufferSize(2048);
    client.setCallback(callback);
#ifndef TestBoard
    if (client.connect("ESP32_Awning", mqtt_user_chars, mqtt_Password_chars)) {
      client.subscribe(mqtt_topic_command);  // iscriviti al topic solo dopo la connessione
      Serial.println("MQTT connesso e iscritto al topic.");
      sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"MQTT_CONNECTED\"}");
      client.publish(mqtt_topic_log, TempString);
    } else {
      Serial.print("MQTT connection failed. Code: ");
      Serial.println(client.state());  // utile per debug
    }
#else
    if (client.connect("ESP32_AwningTest", mqtt_user_chars, mqtt_Password_chars)) {
      client.subscribe(mqtt_topic_command);  // iscriviti al topic solo dopo la connessione
      Serial.println("MQTT connesso e iscritto al topic.");
      sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"MQTT_CONNECTED\"}");
      client.publish(mqtt_topic_log, TempString);
    } else {
      Serial.print("MQTT connection failed. Code: ");
      Serial.println(client.state());  // utile per debug
    }
#endif
  }
}

int readCurrent() {
  int raw = analogRead(CURRENT_SENSOR_PIN);
  if (LCDPresent) {
    lcd.setCursor(0, 3);
    lcd.print("                ");
    lcd.setCursor(0, 3);
    lcd.print(raw);
  }
  return raw;
}

bool isCurrentZero() {
  static unsigned long belowThresholdStart = 0;
  int current = readCurrent();

  if (useCurrent == 0) {
    return false;
  }

  if (current < CURRENT_THRESHOLD) {
    if (belowThresholdStart == 0)
      belowThresholdStart = millis();
    else if (millis() - belowThresholdStart >= CURRENT_ZERO_DELAY)
      return true;
  } else {
    belowThresholdStart = 0;
  }
  return false;
}

void setup() {
  Serial.begin(115200);

#ifdef TestBoard
  Serial.println("TestBoard");
  delay(10000);
#endif
  pinMode(CURRENT_SENSOR_PIN, INPUT);
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
  if (devicePresent(0x27)) {
    LCDPresent = true;
  }
  Serial.print("LCD ");
  Serial.println(LCDPresent);
  Serial.println("");
  setup_wifi();
  prefs.begin("awn", true);
  client.setServer(mqtt_server, 1883);
  client.setBufferSize(2048);
  client.setCallback(callback);

  currentNormal = prefs.getULong("currentNormal", 730);
  currentRun = prefs.getULong("currentRun", 850);
  CURRENT_THRESHOLD = prefs.getULong("CURRENT_THRESHOLD", 820);
  openDuration = prefs.getULong("openTime", 10);
  useCurrent = prefs.getULong("useCurrent", 0);
  closeDuration = prefs.getULong("closeTime", 10);
  WIND_TICK_THRESHOLD = prefs.getULong("tickThreshold", 40);

  DoNotRestart = prefs.getULong("DoNotRestart", 0);
  if(DoNotRestart > 0 && DoNotRestart < 3)
  {
     tickCount = 0;
  }

  prefs.end();

  Serial.println("Awning controller ready");

  if (LCDPresent) {
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Sistema pronto");
    delay(1000);
    lcd.clear();
  }
   
  configTzTime(timezone, ntpServer);
  CheckLocalTime();
}

void loop() {
  currentMillis = millis();
  ActualCurrent = readCurrent();
  CheckLocalTime();
  if (ActualCurrent > (currentRun + (currentRun - currentNormal))) {
    AlarmConsuption = true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      Serial.println("WiFi lost, reconnecting...");
      wifiConnected = false;
    }
  }
  
  if (currentMillis - lastConsuptionCheck >= 500) {
    lastConsuptionCheck = currentMillis;

    if (AlarmConsuption && PrecAlarmConsuption) {
      if (wifiConnected && client.connected())
      {
        sprintf(TempString, "{\"wind\": %u , \"state\": \"ABNORMAL CONSUPTION %ld\"}", tickCount, ActualCurrent);
        client.publish(mqtt_topic_alarm, TempString);
        sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"ABNORMAL CONSUPTION %ld\"}", ActualCurrent);
        client.publish(mqtt_topic_log, TempString);
      }
    }
    if (AlarmConsuption) {
      PrecAlarmConsuption = true;
      AlarmConsuption = false;
    } else {
      PrecAlarmConsuption = false;
    }
  }

  // --- Check wifi every 30 seconds ---
  if (currentMillis - lastWifiCheck >= 30000) {
    lastWifiCheck = currentMillis;

    if (WiFi.status() != WL_CONNECTED) {
      if (wifiConnected) {
        Serial.println("WiFi lost, reconnecting...");
        wifiConnected = false;

      } else {
        Serial.println("try WiFi reconnection...");
        Serial.println(CounterWifiLost);
        WiFi.disconnect();
        WiFi.reconnect(); 
        CounterWifiLost++;
        if(CounterWifiLost > 119) //30 mins
        {
          prefs.begin("awn", false);
          DoNotRestart++;
          prefs.putULong("DoNotRestart", DoNotRestart);
          prefs.end();
          ESP.restart();
        }
      }
    } else {
      if (!wifiConnected) {
        Serial.println("WiFi reconnected");
        if(DoNotRestart > 0)
        {
          prefs.begin("awn", false);
          prefs.putULong("DoNotRestart", 0);
          DoNotRestart = 0;
          prefs.end();
        }
        wifiConnected = true;
      }
      CounterWifiLost = 0;
    }
  }

  if (currentMillis - lastwindMQTTCheck >= 600000 || UpdateState) {
    UpdateState = false;
    if (wifiConnected && client.connected()) {
      lastwindMQTTCheck = currentMillis;
      sprintf(TempString, "{\"wind\": %u , \"WiFi_signal\" : %u, \"position\" : %u}", MAXtickCount, rssi_to_percentage(WiFi.RSSI()),(unsigned int)((float)((float)actualPositionTime/(float)openDuration) * 100));
      client.publish(mqtt_topic_state, TempString);
      sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"wind: %u , WiFi_signal : %u, position : %u\"}", MAXtickCount, rssi_to_percentage(WiFi.RSSI()),(unsigned int)((float)((float)actualPositionTime/(float)openDuration) * 100));
      client.publish(mqtt_topic_log, TempString);
      MAXtickCount = 0;
    }
  }

  if (wifiConnected) {
    if (currentMillis - lastMQTTCheck >= 10000) {
      lastMQTTCheck = currentMillis;
      if (!client.connected())
      {
        reconnect();
        ReconnectionSuccess = true;
      }
    }
    if (client.connected()) {
      client.loop();
    }
    else
    {
      Serial.println("client not connected");
    }
  }

  if (LearningStateMachine != LEARNING_IDLE) {
    switch (LearningStateMachine) {
      case LEARNING_INIT:
        LASTopenDuration = openDuration;
        LASTcloseDuration = closeDuration;
        learnTimer = currentMillis;
        LearningStateMachine = LEARNING_CURRENT_CONSUPTION;
        client.publish(mqtt_topic_learn, "LEARNING_CURRENT_CONSUPTION");
        sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Learn status: LEARNING_CURRENT_CONSUPTION\"}");
        client.publish(mqtt_topic_log, TempString);
        break;
      case LEARNING_CURRENT_CONSUPTION:
        stopAwning();
        delay(1000);
        lowerAwning();
        delay(7000);
        stopAwning();
        delay(2000);
        raiseAwning();
        delay(2000);
        currentRun = readCurrent();
        stopAwning();
        delay(1000);
        currentNormal = readCurrent();
        CURRENT_THRESHOLD = currentNormal + ((currentRun - currentNormal) / 2);
        LearningStateMachine = LEARNING_WAIT_OPEN_INIT;
        client.publish(mqtt_topic_learn, "LEARNING_WAIT_OPEN_INIT");
        sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Learn status: LEARNING_WAIT_OPEN_INIT\"}");
        client.publish(mqtt_topic_log, TempString);
        raiseAwning();
        delay(500);
        break;
      case LEARNING_WAIT_OPEN_INIT:
        if (isCurrentZero() || currentMillis - learnTimer >= 60000) {
          stopAwning();
          delay(1000);
          learnTimer = currentMillis;
          openDuration = currentMillis;
          LearningStateMachine = LEARNING_WAIT_CLOSE;
          client.publish(mqtt_topic_learn, "LEARNING_WAIT_CLOSE");
          sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Learn status: LEARNING_WAIT_CLOSE\"}");
          client.publish(mqtt_topic_log, TempString);
          lowerAwning();
          delay(500);
        } else {
          raiseAwning();
        }
        break;
      case LEARNING_WAIT_CLOSE:
        if (isCurrentZero() || currentMillis - learnTimer >= 90000) {
          if (currentMillis - learnTimer >= 90000) {
            prefs.begin("awn", false);
            prefs.putULong("useCurrent", 0);
            useCurrent = 0;
            prefs.end();
            LearningStateMachine = LEARNING_DONE;
            client.publish(mqtt_topic_learn, "LEARNING_ERROR");
            openDuration = LASTopenDuration;
            closeDuration = LASTcloseDuration;
            sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Learn status: LEARNING_ERROR\"}");
            client.publish(mqtt_topic_log, TempString);
            actualPositionTime = 0;
          } else {
            openDuration = currentMillis - openDuration;
            stopAwning();
            delay(1000);
            learnTimer = currentMillis;
            closeDuration = currentMillis;
            LearningStateMachine = LEARNING_WAIT_OPEN;
            client.publish(mqtt_topic_learn, "LEARNING_WAIT_OPEN");
            sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Learn status: LEARNING_WAIT_OPEN\"}");
            client.publish(mqtt_topic_log, TempString);
            raiseAwning();
            delay(500);
          }
        } else {
          lowerAwning();
        }
        break;
      case LEARNING_WAIT_OPEN:
        if (isCurrentZero() || currentMillis - learnTimer >= 90000) {
          if (currentMillis - learnTimer >= 90000) {
            prefs.begin("awn", false);
            prefs.putULong("useCurrent", 0);
            useCurrent = 0;
            prefs.end();
            LearningStateMachine = LEARNING_DONE;
            client.publish(mqtt_topic_learn, "LEARNING_ERROR");
            openDuration = LASTopenDuration;
            closeDuration = LASTcloseDuration;
            sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Learn status: LEARNING_ERROR\"}");
            client.publish(mqtt_topic_log, TempString);
            actualPositionTime = 0;
          } else {
            closeDuration = currentMillis - closeDuration;
            stopAwning();
            delay(1000);
            learnTimer = currentMillis;
            LearningStateMachine = LEARNING_DONE;
            client.publish(mqtt_topic_learn, "LEARNING_DONE");
            sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"Learn status: LEARNING_DONE\"}");
            client.publish(mqtt_topic_log, TempString);
            actualPositionTime = 0;
            prefs.begin("awn", false);
            prefs.putULong("openTime", openDuration);
            prefs.putULong("closeTime", closeDuration);
            prefs.putULong("CURRENT_THRESHOLD", CURRENT_THRESHOLD);
            prefs.putULong("currentNormal", currentNormal);
            prefs.putULong("currentRun", currentRun);
            prefs.end();
          }
        } else {
          raiseAwning();
        }
        break;
      default:
        LearningStateMachine = LEARNING_IDLE;
        break;
    }
    return;
  }

  // --- LED blinking ---
  if (windAlarmActive) {
    if (currentMillis - lastBlinkTime >= 100) {
      ledState = !ledState;
      digitalWrite(RED_LED, ledState);
      if (raisingInProgress) {
        digitalWrite(GREEN_LED, LOW);
      } else {
        digitalWrite(GREEN_LED, ledState);
      }
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
    if (LCDPresent) {
      lcd.setCursor(0, 0);
      lcd.print("Wind tick:      ");
      lcd.setCursor(11, 0);
      lcd.print(tickCount);
    }

    if (tickCount >= WIND_TICK_THRESHOLD && !windAlarmActive) {
      windAlarmActive = true;
      alarmStartTime = currentMillis;
      retractAwningDueToAlarm();
      if (wifiConnected && client.connected()) {
        sprintf(TempString, "{\"wind\": %u , \"state\": \"WIND ALARM\"}", tickCount);
        client.publish(mqtt_topic_alarm, TempString);
        sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"WIND ALARM %u\"}", tickCount);
        client.publish(mqtt_topic_log, TempString);
      }
    }
    if (MAXtickCount < tickCount && tickCount < 200) {
      MAXtickCount = tickCount;
    }
    tickCount = 0;
  }

  if(FirstStart && wifiConnected && client.connected())
  {
    FirstStart = false;
    sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"START BOARD\"}");
    client.publish(mqtt_topic_log, TempString);
  }

  if(ReconnectionSuccess && wifiConnected && client.connected())
  {
    ReconnectionSuccess = false;
    sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"ReconnectionSuccess\"}");
    client.publish(mqtt_topic_log, TempString);
  }

  // --- Wind alarm reset ---
  if (windAlarmActive) {
    if (LCDPresent) {
      lcd.setCursor(0, 1);
      lcd.print("Allarme!       ");
      lcd.setCursor(9, 1);
      lcd.print((ALARM_RESET_TIMEOUT / 1000) - ((unsigned long)((currentMillis - alarmStartTime) / 1000)));
    }
    actualPositionTime = 0;
    if (currentMillis - alarmStartTime >= ALARM_RESET_TIMEOUT) {
      Serial.println("Wind alarm reset.");
      if (LCDPresent) {
        lcd.setCursor(0, 1);
        lcd.print("Allarm reset   ");
      }
      raisingInProgress = false;
      windAlarmActive = false;
    } else {
      if (currentMillis - alarmStartTime >= ALARM_STOP_UP_TIMEOUT) {
        //Serial.println("Wind alarm stop UP.");
        raisingInProgress = false;
      } else {
        raiseAwning();
      }
    }
  }

  if (movingToTarget && !windAlarmActive) {
    if (isMovingDown) {
      if (actualPositionTime < targetMoveDuration) {
        lowerAwning();
      } else {
        stopAwning();
        movingToTarget = false;
        isMovingDown = false;
      }
    } else if (isMovingUp) {
      if (actualPositionTime > targetMoveDuration) {
        raiseAwning();
      } else {
        stopAwning();
        movingToTarget = false;
        isMovingUp = false;
      }
    }
  }

  // --- Button debounce logic ---
  bool readingUP = digitalRead(PIN_UP);
  if (readingUP != lastButtonStateUP) {
    lastDebounceTimeUP = currentMillis;
  }
  if ((currentMillis - lastDebounceTimeUP) > DEBOUNCE_DELAY) {
    if (readingUP == LOW && !windAlarmActive) {
      movingToTarget = false;
      raiseAwning();
    }
  }
  lastButtonStateUP = readingUP;

  bool readingDOWN = digitalRead(PIN_DOWN);
  if (readingDOWN != lastButtonStateDOWN) {
    lastDebounceTimeDOWN = currentMillis;
  }
  if ((currentMillis - lastDebounceTimeDOWN) > DEBOUNCE_DELAY) {
    if (readingDOWN == LOW && !windAlarmActive) {
      movingToTarget = false;
      lowerAwning();
    }
  }
  lastButtonStateDOWN = readingDOWN;

  // Stop motor if no button pressed and no automatic raise
  if (readingUP == HIGH && readingDOWN == HIGH && !raisingInProgress && !movingToTarget) {
    stopAwning();
  }
}

// --- Relay logic updated to support double-deviator wiring ---

void raiseAwning() {
  Serial.println("UP.");
  if (raisingInProgress == false) {
    if (LCDPresent) {
      lcd.setCursor(0, 1);
      lcd.print("UP...       ");
    }
  }
  digitalWrite(RELAY_DIRECTION, LOW);  // Direction: UP
  delay(100);                          // Small delay to settle relays
  digitalWrite(RELAY_POWER, HIGH);     // Power ON
  if (isCurrentZero() == false) actualPositionTime -= millis() - millisStop;
  if (actualPositionTime > 655350) actualPositionTime = 0;
  millisStop = millis();
}

void lowerAwning() {
  Serial.println("DOWN.");
  if (LCDPresent) {
    lcd.setCursor(0, 1);
    lcd.print("DOWN...      ");
  }
  digitalWrite(RELAY_DIRECTION, HIGH);  // Direction: DOWN
  delay(100);
  digitalWrite(RELAY_POWER, HIGH);  // Power ON
  if (isCurrentZero() == false) actualPositionTime += millis() - millisStop;
  if (actualPositionTime > openDuration)
    actualPositionTime = openDuration;
  millisStop = millis();
}

void stopAwning() {
  millisStop = millis();
  digitalWrite(RELAY_POWER, LOW);
  delay(100);
  digitalWrite(RELAY_DIRECTION, LOW);
  isMovingUp = false;
  isMovingDown = false;
  movingToTarget = false;
  if (!windAlarmActive) {
    if (LCDPresent) {
      lcd.setCursor(0, 1);
      lcd.print("Fermo           ");
    }
  }
}

void retractAwningDueToAlarm() {
  Serial.println("Wind alarm! Retracting awning...");
  targetMoveDuration = 0;
  movingToTarget = false;
  raisingInProgress = true;
}

void startLearning() {
  LearningStateMachine = LEARNING_INIT;
}

void moveToPercentage(float percent) {
  if (percent < 0 || percent > 100 || openDuration == 0 || closeDuration == 0)
    return;

  if (percent == 100) {
    percent = 120;
  }

  targetMoveDuration = (unsigned long)((float)openDuration * (percent * 0.01));

  if (targetMoveDuration > actualPositionTime) {
    isMovingDown = true;
    isMovingUp = false;
  } else if (targetMoveDuration < actualPositionTime) {
    isMovingUp = true;
    isMovingDown = false;
  } else {
    isMovingUp = false;
    isMovingDown = false;
    return;  // already at position
  }
  Serial.println("moveToPercentage");
  Serial.println(targetMoveDuration);

  sprintf(TempString, "{\"sender\": \"Awning1__\" , \"message\": \"moveToPercentage %3.0f\"}", percent);
  client.publish(mqtt_topic_log, TempString);
  movingToTarget = true;
}