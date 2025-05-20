// -----------------------------------------------------------------------------
// BLYNK CONFIGURATION
// -----------------------------------------------------------------------------
#define BLYNK_TEMPLATE_ID "###########"
#define BLYNK_TEMPLATE_NAME "Auto Fish Feeder"
#define BLYNK_AUTH_TOKEN "#################### // Template key to unique ID by us
#define BLYNK_PRINT Serial

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = " " //put your wifi name here
char pass[] = " " //put your wifi password here

// -----------------------------------------------------------------------------
// LIBRARIES
// -----------------------------------------------------------------------------
#include <time.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// -----------------------------------------------------------------------------
// CONSTANT VARIABLES
// -----------------------------------------------------------------------------
// Pin definition
// https://www.studiopieters.nl/esp32-pinout/#google_vignette
const int IN1_PIN = 16;
const int IN2_PIN = 4;
const int ENA_PIN = 17;
const int TRIG_PIN = 32;
const int ECHO_PIN = 35;
const int LED_BUILD_IN_PIN = 2;

// Utils
const unsigned long falseAlarmWindow = 3000;  // milliseconds
const int bufferSize = 5;

// Threshold constant
const int humidity_ThresholdValue = 85;  // AHT10 sensor
const int distance_ThresholdValue = 5 ;  // if distance below 10% then trigger notification

// Ultrasonic constant
const int distance_MaxDistance = 25;  // Maximum valid distance in cm (height of tank)

// Timer
unsigned long previousMillis_1_sec = 0;
const unsigned long interval_1_sec = 1000;
unsigned long previousMillis_debugPrint = 0;
const unsigned long interval_debugPrint = 1000;
unsigned long previousMillis_5_sec = 0;
const unsigned long interval_5_sec = 5000;
unsigned long previousMillis_30_sec = 0;
const unsigned long interval_30_sec = 30000;

// Feeding constant
const int SPEED_VALUE = 1;
const int MINUTE_VALUE = 3;
const int ONTIME_Value = 3;
const int SPEED_1 = 160;  // If motor have buzzing sound and not rotating then increase this value
const int SPEED_2 = 192;
const int SPEED_3 = 255;

// NTP Configuration
const char* ntpServer = "asia.pool.ntp.org";
const long gmtOffset_sec = 8 * 3600;  // UTC+8 for Malaysia
const int daylightOffset_sec = 0;     // No daylight saving time

// Blynk virtual pin
#define HUMIDITY_VPIN V0
#define TEMPERATURE_VPIN V1
#define SPEED_VPIN V2
#define ON_TIME_VPIN V3
#define MINUTE_VPIN V4

// -----------------------------------------------------------------------------
// GLOBAL VARIABLES
// -----------------------------------------------------------------------------
// AHT10 global
Adafruit_AHTX0 aht;
sensors_event_t humid_event, temp_event;
float humidity_Level = 0.0;
float temperature_Level = 0.0;
bool humidity_Trigger = false;
bool humidity_IndeedTrigger = false;
unsigned long humidity_StartTime = 0;

// Ultrasonic global
float distance_cm = 0.0;
int distance_Percentage = 0;
bool distance_Trigger = false;
bool distance_IndeedTrigger = false;
unsigned long distance_StartTime = 0;

// Feeding global
int speedValue = SPEED_VALUE;
int minuteValue = MINUTE_VALUE;
int onTimeValue = ONTIME_Value;
bool feedingInProgress = false;
unsigned long feedingStartTime = 0;
int currentSpeed = SPEED_1;
bool feedingScheduled = false;

// Sensor averaging buffer
float distanceBuffer[bufferSize];
float humidityBuffer[bufferSize];
float temperatureBuffer[bufferSize];
int bufferIndex = 0;
int bufferCount = 0;
float avg_distance = 0;
float avg_humidity = 0;
float avg_temperature = 0;

// Blynk global
bool humidity_Notify = false;
bool distance_Notify = false;

// -----------------------------------------------------------------------------
// UTILS FUNCTIONS
// -----------------------------------------------------------------------------
float calculateMean(float* arr, int len) {
  float sum = 0;
  for (int i = 0; i < len; i++) {
    sum += arr[i];
  }
  return sum / len;
}

// -----------------------------------------------------------------------------
// BLYNK FUNCTIONS
// -----------------------------------------------------------------------------
void initializeWiFiAndBlynk() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    attempts++;
    if (attempts >= 30) {
      Serial.println("\nFailed to connect to WiFi. Force restart in 3 seconds");
      delay(3000);
      ESP.restart();
    }
  }

  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Blynk.begin(auth, ssid, pass);
  delay(100);
  Serial.println("Connected to Blynk!");
  Blynk.virtualWrite(HUMIDITY_VPIN, 0);
  Blynk.virtualWrite(TEMPERATURE_VPIN, 0);
  Blynk.virtualWrite(SPEED_VPIN, SPEED_VALUE);
  Blynk.virtualWrite(MINUTE_VPIN, MINUTE_VALUE);
  Blynk.virtualWrite(ON_TIME_VPIN, ONTIME_Value);

  Serial.println("WIFI & BLYNK have been initialized!");
}

BLYNK_WRITE(SPEED_VPIN) {
  speedValue = param.asInt();  // 1 = slow, 2 = medium, 3 = fast
  Serial.print("[BLYNK] Speed Value Updated: ");
  Serial.println(speedValue);
}

BLYNK_WRITE(ON_TIME_VPIN) {
  onTimeValue = param.asInt();
  Serial.print("[BLYNK] On-Time Value Updated (seconds): ");
  Serial.println(onTimeValue);
}

BLYNK_WRITE(MINUTE_VPIN) {
  minuteValue = param.asInt();
  Serial.print("[BLYNK] Minute Interval Updated: ");
  Serial.println(minuteValue);
}

// -----------------------------------------------------------------------------
// AHT FUNCTIONS
// -----------------------------------------------------------------------------
void initializeAHT10() {
  Wire.begin();
  if (!aht.begin()) {
    Serial.println("AHT10 not found! Check wiring.");
    Serial.println("Run without AHT10");
    delay(3000);
    return;
  }
  Serial.println("AHT10 Sensor Initialized.");
}

void readAHT10() {
  aht.getEvent(&humid_event, &temp_event);
  humidity_Level = humid_event.relative_humidity;
  temperature_Level = temp_event.temperature;
}

void handleHumidityFalseAlarm() {
  if (avg_humidity >= humidity_ThresholdValue) {
    humidity_Trigger = true;
    if (humidity_StartTime == 0) {
      humidity_StartTime = millis();
    } else if (millis() - humidity_StartTime >= falseAlarmWindow) {
      humidity_IndeedTrigger = true;
    }
  } else {
    humidity_StartTime = 0;
    humidity_Trigger = false;
    humidity_IndeedTrigger = false;
  }
}

void handleBlynkNotificationHumidity() {
  if (humidity_IndeedTrigger) {
    if (!humidity_Notify) {
      humidity_Notify = true;
      String notificationMessage = "Trigger Reason: Humidity Level Too High\n"
                                   "Humidity Value: "
                                   + String(humidity_Level) + "\n"
                                   + "Humidity Threshold: " + String(humidity_ThresholdValue);
      Blynk.logEvent("humidity_event", notificationMessage);
      Serial.println("\n// === HUMIDITY EVENT === //");
      Serial.println(notificationMessage);
      Serial.println("// === HUMIDITY EVENT === //\n");
    }
  } else {
    humidity_Notify = false;
  }
}

// -----------------------------------------------------------------------------
// ULTRASONIC FUNCTIONS
// -----------------------------------------------------------------------------
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance = duration * 0.034 / 2;
  return constrain(distance, 0, distance_MaxDistance);
}

float getFilteredDistance(int trigPin, int echoPin) {
  float readings[5];
  for (int i = 0; i < 5; i++) {
    readings[i] = getDistance(trigPin, echoPin);
    delay(20);
  }
  // Simple sort (bubble for small array)
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (readings[j] < readings[i]) {
        float temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  return readings[2];  // Median
}

void readUltrasonic() {
  distance_cm = getFilteredDistance(TRIG_PIN, ECHO_PIN);
  distance_Percentage = map(distance_cm, 0, distance_MaxDistance, 100, 0);
  distance_Percentage = constrain(distance_Percentage, 0, 100);
}

void handleDistanceFalseAlarm() {
  if (avg_distance <= distance_ThresholdValue) {
    distance_Trigger = true;
    if (distance_StartTime == 0) {
      distance_StartTime = millis();
    } else if (millis() - distance_StartTime >= falseAlarmWindow) {
      distance_IndeedTrigger = true;
    }
  } else {
    distance_StartTime = 0;
    distance_Trigger = false;
    distance_IndeedTrigger = false;
  }
}

void handleBlynkNotificationDistance() {
  if (distance_IndeedTrigger) {
    if (!distance_Notify) {
      distance_Notify = true;
      String notificationMessage = "Trigger Reason: Food Level Too Low\n"
                                   "Food Level: "
                                   + String(distance_Percentage) + "\n"
                                   + "Food Threshold: " + String(distance_ThresholdValue);
      Blynk.logEvent("distance_event", notificationMessage);
      Serial.println("\n// === FOOD EVENT === //");
      Serial.println(notificationMessage);
      Serial.println("// === FOOD EVENT === //\n");
    }
  } else {
    distance_Notify = false;
  }
}

// -----------------------------------------------------------------------------
// FEEDING FUNCTIONS
// -----------------------------------------------------------------------------
void motorOn() {
  if (speedValue == 1) currentSpeed = SPEED_1;
  else if (speedValue == 2) currentSpeed = SPEED_2;
  else if (speedValue == 3) currentSpeed = SPEED_3;
  else currentSpeed = SPEED_1;

  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, currentSpeed);
}

void motorOff() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 0);
}

void startFeeding() {
  if (!feedingInProgress) {
    motorOn();
    feedingInProgress = true;
    feedingStartTime = millis();
  }
}

void stopFeeding() {
  if (feedingInProgress && millis() - feedingStartTime > onTimeValue * 1000UL) {
    motorOff();
    feedingInProgress = false;
  }
}

// -----------------------------------------------------------------------------
// NTP FUNCTIONS
// -----------------------------------------------------------------------------
void initializeNTP() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time. Restarting...");
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILD_IN_PIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILD_IN_PIN, LOW);
      delay(500);
    }
    ESP.restart();
  } else {
    Serial.println("Time synchronized using NTP");
  }
}

void getNTP() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    static int lastSecond = -1;
    if (timeinfo.tm_sec != lastSecond) {
      lastSecond = timeinfo.tm_sec;
      Serial.println(&timeinfo, "Current time: %A, %B %d %Y %H:%M:%S");

      if (minuteValue > 0 && timeinfo.tm_min % minuteValue == 0 && timeinfo.tm_sec == 0) {
        if (!feedingScheduled) {
          startFeeding();
          feedingScheduled = true;
        }
      } else {
        feedingScheduled = false;
      }
    }
  }
}

void checkWiFiAndReconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, pass);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Blynk.config(auth);
      Blynk.connect(10000);  // timeout in 10 seconds
      Blynk.syncAll();
    }
  }
}

void reconnectNTP() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    checkWiFiAndReconnect();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(200);
  }
}

// -----------------------------------------------------------------------------
// SERIAL DEBUG PRINT FUNCTIONS
// -----------------------------------------------------------------------------
void printSerial() {
  Serial.println("\n// ------------------- DEBUG DATA ------------------- //");

  // Humidity
  Serial.print("Humidity Level      : ");
  Serial.print(avg_humidity);
  Serial.print("% | Trigger: ");
  Serial.print(humidity_Trigger);
  Serial.print(" | Indeed Trigger: ");
  Serial.println(humidity_IndeedTrigger);

  // Temperature
  Serial.print("Temperature Level: ");
  Serial.print(avg_temperature);
  Serial.println(" °C");

  // Distance
  Serial.print("Distance %          : ");
  Serial.print(avg_distance);
  Serial.print("% | Trigger: ");
  Serial.print(distance_Trigger);
  Serial.print(" | Indeed Trigger: ");
  Serial.println(distance_IndeedTrigger);

  // Feeding
  Serial.print("Feeding In Progress : ");
  Serial.print(feedingInProgress);
  Serial.print(" | Scheduled Feeding: ");
  Serial.println(feedingScheduled);

  // WiFi and Blynk
  Serial.print("WiFi Connected      : ");
  Serial.print(WiFi.isConnected());
  Serial.print(" | Blynk Connected: ");
  Serial.println(Blynk.connected());

  Serial.println("// ------------------- DEBUG DATA ------------------- //\n");
}

// -----------------------------------------------------------------------------
// SETUP FUNCTION
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILD_IN_PIN, OUTPUT);

  motorOff();

  initializeAHT10();
  initializeWiFiAndBlynk();
  initializeNTP();
}

// -----------------------------------------------------------------------------
// LOOP FUNCTION
// -----------------------------------------------------------------------------
void loop() {
  unsigned long currentMillis = millis();

  // 1-sec tasks
  if (currentMillis - previousMillis_1_sec >= interval_1_sec) {
    previousMillis_1_sec = currentMillis;

    readAHT10();
    readUltrasonic();

    // Store into circular buffer
    distanceBuffer[bufferIndex] = distance_Percentage;
    humidityBuffer[bufferIndex] = humidity_Level;
    temperatureBuffer[bufferIndex] = temperature_Level;

    bufferIndex = (bufferIndex + 1) % bufferSize;
    if (bufferCount < bufferSize) bufferCount++;

    // Smooth readings
    avg_distance = calculateMean(distanceBuffer, bufferCount);
    avg_humidity = calculateMean(humidityBuffer, bufferCount);
    avg_temperature = calculateMean(temperatureBuffer, bufferCount);

    getNTP();  // Check if feeding time
  }

  // 5-sec task: reconnect NTP if needed
  if (currentMillis - previousMillis_5_sec >= interval_5_sec) {
    previousMillis_5_sec = currentMillis;
    reconnectNTP();
  }

  // 30-sec task: send data to Blynk
  if (currentMillis - previousMillis_30_sec >= interval_30_sec) {
    previousMillis_30_sec = currentMillis;
    Blynk.virtualWrite(HUMIDITY_VPIN, avg_humidity);
    Blynk.virtualWrite(TEMPERATURE_VPIN, avg_temperature);
  }

  // Realtime jobs
  Blynk.run();
  handleHumidityFalseAlarm();
  handleBlynkNotificationHumidity();
  handleDistanceFalseAlarm();
  handleBlynkNotificationDistance();
  stopFeeding();  // stop if feeding time has elapsed

  // 1-sec task: serial debug print
  if (currentMillis - previousMillis_debugPrint >= interval_debugPrint) {
    previousMillis_debugPrint = currentMillis;
    printSerial();
  }
}
