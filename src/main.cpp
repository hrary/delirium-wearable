#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

const char* ssid     = "HARRYSGADGET 4563";
const char* password = "12345678";
const char* apiRoute = "http://192.168.137.1:3000/api/data";

int num = 0;
String payload = "";

void initWifi() {
    WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.println(String("Connected to the WiFi network: ") + WiFi.localIP().toString());
  // configure NTP and wait briefly for time to be set so we can produce UTC timestamps
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Synchronizing time");
  time_t now = time(nullptr);
  int retry = 0;
  while (now < 1000000000 && retry < 15) {
    Serial.print('.');
    delay(1000);
    now = time(nullptr);
    retry++;
  }
  if (now < 1000000000) {
    Serial.println("\nTime sync failed");
  } else {
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    Serial.println(String("\nTime synchronized: ") + buf);
  }
}

// return current UTC timestamp in ISO8601 format like 2025-11-04T20:49:07Z
String getUTCTimestamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  // use gmtime_r (reentrant) on ESP32
  gmtime_r(&now, &timeinfo);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Booting...");
  delay(100);
  initWifi();
}

void sendPOSTRequest(const String& payload) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(apiRoute);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(payload);
    Serial.println("Payload: " + String(payload));
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("HTTP Response code: " + String(httpResponseCode));
      Serial.println("Response: " + response);
    } else {
      Serial.println("Error on sending POST: " + String(httpResponseCode));
    }
    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void createPayload(String& payload, String deviceId, int hr, int o2, int skinTemp, int accX, int accY, int accZ, int gyroX, int gyroY, int gyroZ) {
  String timestamp = getUTCTimestamp();
  payload = "{\"deviceId\": \"" + deviceId + "\", \"timestamp\": \"" + timestamp + "\", \"heartRate\": " + String(hr + random(-5, 60)) + ", \"o2Sat\": " + String(o2 + random(-2, 3)) + ", \"skinTemp\": " + String(skinTemp + random(0, 2)) +
            ", \"accX\": " + String(accX + random(-1, 2)) + ", \"accY\": " + String(accY + random(-1, 2)) + ", \"accZ\": " + String(accZ + random(-1, 2)) +
            ", \"gyroX\": " + String(gyroX + random(-1, 2)) + ", \"gyroY\": " + String(gyroY + random(-1, 2)) + ", \"gyroZ\": " + String(gyroZ + random(-1, 2)) + "}";
}

void loop() {
  num++;
  createPayload(payload, "aaaaaa", 70, 95, 36, 0, 0, 0, 0, 0, 0);
  sendPOSTRequest(payload);
  delay(2000);
}