#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const char *ssid = "HARRYSGADGET 4563";
const char *password = "12345678";
const char *apiRoute = "http://192.168.137.1:3000/api/data";
const char *deviceId = "aaaaaa";
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

float sumAccX{0};
float sumAccY{0};
float sumAccZ{0};
float meanAccX{0};
float meanAccY{0};
float meanAccZ{0};
int numMeanElements{1};

sensors_event_t a, g, temp;


bool sensorReady = false;

struct Timer
{
  unsigned long lastTime = millis();
  unsigned long interval;

  Timer(unsigned long intervalMs) : interval(intervalMs) {}

  bool isExpired()
  {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval)
    {
      lastTime = currentTime;
      return true;
    }
    return false;
  }
};

struct HeartRateData
{
  int lastHeartBeat = millis();
  int totalDeltaHR = 0;
  int averageDeltaHR = 0;
  int beatCount = 0;
  int bpm = 0;
  HeartRateData() {}
};

struct MovementData
{
  float sumAccX;
  float sumAccY;
  float sumAccZ;
  float sumGyroX;
  float sumGyroY;
  float sumGyroZ;
  int numElements = 0;
  MovementData() {}
};

HeartRateData hrData = {};
Timer twoSecondTimer(2000);

String payload = "";

void initWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
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
  while (now < 1000000000 && retry < 15)
  {
    Serial.print('.');
    delay(1000);
    now = time(nullptr);
    retry++;
  }
  if (now < 1000000000)
  {
    Serial.println("\nTime sync failed");
  }
  else
  {
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    Serial.println(String("\nTime synchronized: ") + buf);
  }
}

// return current UTC timestamp in ISO8601 format like 2025-11-04T20:49:07Z
String getUTCTimestamp()
{
  time_t now = time(nullptr);
  struct tm timeinfo;
  // use gmtime_r (reentrant) on ESP32
  gmtime_r(&now, &timeinfo);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting...");
  delay(100);
  initWifi();

  Wire.setPins(47, 21); // Use SDA=47, SCL=21

  // Initialize MAX30105 sensor
  Serial.println("Initializing MAX30105 sensor...");
  if (particleSensor.begin() == false)
  {
    Serial.println("MAX30105 was not found. Sensor reads will be skipped.");
    sensorReady = false;
  }
  else
  {
    Serial.println("MAX30105 found.");
    sensorReady = true;
    // optional basic configuration (safe defaults), adjust as needed
    particleSensor.setup();
  }

  if (!mpu.begin(0x68, &Wire))
  {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  // SET BANDWIDTH FOR MPU6050
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void sendPOSTRequest(const String &payload)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    http.begin(apiRoute);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(payload);
    Serial.println("Payload: " + String(payload));
    if (httpResponseCode > 0)
    {
      String response = http.getString();
      Serial.println("HTTP Response code: " + String(httpResponseCode));
      Serial.println("Response: " + response);
    }
    else
    {
      Serial.println("Error on sending POST: " + String(httpResponseCode));
    }
    http.end();
  }
  else
  {
    Serial.println("WiFi Disconnected");
  }
}

void createPayload(String &payload, String deviceId, int hr, int o2, int skinTemp, int accX, int accY, int accZ, int gyroX, int gyroY, int gyroZ)
{
  String timestamp = getUTCTimestamp();
  payload = "{\"deviceId\": \"" + deviceId + "\", \"timestamp\": \"" + timestamp + "\", \"heartRate\": " + hr + ", \"o2Sat\": " + String(o2 + random(-2, 3)) + ", \"skinTemp\": " + String(skinTemp + random(0, 2)) +
            ", \"accX\": " + accX + ", \"accY\": " + accY + ", \"accZ\": " + accZ +
            ", \"gyroX\": " + gyroX + ", \"gyroY\": " + gyroY + ", \"gyroZ\": " + gyroZ + "}";
}

void loop()
{
  // Only read from the sensor if it was initialized
  if (sensorReady && checkForBeat(particleSensor.getIR()) == true)
  {
    // Calculate beatsPerMinute
    hrData.beatCount++;
    int delta = millis() - hrData.lastHeartBeat;
    hrData.totalDeltaHR += delta;
    hrData.averageDeltaHR = hrData.totalDeltaHR / hrData.beatCount;
    hrData.lastHeartBeat = millis();
    hrData.bpm = 60000 / delta;
    Serial.println("Beat detected! Average Delta HR: " + String(hrData.averageDeltaHR));
  }

  mpu.getEvent(&a, &g, &temp); // check if this works - only if MPU has correct address

  sumAccX += sqrt(a.acceleration.x * a.acceleration.x);
  sumAccY += sqrt(a.acceleration.y * a.acceleration.y);
  sumAccZ += sqrt(a.acceleration.z * a.acceleration.z);
  meanAccX = sumAccX / numMeanElements;
  meanAccY = sumAccY / numMeanElements;
  meanAccZ = sumAccZ / numMeanElements;
  numMeanElements += 1;

  if (twoSecondTimer.isExpired())
  {
    Serial.println(sensorReady);
    createPayload(payload, deviceId, hrData.bpm, 95, 36, meanAccX, meanAccY, meanAccZ, g.gyro.x, g.gyro.y, g.gyro.z);
    sendPOSTRequest(payload);
    hrData = {};
    numMeanElements = 1;
    sumAccX = 0;
    sumAccY = 0;
    sumAccZ = 0;
    meanAccX = 0;
    meanAccY = 0;
    meanAccZ = 0;
  }
}