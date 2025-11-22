#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <Wire.h>
#include <math.h>
#include "MAX30105.h"
#include "heartRate.h"

const char *ssid = "HARRYSGADGET 4563";
const char *password = "12345678";
const char *apiRoute = "http://192.168.137.1:3000/api/data";
const char *deviceId = "aaaaaa";
MAX30105 particleSensor;
const int MPU = 0x68;
//Create infared sensor LED data:
const int buffer_length = 250;
uint32_t ir_Buffer[buffer_length]; //infrared LED sensor data
uint32_t red_Buffer[buffer_length];  //red LED sensor data
int bufferIndex = 0;

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
  unsigned long lastHeartBeat = millis();
  int totalDeltaHR = 0;
  int averageDeltaHR = 0;
  int beatCount = 0;
  int bpm = 0;
  int btb = 0;
  HeartRateData() {}
};

struct MovementData
{
  float sumAccX = 0.0;
  float sumAccY = 0.0;
  float sumAccZ = 0.0;
  float sumGyroX = 0.0;
  float sumGyroY = 0.0;
  float sumGyroZ = 0.0;

  float sumAccMagnitude = 0.0;
  float sumGyroMagnitude = 0.0;

  float acc_peak_threshold = 1.5;
  float gyro_peak_threshold = 1.5;
  int num_peaks_acc = 0;
  int num_peaks_gyro = 0;
  float max_acc = 0.0;

  int numElements = 0;
  MovementData() {}
};

struct LastData
{
  float lastSpO2 = 0.0;
  int lastHR = 0;
  int lastBTB = 0;
  float lastTemp = 0.0;
  LastData() {}
};

HeartRateData hrData = {};
MovementData mvData = {};
Timer twoSecondTimer(2000);
Timer movementTimer(10);
Timer SpO2Timer(100);
LastData lastData = {};

static char payload_buffer[512];
float SpO2 = 0.0;

struct MovementPacket {
  float acc;
  float gyro;
};

MovementPacket movementPackets[250];
float temperature = 0;

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

void setup()
{
  Serial.begin(115200);
  Serial.println("Booting...");
  delay(1000);
  initWifi();

  Wire.setPins(47, 21); // Use SDA=47, SCL=21

  // Initialize MAX30105 sensor
  Serial.println("Initializing MAX30105 sensor...");
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD) == false)
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
    particleSensor.setPulseAmplitudeRed(0x0A);   //Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
    particleSensor.enableDIETEMPRDY(); //starts temperature measurement
  }

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  particleSensor.enableDIETEMPRDY(); //starts temperature measurement

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void sendPOSTRequest(const char *payload, size_t len)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    http.begin(apiRoute);
    http.addHeader("Content-Type", "application/json");
    Serial.print("Payload: ");
    Serial.println(payload);
    int httpResponseCode = http.POST((uint8_t *)payload, len);
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

int createPayload(char *buf, size_t bufSize, const char *deviceId, int HR, int BTB, float SpO2, float Temp, float acc_mean,
                  float acc_std, float acc_max, int acc_peaks, float gyro_mean, float gyro_std, int gyro_peaks,
                  float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ)
{
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char tbuf[32];
  strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

  int written = snprintf(buf, bufSize,
    "{\"deviceId\":\"%s\",\"timestamp\":\"%s\",\"HR\":%d,\"BTB\":%d,\"SpO2\":%.2f,\"Temp\":%.2f,\"accel_mean_dyn_2s\":%.2f,\"accel_std_dyn_2s\":%.2f,\"accel_max_2s\":%.2f,\"accel_peak_count_2s\":%d,\"gyro_mean_2s\":%.2f,\"gyro_std_2s\":%.2f,\"gyro_large_change_count_2s\":%d,\"accX\":%.2f,\"accY\":%.2f,\"accZ\":%.2f,\"gyroX\":%.2f,\"gyroY\":%.2f,\"gyroZ\":%.2f}",
    deviceId, tbuf, HR, BTB, SpO2, Temp, acc_mean, acc_std, acc_max, acc_peaks, gyro_mean, gyro_std, gyro_peaks, accX, accY, accZ, gyroX, gyroY, gyroZ);

  if (written < 0) return -1; // encoding error
  if ((size_t)written >= bufSize) return -2; // truncated
  return written;
}

float calculate_SpO2(uint32_t *redBuffer, uint32_t *irBuffer, std::size_t length = bufferIndex) {
  if (length == 0) return NAN;

  float r_mean = redBuffer[0];
  float ir_mean = irBuffer[0];

  float r_min = redBuffer[0];
  float r_max = redBuffer[0];
  float ir_min = irBuffer[0];
  float ir_max = irBuffer[0];

  for (int i = 1; i < length; i++) {
    r_mean += redBuffer[i];
    ir_mean += irBuffer[i];
    r_max = max(r_max, (float)redBuffer[i]);
    r_min = min(r_min, (float)redBuffer[i]);
    ir_max = max(ir_max, (float)irBuffer[i]);
    ir_min = min(ir_min, (float)irBuffer[i]);
  }

  r_mean /= length;
  ir_mean /= length;

  float r_red = (r_max - r_min) / r_mean;
  float r_ir = (ir_max - ir_min) / ir_mean;

  float kSpO2_A = 1.5958422;      // constants to approximate SPO2
  float kSpO2_B = -34.6596622;
  float kSpO2_C = 112.6898759;

  float ratio = r_red / r_ir;
  float SpO2 = kSpO2_A * ratio * ratio + kSpO2_B * ratio + kSpO2_C;
  return SpO2;
}


float getMagnitude(float x, float y, float z, float bias)
{
  return fabsf(sqrt(x * x + y * y + z * z) + bias);
}

float stdDev(MovementPacket data[], int size, float mean, bool isMovement)
{
  float sum = 0.0;
  if (!isMovement)
  {
    for (int i = 0; i < size; i++)
    {
      sum += (data[i].gyro - mean) * (data[i].gyro - mean);
    }
    return sqrt(sum / (size-1));
  }
  for (int i = 0; i < size; i++)
  {
    sum += (data[i].acc - mean) * (data[i].acc - mean);
  }
  return sqrt(sum / (size-1));
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
    hrData.bpm = 60000/hrData.averageDeltaHR;
    hrData.btb = 60000/delta;
    Serial.println("Beat detected! Average Delta HR: " + String(hrData.averageDeltaHR));
  }

  particleSensor.check();
  if (particleSensor.available()) {
    if (bufferIndex < buffer_length) {
      red_Buffer[bufferIndex] = particleSensor.getRed();
      ir_Buffer[bufferIndex] = particleSensor.getIR();
      bufferIndex++;
      particleSensor.nextSample(); 
    }
  }

  if (movementTimer.isExpired()) {

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true);

    // Initialize readings with safe defaults
    float current_acc_X = 0.0f, current_acc_Y = 0.0f, current_acc_Z = 0.0f;
    float mpu_temp = 0.0f;
    float current_gyro_X = 0.0f, current_gyro_Y = 0.0f, current_gyro_Z = 0.0f;

    // Ensure we actually received 14 bytes (6 accel, 2 temp, 6 gyro)
    if (Wire.available() >= 14) {
      int16_t raw_acc_x = ((Wire.read() << 8) + Wire.read());
      int16_t raw_acc_y = ((Wire.read() << 8) + Wire.read());
      int16_t raw_acc_z = ((Wire.read() << 8) + Wire.read());
      int16_t raw_temp  = ((Wire.read() << 8) + Wire.read());
      int16_t raw_gyro_x = ((Wire.read() << 8) + Wire.read());
      int16_t raw_gyro_y = ((Wire.read() << 8) + Wire.read());
      int16_t raw_gyro_z = ((Wire.read() << 8) + Wire.read());

    Serial.println("raw accel X: " + String(raw_acc_x) + " Y: " + String(raw_acc_y) + " Z: " + String(raw_acc_z));

      // Convert to engineering units
      // Accel: raw / 16384 LSB per g (assuming default ±2g), convert g -> m/s^2
      current_acc_X = (raw_acc_x / 16384.0f) * 9.81f;
      current_acc_Y = (raw_acc_y / 16384.0f) * 9.81f;
      current_acc_Z = (raw_acc_z / 16384.0f) * 9.81f;

      // Temp: per MPU6050 datasheet: Temp (°C) = (raw / 340) + 36.53
      mpu_temp = (raw_temp / 340.0f) + 36.53f;

      // Gyro: raw / 131 gives deg/s (assuming default ±250°/s)
      current_gyro_X = raw_gyro_x / 131.0f;
      current_gyro_Y = raw_gyro_y / 131.0f;
      current_gyro_Z = raw_gyro_z / 131.0f;

      Serial.println("Acc X: " + String(current_acc_X) + " Y: " + String(current_acc_Y) + " Z: " + String(current_acc_Z));

   } else {
      Serial.println("MPU6050: insufficient I2C data (expected 14 bytes)");
   }

    mvData.sumAccX += fabsf(current_acc_X);
    mvData.sumAccY += fabsf(current_acc_Y);
    mvData.sumAccZ += fabsf(current_acc_Z);
    mvData.sumGyroX += fabsf(current_gyro_X);
    mvData.sumGyroY += fabsf(current_gyro_Y);
    mvData.sumGyroZ += fabsf(current_gyro_Z);


    float acc_magnitude = getMagnitude(current_acc_X, current_acc_Y, current_acc_Z, -9.81f);
    float gyro_magnitude = getMagnitude(current_gyro_X, current_gyro_Y, current_gyro_Z, -5.0f);
    if (acc_magnitude > mvData.acc_peak_threshold) {
      mvData.num_peaks_acc += 1;
    }
    if (gyro_magnitude > mvData.gyro_peak_threshold) {
      mvData.num_peaks_gyro += 1;
    }
    
    mvData.max_acc = max(mvData.max_acc, acc_magnitude);

    movementPackets[mvData.numElements] = {
      acc_magnitude,
      gyro_magnitude
    };

    mvData.sumAccMagnitude += acc_magnitude;
    mvData.sumGyroMagnitude += gyro_magnitude;

    mvData.numElements += 1; 
  }

  if (twoSecondTimer.isExpired())
  {
    // FOR DISPLAY ONLY
    float meanAccX, meanAccY, meanAccZ, meanGyroX, meanGyroY, meanGyroZ, meanAccMagnitude, meanGyroMagnitude, stdDevAcc, stdDevGyro;
    if (mvData.numElements == 0) {
      mvData.numElements = 1;
      meanAccX = 0;
      meanAccY = 0;
      meanAccZ = 0;
      meanGyroX = 0;
      meanGyroY = 0;
      meanGyroZ = 0;
      meanAccMagnitude = 0;
      meanGyroMagnitude = 0;
      stdDevAcc = 0;
      stdDevGyro = 0;
    } else {
      meanAccX = mvData.sumAccX / mvData.numElements;
      meanAccY = mvData.sumAccY / mvData.numElements;
      meanAccZ = mvData.sumAccZ / mvData.numElements;
      meanGyroX = mvData.sumGyroX / mvData.numElements;
      meanGyroY = mvData.sumGyroY / mvData.numElements;
      meanGyroZ = mvData.sumGyroZ / mvData.numElements;

      temperature = particleSensor.readTemperature();

      meanAccMagnitude = mvData.sumAccMagnitude / mvData.numElements;
      meanGyroMagnitude = mvData.sumGyroMagnitude / mvData.numElements;

      stdDevAcc = stdDev(movementPackets, mvData.numElements, meanAccMagnitude, true);
      stdDevGyro = stdDev(movementPackets, mvData.numElements, meanGyroMagnitude, false);
    }

    SpO2 = calculate_SpO2(red_Buffer, ir_Buffer, bufferIndex);

    if (hrData.beatCount == 0 || hrData.bpm < 20 || hrData.bpm > 250) {
      hrData.bpm = lastData.lastHR;
      hrData.btb = lastData.lastBTB;
    }
    if (SpO2 > 100.0 || SpO2 < 75.0 || isnan(SpO2)) {
      SpO2 = lastData.lastSpO2;
    }

    int len = createPayload(
      payload_buffer, sizeof(payload_buffer), deviceId, hrData.bpm, hrData.btb, SpO2, temperature,
      meanAccMagnitude, stdDevAcc, mvData.max_acc, mvData.num_peaks_acc, meanGyroMagnitude,
      stdDevGyro, mvData.num_peaks_gyro, meanAccX, meanAccY, meanAccZ, meanGyroX, meanGyroY, meanGyroZ);
    if (len > 0) {
      sendPOSTRequest(payload_buffer, (size_t)len);
    } else if (len == -2) {
      Serial.println("Payload was truncated; increase buffer size or reduce fields.");
    } else {
      Serial.println("Failed to create payload");
    }
    lastData.lastSpO2 = SpO2;
    lastData.lastHR = hrData.bpm;
    lastData.lastBTB = hrData.btb;
    lastData.lastTemp = temperature;
    mvData = MovementData();
    hrData = HeartRateData();
    bufferIndex = 0;
  }
}