#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <Wire.h>
#include <math.h>
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

//Create infared sensor LED data:
const int buffer_length = 250;
uint32_t ir_Buffer[buffer_length]; //infrared LED sensor data
uint32_t red_Buffer[buffer_length];  //red LED sensor data
int bufferIndex = 0;

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
  float sumAccX;
  float sumAccY;
  float sumAccZ;
  float sumGyroX;
  float sumGyroY;
  float sumGyroZ;

  float sumAccMagnitude;
  float sumGyroMagnitude;


  float acc_peak_threshold = 1.5;
  float gyro_peak_threshold = 1.5;
  int num_peaks_acc;
  int num_peaks_gyro;
  float max_acc;

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

static char payload_buffer[256];
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
  delay(100);
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


float getMagnitude(float x, float y, float z)
{
  return sqrt(x * x + y * y + z * z);
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
    mpu.getEvent(&a, &g, &temp); // check if this works - only if MPU has correct address

    mvData.sumAccX += fabsf(a.acceleration.x);
    mvData.sumAccY += fabsf(a.acceleration.y);
    mvData.sumAccZ += fabsf(a.acceleration.z);
    mvData.sumGyroX += fabsf(g.gyro.x);
    mvData.sumGyroY += fabsf(g.gyro.y);
    mvData.sumGyroZ += fabsf(g.gyro.z);

    float acc_magnitude = getMagnitude(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    float gyro_magnitude = getMagnitude(g.gyro.x, g.gyro.y, g.gyro.z);
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
    temperature = temp.temperature;     
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