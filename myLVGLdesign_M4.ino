#include <Arduino.h>
#include <RPC.h>
#include <dhtnew.h>

// DHT PIN layout from left to right
// =================================
// FRONT : DESCRIPTION
// pin 1 : VCC 3,3 - 6,0 V (use 5V as line is long to loft ceiling)
// pin 2 : DATA
// pin 3 : Not Connected
// pin 4 : GND

// Define pins for DHT22 sensors
#define DHT1_PIN 2   // living space r/h
#define DHT2_PIN 3   // living space l/h
#define DHT3_PIN 4   // shower room
#define DHT4_PIN 5   // loft ceiling

// Named SensorData struct initialised with faulty data
struct SensorData {
    float temp1 = 999.9f;
    float temp2 = 999.9f;
    float temp3 = 999.9f;
    float temp4 = 999.9f;
    float rh1 = 999.9f;
    float rh2 = 999.9f;
    float rh3 = 999.9f;
    float rh4 = 999.9f;
    float avg_temp = 999.9f;

    MSGPACK_DEFINE_ARRAY(temp1, temp2, temp3, temp4, rh1, rh2, rh3, rh4, avg_temp);
};

// Declare struct instances (non static) to refresh data
SensorData sensorData;

// Global Variables
uint16_t delay_ms = 2000; // DHT22 sensor 2000ms recommended for accuracy but sensor will read around 360-370ms
uint32_t last_sensor_read_ms;

float rh_offset = 0;  // Can expand on this and temp for individual sensors
float temp_offset = 0.7;

// Set sensor type 22 = dht22/am2320-22
DHTNEW setType(22);

// Create instances of DHT sensors
DHTNEW dht1(DHT1_PIN);
DHTNEW dht2(DHT2_PIN);
DHTNEW dht3(DHT3_PIN);
DHTNEW dht4(DHT4_PIN);

// READ DHT SENSOR
bool readDHT(DHTNEW &sensor, float* dhtArr, byte retries) {

  // IF SENSOR READY RETURN READINGS
  if (sensor.read() == DHTLIB_OK) {
    dhtArr[0] = sensor.getTemperature();
    dhtArr[1] = sensor.getHumidity();
    return true;
  }
  // IF RETRIES EXCEED 5 SET DATA TO INVALID AKA 999.9
  else if (retries > 5) {
    dhtArr[0] = 999.9f;
    dhtArr[1] = 999.9f;
    // RESTART AND RESET SENSOR
    sensor.powerUp();
    sensor.reset();
    RPC.println("M4 Restarting and resetting DHT22 sensor");
    return true;
  }
  // IF SENSOR NOT READY RETURN FALSE TO INCREMENT RETRIES IN CALLBACK
  else {
    // POWER DOWN SENSOR IF 5 RETRIES
    if (retries == 5) {
      sensor.powerDown();
      RPC.println("M4 Shutting down DHT22 sensor after 5 retries");
    }
    return false;
  }
}

// STORE SENSOR DATA
void read_sensors() {

  float dht1Arr[2];
  float dht2Arr[2];
  float dht3Arr[2];
  float dht4Arr[2];

  static byte dht_retries[4] = {0, 0, 0, 0};

  bool dht1Read = readDHT(dht1, dht1Arr, dht_retries[0]);
  bool dht2Read = readDHT(dht2, dht2Arr, dht_retries[1]);
  bool dht3Read = readDHT(dht3, dht3Arr, dht_retries[2]);
  bool dht4Read = readDHT(dht4, dht4Arr, dht_retries[3]);

  // Verify data has been received before adding to struct
  if (dht1Read) {
    sensorData.temp1 = dht1Arr[0];
    sensorData.rh1 = dht1Arr[1];
    dht_retries[0] = 0;
  }
  else {
    dht_retries[0]++;
  }
  // LIVING ROOM L/H SENSOR
  if (dht2Read) {
    sensorData.temp2 = dht2Arr[0];
    sensorData.rh2 = dht2Arr[1];
    dht_retries[1] = 0;
  }
  else {
    dht_retries[1]++;
  }
  // SHOWER ROOM SENSOR
  if (dht3Read) {
    sensorData.temp3 = dht3Arr[0];
    sensorData.rh3 = dht3Arr[1];
    dht_retries[2] = 0;
  }
  else {
    dht_retries[2]++;
  }
  // LOFT CEILING SENSOR
  if (dht4Read) {
    sensorData.temp4 = dht4Arr[0];
    sensorData.rh4 = dht4Arr[1];
    dht_retries[3] = 0;
  }
  else {
    dht_retries[3]++;
  }

  // Check for valid data before calculating avg_temp
  if (dht1Read && dht2Read && dht4Read) {
    sensorData.avg_temp = (dht1Arr[0] + dht2Arr[0] + dht4Arr[0]) / 3;
  }
  else {
    sensorData.avg_temp = 999.9f; // Set to invalid value if any sensor data is missing
  }
}

// RPC get Sensor data function
SensorData getSensorData() {
  return sensorData;
}

// SETUP FUNCTION
void setup() {
    // Initialise RPC on M4 only to avoid M4 bootup while testing on M7
    if (RPC.cpu_id() == CM4_CPUID) {
      RPC.begin();
      RPC.println("M4 Core online");
    }
    
    // Make M4 functions available on M7
    RPC.bind("getSensorData", getSensorData);

    // Set offset for all sensors
    dht1.setHumOffset(rh_offset);
    dht2.setHumOffset(rh_offset);
    dht3.setHumOffset(rh_offset);
    dht4.setHumOffset(rh_offset);
    dht1.setTempOffset(temp_offset);
    dht2.setTempOffset(temp_offset);
    dht3.setTempOffset(temp_offset);
    dht4.setTempOffset(temp_offset);

    // initialise timer and sensors
    last_sensor_read_ms = millis();
    read_sensors();
}

// LOOP FUNCTION
void loop() {
    // DHT22 SENSORS READ NON-BLOCKING
    if ( millis() - delay_ms > last_sensor_read_ms ) {
      last_sensor_read_ms = millis();
      read_sensors();

      //DEBUG
      /*RPC.print("M4 Temperature 1: ");
      RPC.println(sensorData.temp1);
      RPC.print("M4 Temperature 2: ");
      RPC.println(sensorData.temp2);
      RPC.print("M4 Temperature 3: ");
      RPC.println(sensorData.temp3);
      RPC.print("M4 Temperature 4: ");
      RPC.println(sensorData.temp4);*/
    }

    delay(5);
}
