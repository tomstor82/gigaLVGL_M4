#include <Arduino.h>
#include <RPC.h>
#include <dhtnew.h>

// DHT PIN layout from left to right
// =================================
// FRONT : DESCRIPTION
// pin 1 : VCC
// pin 2 : DATA
// pin 3 : Not Connected
// pin 4 : GND

// Define pins for DHT22 sensors
#define DHT1_PIN 2   // living space r/h
#define DHT2_PIN 3   // living space l/h
#define DHT3_PIN 4   // shower room
#define DHT4_PIN 5   // loft

// Named SensorData struct initialised with faulty data
struct SensorData {
    float temp1 = 999.0f;
    float temp2 = 999.0f;
    float temp3 = 999.0f;
    float temp4 = 999.0f;
    float humi1 = 999.0f;
    float humi2 = 999.0f;
    float humi3 = 999.0f;
    float humi4 = 999.0f;
    float avg_temp = 999.0f;

    MSGPACK_DEFINE_ARRAY(temp1, temp2, temp3, temp4, humi1, humi2, humi3, humi4, avg_temp);
};

// Declare struct instances (non static) to refresh data
SensorData sensorData;

// Global Variables
uint16_t _delay = 2000; // DHT22 sensor 2000ms recommended for accuracy but sensor will read around 360-370ms
uint32_t _lastSensorReadTime;

float _humOffset = 0;  // Can expand on this and temp for individual sensors
float _tempOffset = 0.7;

// Set sensor type 22 = dht22/am2320-22
DHTNEW setType(22);

// Create instances of DHT sensors
DHTNEW dht1(DHT1_PIN);
DHTNEW dht2(DHT2_PIN);
DHTNEW dht3(DHT3_PIN);
DHTNEW dht4(DHT4_PIN);

// READ DHT SENSOR
bool readDHT(DHTNEW &sensor, float* dhtArr) {
  static uint8_t count = 0; // static to be remembered between calls

  // if sensor ready return value else return 0
  if (sensor.read() == DHTLIB_OK) {
    dhtArr[0] = sensor.getTemperature();
    dhtArr[1] = sensor.getHumidity();
    count = 0;
    return true;
  } else if (count > 5) { // 5 reads or else it won't work
    dhtArr[0] = 999.0f;
    dhtArr[1] = 999.0f;
    count = 0;
    return true;
  } else {
    count++;
    return false;
  }
}

// STORE SENSOR DATA
void read_sensors() {
    float dht1Arr[2];
    float dht2Arr[2];
    float dht3Arr[2];
    float dht4Arr[2];

    bool dht1Read = readDHT(dht1, dht1Arr);
    bool dht2Read = readDHT(dht2, dht2Arr);
    bool dht3Read = readDHT(dht3, dht3Arr);
    bool dht4Read = readDHT(dht4, dht4Arr);

    // Verify data has been received before adding to struct
    if (dht1Read) {
        sensorData.temp1 = dht1Arr[0];
        sensorData.humi1 = dht1Arr[1];
    }
    if (dht2Read) {
        sensorData.temp2 = dht2Arr[0];
        sensorData.humi2 = dht2Arr[1];
    }
    if (dht3Read) {
        sensorData.temp3 = dht3Arr[0];
        sensorData.humi3 = dht3Arr[1];
    }
    if (dht4Read) {
        sensorData.temp4 = dht4Arr[0];
        sensorData.humi4 = dht4Arr[1];
    }

    // Check for valid data before calculating avg_temp
    if (dht1Read && dht2Read && dht4Read) {
        sensorData.avg_temp = (dht1Arr[0] + dht2Arr[0] + dht4Arr[0]) / 3;
    } else {
        sensorData.avg_temp = 999.0f; // Set to invalid value if any sensor data is missing
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

    // Set read delay and temperature offset for all sensors
    /*dht1.setReadDelay(_delay);
    dht2.setReadDelay(_delay);
    dht3.setReadDelay(_delay);
    dht4.setReadDelay(_delay);*/
    dht1.setHumOffset(_humOffset);
    dht2.setHumOffset(_humOffset);
    dht3.setHumOffset(_humOffset);
    dht4.setHumOffset(_humOffset);
    dht1.setTempOffset(_tempOffset);
    dht2.setTempOffset(_tempOffset);
    dht3.setTempOffset(_tempOffset);
    dht4.setTempOffset(_tempOffset);
}

// LOOP FUNCTION
void loop() {
    // DHT22 SENSORS READ NON-BLOCKING
    if (millis() > _lastSensorReadTime + _delay) {
      _lastSensorReadTime = millis();
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

    // M4 Core only
    if (!Serial) {
      
    }

    // M7 Core only
    if (Serial) {
      /*Serial.print("M7 Temperature: ");
      Serial.println(sensorData.temp3);*/
    }
    delay(50);
}
