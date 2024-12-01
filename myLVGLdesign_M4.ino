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

// Sensor delay (2000 default for DHT22)
int _delay = 2000;

// Named SensorData struct
struct SensorData {
    float temp1;
    float temp2;
    float temp3;
    float temp4;
    float humi1;
    float humi2;
    float humi3;
    float humi4;
    float avg_temp;

    MSGPACK_DEFINE_ARRAY(temp1, temp2, temp3, temp4, humi1, humi2, humi3, humi4, avg_temp);
};

// Declare struct instances (non static) to refresh data
SensorData sensorData;

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
  } else if (count > 10) {
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
    float dht1Arr[2] = {999.0f, 999.0f};
    float dht2Arr[2] = {999.0f, 999.0f};
    float dht3Arr[2] = {999.0f, 999.0f};
    float dht4Arr[2] = {999.0f, 999.0f};

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

    DHTNEW setReadDelay(_delay);
}

// LOOP FUNCTION
void loop() {
    
    // DHT22 SENSORS READ
    read_sensors();
    delay(50);

    // M4 Core only
    if (!Serial) {
      /*RPC.print("M4 Temperature 1: ");
      RPC.println(sensorData.temp1);
      RPC.print("M4 Temperature 2: ");
      RPC.println(sensorData.temp2);
      RPC.print("M4 Temperature 3: ");
      RPC.println(sensorData.temp3);
      RPC.print("M4 Temperature 4: ");
      RPC.println(sensorData.temp4);*/
    }

    // M7 Core only
    if (Serial) {
      Serial.print("M7 Temperature: ");
      Serial.println(sensorData.temp3);
    }
}
