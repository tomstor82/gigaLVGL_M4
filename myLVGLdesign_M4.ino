#include <Arduino.h>
#include <RPC.h>
#include <dhtnew.h>
//#include <Arduino_CAN.h>

// Define pins for DHT22 sensors
#define DHTPIN1 2   // living space r/h
#define DHTPIN2 3   // living space l/h
#define DHTPIN3 4   // shower room
#define DHTPIN4 5   // loft

// Sensor delay (2000 default for DHT22)
int _delay = 2000;

// TEMPERATURE offset
//const float temp_offset = 0.7;

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

// Initiate struct instance
SensorData sensorData;

// READ DHT SENSOR
float* readDHT(byte pin) {
  // Create instance and initialise DHT pin
  DHTNEW sensor(pin);
  static float dhtArr[2];
  static uint8_t count = 0;
  
  // if sensor ready return value else return 0
  if ( sensor.read() == DHTLIB_OK ) {
    dhtArr[0] = sensor.getTemperature();
    dhtArr[1] = sensor.getHumidity();

    count = 0; // reset count
  }
  // try 10 times to read data before setting 999
  else if ( count > 10 ) {
    for (uint8_t i = 0; i < 2; i++ ) {
      dhtArr[i] = 999.0f;
    }
    count = 0; // reset counter
  }

  count++;
  
  return dhtArr;
}

// STORE SENSOR DATA
void read_sensors() {
    float* DHTPIN1_readArr = readDHT(DHTPIN1);
    float* DHTPIN2_readArr = readDHT(DHTPIN2);
    float* DHTPIN3_readArr = readDHT(DHTPIN3);
    float* DHTPIN4_readArr = readDHT(DHTPIN4);

    // verify data has been received before adding to struct
    if (DHTPIN1_readArr) {
        sensorData.temp1 = DHTPIN1_readArr[0];
        sensorData.humi1 = DHTPIN1_readArr[1];
    }
    if (DHTPIN2_readArr) {
        sensorData.temp2 = DHTPIN2_readArr[0];
        sensorData.humi2 = DHTPIN2_readArr[1];
    }
    if (DHTPIN3_readArr) {
        sensorData.temp3 = DHTPIN3_readArr[0];
        sensorData.humi3 = DHTPIN3_readArr[1];
    }
    if (DHTPIN4_readArr) {
        sensorData.temp4 = DHTPIN4_readArr[0];
        sensorData.humi4 = DHTPIN4_readArr[1];
    }
    if (DHTPIN1_readArr && DHTPIN2_readArr) {
       sensorData.avg_temp = (DHTPIN1_readArr[0] + DHTPIN2_readArr[0]) / 2;
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
    
    DHTNEW setReadDelay(_delay);
    // Make M4 functions available on M7
    RPC.bind("getSensorData", getSensorData);
}

// LOOP FUNCTION
void loop() {
    
    // DHT22 SENSORS READ
    read_sensors();
    delay(50);
     // M4 Core only
    if ( !Serial ) {
      RPC.print("M4 Temperature: ");
      RPC.println(sensorData.temp3);
    }

    // M7 Core only
    if ( Serial ) {
      Serial.print("M7 Temperature: ");
      Serial.println(sensorData.temp3);
    }
}
