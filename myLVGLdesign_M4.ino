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

// MACRO FOR DHT PINS
#define DHT1_PIN 2   // living space r/h
#define DHT2_PIN 3   // living space l/h
#define DHT3_PIN 4   // shower room
#define DHT4_PIN 5   // loft ceiling

// NAMED STRUCTURE USED AS DATA-TYPE, INITIALISED WITH FAULTY DATA
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

// DECLARE STRUCT VARIABLE
SensorData sensorData;

// GLOBAL VARIABLES
uint16_t initial_read_delay_ms = 1000; // DHT22 sensor 2000ms recommended for accuracy but sensor will read around 360-370ms
uint32_t last_sensor_read_ms = 0;

float rh_offset[4] = {0, 0, 0, 0};
float temp_offset[4] = {0.7, 0.7, 0.7, 0.7};

// SET SENSOR TYPE 22 = dht22/am2320-22
DHTNEW setType(22);

// CREATE INSTANCES OF DHT SENSORS
DHTNEW dht1(DHT1_PIN);
DHTNEW dht2(DHT2_PIN);
DHTNEW dht3(DHT3_PIN);
DHTNEW dht4(DHT4_PIN);

// READ DHT SENSOR - RETURNING 200 IF READ OK, 100 FAILED READ, 0 DELAY NOT MET
byte readDHT(DHTNEW &sensor, float* dhtArr, byte retries) {

  static uint16_t read_delay_ms = 0;

  // INCREMENT READ DELAY TO SEE IF IT IS A SYNC ISSUE
  if ( retries ) {
     read_delay_ms += 250;
     Serial.print("readDHT incrementing delay to: "); Serial.println(read_delay_ms+initial_read_delay_ms);
  }
  else {
    read_delay_ms = 0;
  }
  
  // RETURN IF READ DELAY NOT YET PASSED
  if ( millis() - ( initial_read_delay_ms + read_delay_ms ) > last_sensor_read_ms ) {
    last_sensor_read_ms = millis();
  }
  else {
    return 0;
  }
  

  // IF SENSOR READY RETURN READINGS
  if (sensor.read() == DHTLIB_OK) {
    dhtArr[0] = sensor.getTemperature();
    dhtArr[1] = sensor.getHumidity();
    return 200;
  }
  // IF RETRIES EXCEED 5 SET DATA TO INVALID AKA 999.9
  else if (retries > 5) {
    dhtArr[0] = 999.9f;
    dhtArr[1] = 999.9f;
    // RESTART SENSOR
    sensor.powerUp();
    RPC.println("M4 Restarting DHT22 sensor after 6 retries");
    return 200;
  }
  // IF SENSOR NOT READY RETURN FALSE TO INCREMENT RETRIES IN CALLBACK
  else {
    // RESET SENSOR AFTER 3 RETRIES
    if (retries == 3) {
      sensor.reset();
      RPC.println("M4 Resetting DHT22 sensor after 3 retries");
    }
    // POWER DOWN SENSOR AFTER 5 RETRIES
    else if (retries == 5) {
      sensor.powerDown();
      RPC.println("M4 Shutting down DHT22 sensor after 5 retries");
    }
    return 100;
  }
}

// STORE SENSOR DATA
void read_sensors() {

  // NESTED DATA ARRAY
  float dht_data[4][2];

  // SENSOR READ RETRY ARRAY
  static byte dht_retries[4] = {0, 0, 0, 0};

  byte dht1Read = readDHT(dht1, dht_data[0], dht_retries[0]);
  byte dht2Read = readDHT(dht2, dht_data[1], dht_retries[1]);
  byte dht3Read = readDHT(dht3, dht_data[2], dht_retries[2]);
  byte dht4Read = readDHT(dht4, dht_data[3], dht_retries[3]);

  // LIVING ROOM R/H SENSOR
  if (dht1Read == 200) {
    sensorData.temp1 = dht_data[0][0];
    sensorData.rh1 = dht_data[0][1];
    dht_retries[0] = 0;
  }
  else if (dht1Read == 100) {
    dht_retries[0]++;
  }
  // LIVING ROOM L/H SENSOR
  if (dht2Read == 200) {
    sensorData.temp2 = dht_data[1][0];
    sensorData.rh2 = dht_data[1][1];
    dht_retries[1] = 0;
  }
  else if (dht2Read == 100) {
    dht_retries[1]++;
  }
  // SHOWER ROOM SENSOR
  if (dht3Read == 200) {
    sensorData.temp3 = dht_data[2][0];
    sensorData.rh3 = dht_data[2][1];
    dht_retries[2] = 0;
  }
  else if (dht3Read == 100) {
    dht_retries[2]++;
  }
  // LOFT CEILING SENSOR
  if (dht4Read == 200) {
    sensorData.temp4 = dht_data[3][0];
    sensorData.rh4 = dht_data[3][1];
    dht_retries[3] = 0;
  }
  else if (dht4Read == 100) {
    dht_retries[3]++;
  }

  // VERIFY DATA BEFORE CALCULATION
  if (dht1Read == 200 && dht2Read == 200 && dht4Read == 200) {
    sensorData.avg_temp = (dht_data[0][0] + dht_data[1][0] + dht_data[3][0]) / 3;
  }
  else {
    sensorData.avg_temp = 999.9f; // Set to invalid value if any sensor data is missing
  }
}

// GET SENSOR DATA FUNCTION FOR RPC USAGE
SensorData getSensorData() {
  return sensorData;
}

// SETUP FUNCTION
void setup() {

  // INITIALISE RPC ON M4 CORE
  if (RPC.cpu_id() == CM4_CPUID) {
    RPC.begin();
    RPC.println("M4 Core online");
  }

  // BIND M4 FUNCTION TO RPC TO ALLOW M7 TO CALL IT RETURNING STRUCT DATA
  RPC.bind("getSensorData", getSensorData);

  // SET SENSOR OFFSETS
  dht1.setHumOffset(rh_offset[0]);
  dht2.setHumOffset(rh_offset[1]);
  dht3.setHumOffset(rh_offset[2]);
  dht4.setHumOffset(rh_offset[3]);
  dht1.setTempOffset(temp_offset[0]);
  dht2.setTempOffset(temp_offset[1]);
  dht3.setTempOffset(temp_offset[2]);
  dht4.setTempOffset(temp_offset[3]);

  // INITALISE TIMER
  last_sensor_read_ms = millis();
}

// LOOP FUNCTION
void loop() {

  read_sensors();

  delay(50);
}
