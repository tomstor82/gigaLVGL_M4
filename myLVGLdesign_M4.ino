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

// NAMED STRUCTURE USED AS DATA-TYPE, INITIALISED WITH FAULTY DATA
struct SensorData {
    float temp1 = 99.9f;
    float temp2 = 99.9f;
    float temp3 = 99.9f;
    float temp4 = 99.9f;
    float rh1 = 99.9f;
    float rh2 = 99.9f;
    float rh3 = 99.9f;
    float rh4 = 99.9f;
    float avg_temp = 99.9f;

    MSGPACK_DEFINE_ARRAY(temp1, temp2, temp3, temp4, rh1, rh2, rh3, rh4, avg_temp);
};

// DECLARE STRUCT VARIABLE
SensorData sensorData;

// GLOBAL VARIABLES
uint16_t read_delay_ms = 3000; // DHT22 sensor 2000ms recommended for accuracy but sensor will read around 360-370ms. 3000 set as some sensors struggle and reboot doesn't work
uint32_t last_sensor_read_ms = 0;

// OFFSETS FOR INDIVIDUAL SENSORS
float rh_offset[4] = {0, 0, 0, 0};
float temp_offset[4] = {0.7, 0.7, 0.7, 0.7};

// SET SENSOR TYPE 22 = dht22/am2320-22
DHTNEW setType(22);

// STORE DHT22 PINS TO ARRAY
DHTNEW dht[4] = {2, 3, 4, 5}; // living space right, left, shower room and loft ceiling


// READ DHT SENSOR /////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool read_sensor(DHTNEW &sensor, float* dhtArr, byte retries) {  

  // IF SENSOR READY RETURN READINGS
  if (sensor.read() == DHTLIB_OK) {
    dhtArr[0] = sensor.getTemperature();
    dhtArr[1] = sensor.getHumidity();
    return 1;
  }
  // IF 5 RETRIES SET DATA TO INVALID AKA 99.9
  else if (retries == 5) {
    dhtArr[0] = 99.9f;
    dhtArr[1] = 99.9f;
    return 0;
  }
  // IF SENSOR NOT READY RETURN FALSE TO INCREMENT RETRIES
  else {
    return 0;
  }
}


// CHECK AND VALIDATE SENSOR DATA ///////////////////////////////////////////////////////////////////////////////////////////////
void check_sensors() {

  // SENSOR ARRAY
  bool dht_read[4];

  // NESTED DATA ARRAY
  float dht_data[4][2];

  // SENSOR READ RETRY ARRAY
  static byte dht_retries[4] = {0, 0, 0, 0};

  // ITERATE THROUGH SENSOR ARRAY AND STORE DATA TO STRUCT OR INCREMENT RETRIES
  for (byte i = 0; i < 3; i++) {

    // CALL READ FUNCTION
    dht_read[i] = read_sensor(dht[i], dht_data[i], dht_retries[i]);

    // CHECK RESULT
    if (dht_read[i] || dht_retries[i] == 5) {
      dht_retries[i] = 0;
      switch (i) {
        case 0:
          sensorData.temp1 = dht_data[i][0];
          sensorData.rh1 = dht_data[i][1];
          break;
        case 1:
          sensorData.temp2 = dht_data[i][0];
          sensorData.rh2 = dht_data[i][1];
          break;
        case 2:
          sensorData.temp3 = dht_data[i][0];
          sensorData.rh3 = dht_data[i][1];
          break;
        case 3:
          sensorData.temp4 = dht_data[i][0];
          sensorData.rh4 = dht_data[i][1];
      }
    }
    else {
      dht_retries[i]++;
    }
  }

  // VERIFY DATA BEFORE CALCULATION AVERAGE
  if (dht_read[0] && dht_read[1] && dht_read[3]) {
    sensorData.avg_temp = (dht_data[0][0] + dht_data[1][0] + dht_data[3][0]) / 3;
  }
  else {
    sensorData.avg_temp = 99.9f; // Set to invalid value if any sensor data is missing
  }
}


// GET SENSOR DATA FUNCTION FOR RPC USAGE ///////////////////////////////////////////////////////////////////////////////////
SensorData getSensorData() {
  return sensorData;
}


// SETUP FUNCTION ///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  // INITIALISE RPC ON M4 CORE
  if (RPC.cpu_id() == CM4_CPUID) {
    RPC.begin();
    RPC.println("M4 Core online");
  }

  // BIND M4 FUNCTION TO RPC TO ALLOW M7 TO CALL IT RETURNING STRUCT DATA
  RPC.bind("getSensorData", getSensorData);

  // SET SENSOR OFFSETS
  for (byte i = 0; i < 3; i++) {
    dht[i].setHumOffset(rh_offset[i]);
    dht[i].setTempOffset(temp_offset[i]);
  }

  // INITALISE TIMER
  last_sensor_read_ms = millis();
}


// LOOP FUNCTION /////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // UNBLOCKING SENSORS CHECK ONLY AFTER READ DELAY HAS BEEN EXCEEDED
  if (millis() - read_delay_ms > last_sensor_read_ms) {
    last_sensor_read_ms = millis();
    check_sensors();
  }

  delay(5);
}
