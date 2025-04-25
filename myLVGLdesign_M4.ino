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
uint16_t initial_read_delay_ms = 2000; // DHT22 sensor 2000ms recommended for accuracy but sensor will read around 360-370ms. 3000 set as some sensors struggle and reboot doesn't work
uint16_t added_read_delay_ms[4] = {0, 0, 0, 0};
uint32_t last_sensor_read_ms[4] = {0, 0, 0, 0};

// OFFSETS FOR INDIVIDUAL SENSORS
float rh_offset[4] = {0, 0, 0, 0};
float temp_offset[4] = {0.7, 0.7, 0.7, 0.7};

// SET SENSOR TYPE 22 = dht22/am2320-22
DHTNEW setType(22);

// STORE DHT22 PINS TO ARRAY
DHTNEW dht[4] = {2, 3, 4, 5}; // living space right, left, shower room and loft ceiling


// READ DHT SENSOR /////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool read_sensor(DHTNEW &sensor, float* dhtArr, byte retries, byte index) {

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
    sensor.reset();
    added_read_delay_ms[index] = 0;
    //RPC.print("M4 resetting sensor ");
    //RPC.println(index+1);
  }
  // IF SENSOR NOT READY RETURN FALSE TO INCREMENT RETRIES AND ADD EXTRA READ DELAY
  else {
    added_read_delay_ms[index] += 500;
    //char debug_msg[31];
    //snprintf(debug_msg, sizeof(debug_msg), "M4 Sensor %d read delay %d ms", index+1, (initial_read_delay_ms + added_read_delay_ms[index]));
    //RPC.println(debug_msg);
  }
  return 0;
}


// CHECK AND VALIDATE SENSOR DATA ///////////////////////////////////////////////////////////////////////////////////////////////
void check_sensors(DHTNEW &sensor, byte index) {

  // SENSOR READ RETRY ARRAY
  static byte dht_retries[4] = {0, 0, 0, 0};

  float dht_data[2];
  bool dht_read;
  dht_read = read_sensor(sensor, dht_data, dht_retries[index], index);

  if (dht_read || dht_retries[index] == 5) {
      dht_retries[index] = 0;
      switch (index) {
        case 0:
          sensorData.temp1 = dht_data[0];
          sensorData.rh1 = dht_data[1];
          break;
        case 1:
          sensorData.temp2 = dht_data[0];
          sensorData.rh2 = dht_data[1];
          break;
        case 2:
          sensorData.temp3 = dht_data[0];
          sensorData.rh3 = dht_data[1];
          break;
        case 3:
          sensorData.temp4 = dht_data[0];
          sensorData.rh4 = dht_data[1];
      }
    }
    else {
      dht_retries[index]++;
    }

  // VERIFY DATA BEFORE CALCULATION AVERAGE
  if (sensorData.temp1 != 99.9f && sensorData.temp2 != 99.9f && sensorData.temp4 != 99.9f) {
    sensorData.avg_temp = (sensorData.temp1 + sensorData.temp2 + sensorData.temp4) / 3;
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
  for (byte i = 0; i < 4; i++) {
    dht[i].setHumOffset(rh_offset[i]);
    dht[i].setTempOffset(temp_offset[i]);
  }
}


// LOOP FUNCTION /////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // UNBLOCKING SENSORS CHECK ONLY AFTER READ DELAY HAS BEEN EXCEEDED
  for (byte i = 0; i < 4; i++) {
    if (millis() - initial_read_delay_ms - added_read_delay_ms[i] > last_sensor_read_ms[i]) {
      last_sensor_read_ms[i] = millis();
      check_sensors(dht[i], i);
    }
  }

  delay(5);
}
