#include <Arduino.h>
#include <RPC.h>
#include <dhtnew.h>
#include <Arduino_CAN.h>

// Define pins for DHT22 sensors
#define DHTPIN1 2   // living space r/h
#define DHTPIN2 3   // living space l/h
#define DHTPIN3 4   // shower room
#define DHTPIN4 5   // loft

// Define DHT type
#define DHTTYPE DHT22

// Number of retries for sensor
uint8_t _retries = 3;

// Sensor delay (2000 default for DHT22)
int _delay = 2000;

//  CANBUS data Identifier List

// CAN data Orion2jr ECU ID 0x7E3
// ID    Byte  1     2     3     4     5     6     7     8
// 0x6B0       Amps  "     Volt  "     SOC   Relay "     CRC
// 0x6B1       DCL   "     "     "     HighT LowT  free  CRC
// 0x252       CCL   "     LowC HighC  Helth Count Cycl  CRC

// INVESTIGATE ABS_AMP FROM ORION

// CANBUS receive data
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

// CANBUS send data
static uint8_t const CAN_ID = 0x20; // Must be different from other devices on CANbus
uint8_t const msg_data[] = {0x6B2,0x01,0,0,0,0,0,0};
static uint8_t msg_cnt = 0;

// TEMPERATURE offset
const float temp_offset = 0.7;

// Named SensorData struct so I can use it as a data type
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

// CanData struct
struct CanData {
    unsigned int rawU;
    int rawI;
    byte soc;
    int hC;
    int lC;
    byte h;
    int fu;
    byte tH;
    byte tL;
    float ah;
    byte ry;
    byte dcl;
    byte ccl;
    byte ct;
    byte st;
    int cc;
    int fs;
    int avgI;
    unsigned int p;

    MSGPACK_DEFINE_ARRAY(rawU, rawI, soc, hC, lC, h, fu, tH, tL, ah, ry, dcl, ccl, ct, st, cc, fs, avgI, p);
};

// Declare struct variables
SensorData sensorData;
CanData canData;

// READ DHT SENSOR
float* readDHT(int pin) {
    DHTNEW sensor(pin);
    uint8_t i = 0;
    static float dhtArr[2];

    while (sensor.read() != DHTLIB_OK && i < _retries) {
        i++;
        delay(400 * i);
        RPC.println("DHT sensor read error");
    }

    if (i == _retries) {
        RPC.print(_retries);
        RPC.print(" retries from sensor on pin ");
        RPC.println(pin);
        return nullptr;
    }

    dhtArr[0] = sensor.getTemperature();
    dhtArr[1] = sensor.getHumidity();
    RPC.println("Number of Retries: ");
    RPC.println(i);
    RPC.println("Temperature reading: ");
    RPC.println(dhtArr[0]);
    RPC.println("Humidity reading: ");
    RPC.println(dhtArr[1]);

    return dhtArr;
}

// STORE SENSOR DATA
SensorData read_sensors() {
    float* DHTPIN1_readArr = readDHT(DHTPIN1);
    float* DHTPIN2_readArr = readDHT(DHTPIN2);
    float* DHTPIN3_readArr = readDHT(DHTPIN3);
    float* DHTPIN4_readArr = readDHT(DHTPIN4);

    if (DHTPIN1_readArr && DHTPIN2_readArr && DHTPIN3_readArr && DHTPIN4_readArr) {
        sensorData.temp1 = DHTPIN1_readArr[0] + temp_offset;
        sensorData.temp2 = DHTPIN2_readArr[0] + temp_offset;
        sensorData.temp3 = DHTPIN3_readArr[0] + temp_offset;
        sensorData.temp4 = DHTPIN4_readArr[0] + temp_offset;
        sensorData.humi1 = DHTPIN1_readArr[1];
        sensorData.humi2 = DHTPIN2_readArr[1];
        sensorData.humi3 = DHTPIN3_readArr[1];
        sensorData.humi4 = DHTPIN4_readArr[1];
        sensorData.avg_temp = ((DHTPIN1_readArr[0] + DHTPIN2_readArr[0]) / 2) + temp_offset;
    }
}

// SORT CANBUS MSG
CanData sort_can() {
    if (rxId == 0x6B0) {
        canData.avgI = 0;
        canData.rawU = ((rxBuf[2] << 8) + rxBuf[3]);
        canData.rawI = ((rxBuf[0] << 8) + rxBuf[1]);
        canData.soc = (rxBuf[4]);
        canData.ry = ((rxBuf[5] << 8) + rxBuf[6]);
    }
    if (rxId == 0x6B1) {
        canData.dcl = ((rxBuf[0] << 8) + rxBuf[1]);
        canData.tH = rxBuf[4];
        canData.tL = rxBuf[5];
    }
    if (rxId == 0x252) {
        canData.ccl = ((rxBuf[0] << 8) + rxBuf[1]);
        canData.lC = rxBuf[2];
        canData.hC = rxBuf[3];
        canData.h = rxBuf[4];
        canData.ct = rxBuf[5];
        canData.cc = rxBuf[6];
    }
    canData.p = (abs(canData.rawI) / 10.0) * canData.rawU / 10.0;
}

// Send can message function
void sendCan() {
  msg_cnt = 1;
}

// RPC get Sensor data function
SensorData getSensorData() {
  SensorData sensorData;
  return sensorData;
}
    
// RPC get CAN data function
CanData getCanData() {
  CanData canData;
  return canData;
}



// SETUP FUNCTION
void setup() {

    RPC.begin();
    
    Serial.begin(115200); // Initialize Serial Monitor
    while (!Serial);
    
    DHTNEW setReadDelay(_delay);

    if (!CAN.begin(CanBitRate::BR_500k)) {
        RPC.println("CAN.begin(...) failed.");
        for (;;) {}
    }
    // Make M4 functions available on M7
    RPC.bind("getSensorData", getSensorData);
    RPC.bind("getCanData", getCanData);
    RPC.bind("sendCan", sendCan);
}

// LOOP FUNCTION
void loop() {
    // CANBUS READ AND WRITE
    if (CAN.available()) {
        CanMsg msg = CAN.read();
        rxId = msg.id;
        len = msg.data_length;
        memcpy(rxBuf, msg.data, len);
        sort_can();

        // send CAN if commanded
        if ( msg_cnt && msg_cnt < 3 ) {
          memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
          CanMsg msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);
          
          // retry if send failed
          if ( int const rc = CAN.write(msg); rc <= 0 ) {
            RPC.print("CAN.write(...) failed with error code ");
            RPC.println(rc);
            msg_cnt++;
          }
          else msg_cnt = 0; // sent successfully
        }
    }
    // DHT22 SENSORS READ
    read_sensors();

    // SLOW THE LOOP A TAD
    delay(50);
}
