#include <Arduino.h>
#include <RPC.h>
#include <dhtnew.h>
#include <Arduino_CAN.h>

// Define pins for DHT22 sensors
#define DHTPIN1 2   // living space r/h
#define DHTPIN2 3   // living space l/h
#define DHTPIN3 4   // shower room
#define DHTPIN4 5   // loft

// Sensor delay (2000 default for DHT22)
int _delay = 2000;

// CANBUS receive data
uint32_t rxId;
uint8_t len = 0;
uint8_t rxBuf[8];

// CANBUS send data MPI through MPO
static uint8_t const CAN_ID = 0x20; // Must be different from other devices on CANbus
uint8_t const msg_data[] = {0x002, 0x01, 0, 0, 0, 0, 0, 0};
static uint8_t msg_cnt = 0;

// TEMPERATURE offset
const float temp_offset = 0.7;

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

// CanData struct
struct CanData {
    unsigned int rawU;
    unsigned int p;
    
    int fu;
    int rawI;
    int cc;
    int fs;
    int avgI;
    int kw;
    int cap;
    
    byte soc;
    byte h;
    byte hT;
    byte lT;
    byte ry;
    byte dcl;
    byte ccl;
    byte ct;
    byte st;
    
    float hC;
    float lC;
    float ah;

    MSGPACK_DEFINE_ARRAY(rawU, rawI, soc, hC, lC, h, fu, hT, lT, ah, ry, dcl, ccl, ct, st, cc, fs, avgI, kw, cap, p);
};

// Declare struct variables
static SensorData sensorData;
static CanData canData;

// READ DHT SENSOR
float* readDHT(byte pin) {
  // Create instance and initialise DHT pin
  DHTNEW sensor(pin);
  static float dhtArr[2];
  
  // if sensor ready return value else return 0
  if ( sensor.read() == DHTLIB_OK ) {
    dhtArr[0] = sensor.getTemperature();
    dhtArr[1] = sensor.getHumidity();

    return dhtArr;
  }
  else return nullptr;
}

// STORE SENSOR DATA
void read_sensors() {
    float* DHTPIN1_readArr = readDHT(DHTPIN1);
    float* DHTPIN2_readArr = readDHT(DHTPIN2);
    float* DHTPIN3_readArr = readDHT(DHTPIN3);
    float* DHTPIN4_readArr = readDHT(DHTPIN4);

    // verify data has been received before adding to struct
    if (DHTPIN1_readArr) {
        sensorData.temp1 = DHTPIN1_readArr[0] + temp_offset;
        sensorData.humi1 = DHTPIN1_readArr[1];
    }
    if (DHTPIN2_readArr) {
        sensorData.temp2 = DHTPIN2_readArr[0] + temp_offset;
        sensorData.humi2 = DHTPIN2_readArr[1];
    }
    if (DHTPIN3_readArr) {
        sensorData.temp3 = DHTPIN3_readArr[0] + temp_offset;
        sensorData.humi3 = DHTPIN3_readArr[1];
    }
    if (DHTPIN4_readArr) {
        sensorData.temp4 = DHTPIN4_readArr[0] + temp_offset;
        sensorData.humi4 = DHTPIN4_readArr[1];
    }
    if (DHTPIN1_readArr && DHTPIN2_readArr) {
       sensorData.avg_temp = ((DHTPIN1_readArr[0] + DHTPIN2_readArr[0]) / 2) + temp_offset;
    }
}

// SORT CANBUS MSG
void sort_can() {
    if (rxId == 0x6B0) {
        canData.rawI = ((rxBuf[0] << 8) + rxBuf[1]) / 10;
        canData.rawU = ((rxBuf[2] << 8) + rxBuf[3]) / 10;
        canData.soc = rxBuf[4] / 2;
        canData.ry = rxBuf[5];
        canData.st = rxBuf[6];
    }
    if (rxId == 0x6B1) {
        canData.dcl = ((rxBuf[0] << 8) + rxBuf[1]);
        canData.ccl = ((rxBuf[2] << 8) + rxBuf[3]);
        canData.hT = rxBuf[4];
        canData.lT = rxBuf[5];
        canData.fu = rxBuf[6];
    }
    if (rxId == 0x001) {
        canData.hC = rxBuf[0] / 1000.0;
        canData.lC = rxBuf[1] / 1000.0;
        canData.h = rxBuf[2];
        canData.ah = rxBuf[3];
        canData.avgI = rxBuf[4];
        canData.kw = rxBuf[5];
        canData.cap = rxBuf[6];
    }
    canData.p = (abs(canData.rawI) / 10.0) * canData.rawU / 10.0;
}

// Send CAN message function
void sendCan() {
  msg_cnt = 1;
}

// RPC get Sensor data function
SensorData getSensorData() {
  return sensorData;
}

// RPC get CAN data function
CanData getCanData() {
  return canData;
}

// SETUP FUNCTION
void setup() {
    // Initialize RPC if this is M4 core to allow testing on M7 without booting M4
    if ( RPC.cpu_id() == CM7_CPUID ) {
      Serial.begin(115200);
      while(!Serial); // wait for serial connection
      Serial.println("Serial Communication Enabled at 115200kbps");
    }
    else {
      // if this is M4 core start RPC
      RPC.begin();
    }
    
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
        CanMsg const msg = CAN.read();
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
    else { RPC.println("CAN not available"); }
    
    // DHT22 SENSORS READ
    read_sensors();
    delay(50);

    RPC.print("M4 Temperature: ");
    RPC.println(sensorData.temp3);
    RPC.print("M4 SOC: ");
    RPC.println(canData.soc);
    // if M7 running script
    if (Serial) {
      Serial.print("SOC: ");
      Serial.println(canData.soc);
      Serial.print("Temp: ");
      Serial.println(sensorData.temp3);
    }
}
