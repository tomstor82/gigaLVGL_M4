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

// CANBUS receive data
/*uint32_t rxId;
uint8_t len = 0;
uint8_t rxBuf[8];

// CANBUS send data MPI through MPO
static uint8_t const CAN_ID = 0x20; // Must be different from other devices on CANbus
uint8_t const msg_data[] = {0x002, 0x01, 0, 0, 0, 0, 0, 0};
static uint8_t msg_cnt = 0;*/

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
/*struct CanData {

    int p;                  // watt calculated by script

    float instU;             // Voltage - multiplied by 10
    float instI;             // Current - multiplied by 10 - negative value indicates charge
    float avgI;             // Average current for clock and sun symbol calculations
    float absI;             // Absolute Current NOT WORKING CORRECTLY
    float ah;               // Amp hours
    float hC;               // High Cell Voltage in 0,0001V
    float lC;               // Low Cell Voltage in 0,0001V

    byte soc;               // State of charge - multiplied by 2
    byte hT;                // Highest cell temperature * was int
    byte lT;                // Lowest cell temperature * was int
    byte ry;                // Relay status
    byte dcl;               // Discharge current limit * was unsigned int
    byte ccl;               // Charge current limit * was unsigned int
    byte ct;                // Counter to observe data received
    byte st;                // BMS Status
    byte h;                 // Health
    byte hCid;              // High Cell ID
    byte lCid;              // Low Cell ID

    int fu;                 // BMS faults
    int fs;                 // Fault messages & status from CANBus for displaying wrench icon
    int cc;                 // Total pack cycles

    float kw;               // Active power 0,1kW
    byte hs;                // Internal Heatsink
    
    MSGPACK_DEFINE_ARRAY(instU, instI, soc, hC, lC, h, fu, hT, lT, ah, ry, dcl, ccl, ct, st, cc, fs, avgI, hCid, lCid, p, absI, kw, hs);
};*/

// Declare struct instances (non static) to refresh data
SensorData sensorData;
//CanData canData;

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
SensorData read_sensors() {
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

// CONVERT UNSIGNED TO SIGNED FUNCTION ////////////////////////////////////////
/*int16_t signValue(uint16_t canValue) {
  int16_t signedValue = (canValue > 32767) ? canValue - 65536 : canValue;
  return signedValue;
}

// SORT CANBUS DATA ////////////////////////////////////////////////////////////
void sort_can() {

    // I WOULD LIKE TO COMPARE CHECKSUM BUT CPP STD LIBRARY NOT AVAILABLE I BELIEVE
    if (rxId == 0x3B) {
        canData.instU = ((rxBuf[0] << 8) + rxBuf[1]) / 10.0;
        canData.instI = (signValue((rxBuf[2] << 8) + rxBuf[3])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
        canData.absI = ((rxBuf[4] << 8) + rxBuf[5]) / 10.0; // orion2jr issue: set signed and -32767 to fix
        canData.soc = rxBuf[6] / 2;
    }
    if(rxId == 0x6B2) {
        canData.lC = ((rxBuf[0] << 8) + rxBuf[1]) / 10000.00;
        canData.hC = ((rxBuf[2] << 8) + rxBuf[3]) / 10000.00;
        canData.h = rxBuf[4];
        canData.cc = (rxBuf[5] << 8) + rxBuf[6];
    }
    if(rxId == 0x0A9) {    
        canData.ry = rxBuf[0];
        canData.ccl = rxBuf[1];
        canData.dcl = rxBuf[2];
        canData.ah = ((rxBuf[3] << 8) + rxBuf[4]) / 10.0;
        canData.avgI = (signValue((rxBuf[5] << 8) + rxBuf[6])) / 10.0; // orion2jr issue: unsigned value despite ticket as signed
    }
    if(rxId == 0x0BD) {
        canData.fu = (rxBuf[0] << 8) + rxBuf[1];
        canData.hT = rxBuf[2];
        canData.lT = rxBuf[3];
        canData.ct = rxBuf[4];
        canData.st = rxBuf[5];
    }
    if(rxId == 0x0BE) {
        canData.hCid = rxBuf[0];
        canData.lCid = rxBuf[1];
        canData.hs = rxBuf[2];
        canData.kw = ((rxBuf[3] << 8) + rxBuf[4]) / 10.0;
    }
    canData.p = canData.avgI * canData.instU;
}*/

// Send CAN message function
/*void sendCan() {
  msg_cnt = 1;
}*/

// RPC get Sensor data function
SensorData getSensorData() {
  //SensorData sensorData; // is this neccessary
  return sensorData;
}

// RPC get CAN data function
/*CanData getCanData() {
  CanData canData; // is this neccessary
  return canData;
}*/

// SETUP FUNCTION
void setup() {

    // Initialise RPC on M4 only to avoid M4 bootup while testing on M7
    if (RPC.cpu_id() == CM4_CPUID) {
      RPC.begin();
      RPC.println("M4 Core online");
    }
    
    DHTNEW setReadDelay(_delay);

    /*if (!CAN.begin(CanBitRate::BR_500k)) {
        RPC.println("CAN.begin(...) failed.");
        for (;;) {}
    }
    else RPC.println("CAN started");*/
    // Make M4 functions available on M7
    RPC.bind("getSensorData", getSensorData);
    /*RPC.bind("getCanData", getCanData);
    RPC.bind("sendCan", sendCan);*/
}

// LOOP FUNCTION
void loop() {
    // CANBUS READ AND WRITE 
    /*if (CAN.available()) {
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
    else { RPC.println("M4 CAN not available"); }*/
    
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
