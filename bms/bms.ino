#include <SPI.h>
#include <limits.h>

#include "src/mcp2515.h"

struct can_frame canMsg;
MCP2515 mcp2515(53);

const int buzzerPin = 8;

#define CANBUS_CAN_ID_SHUNT 40
#define CANBUS_CAN_ID_BMS 300

typedef struct {
  int16_t cellVoltage[8];
  int16_t temperature[2];
  int32_t chargeMilliAmps;
  int16_t balanceVoltage;
} bms_t;

static bms_t bms;

const byte gridPowerPin = 2;
const byte inverterPowerPin = 3;

static volatile unsigned long gridPulsePeriod = ULONG_MAX;
static volatile unsigned long gridPulseCount = 0;
static volatile unsigned long gridPulseLastTime = 0;

static volatile unsigned long inverterPulsePeriod = ULONG_MAX;
static volatile unsigned long inverterPulseCount = 0;
static volatile unsigned long inverterPulseLastTime = 0;

void gridPowerISR(void) {
  unsigned long m = millis();
  unsigned long p = m - gridPulseLastTime;

  if (p > 100) { // ignore pulses less than 100 ms apart
    gridPulseLastTime = m;
    gridPulsePeriod = p;
    gridPulseCount++;
  }
}

void inverterPowerISR(void) {
  unsigned long m = millis();
  unsigned long p = m - inverterPulseLastTime;

  if (p > 100) { // ignore pulses less than 100 ms apart
    inverterPulseLastTime = m;
    inverterPulsePeriod = p;
    inverterPulseCount++;
  }
}

void powerMeterSetup(void) {
  pinMode(gridPowerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(gridPowerPin), gridPowerISR,
                    FALLING);
  pinMode(inverterPowerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inverterPowerPin), inverterPowerISR,
                    FALLING);
}

unsigned long getGridPower(void) {
  // static unsigned long period = ULONG_MAX;
  // static unsigned long lastCallTime = 0L;
  //
  // unsigned long m = millis();
  // unsigned long timeSinceLastCall = m - lastCallTime;

  noInterrupts();
  unsigned long p = gridPulsePeriod;
  // unsigned long c = gridPulseCount;
  interrupts();

  // if (t) {
  //   // update period
  //   if (gridPulseLastTime) {
  //     unsigned long p = t - gridPulseLastTime;
  //     // debounce / supress noise for min 200 ms
  //     if (p > 200) {
  //       period = p;
  //       gridPulseLastTime = t;
  //     }
  //   } else {
  //     gridPulseLastTime = t;
  //   }
  // }

  return 1800000L / p;
}

unsigned long getInverterPower(void) {
  noInterrupts();
  unsigned long p = inverterPulsePeriod;
  interrupts();
  return 1800000L / p;
}

unsigned long gridGetEnergy(void) {
  noInterrupts();
  unsigned long c = gridPulseCount;
  interrupts();
  return c / 2;
}

unsigned long inverterGetEnergy(void) {
  noInterrupts();
  unsigned long c = gridPulseCount;
  interrupts();
  return c / 2;
}

void process(void) {
  // called 4 times per second, triggered by current shunt canbus message
  static int32_t currentAverageSum = 0;
  static int32_t chargeMilliCoulombs = 0;
  static int8_t count = 0;

  uint16_t cellMin = 5000;
  uint16_t cellMax = 0;
  uint16_t cellSum = 0;
  uint8_t i = 8;
  while (i--) {
    uint16_t v = bms.cellVoltage[i];
    if (v > cellMax)
      cellMax = v;
    if (v < cellMin)
      cellMin = v;
    cellSum += v;
  }

  if (cellMax > 0) {
    bms.balanceVoltage = cellMax - 1; //(cellSum / 8);
    if (bms.balanceVoltage < 3000) {
      bms.balanceVoltage = 3000;
    }
  } else {
    bms.balanceVoltage = 0;
  }

  currentAverageSum += bms.chargeMilliAmps;

  count++;
  bool buzzer = (count & 0x08) && (bms.chargeMilliAmps < -5000);
  digitalWrite(buzzerPin, buzzer);

  if ((count & 0x03) == 0) {
    // update once per second

    //  nohup socat /dev/ttyUSB1,echo=0,b9600
    //  UDP4-DATAGRAM:192.168.8.255:12345,broadcast </dev/null >/dev/null 2>&1 &

    int32_t chargeMilliAmps = currentAverageSum / 4;
    currentAverageSum = 0;

    chargeMilliCoulombs += chargeMilliAmps;

    Serial.print("vbat=");
    Serial.print((float)cellSum / 1000.0);
    for (int8_t i = 0; i < 8; i++) {
      Serial.print(",vc");
      Serial.print(i + 1);
      Serial.print("=");
      Serial.print((float)bms.cellVoltage[i] / 1000.0, 3);
    }
    Serial.print(",vbal=");
    Serial.print((float)bms.balanceVoltage / 1000.0, 3);
    Serial.print(",ibat=");
    Serial.print((float)chargeMilliAmps / 1000.0);
    Serial.print(",qbat=");
    Serial.print((float)chargeMilliCoulombs / 3600000.0);
    Serial.print(",tbat=");
    Serial.print(bms.temperature[0]);
    Serial.print(",pgrd=");
    Serial.print(getGridPower());
    Serial.print("\n");
  }
}

void setup() {
  Serial.begin(9600);
  SPI.begin();

  powerMeterSetup();

  pinMode(buzzerPin, OUTPUT);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.print("bms=1\n");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == (CANBUS_CAN_ID_SHUNT | CAN_EFF_FLAG)) {
      if (canMsg.can_dlc == 8) {
        int32_t current = (int32_t)(((uint32_t)canMsg.data[0] << 16) +
                                    ((uint32_t)canMsg.data[1] << 8) +
                                    (uint32_t)canMsg.data[2]) -
                          8388608L;
        bms.chargeMilliAmps = current;

        process();

        // send can message
        canMsg.can_id = (CANBUS_CAN_ID_BMS | CAN_EFF_FLAG);
        canMsg.can_dlc = 2;
        canMsg.data[0] = bms.balanceVoltage >> 8;
        canMsg.data[1] = bms.balanceVoltage;
        mcp2515.sendMessage(&canMsg);
      }
    }
    if (canMsg.can_id == ((CANBUS_CAN_ID_BMS + 1) | CAN_EFF_FLAG)) {
      if (canMsg.can_dlc == 8) {
        // update cell voltages
        bms.cellVoltage[0] =
            ((uint16_t)canMsg.data[0] << 8) + (uint16_t)canMsg.data[1];
        bms.cellVoltage[1] =
            ((uint16_t)canMsg.data[2] << 8) + (uint16_t)canMsg.data[3];
        bms.cellVoltage[2] =
            ((uint16_t)canMsg.data[4] << 8) + (uint16_t)canMsg.data[5];
        bms.cellVoltage[3] =
            ((uint16_t)canMsg.data[6] << 8) + (uint16_t)canMsg.data[7];
      }
    }
    if (canMsg.can_id == ((CANBUS_CAN_ID_BMS + 2) | CAN_EFF_FLAG)) {
      if (canMsg.can_dlc == 8) {
        // update cell voltages
        bms.cellVoltage[4] =
            ((uint16_t)canMsg.data[0] << 8) + (uint16_t)canMsg.data[1];
        bms.cellVoltage[5] =
            ((uint16_t)canMsg.data[2] << 8) + (uint16_t)canMsg.data[3];
        bms.cellVoltage[6] =
            ((uint16_t)canMsg.data[4] << 8) + (uint16_t)canMsg.data[5];
        bms.cellVoltage[7] =
            ((uint16_t)canMsg.data[6] << 8) + (uint16_t)canMsg.data[7];
      }
    }
    if (canMsg.can_id == ((CANBUS_CAN_ID_BMS + 4) | CAN_EFF_FLAG)) {
      if (canMsg.can_dlc == 8) {
        // update temperatures
        bms.temperature[0] = (int16_t)canMsg.data[0] - 40;
        bms.temperature[1] = (int16_t)canMsg.data[1] - 40;
      }
    }
  }
}
