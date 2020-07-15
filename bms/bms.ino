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
  int32_t loadCurrent;
  int16_t balanceVoltage;
} bms_t;

static bms_t bms;

const byte extInterruptPin = 2;

static volatile unsigned long pulsePeriod = ULONG_MAX;
static volatile unsigned long pulseCount = 0;
static volatile unsigned long pulseLastTime = 0;

void extInterruptISR(void) {
  unsigned long m = millis();
  unsigned long p = m - pulseLastTime;
  // ignore pulses less than 100 ms apart
  if (p > 100) {
    pulseLastTime = m;
    pulsePeriod = p;
    pulseCount++;
  }
  //
  // pulseTime = millis();
  // pulseCount++;
}

void powerMeterSetup(void) {
  pinMode(extInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(extInterruptPin), extInterruptISR,
                  FALLING);
}

unsigned long getPower(void) {
  // static unsigned long period = ULONG_MAX;
  // static unsigned long pulseLastTime = 0L;

  noInterrupts();
  unsigned long p = pulsePeriod;
  interrupts();

  // if (t) {
  //   // update period
  //   if (pulseLastTime) {
  //     unsigned long p = t - pulseLastTime;
  //     // debounce / supress noise for min 200 ms
  //     if (p > 200) {
  //       period = p;
  //       pulseLastTime = t;
  //     }
  //   } else {
  //     pulseLastTime = t;
  //   }
  // }
  return 1800000L / p;
}

unsigned long getEnergy(void) {
  noInterrupts();
  unsigned long p = pulseCount;
  interrupts();
  return p / 2;
}

void process(void) {
  static int32_t currentFilter = 0;
  static int8_t count = 0;

  uint16_t batteryMin = 5000;
  uint16_t batteryMax = 0;
  uint16_t batterySum = 0;
  uint8_t i = 8;
  while (i--) {
    uint16_t v = bms.cellVoltage[i];
    if (v > batteryMax)
      batteryMax = v;
    if (v < batteryMin)
      batteryMin = v;
    batterySum += v;
  }

  if (batteryMin > 0) {
    bms.balanceVoltage = (batterySum / 8);
    if (bms.balanceVoltage < 3000) {
      bms.balanceVoltage = 3000;
    }
  } else {
    bms.balanceVoltage = 0;
  }

  currentFilter += bms.loadCurrent - (currentFilter / 256L);
  int32_t averageCurrent = (currentFilter / 256L);

  count++;

  bool output = (count & 0x08);
  digitalWrite(buzzerPin, output);

  if ((count & 0x03) == 0) {
    // send json status string
    Serial.print("{\"BMS\":");
    Serial.print("{\"VC\":[");
    for (int8_t i = 0; i < 8; i++) {
      Serial.print(bms.cellVoltage[i]);
      if (i < 7)
        Serial.print(",");
    }
    Serial.print("],\"T\":[");
    for (int8_t i = 0; i < 2; i++) {
      Serial.print(bms.temperature[i]);
      if (i < 1)
        Serial.print(",");
    }
    Serial.print("],\"IC\":");
    Serial.print(bms.loadCurrent);
    Serial.print(",\"IA\":");
    Serial.print(averageCurrent);
    Serial.print(",\"VB\":");
    Serial.print(batterySum);

    Serial.print(",\"P\":");
    Serial.print(getPower());
    Serial.print(",\"E\":");
    Serial.print(getEnergy());
    Serial.println("}");
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

  Serial.println("{\"BMS\": \"BOOT\"}");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == (CANBUS_CAN_ID_SHUNT | CAN_EFF_FLAG)) {
      if (canMsg.can_dlc == 8) {
        int32_t current = (int32_t)(((uint32_t)canMsg.data[0] << 16) +
                                    ((uint32_t)canMsg.data[1] << 8) +
                                    (uint32_t)canMsg.data[2]) -
                          8388608L;
        bms.loadCurrent = current;

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
