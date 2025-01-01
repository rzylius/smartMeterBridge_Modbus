#include "secrets.h"

#include <ModbusRTU.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <SimpleSyslog.h>

#define MASTER_SLAVE_ID 2         // Smart meter ID for Modbus RTU
#define START_REG 20482           // Starting register to read from the smart meter
#define REG_COUNT 48              // Number of registers to read from the smart meter
#define REFRESH_INTERVAL 500      // refresh interval in ms of smartmeter
#define BATT_REFRESH_INTERVAL 60000 // maximum interval of information from battery update

#define OVUM_SLAVE_ID 18           // Slave ID of mbOvum device
#define OVUM_REGISTER 50          // Register to write in mbOvum device

// Syslog server configuration
SimpleSyslog syslog(SYSLOG_NAME, HOSTNAME, SYSLOG_SERVER_IP);
#define SYSLOG_FACILITY FAC_USER
#define LOG_INFO(fmt, ...)    syslog.printf(SYSLOG_FACILITY, PRI_INFO, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   syslog.printf(SYSLOG_FACILITY, PRI_ERROR, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)   syslog.printf(SYSLOG_FACILITY, PRI_DEBUG, fmt, ##__VA_ARGS__)

ModbusRTU mbMeter;               // Modbus RTU master instance on Serial2 (smart meter)
ModbusRTU mbOvum;           // Modbus RTU master instance on Serial1 (mbOvum)
ModbusIP mbTCP;                   // Modbus TCP server instance

float battPower;
float householdPower;
unsigned long battUpdateTime;
unsigned long lastReadTime = 0;
uint16_t meterRegisters[REG_COUNT]; // Buffer to store values read from the smart meter

bool meterCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event != Modbus::EX_SUCCESS) {
    LOG_ERROR("Master request error: 0x%02X", event );
  }
  return true;
}

bool ovumCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event != Modbus::EX_SUCCESS) {
    LOG_ERROR("Ovum Master write error: 0x%02X", event);
  } 
  return true;
}

uint16_t cbOnGet50(TRegister* reg, uint16_t val) {
  // register 51, if not 0, overrides 
  if (mbTCP.Hreg(51) != 0) {
    return mbTCP.Hreg(51);
  } else {
    float totalPower = householdPower; // no battPower is considered in this formula
    int16_t totalPowerOvum = (int16_t)(totalPower * -100.0f);
    LOG_DEBUG("total power calculation. householdPower=%.2f, battPower=%.2f, totalPower=%.2f, totalPowerOvum=%d",
                householdPower, battPower, totalPower, totalPowerOvum);
    return totalPowerOvum;
  }
}

uint16_t cbOnSetBattery(TRegister* reg, uint16_t val) {
  battPower = (mbTCP.Hreg(769) - mbTCP.Hreg(770)) * mbTCP.Hreg(768) / 100 /1000; // calculate battery charge(+) or discharge (-) power in kw
  LOG_DEBUG("cbOnSetBattery callback address: %d, battPower=%.2f", reg->address.address, battPower); 
  battUpdateTime = millis();
  return val;
}

float combineRegistersToFloat(uint16_t reg1, uint16_t reg2) {
  // Combine the two registers into a single 32-bit variable
  uint32_t combinedValue = (reg1 << 16) | reg2;

  // Convert the 32-bit value to a float
  float floatValue;
  memcpy(&floatValue, &combinedValue, sizeof(float));

  return floatValue;
}

void setup() {
  Serial.begin(115200);

  // Initialize Modbus RTU Master Serial (Smart Meter)
  Serial2.begin(9600, SERIAL_8E1, 17, 16); // GPIO17 for RX, GPIO16 for TX
  mbMeter.begin(&Serial2);
  mbMeter.master();

  // Initialize Modbus RTU Master Serial1 (mbOvum)
  Serial1.begin(19200, SERIAL_8N1, 19, 18); // GPIO19 for RX, GPIO18 for TX
  mbOvum.begin(&Serial1);
  mbOvum.slave(OVUM_SLAVE_ID);

  // Set the hostname before connecting to WiFi
  WiFi.setHostname(HOSTNAME);
  // Initialize WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  // Print IP address and hostname when connected
  Serial.println("Connected to WiFi!");
  Serial.print("Hostname: ");
  Serial.println(WiFi.getHostname());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up Modbus TCP server
  mbTCP.server();

  mbOvum.addHreg(OVUM_REGISTER, 20);
  mbOvum.onGetHreg(OVUM_REGISTER, cbOnGet50);

  mbTCP.addHreg(OVUM_REGISTER, 20);
  mbTCP.addHreg(OVUM_REGISTER + 1, 0);   // register to override 50
    
  mbTCP.addHreg(768, 0); // battery voltage 
  mbTCP.addHreg(769, 0); // battery charge current
  mbTCP.addHreg(770, 0); // battery discharge current
  mbTCP.addHreg(771, 0); // battery SOC
  mbTCP.onSetHreg(768, cbOnSetBattery, 3); // callback

  for (int i = START_REG; i < START_REG + REG_COUNT; ++i) {
    mbTCP.addHreg(i, 0);
  }
}

void loop() {
  // Get the current time
  unsigned long currentTime = millis();

  // Perform Modbus RTU read from smart meter every REFRESH_INTERVAL
  if (currentTime - lastReadTime >= REFRESH_INTERVAL) {
    lastReadTime = currentTime;  // Update last read time

    // Poll data from the smart meter as a Modbus RTU master
    if (!mbMeter.slave()) {  // If no Modbus RTU transaction is in progress
      mbMeter.readHreg(MASTER_SLAVE_ID, START_REG, meterRegisters, REG_COUNT, meterCallback);
    }
  }

  // check if battery information timeout occured
  if (currentTime - battUpdateTime >= BATT_REFRESH_INTERVAL) {
    mbTCP.Hreg(768, 0);
    mbTCP.Hreg(769, 0);
    mbTCP.Hreg(770, 0);
    LOG_ERROR("Battery information receiving timeout");
  }
  
  // Process Modbus RTU smart meter task
  mbMeter.task();

  // Check if we have new data from smart meter
  static bool dataProcessed = false; // To prevent re-processing data
  if (!mbMeter.slave() && !dataProcessed) { // If transaction is complete and data not yet processed
    dataProcessed = true;

    for (uint16_t i = 0; i < REG_COUNT; i++) {
      mbTCP.Hreg(START_REG + i, meterRegisters[i]);
    }
  }
  
  // Process Modbus RTU smart meter task
  mbMeter.task();

  // Process registers 20498 and 20499
  householdPower = combineRegistersToFloat(mbTCP.Hreg(20498), mbTCP.Hreg(20499));
  int16_t totalPowerOvum = (int16_t)(householdPower * -100.0f);

  Serial.printf("battery totalPower: %.2f, powerOvum=%d", householdPower, totalPowerOvum);
  
  // Write to mbOvum slave register 50
  mbTCP.Hreg(OVUM_REGISTER, totalPowerOvum);

  // Reset dataProcessed flag when new transaction starts
  if (mbMeter.slave()) {
    dataProcessed = false;
  }

  // Process mbOvumMaster task
  mbOvum.task();

  // Continuously handle Modbus TCP server requests
  mbTCP.task();

  // Small delay to yield to other tasks
  delay(10);
}
