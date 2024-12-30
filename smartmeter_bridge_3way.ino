#include "secrets.h"

#include <ModbusRTU.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>

#define MASTER_SLAVE_ID 2         // Smart meter ID for Modbus RTU
#define START_REG 20482           // Starting register to read from the smart meter
#define REG_COUNT 48              // Number of registers to read from the smart meter
#define TARGET_REGISTER_INDEX 16  // Index of register 20498 in the read data
#define REFRESH_INTERVAL 500      // refresh interval in ms of smartmeter

#define OVUM_SLAVE_ID 18           // Slave ID of mbOvum device
#define OVUM_REGISTER 50          // Register to write in mbOvum device

#define LOG_LEVEL 0

ModbusRTU mbMaster;               // Modbus RTU master instance on Serial2 (smart meter)
ModbusRTU mbOvum;           // Modbus RTU master instance on Serial1 (mbOvum)
ModbusIP mbTCP;                   // Modbus TCP server instance

uint16_t slaveRegisters[REG_COUNT]; // Buffer to store values read from the smart meter
unsigned long lastReadTime = 0;     // Variable to store the last RTU read time

struct RegisterRange {
  uint16_t start;   // Starting register number
  uint16_t length;  // Length of the range
};
const RegisterRange predefinedRanges[] = {
    {256, 38},
    {512, 26},
    {768, 10},
    {1024, 24},
    {1283, 10},
    {4096, 10},
    {4352, 10},
    {4864, 20},
    {4899, 10},
    {5125, 10}
};
const int rangeCount = sizeof(predefinedRanges) / sizeof(predefinedRanges[0]); // Get the size of the array

bool masterCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event != Modbus::EX_SUCCESS) {
    Serial.print("Master request error: 0x");
    Serial.println(event, HEX);
  }
  return true;
}

bool ovumCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event != Modbus::EX_SUCCESS) {
    Serial.print("Ovum Master write error: 0x");
    Serial.println(event, HEX);
  } else {
    Serial.println("Ovum Master write completed successfully.");
  }
  return true;
}

// Callback function to log Modbus TCP requests
bool tcpCallback(TRegister* reg, uint16_t val) {
  Serial.print(" cbReg: ");
  Serial.print(reg->address.address);
  Serial.print(", cbValue: ");
  Serial.print(val);
  Serial.print(" loopRegVal: ");
  return true;
}

// Callback function for Modbus TCP connection
bool cbConn(IPAddress ip) {
  Serial.print("Connected to IP: ");
  Serial.println(ip);
  return true;
}

int16_t processRegisters(uint16_t* slaveRegisters, int index) {
    // Get the two words from the specified index
    uint16_t word1 = slaveRegisters[index];
    uint16_t word2 = slaveRegisters[index + 1];

    // Combine words (assuming big-endian format)
    uint32_t combinedValue = ((uint32_t)word1 << 16) | word2;

    // Convert to float
    float floatValue;
    memcpy(&floatValue, &combinedValue, sizeof(float));

    // Multiply by -100.0f
    //floatValue *= 1000.0f;

    // Convert to int16_t
    //int16_t intValue = (int16_t)(floatValue);

    return floatValue;
}


void setup() {
  Serial.begin(115200);

  // Initialize Modbus RTU Master Serial (Smart Meter)
  Serial2.begin(9600, SERIAL_8E1, 17, 16); // GPIO17 for RX, GPIO16 for TX
  mbMaster.begin(&Serial2);
  mbMaster.master();

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
  mbTCP.onConnect(cbConn);
  mbTCP.server();

  for (int i = 0; i < REG_COUNT; i++) {
    mbTCP.addHreg(START_REG + i, 123);  // Initialize holding registers for TCP
  }

  mbTCP.addHreg(50, 20);
  mbTCP.addHreg(51, 0);   // synthetic register to influence hreg 50
  mbTCP.addHreg(52, 0);
  mbTCP.addHreg(768, 11); 
  mbTCP.addHreg(769, 11); 
  mbTCP.addHreg(770, 11); 
  mbTCP.addHreg(771, 11); 
  
  // Iterate through each range and print all individual registers
  for (int i = 0; i < rangeCount; ++i) {
    uint16_t start = predefinedRanges[i].start;
    uint16_t length = predefinedRanges[i].length;
    for (uint16_t reg = start; reg < start + length; ++reg) {
      mbTCP.addHreg(reg, 0); // add each register 
    }
  }
  
  
  // Register the TCP callback to log requests
  
  //mbTCP.onGetHreg(START_REG, tcpCallback, REG_COUNT);
  //mbTCP.onGetHreg(50, tcpCallback, 1);
  
  mbOvum.addHreg(OVUM_REGISTER, 100);
}

void loop() {
  // Get the current time
  unsigned long currentTime = millis();

  // Perform Modbus RTU read from smart meter every 1 second
  if (currentTime - lastReadTime >= REFRESH_INTERVAL) {
    lastReadTime = currentTime;  // Update last read time

    // Poll data from the smart meter as a Modbus RTU master
    if (!mbMaster.slave()) {  // If no Modbus RTU transaction is in progress
      mbMaster.readHreg(MASTER_SLAVE_ID, START_REG, slaveRegisters, REG_COUNT, masterCallback);
    }
  }

  // Process Modbus RTU master task (smart meter)
  mbMaster.task();

  // Check if we have new data from smart meter
  static bool dataProcessed = false; // To prevent re-processing data
  if (!mbMaster.slave() && !dataProcessed) { // If transaction is complete and data not yet processed
    dataProcessed = true;

    for (uint16_t i = 0; i < REG_COUNT; i++) {
      // Perform some calculation for each register (e.g., increase value by i * 2)
      uint16_t newValue = mbTCP.Hreg(START_REG + i) + i * 2;
      mbTCP.Hreg(START_REG + i, slaveRegisters[i]);
    }
  }

  // Process registers 20498 and 20499
  uint16_t word1 = slaveRegisters[TARGET_REGISTER_INDEX];     // Index 16 corresponds to register 20498
  uint16_t word2 = slaveRegisters[TARGET_REGISTER_INDEX + 1]; // Index 17 corresponds to register 20499

  // Combine words (assuming big-endian format)
  uint32_t combinedValue = ((uint32_t)word1 << 16) | word2;

  // Convert to float
  float floatValue;
  memcpy(&floatValue, &combinedValue, sizeof(float));


  Serial.print("floatVal: ");
  Serial.print(floatValue);

  // Multiply by 100
  floatValue *= -100.0f;

  // Convert to int16_t
  int16_t intValue = (int16_t)(floatValue);

  // Write to mbOvum slave register 50
  // mbOvum.Hreg(OVUM_REGISTER, (uint16_t)intValue);
  mbTCP.Hreg(OVUM_REGISTER, (int16_t)intValue + mbTCP.Hreg(51));
  mbTCP.Hreg(OVUM_REGISTER + 2, (int16_t)intValue);

  Serial.print("  hreg50: ");
  Serial.print(intValue);
  Serial.print("  batValue: ");
  Serial.print(mbTCP.Hreg(768));
  Serial.print(" finalVal: ");
  Serial.println(intValue + mbTCP.Hreg(51));

  // Reset dataProcessed flag when new transaction starts
  if (mbMaster.slave()) {
    dataProcessed = false;
  }

  // Process mbOvumMaster task
  mbOvum.task();

  // Continuously handle Modbus TCP server requests
  mbTCP.task();

  // Small delay to yield to other tasks
  delay(10);
}
