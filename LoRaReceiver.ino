#include <LoRa.h>
#include <HardwareSerial.h>
HardwareSerial SerialPort(2);

#define SS 5
#define RST 14
#define DI0 26

void setup() {
  Serial.begin(115200);
  while (!Serial);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("LoRa Receiver");

  LoRa.setPins(SS, RST, DI0);

  if (!LoRa.begin(436.5E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() 
{
  int packetSize = LoRa.parsePacket();  // Check if a packet has been received
  if (packetSize) 
  {
    Serial.print("Received packet: ");
    
    while (LoRa.available()) 
    {
      byte byteRead = LoRa.read();  // Read each byte from LoRa
      Serial.print(byteRead);
      Serial.println(byteRead, HEX);  // Print byte in hexadecimal format
      Serial.print(" ");            // Add space between hex values for clarity

      // Send the received byte to SerialPort
      SerialPort.write(byteRead);  // This will write the byte to SerialPort
    }

    // Print the RSSI value of the received packet
    Serial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
