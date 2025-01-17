#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <SparkFun_AS7331.h>

bool rs_flag1 = false;
unsigned long rs_flag_start_time = 0;

TaskHandle_t serialTask;

#define LORA_TRANSMISSION_TIMEOUT_MS 600000 //10 mins (10*60*1000)

bool debug_mode = false;

uint8_t quantized_humidity;
uint8_t quantized_temperature;
uint8_t quantized_UVa;
uint8_t quantized_UVb;
uint8_t quantized_UVc;
uint32_t chipId = 0;
uint8_t ESP_ID = 0;

typedef struct UHF_telemtry_packet_t
{
    uint8_t Header;
    uint8_t Call_sign[6];
    
    uint8_t ESP_CHIP;
    uint8_t Lora_frequency[4];
    uint8_t Sample;
    uint8_t temperature1;
    uint8_t temperature2;
    uint8_t humidity;
    uint8_t UV_AT;
    uint8_t UV_BT;
    uint8_t UV_CT;
    uint8_t Health[4];
    uint8_t Footer;    
} UHF_telemtry_packet;

typedef struct UHF_telemtry_packet_buffer_t
{
  UHF_telemtry_packet tlm_packet;
  UHF_telemtry_packet_buffer_t *next;
} UHF_telemtry_packet_buffer;

UHF_telemtry_packet uhf_telemetryPacket;
UHF_telemtry_packet_buffer *uhf_tlm_packet_buffer_head;

typedef struct payload_telemetry_t
{
	//in degree celcius (-127 to +127)
	int8_t temp1;
	int8_t temp2;
	uint8_t mcu_temp;
	uint8_t voltage;
	//(-127 to +127)
	uint8_t humidity;
	uint8_t padding[3];

	float uva;
	float uvb;
	float uvc;
} payload_telemetry;

payload_telemetry poem_telemetry_data;

typedef struct payload_health_t
{
  uint8_t byte;
  uint8_t spare[3];
} payload_health;

payload_health  poem_health_data;

HardwareSerial SerialPort(2); // Use UART2 for additional communication

// SD card and LoRa pin definitions
//const int CS = 4;  // CS pin for SD card
#define SS 5        // LoRa CS pin
#define RST 14      // LoRa reset pin
#define DI0 26      // LoRa interrupt pin

// HS400 Sensor defines
#define HS400_ADDR         0x54
#define READ_RH_NOHOLD     0xf5
#define SOFT_RESET         0xfe

// AS7331 UV Sensor
SfeAS7331ArdI2C myUVSensor;

// Variables for sensors
uint8_t AS_health,HS_health,tx_health,lora_health;
float temperature, humidity;

// Function to quantize the sensor data
uint8_t quantize(float value, float min, float max, int bits) 
{
    if (min == max) 
    return 0; // Avoid division by zero
    int levels = pow(2, bits); // Number of quantization levels
    float step = (max - min) / levels; // Calculate step size
    return (uint8_t)((value - min) / step); // Quantize value
}

void initLoRa() 
{
    Serial.println("Initializing LoRa...");
    LoRa.setPins(SS, RST, DI0);
    if (!LoRa.begin(436.48E6)) 
    {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    Serial.println("LoRa initialized.");
    lora_health = 0xFD;
}

void initHS400() 
{
    Wire.begin(21, 22);
    Wire.beginTransmission(HS400_ADDR);
    Wire.write(SOFT_RESET);
    Wire.endTransmission();
    delay(500);
}

void readUVData(float &uva, float &uvb, float &uvc) 
{
  // Read UV data from AS7331 sensor
  // Send a start measurement command.
  if(kSTkErrOk != myUVSensor.setStartState(true))
    Serial.println("Error starting reading!");
      
  // Wait for a bit longer than the conversion time.
  delay(2+myUVSensor.getConversionTimeMillis());

  // Read UV values.
  if(kSTkErrOk != myUVSensor.readAllUV())
  {
    Serial.println("Error reading UV.");
    AS_health = 0x20;
  }
   
  uva = myUVSensor.getUVA();
  poem_telemetry_data.uva = uva;
  uvb = myUVSensor.getUVB();
  poem_telemetry_data.uva = uva;
  uvc = myUVSensor.getUVC();
  poem_telemetry_data.uva = uva;

  uhf_telemetryPacket.UV_AT = quantized_UVa = quantize(uva, 0.0f, 1000.0f, 8);
  uhf_telemetryPacket.UV_BT = quantized_UVb = quantize(uvb, 0.0f, 1000.0f, 8);
  uhf_telemetryPacket.UV_CT = quantized_UVc = quantize(uvc, 0.0f, 1000.0f, 8);

  AS_health = 0x60;
}

void readSensorData() 
{
  for (int i = 0; i < 17; i = i + 8) 
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  ESP_ID = (chipId == 11241924) ? 0xAB : 0xBA;  // to read ESP chip ID tx-sqm 13396600

    // Request humidity and temperature
  Wire.beginTransmission(HS400_ADDR);
  Wire.write(READ_RH_NOHOLD);
  
  if (Wire.endTransmission() == 0) 
  {
    //NOTE: Do not remove this delay(10). its important for the sensor read to work predictably
    delay(50);
    Wire.requestFrom(HS400_ADDR, 4);
    if (Wire.available() == 4) 
    {
      uint8_t hum_msb = Wire.read() & 0x3F;
      uint8_t hum_lsb = Wire.read();
      uint8_t temp_msb = Wire.read() & 0x3F;
      uint8_t temp_lsb = Wire.read();

      humidity = (((hum_msb << 8) | hum_lsb) / 16383.0) * 100.0;
      temperature = ((((temp_msb << 8) | temp_lsb) / 16383.0) * 165.0) - 40.0;

      poem_telemetry_data.humidity = humidity; 
      poem_telemetry_data.temp1 = poem_telemetry_data.temp2 =  temperature;

      uhf_telemetryPacket.humidity = quantized_humidity = quantize(humidity, 0, 100, 8); 
      uhf_telemetryPacket.temperature1 = uhf_telemetryPacket.temperature2 = quantized_temperature = quantize(temperature, -40.0f, 125.0f, 8);

      //Serial.println(quantized_humidity);

      HS_health = 0x16;

      poem_telemetry_data.mcu_temp=23;
      poem_telemetry_data.voltage=28;
      poem_telemetry_data.padding[0]=0;
      poem_telemetry_data.padding[1]=0;
      poem_telemetry_data.padding[2]=0;
    }    
  }
	else 
	{
    if(debug_mode) Serial.println("Failed to read from HS400 sensor!");
    HS_health = 0x12;
  }
}

void setup() 
{
  rs_flag_start_time = 0;
  tx_health = 0xBB;
  uhf_tlm_packet_buffer_head = NULL;
  memset((uint8_t*)&poem_health_data, 0, sizeof(poem_health_data));
  memset((uint8_t*)&poem_telemetry_data, 0, sizeof(poem_telemetry_data));
 
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  while (!Serial) { ; }  // Wait for serial port to connect

// Configure for FSK mode
  //  writeRegister(0x01, 0x0C); // FSK mode
  writeRegister(0x02, 0x34); //2.4 kbps MSB
  writeRegister(0x03, 0x15); // 2.4 kbps LSB

  LoRa.setTxPower(0);

  initLoRa();
  initHS400();
 
  Wire.begin();

 //// writeRegister(0x1D, 0x31);  //20 KHz
  
  // Initialize sensor and run default setup.
  if(!myUVSensor.begin()) 
  {
    delay(10);
    //try again
    if(!myUVSensor.begin())
    {
      if(debug_mode) Serial.println("UV Sensor failed to begin. Please check your wiring!");
    }
  }
  // Set measurement mode and change device operating mode to measure.
  if(!myUVSensor.prepareMeasurement(MEAS_MODE_CMD)) 
  {
    delay(10);
    //try again
    if(!myUVSensor.prepareMeasurement(MEAS_MODE_CMD))
    {
      if(debug_mode) Serial.println("UV Sensor did not get set properly.");
    }
  }

   init_uhf_packet();

   xTaskCreatePinnedToCore(
      handleSerialComms, /* Function to implement the task */
      "SerialTask",      /* Name of the task */
      10000,             /* Stack size in words */
      NULL,              /* Task input parameter */
      0,                 /* Priority of the task */
      &serialTask,       /* Task handle. */
      0);                /* Core where the task should run */
 
  delay(500); 
}

uint8_t readRegister(uint8_t addr) 
{
  uint8_t value;
  
  digitalWrite(SS, LOW);        // Select the LoRa chip
  SPI.transfer(addr & 0x7F);    // Send the register address (MSB bit 7 should be 0 for read)
  value = SPI.transfer(0x00);   // Read the value from the register
  digitalWrite(SS, HIGH);       // Deselect the LoRa chip
  
  return value;
}

void writeRegister(uint8_t addr, uint8_t value) 
{
  digitalWrite(SS, LOW);        // Select the LoRa chip
  SPI.transfer(addr | 0x80);    // Send the register address with write flag (MSB bit 7 should be 1 for write)
  SPI.transfer(value);          // Write the value to the register
  digitalWrite(SS, HIGH);       // Deselect the LoRa chip
}


void init_uhf_packet()
{
  float uva, uvb, uvc;
  readSensorData();  // Read temperature and humidity
  readUVData(uva, uvb, uvc);  // Read UV data need to on
  
  uhf_telemetryPacket.Header = 0xDE;

  uhf_telemetryPacket.Call_sign[0] = 'V';
  uhf_telemetryPacket.Call_sign[1] = 'U';
  uhf_telemetryPacket.Call_sign[2] = '2';
  uhf_telemetryPacket.Call_sign[3] = 'E';
  uhf_telemetryPacket.Call_sign[4] = 'U';
  uhf_telemetryPacket.Call_sign[5] = 'S';

  uhf_telemetryPacket.Lora_frequency[0] = 0x43;
  uhf_telemetryPacket.Lora_frequency[1] = 0xDA;
  uhf_telemetryPacket.Lora_frequency[2] = 0x41;
  uhf_telemetryPacket.Lora_frequency[3] = 0x11;
  uhf_telemetryPacket.ESP_CHIP = ESP_ID;
  uhf_telemetryPacket.Health[0] = AS_health;
  uhf_telemetryPacket.Health[1] = HS_health;
  uhf_telemetryPacket.Health[2] = tx_health;
  uhf_telemetryPacket.Health[3] = lora_health;
  uhf_telemetryPacket.Sample = 0xEE;
  uhf_telemetryPacket.Footer = 0xAD;
}


void handleSerialComms(void *params)
{
  for(;;) {
    // Handle serial input
    if (SerialPort.available()) 
    {
      int Cmd = SerialPort.read();
      if(debug_mode) Serial.println(Cmd, HEX); // Print in hexadecimal format

      switch(Cmd)
      {
        case 0x22:
          SerialPort.write((uint8_t*)&poem_telemetry_data, sizeof(poem_telemetry_data));
        break;
        case 0x11:
          poem_health_data.byte =  AS_health | HS_health;
          
          SerialPort.write((uint8_t*)(&poem_health_data), sizeof(poem_health_data));
        break;
        case 0x33:
          if(debug_mode) Serial.println("33 command recieved");
          rs_flag1 = true;
          tx_health = 0x33;
          rs_flag_start_time = millis();
        break;
        case 0x44:
          writeRegister(0x01, 0x01); // lora stdby mode
          if(debug_mode) Serial.println("44 lora off");
          rs_flag1 = false;
      
        break;
        default:
        break;
      }
    }
  }

}

void loop() 
{
    // uint8_t invisible_flag;
    float uva, uvb, uvc;
    
    //  unsigned long lastPrintTime = 0;
    //  const unsigned long printInterval = 1000; // 1 second

    readSensorData();  // Read temperature and humidity
    readUVData(uva, uvb, uvc);  // Read UV data need to on
    
    // if (millis() - lastPrintTime >= printInterval)
    // {
    //     lastPrintTime = millis();
    //     print_uhf_packet();  // Print telemetry data every 1 second
    // }

    if( rs_flag1)
    {
      if((millis()-rs_flag_start_time) >= LORA_TRANSMISSION_TIMEOUT_MS)
      {
        rs_flag1 = false;
      }
      else
      {
        writeRegister(0x01, 0x0C);  //FSK_mode

        LoRa.beginPacket();
        LoRa.write((uint8_t*)&uhf_telemetryPacket, sizeof(uhf_telemetryPacket));
        LoRa.endPacket();
        tx_health = 0x44;
      }
    }
}

