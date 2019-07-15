
#include "ETT_ModbusRTU.h"
//=================================================================================================
// RS485 Module Connect ESP32 DoitKitV1
// DI Connect Pin GPIO 17
// DE Connect Pin GPIO 12
// RE Connect Pin GPIO 12
// R0 Connect Pin GPIO 16
// A to port A (Orange) on ModbusRTU H/T Sensor
// B to port B (Green) on ModbusRTU H/T Sensor
// V to GPIO 15
// Ground To Ground Arduino 

//=================================================================================================
// Modbus RTU H/T
// port A(Orange) to A on Rs485 Module
// port B(Green) to B on Rs485 Module
// port Vin to +12V (Yellow)
// port Ground (Black) to Ground Arduino




//=================================================================================================
#define SerialDebug Serial                                                                        // USB Serial
//=================================================================================================
const int RS485_DIRECTION_PIN =     12;                                                            // RS485 TXD Enable,Disable
const int RS485_RXD_SELECT    =     LOW;
const int RS485_TXD_SELECT    =     HIGH;

//=================================================================================================
const int POWER_PIN        =     15;
const int LED_LINK_PIN        =     2;
const int LED_OFF             =     LOW;
const int LED_ON              =     HIGH;
//=================================================================================================
// data array for modbus network sharing
//=================================================================================================
uint16_t au16dataSlave1[2];     //!< data array for modbus network sharing

uint8_t u8state;                //!< machine state
uint8_t u8query;                //!< pointer to message query
//=================================================================================================

/**
    Modbus object declaration
    u8id : node id = 0 for master, = 1..247 for slave
    u8serno : serial port (use 0 for Serial)
    u8txenpin : 0 for RS-232 and USB-FTDI
                 or any pin number > 1 for RS-485
*/
Modbus master(0,                        // node id = 0(master)
              Serial2,              // Serial2
              RS485_DIRECTION_PIN);     // RS485 Modbus

/**
   This is an structe which contains a query to an slave device
*/
modbus_t telegram[2];                   // 2-Modbus Device
unsigned long u32wait;
//=================================================================================================
unsigned long lastScanBusTime = 0;
//=================================================================================================

void setup()
{

  //===============================================================================================
  //===============================================================================================
  SerialDebug.begin(115200);
  SerialDebug.println();
  while (!SerialDebug);                                                                           // Wait MEGA32U4 USB Serial Complete
  //===============================================================================================


  //===============================================================================================
  // Initial RS485
  //===============================================================================================
  Serial2.begin(9600, SERIAL_8N1);
  //===============================================================================================
  pinMode(RS485_DIRECTION_PIN, OUTPUT);
  digitalWrite(RS485_DIRECTION_PIN, RS485_RXD_SELECT);
  
  pinMode(LED_LINK_PIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  

  //===============================================================================================

  //===============================================================================================
  // telegram 0: read registers device1
  //===============================================================================================
  telegram[0].u8id = 1;                       // slave address
  telegram[0].u8fct = 3;                      // function code (this one is registers read)
  telegram[0].u16RegAdd = 0;                  // start address in slave
  telegram[0].u16CoilsNo = 2;                 // number of elements (coils or registers) to read
  telegram[0].au16reg = au16dataSlave1;       // pointer to a memory array in the Arduino
  //===============================================================================================
  master.begin(Serial2);                  // Mosbus Interface
  Serial2.flush();

  master.setTimeOut(2000);                    // if there is no answer in 2000 ms, roll over
  u32wait = millis() + 2000;
  u8state = u8query = 0;
  //===============================================================================================
  lastScanBusTime = millis();
  //===============================================================================================
}

void loop()
{
  switch ( u8state )
  {
    case 0:
      if (millis() > u32wait) u8state++;        // wait state
      break;

    case 1:
      master.query(telegram[u8query]);          // send query (only once)
      u8state++;
      u8query++;
      if (u8query > 2) u8query = 0;
      break;

    case 2:
      if (master.poll())                       // check incoming messages
      {
        switch (master.getAnswerID())
        {
          case 1:
            //========================================================================
            Serial.print("Modbus Sensor(1) : Temperature = ");
            Serial.print(au16dataSlave1[0] / 10.0, 1);
            Serial.print(" C, Humidity = ");
            Serial.print(au16dataSlave1[1] / 10.0, 1);
            Serial.println(" %RH");
            //========================================================================
            break;


        }
      }

      if (master.getState() == COM_IDLE)
      {
        u8state = 0;
        u32wait = millis() + 1000;
      }

      break;
  }
}
