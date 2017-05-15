// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
#include "mcp_can.h"
#include <mcp_can_dfs.h>


// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
uint32_t encoder_data = 0;
uint32_t loadcell_data = 0;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
}


void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    unsigned int canId = CAN.getCanId();

    // encoder information is read from buf[4] and buf[5] and converted to decimal
    encoder_data = buf[4];
    encoder_data <<= 8;
    encoder_data = encoder_data | buf[5];

    // // load cell information is read from buf[1], buf[2] and buf[3] and converted to decimal
    // loadcell_data = buf[1];
    // loadcell_data <<= 8;
    // loadcell_data = loadcell_data | buf[2];
    // loadcell_data <<= 8;
    // loadcell_data = loadcell_data | buf[3];

    // if (buf[1] >= 128) {
    //        loadcell_data = loadcell_data - 16777216;
    // }
    
       Serial.print("Encoder: ");
       Serial.println(encoder_data, DEC);

    // Serial.print("Loadcell: ");
    // Serial.println(loadcell_data/1000, DEC);

    // delay(400);
  }

}



/*********************************************************************************************************
  END FILE
*********************************************************************************************************/



