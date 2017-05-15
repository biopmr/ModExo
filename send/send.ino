// demo: CAN-BUS Shield, receive data with check mode
// send data coming to fast, such as less than 10ms, you can use this way
// loovee, 2014-6-13


#include <SPI.h>
#include "mcp_can.h"
#include <mcp_can_dfs.h>

// Define States
#define Startup             1
#define Pre_Operational     2
#define Operational         3

// Statemachine State variable and initial value
byte State = Startup;

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
// Set CS pin
MCP_CAN CAN(SPI_CS_PIN); 

// Sensor variables
uint32_t encoder_data = 0;
uint32_t loadcell_data = 0;

// ***************************
// EPOS2 CANOpen Communication
// ***************************

// Statemachine
unsigned char set_pre_operational[2] = {0x80, 0};
unsigned char set_operational[2] = {0x1, 0};
unsigned char enable_epos[8] = {0x2B, 0x40, 0x60, 0, 0x0F, 0, 0, 0};
unsigned char disable_epos[8] = {0x2B, 0x40, 0x60, 0, 0x06, 0, 0, 0};

// PDO Configuration
unsigned char pdo_actual_position_1[8] = {0x22, 0x02, 0x18, 0x01, 0x81, 0x03, 0, 0};
unsigned char pdo_actual_position_2[8] = {0x22, 0x02, 0x18, 0x02, 0x01, 0, 0, 0};

unsigned char pdo_actual_velocity_1[8] = {0x22, 0x02, 0x18, 0x01, 0x81, 0x03, 0, 0};
unsigned char pdo_actual_velocity_2[8] = {0x22, 0x02, 0x18, 0x02, 0x01, 0, 0, 0};

unsigned char pdo_actual_current_1[8] = {0x22, 0x00, 0x1A, 0x00, 0x00, 0, 0, 0};
unsigned char pdo_actual_current_2[8] = {0x22, 0x00, 0x1A, 0x01, 0x10, 0, 0x78, 0x60};
unsigned char pdo_actual_current_3[8] = {0x22, 0x00, 0x1A, 0x02, 0x10, 0, 0x27, 0x20};
unsigned char pdo_actual_current_4[8] = {0x22, 0x00, 0x18, 0x02, 0x01, 0, 0, 0};
unsigned char pdo_actual_current_5[8] = {0x22, 0x00, 0x1A, 0x00, 0x01, 0, 0, 0};

// Objects Writing
unsigned char set_max_following_error[8] = {0x22, 0x65, 0x60, 0, 0xD0, 7, 0, 0};
unsigned char set_max_acceleration[8] = {0x22, 0xC5, 0x60, 0, 0x88, 0x13, 0, 0};
unsigned char set_max_profile_velocity[8] = {0x22, 0x7F, 0x60, 0, 0xD0, 7, 0, 0};
unsigned char setpoint_position[8] = {0x22, 0x62, 0x20, 0, 0, 0, 0, 0};

// Objects Reading
unsigned char get_actual_position[8] = {0x40, 0x64, 0x60, 0, 0, 0, 0, 0};
unsigned char get_actual_velocity[8] = {0x40, 0x6C, 0x60, 0, 0, 0, 0, 0};
unsigned char get_actual_current[8] = {0x40, 0x78, 0x60, 0, 0, 0, 0, 0};

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
