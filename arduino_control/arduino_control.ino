/* CAN COMMUNICATION

   References
    http://wiki.seeed.cc/CAN-BUS_Shield_V1.2/
    https://www.arduino.cc/en/Reference/SPI
    http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/

   Hardware
    Pins for Genuino 101
      MOSI: 11 or ICSP-4
      MISO: 12 or ICSP-1
      SCK: 13 or ICSP-3
      LEVEL: 3,3V

*/

/*
  Analog setpoint_position_position, analog output, serial output

  Reads an analog setpoint_position_position pin, maps the result to a range from 0 to 255
  and uses the result to set the pulsewidth modulation (PWM) of an output pin.
  Also prints the results to the serial monitor.

  The circuit:
   potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
   LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/

// demo: CAN-BUS Shield, send data
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

// Define States
#define Startup             1
#define Pre_Operational     2
#define Operational         3

//Statemachine State variable and initial value
byte State = Startup;

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10

const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

// EPOS2 Communication

unsigned char set_pre_operational[2] = {0x80, 0};
unsigned char set_operational[2] = {0x1, 0};
unsigned char pdo_actual_position_1[8] = {0x22, 0x02, 0x18, 0x01, 0x81, 0x03, 0, 0};
unsigned char pdo_actual_position_2[8] = {0x22, 0x02, 0x18, 0x02, 0x01, 0, 0, 0};

unsigned char pdo_actual_velocity_1[8] = {0x22, 0x02, 0x18, 0x01, 0x81, 0x03, 0, 0};
unsigned char pdo_actual_velocity_2[8] = {0x22, 0x02, 0x18, 0x02, 0x01, 0, 0, 0};

unsigned char pdo_actual_current_1[8] = {0x22, 0x00, 0x1A, 0x00, 0x00, 0, 0, 0};
unsigned char pdo_actual_current_2[8] = {0x22, 0x00, 0x1A, 0x01, 0x10, 0, 0x78, 0x60};
unsigned char pdo_actual_current_3[8] = {0x22, 0x00, 0x1A, 0x02, 0x10, 0, 0x27, 0x20};
unsigned char pdo_actual_current_4[8] = {0x22, 0x00, 0x18, 0x02, 0x01, 0, 0, 0};
unsigned char pdo_actual_current_5[8] = {0x22, 0x00, 0x1A, 0x00, 0x01, 0, 0, 0};

unsigned char enable_epos[8] = {0x2B, 0x40, 0x60, 0, 0x0F, 0, 0, 0};
unsigned char disable_epos[8] = {0x2B, 0x40, 0x60, 0, 0x06, 0, 0, 0};
unsigned char set_max_following_error[8] = {0x22, 0x65, 0x60, 0, 0xD0, 7, 0, 0};
unsigned char set_max_acceleration[8] = {0x22, 0xC5, 0x60, 0, 0x88, 0x13, 0, 0};
unsigned char set_max_profile_velocity[8] = {0x22, 0x7F, 0x60, 0, 0xD0, 7, 0, 0};
unsigned char setpoint_position[8] = {0x22, 0x62, 0x20, 0, 0, 0, 0, 0};
unsigned char get_actual_position[8] = {0x40, 0x64, 0x60, 0, 0, 0, 0, 0};
unsigned char get_actual_velocity[8] = {0x40, 0x6C, 0x60, 0, 0, 0, 0, 0};
unsigned char get_actual_current[8] = {0x40, 0x78, 0x60, 0, 0, 0, 0, 0};

// setpoint_position_position
float outputValue = 0;        // value output to the PWM (analog out)

// Sensor
const int analogInPin = A0;  // Analog setpoint_position_position pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  Serial.begin(115200);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  delay(50);

  while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 1000k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
    Serial.println("CAN BUS Shield init ok!");

      // TIMER SETUP- the timer interrupt allows precise timed measurements of the reed switch
  //for mor info about configuration of arduino timers see http://arduino.cc/playground/Code/Timer1

  cli();//stop interrupts

  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B\
  TCNT1  = 0;//initialize counter value to 0
  // set timer count for 1khz increments
  OCR1A = 1999;// = (16*10^6) / (1000*8) - 1
  //had to use 16 bit timer1 for this bc 1999>255, but could switch to timers 0 or 2 with larger prescaler
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

  Serial.println("Timer Setup OK!");

  //END TIMER SETUP
}

//***************
// CAN STARTUP
//***************
void doStartup(void) {
  CAN.sendMsgBuf(0x601, 0, 8, disable_epos);
  Serial.println("EPOS diabled");
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, enable_epos);
  Serial.println("EPOS enabled");
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, set_max_following_error);
  Serial.println("Max Following Error set as 2000qc");
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, set_max_acceleration);
  Serial.println("Max Following Error set as 5000rpm/s");
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, set_max_profile_velocity);
  Serial.println("Max Profile Velocity set as 2000rpm");
  delay(10);
  
  delay(10);
  State = Pre_Operational;
}

//***************
// PDO Config
//***************
void PDOConfig(void) {

  CAN.sendMsgBuf(0x00, 0, 2, set_pre_operational);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_position_1);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_position_2);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_velocity_1);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_velocity_2);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_current_1);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_current_2);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_current_3);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_current_4);
  delay(10);
  CAN.sendMsgBuf(0x601, 0, 8, pdo_actual_current_5);
  delay(10);
  CAN.sendMsgBuf(0x00, 0, 2, set_operational);
  delay(10);

  Serial.println("TPDO Configured!");
  
  delay(10);
  State = Operational;
}

//***************
// Position Setpoint
//***************
void PositionSetpoint(void) {

  unsigned char len = 0;
  unsigned char buf[8];

    Serial.println("Enviei a mensagem");
    // clear the string:
    CAN.sendMsgBuf(0x601, 0, 8, setpoint_position);
    delay(10);


    delay(1000);
}

//***************
// DATA READ
//***************
void DataRead(void) {
  unsigned char len = 0;
  unsigned char buf[8];

    CAN.sendMsgBuf(0x80, 0, 1, 0);
    inputString = "";
    stringComplete = false;

    if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
    {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

      unsigned int canId = CAN.getCanId();

      Serial.println("-----------------------------");
      Serial.print("Get data from ID1: ");
      Serial.println(canId, HEX);

      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i], HEX);
        Serial.print("\t");
      }
      Serial.println();
    }
    
    if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
    {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

      unsigned int canId = CAN.getCanId();

      Serial.println("-----------------------------");
      Serial.print("Get data from ID2: ");
      Serial.println(canId, HEX);

      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i], HEX);
        Serial.print("\t");
      }
      Serial.println();
    }

     if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
    {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

      unsigned int canId = CAN.getCanId();

      Serial.println("-----------------------------");
      Serial.print("Get data from ID3: ");
      Serial.println(canId, HEX);

      for (int i = 0; i < len; i++) // print the data
      {
        Serial.print(buf[i], HEX);
        Serial.print("\t");
      }
      Serial.println();
    }
    // Serial.println(angulo);
    delay(300);
}

//*****************************************
// POTENTIOMETER INPUT
//*****************************************
void PotRead(void) {

  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  delay(10);

  // map it to the range of the analog out:
  uint32_t angulo = map(sensorValue, 0, 960, 0, 3600);

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t");
  Serial.print("\t output = ");
  Serial.println(angulo);

  //setpoint_position_position[6]+=10;
  angulo *= 200000 / 3600;
  setpoint_position[4] = angulo & 0xFF;
  setpoint_position[5] = (angulo >> 8) & 0xFF;
  setpoint_position[6] = (angulo >> 16) & 0xFF;
  setpoint_position[7] = (angulo >> 24) & 0xFF;

}
void loop() {


  PotRead()

  switch (State) {
    case Startup:
      doStartup();
      break;
    case Pre_Operational:
      PDOConfig();
      break;  
    case Operational:
      PositionSetpoint();
      DataRead();
      break;
  }
}

