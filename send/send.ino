// demo: CAN-BUS Shield, send data
#include <mcp_can.h>
#include <SPI.h>

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

int sensorValue = 0;        // value read from the pot
unsigned char setpoint_position[8] = {0, 1, 2, 3, 4, 5, 6, 7};

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

uint32_t angulo = 0;

void setup()
{
    Serial.begin(115200);

  // TIMER SETUP- the timer interrupt allows precise timed measurements of the reed switch
  //for mor info about configuration of arduino timers see http://arduino.cc/playground/Code/Timer1

  cli();//stop interrupts

  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B\
  TCNT1  = 0;//initialize counter value to 0
  // set timer count for 1khz increments
  OCR1A = 19990/2;// = (16*10^6) / (1000*8) - 1
  //had to use 16 bit timer1 for this bc 1999>255, but could switch to timers 0 or 2 with larger prescaler
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  Serial.println("Timer Setup OK!");
  delay(100);

  //END TIMER SETUP

    while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        delay(100);
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

    delay(100);
}

unsigned char stmp[8] = {0, 1, 2, 3, 4, 5, 6, 7};

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void PotRead(void) {

  // read the analog in value:
  sensorValue = analogRead(A0);
  delay(10);

  // map it to the range of the analog out:
  angulo = map(sensorValue, 0, 960, 0, 3600);
//angulo = angulo + 1;

  //setpoint_position_position[6]+=10;
  setpoint_position[4] = angulo & 0xFF;
  setpoint_position[5] = (angulo >> 8) & 0xFF;
  setpoint_position[6] = (angulo >> 16) & 0xFF;
  setpoint_position[7] = (angulo >> 24) & 0xFF;
}

ISR(TIMER1_COMPA_vect) { //timer1 interrupt 1kHz toggles pin 13 (LED)
      
      PotRead();
      Serial.println(angulo);
}

void loop()
{
    Serial.println("Entrou no Loop");
    sei();//allow interrupts
    // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    CAN.sendMsgBuf(0x00, 0, 8, setpoint_position);
    delay(100);                       // send data per 100ms
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
