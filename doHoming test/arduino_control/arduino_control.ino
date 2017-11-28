//**************************************************************************
//------- Laboratorio de Biomecatronica @ Universidade de Sao Paulo --------
//
// Version: 1.0
// Date: 15.06.2017 [mm.dd.yyyy]
// Description
//  CAN Communication between Arduino and EPOS2 for Exoskeleton Control
// Input files:
// (None)
//
// Final Output files:
//   (None)
//
// References
//  http://wiki.seeed.cc/CAN-BUS_Shield_V1.2/
//  http://www.instructables.com/id/Arduino-Timer-Interrupts/
//  http://forum.arduino.cc/index.php?topic=225789.0
//  
//**************************************************************************
// Steps
//**************************************************************************

#include "ModExo.h"

void setup() 
{
  setupModExo();
}

void loop()
{
  loopModExo();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
