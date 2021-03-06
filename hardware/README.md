The actual ModExo system is divided in two application layers: the high level layer which consists of the central control processing and the low level layer, which consists of actuation nodes or joints, composed by sensors, motor and driver. The number of nodes may vary depending on the application (e.g., for elbow rehabilitation, only one actuation node is necessary; for elbow and wrist, two actuation nodes will be needed). This architecture aims at obtaining a flexible exoskeleton, which can increase or decrease the number of working joints, a property desired for modular systems.

## High Level Layer
The High Level Layer (HLL) consists of a central controller, which operates in real time, receives data from the nodes' sensors, processes the data to obtain relevant state parameters and sends the desired joint angular position to the driver. The central controller is composed by the Arduino UNO and the SeeedStudio CAN-BUS Shield.

A desired property of the HLL is that, regardless of the number of nodes, it should not change. A consequence of that is that the software must be modular, but that is a future development to me made.

### Arduino UNO
[![Arduino UNO](https://github.com/biopmr/biopmr.github.io/blob/master/images/system_Arduino.jpg)](https://store.arduino.cc/usa/arduino-uno-rev3)

Arduino UNO is a microcontroller board based on the ATmega328P. Was chosen for being a fast prototyping tool and well succeeded implementation in many robotic applications. There is plenty documentation shared on the internet which provides many different solutions for problems that eventually come up during the implementation of the system. It is smaller and cheaper and also has libraries that communicate directly with Matlab, which is the simulation tool actually used in the Biomechatronics Laboratory. In this project, acts as the high level controller, implementing the Impedance Control.

### SeeedStudio CAN-BUS Shield
[![EPOS2 70/10](https://github.com/biopmr/biopmr.github.io/blob/master/images/system_seeedcan.jpg)](https://www.seeedstudio.com/CAN-BUS-Shield-V1.2-p-2256.html)

A CAN-BUS interface is also needed to communicate the central controller to the rest of the system. Most options available adopts the MCP2515 CAN BUS controller and the MCP2551 CAN transceiver. For this project, we chose the Seeedstudio CAN-BUS Shield V1.2 which has SPI interface up to 10 MHz and CAN communication up to 1 Mb/s. The reason for this choice was the reliability of the seller, the documentation available, and because the board is a shield that plugs directly on the top of the Arduino, offering a robust connection between them. 

## Low Level Layer

The Low Level Layer (LLL) consists of the actuation nodules of the system. This nodules communicate with the master device - the Arduino - and are composed by the EPOS2 70/10, the Maxon Motor EC90 flat and the sensors.

A property of the LLL is that the number of nodes will increase as the number of degrees of freedom of the system increase. A future hardware development to be made is that the attachment of new joints should be easy, plug'n play, something similar to what is seen with [LittleBits](https://www.youtube.com/watch?v=MMYqtCUN5X4).

### EPOS2 70/10
[![EPOS2 70/10](https://github.com/biopmr/biopmr.github.io/blob/master/images/system_epos2.jpg)](http://www.maxonmotor.com/maxon/view/product/control/Positionierung/375711)

EPOS2 is a digital positioning controller manufactured by Maxon which offers an integrated solution of driver and motor. Chosen for it's flexibility: has a wide variety of operating modes, receives external commands via analog and digital inputs and communicates via USB, RS232 and CANOpen. In this project, works as a low the level controller, receiving position commands from the Arduino and transmitting them to the motor.

### Maxon Motor EC90 Flat
[![Maxon Motor EC90 Flat](https://github.com/biopmr/biopmr.github.io/blob/master/images/system_motor.jpg)](http://www.maxonmotor.com/maxon/view/product/motor/ecmotor/ecflat/ecflat90/244879?etcc_cu=onsite&etcc_med=Header%20Suche&etcc_cmp=mit%20Ergebnis&etcc_ctv=Layer&query=ec90%20flat)

The Maxon Motor EC90 Flat comes with the MILE encoder and was chosen for it's high torque-velocity curve. It also offers power with a slim design, making it easy to fit in an exoskeleton. 
	
	Part Number: 323772
    Brushless Motor
    90 Watt
    Nominal Speed: 2590 rpm
    Nominal Torque: 444 mNm
    Nominal Current: 6.06 A
    Weight: 600 g

Sensors: 
   Human-Robotic Joint Torque:   Strain Gauges in Full Wheatstone Bridge
   Motor Position (Integrated EPOS Solution): Combination of Hall Sensor and Encoder Mile
   Position Control: Encoder USDigital MA3


## Other

### CAN Bus Analyzer

The communication between the Arduino and the EPOS is made by CAN BUS protocol. It has shown good performance so far, I have learned a lot with this post here: http://forum.arduino.cc/index.php?topic=225789.0