The system is divided in two control layers: the high level layer which consists of the central control processing and the low level layer, which consists of actuation nodes or joints, composed by sensors, motor and driver. The number of nodes may vary depending on the application (e.g., for elbow rehabilitation, only one actuation node is necessary; for elbow and wrist, two actuation nodes will be needed). This architecture aims at obtaining a flexible exoskeleton, which can increase or decrease the number of working joints, a property desired for modular systems.

### Arduino UNO
[![Arduino UNO](https://github.com/biopmr/biopmr.github.io/blob/master/images/system_Arduino.jpg)](https://store.arduino.cc/usa/arduino-uno-rev3)

Arduino UNO is a microcontroller board based on the ATmega328P. Was chosen for being a fast prototyping tool and well succeeded implementation in many robotic applications. There is plenty documentation shared on the internet which provides many different solutions for problems that eventually come up during the implementation of the system. It is smaller and cheaper and also has libraries that communicate directly with Matlab, which is the simulation tool actually used in the Biomechatronics Laboratory. In this project, acts as the high level controller, implementing the Impedance Control.

### EPOS2 70/10
[![EPOS2 70/10](https://github.com/biopmr/biopmr.github.io/blob/master/images/system_epos2.jpg)](http://www.maxonmotor.com/maxon/view/product/control/Positionierung/375711)

EPOS2 is a digital positioning controller manufactered by Maxon which offers an integrated solution of Driver, Motor and Sensors. Chosen for it's flexibility: has a wide variety of operating modes, receives external commands via analog and digital inputs and comunicates via USB, RS232 and CANOpen. In this project, works as a Low Level Controller, receiving commands from the Arduino.

Motor: Maxon Motor EC90 flat
      Brushless
      90 Watt

Sensors: 
   Human-Robotic Joint Torque:   Strain Gauges in Full Wheatstone Bridge
   Motor Position (Integrated EPOS Solution): Combination of Hall Sensor and Encoder Mile
   Position Control: Encoder USDigital MA3



The communication between the Arduino and the EPOS is made by CAN BUS protocol. It has shown good performance so far, I have learned a lot with this post here: http://forum.arduino.cc/index.php?topic=225789.0