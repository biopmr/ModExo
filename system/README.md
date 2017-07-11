## Driver: EPOS2 70/10
[![EPOS2 70/10](https://biopmr.github.io/images/modexo-logo.svg)](https://biopmr.github.io/images/logo/modexo.svg)

Motor: Maxon Motor EC90 flat
      Brushless
      90 Watt

Sensors: 
   Human-Robotic Joint Torque:   Strain Gauges in Full Wheatstone Bridge
   Motor Position (Integrated EPOS Solution): Combination of Hall Sensor and Encoder Mile
   Position Control: Encoder USDigital MA3

High Level Controller: Arduino UNO

The communication between the Arduino and the EPOS is made by CAN BUS protocol. It has shown good performance so far, I have learned a lot with this post here: http://forum.arduino.cc/index.php?topic=225789.0