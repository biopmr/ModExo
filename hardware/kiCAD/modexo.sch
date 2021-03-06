EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:modexo
EELAYER 25 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ -1900 4075
$Sheet
S 4325 4375 2425 1550
U 58E6F2F2
F0 "High_Level_Layer" 60
F1 "high_level_layer.sch" 60
F2 "CAN" I R 6750 5700 60 
$EndSheet
$Sheet
S 4275 6300 2425 1250
U 58E6FD99
F0 "Low_Level_Layer" 60
F1 "low_level_layer.sch" 60
F2 "24V" I L 4275 6525 60 
F3 "CAN" I R 6700 6575 60 
$EndSheet
$Comp
L AC #PWR?
U 1 1 58E70616
P 1625 6000
F 0 "#PWR?" H 1625 5900 50  0001 C CNN
F 1 "AC" H 1625 6250 50  0000 C CNN
F 2 "" H 1625 6000 50  0001 C CNN
F 3 "" H 1625 6000 50  0001 C CNN
	1    1625 6000
	1    0    0    -1  
$EndComp
$Comp
L POWER_SUPPLY PWR?
U 1 1 58E70808
P 2525 6500
F 0 "PWR?" H 3075 5700 60  0000 C CNN
F 1 "POWER_SUPPLY" H 2875 7300 60  0000 C CNN
F 2 "" H 2525 6500 60  0001 C CNN
F 3 "" H 2525 6500 60  0001 C CNN
	1    2525 6500
	1    0    0    -1  
$EndComp
$Comp
L GNDREF #PWR?
U 1 1 58E70862
P 1625 6100
F 0 "#PWR?" H 1625 5850 50  0001 C CNN
F 1 "GNDREF" H 1625 5950 50  0000 C CNN
F 2 "" H 1625 6100 50  0001 C CNN
F 3 "" H 1625 6100 50  0001 C CNN
	1    1625 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3425 6400 3850 6400
Wire Wire Line
	3850 6400 3850 6525
Wire Wire Line
	3850 6525 4275 6525
Wire Bus Line
	6700 6575 7225 6575
Wire Bus Line
	7225 6575 7225 5700
Wire Bus Line
	7225 5700 6750 5700
Text GLabel 6700 3350 2    60   Input ~ 0
GND
Wire Bus Line
	1350 3350 3650 3350
Wire Bus Line
	3650 3350 6700 3350
Entry Wire Line
	3650 3350 3750 3450
Wire Wire Line
	3750 3450 3750 7100
Wire Wire Line
	3750 7100 3425 7100
$EndSCHEMATC
