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
Sheet 2 3
Title "High Level Layer Communication"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Arduino101 MC?
U 1 1 58E6F935
P 3025 6050
F 0 "MC?" H 3425 4600 60  0000 C CNN
F 1 "Arduino101" H 3525 7550 60  0000 C CNN
F 2 "" H 3025 6050 60  0001 C CNN
F 3 "" H 3025 6050 60  0001 C CNN
	1    3025 6050
	1    0    0    -1  
$EndComp
Text Label 6775 5350 0    60   ~ 0
IOREF
Text Label 6775 5500 0    60   ~ 0
RESET
Text Label 1750 5200 2    60   ~ 0
IOREF
Text Label 1750 5350 2    60   ~ 0
RESET
Text HLabel 7400 3350 2    60   Input ~ 0
CAN
Text GLabel 7400 3950 2    60   Input ~ 0
GND
Entry Bus Bus
	5575 3350 5675 3450
Entry Wire Line
	900  3950 1000 4050
Entry Wire Line
	750  3950 850  4050
Entry Wire Line
	4225 3950 4325 4050
$Comp
L CAN_BUS_Shield CAN_BUS?
U 1 1 58E94A64
P 5775 6050
F 0 "CAN_BUS?" H 6175 4600 60  0000 C CNN
F 1 "CAN_BUS_Shield" H 6275 7550 60  0000 C CNN
F 2 "" H 5775 6050 60  0001 C CNN
F 3 "" H 5775 6050 60  0001 C CNN
	1    5775 6050
	-1   0    0    -1  
$EndComp
Entry Wire Line
	7250 3950 7350 4050
Entry Wire Line
	7100 3350 7200 3450
Entry Wire Line
	6525 3350 6625 3450
Entry Wire Line
	6700 3350 6800 3450
Text Label 7200 3750 1    60   ~ 0
5V
Text Label 6800 3850 1    60   ~ 0
CAN_L
Text Label 6625 3850 1    60   ~ 0
CAN_H
Wire Wire Line
	3725 5050 5075 5050
Wire Wire Line
	3725 5200 5075 5200
Wire Wire Line
	3725 5350 5075 5350
Wire Wire Line
	3725 5500 5075 5500
Wire Wire Line
	3725 5650 5075 5650
Wire Wire Line
	3725 5950 5075 5950
Wire Wire Line
	3725 7000 5075 7000
Wire Wire Line
	2275 5200 1750 5200
Wire Wire Line
	2275 5350 1750 5350
Wire Bus Line
	650  3350 7400 3350
Wire Bus Line
	600  3950 7400 3950
Wire Wire Line
	1000 5800 2275 5800
Wire Bus Line
	5675 3450 5675 4450
Wire Wire Line
	1000 5800 1000 4050
Wire Wire Line
	2275 5950 850  5950
Wire Wire Line
	850  5950 850  4050
Wire Wire Line
	4325 4050 4325 5200
Connection ~ 4325 5200
Wire Wire Line
	6775 5350 6525 5350
Wire Wire Line
	6525 5500 6775 5500
Wire Wire Line
	6525 5950 7350 5950
Wire Wire Line
	7350 5950 7350 4050
Wire Wire Line
	7200 5800 6525 5800
Wire Wire Line
	7200 3450 7200 5800
Wire Wire Line
	6625 4800 6525 4800
Wire Wire Line
	6625 3450 6625 4800
Wire Wire Line
	6800 4950 6525 4950
Wire Wire Line
	6800 3450 6800 4950
Text Label 5675 3725 1    60   ~ 0
DB9
$EndSCHEMATC
