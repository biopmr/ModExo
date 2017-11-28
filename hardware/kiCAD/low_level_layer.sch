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
Sheet 3 3
Title "Low Level Layer Communication"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L EPOS2.24/5 U?
U 1 1 58E6FED9
P 2850 6550
F 0 "U?" H 3500 4825 60  0000 C CNN
F 1 "EPOS2.24/5" H 3375 8250 60  0000 C CNN
F 2 "" H 2675 6625 60  0001 C CNN
F 3 "" H 2675 6625 60  0001 C CNN
	1    2850 6550
	-1   0    0    -1  
$EndComp
Text HLabel 4100 5075 2    60   Input ~ 0
24V
NoConn ~ 3800 7900
NoConn ~ 3800 8000
Entry Wire Line
	1175 5450 1275 5550
Entry Wire Line
	1175 5550 1275 5650
Entry Wire Line
	1175 5650 1275 5750
Entry Wire Line
	1175 5750 1275 5850
Entry Bus Bus
	1075 3850 1175 3950
Entry Wire Line
	4325 4250 4425 4350
Entry Wire Line
	4450 4250 4550 4350
Entry Wire Line
	4575 4250 4675 4350
$Comp
L Earth_Protective #PWR?
U 1 1 58E84D2D
P 3800 5900
F 0 "#PWR?" H 4050 5650 50  0001 C CNN
F 1 "Earth_Protective" H 4250 5750 50  0001 C CNN
F 2 "" H 3800 5800 50  0001 C CNN
F 3 "" H 3800 5800 50  0001 C CNN
	1    3800 5900
	1    0    0    -1  
$EndComp
$Comp
L Earth_Protective #PWR?
U 1 1 58E84D66
P 3800 6975
F 0 "#PWR?" H 4050 6725 50  0001 C CNN
F 1 "Earth_Protective" H 4250 6825 50  0001 C CNN
F 2 "" H 3800 6875 50  0001 C CNN
F 3 "" H 3800 6875 50  0001 C CNN
	1    3800 6975
	1    0    0    -1  
$EndComp
Text HLabel 7050 3850 2    60   Input ~ 0
CAN
Text GLabel 7050 4250 2    60   Input ~ 0
GND
$Comp
L EC90 U?
U 1 1 58E945FF
P 6075 6150
F 0 "U?" H 6525 5525 60  0000 C CNN
F 1 "EC90" H 6075 6775 60  0000 C CNN
F 2 "" H 6025 6175 60  0001 C CNN
F 3 "" H 6025 6175 60  0001 C CNN
	1    6075 6150
	1    0    0    -1  
$EndComp
$Comp
L Amplifier U?
U 1 1 58E94F57
P 5975 4950
F 0 "U?" H 6475 4575 60  0000 C CNN
F 1 "Amplifier" H 6325 5500 60  0000 C CNN
F 2 "" H 5975 4950 60  0001 C CNN
F 3 "" H 5975 4950 60  0001 C CNN
	1    5975 4950
	1    0    0    -1  
$EndComp
Entry Wire Line
	11425 -350 11525 -250
Entry Wire Line
	13175 600  13275 700 
Entry Wire Line
	6750 3850 6850 3950
Entry Wire Line
	6950 4250 7050 4350
Text Label 4975 4225 1    60   ~ 0
CAN_L
Text Label 5175 4225 1    60   ~ 0
CAN_H
Text Label 6850 4150 1    60   ~ 0
5V
Text Label 1175 4150 1    60   ~ 0
DB9
Wire Bus Line
	700  4250 7050 4250
Wire Bus Line
	4325 4250 4450 4250
Wire Bus Line
	700  3850 7050 3850
Wire Bus Line
	1175 3950 1175 5750
Wire Wire Line
	7450 7800 3800 7800
Wire Wire Line
	7450 5900 7450 7800
Wire Wire Line
	6775 5900 7450 5900
Wire Wire Line
	7350 6000 6775 6000
Wire Wire Line
	7350 6000 7350 7675
Wire Wire Line
	7350 7675 3800 7675
Wire Wire Line
	7250 7575 3800 7575
Wire Wire Line
	7250 6100 7250 7575
Wire Wire Line
	6775 6100 7250 6100
Wire Wire Line
	7125 6200 6775 6200
Wire Wire Line
	7125 7475 7125 6200
Wire Wire Line
	3800 7475 7125 7475
Wire Wire Line
	3800 7375 7025 7375
Wire Wire Line
	7025 7375 7025 6400
Wire Wire Line
	7025 6400 6775 6400
Wire Wire Line
	6925 6500 6775 6500
Wire Wire Line
	6925 7275 6925 6500
Wire Wire Line
	3800 7275 6925 7275
Wire Wire Line
	1275 5850 1950 5850
Wire Wire Line
	1275 5750 1950 5750
Wire Wire Line
	1275 5650 1950 5650
Wire Wire Line
	1275 5550 1950 5550
Wire Wire Line
	4425 5175 3800 5175
Wire Wire Line
	3800 5075 4100 5075
Wire Wire Line
	5150 6875 5150 6400
Wire Wire Line
	3800 6875 5150 6875
Wire Wire Line
	5050 6775 5050 6300
Wire Wire Line
	3800 6775 5050 6775
Wire Wire Line
	4950 6675 4950 6200
Wire Wire Line
	3800 6675 4950 6675
Wire Wire Line
	4850 6575 4850 6100
Wire Wire Line
	3800 6575 4850 6575
Wire Wire Line
	4850 6100 5300 6100
Wire Wire Line
	4950 6200 5300 6200
Wire Wire Line
	5050 6300 5300 6300
Wire Wire Line
	5150 6400 5300 6400
Wire Wire Line
	4750 6000 5300 6000
Wire Wire Line
	4750 6475 4750 6000
Wire Wire Line
	3800 6475 4750 6475
Wire Wire Line
	4750 5900 5300 5900
Wire Wire Line
	4750 5800 4750 5900
Wire Wire Line
	3800 5800 4750 5800
Wire Wire Line
	4850 5800 5300 5800
Wire Wire Line
	4850 5700 4850 5800
Wire Wire Line
	3800 5700 4850 5700
Wire Wire Line
	4950 5700 5300 5700
Wire Wire Line
	4950 5600 4950 5700
Wire Wire Line
	3800 5600 4950 5600
Wire Wire Line
	4425 4350 4425 5175
Wire Wire Line
	4675 4350 4675 7375
Connection ~ 4675 7375
Wire Wire Line
	4550 4350 4550 6775
Connection ~ 4550 6775
Wire Wire Line
	12675 0    12675 1650
Wire Wire Line
	12975 1050 13075 1050
Wire Wire Line
	12625 -100 12625 1650
Wire Wire Line
	7050 4350 7050 4900
Wire Wire Line
	7050 4900 6700 4900
Wire Wire Line
	6850 3950 6850 4800
Wire Wire Line
	6850 4800 6700 4800
Entry Wire Line
	5100 3850 5200 3950
Wire Wire Line
	5200 3950 5200 4800
Wire Wire Line
	5200 4800 5400 4800
Entry Wire Line
	4900 3850 5000 3950
Wire Wire Line
	5000 3950 5000 4900
Wire Wire Line
	5000 4900 5350 4900
$EndSCHEMATC
