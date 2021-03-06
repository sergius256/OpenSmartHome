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
LIBS:irrigator-arduino-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R1
U 1 1 58E77BBC
P 1250 1300
F 0 "R1" V 1330 1300 50  0000 C CNN
F 1 "10k" V 1250 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 1180 1300 50  0001 C CNN
F 3 "" H 1250 1300 50  0000 C CNN
	1    1250 1300
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 58E77D53
P 1450 1300
F 0 "R2" V 1530 1300 50  0000 C CNN
F 1 "10k" V 1450 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 1380 1300 50  0001 C CNN
F 3 "" H 1450 1300 50  0000 C CNN
	1    1450 1300
	-1   0    0    1   
$EndComp
$Comp
L R R3
U 1 1 58E77D8F
P 1650 1300
F 0 "R3" V 1730 1300 50  0000 C CNN
F 1 "10k" V 1650 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 1580 1300 50  0001 C CNN
F 3 "" H 1650 1300 50  0000 C CNN
	1    1650 1300
	-1   0    0    1   
$EndComp
$Comp
L R R4
U 1 1 58E77E77
P 1850 1300
F 0 "R4" V 1930 1300 50  0000 C CNN
F 1 "10k" V 1850 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 1780 1300 50  0001 C CNN
F 3 "" H 1850 1300 50  0000 C CNN
	1    1850 1300
	-1   0    0    1   
$EndComp
$Comp
L R R5
U 1 1 58E77EBF
P 2050 1300
F 0 "R5" V 2130 1300 50  0000 C CNN
F 1 "10k" V 2050 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 1980 1300 50  0001 C CNN
F 3 "" H 2050 1300 50  0000 C CNN
	1    2050 1300
	-1   0    0    1   
$EndComp
$Comp
L R R6
U 1 1 58E77F38
P 2250 1300
F 0 "R6" V 2330 1300 50  0000 C CNN
F 1 "10k" V 2250 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2180 1300 50  0001 C CNN
F 3 "" H 2250 1300 50  0000 C CNN
	1    2250 1300
	-1   0    0    1   
$EndComp
$Comp
L R R7
U 1 1 58E77F96
P 2450 1300
F 0 "R7" V 2530 1300 50  0000 C CNN
F 1 "10k" V 2450 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2380 1300 50  0001 C CNN
F 3 "" H 2450 1300 50  0000 C CNN
	1    2450 1300
	-1   0    0    1   
$EndComp
$Comp
L R R8
U 1 1 58E78018
P 2650 1300
F 0 "R8" V 2730 1300 50  0000 C CNN
F 1 "10k" V 2650 1300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2580 1300 50  0001 C CNN
F 3 "" H 2650 1300 50  0000 C CNN
	1    2650 1300
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X16 P1
U 1 1 58E780B6
P 950 2250
F 0 "P1" H 950 3100 50  0000 C CNN
F 1 "CONN_01X16" V 1050 2250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x16_Pitch2.54mm" H 950 2250 50  0001 C CNN
F 3 "" H 950 2250 50  0000 C CNN
	1    950  2250
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR01
U 1 1 58E78B6E
P 1250 3100
F 0 "#PWR01" H 1250 2850 50  0001 C CNN
F 1 "GND" H 1250 2950 50  0000 C CNN
F 2 "" H 1250 3100 50  0000 C CNN
F 3 "" H 1250 3100 50  0000 C CNN
	1    1250 3100
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q1
U 1 1 58E790BE
P 4400 1850
F 0 "Q1" H 4700 1900 50  0000 R CNN
F 1 "IRLML2502" H 5050 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 4600 1950 50  0001 C CNN
F 3 "" H 4400 1850 50  0000 C CNN
	1    4400 1850
	1    0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 58E792FE
P 4500 1400
F 0 "D1" H 4500 1500 50  0000 C CNN
F 1 "BAW16W" H 4500 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 4500 1400 50  0001 C CNN
F 3 "" H 4500 1400 50  0000 C CNN
	1    4500 1400
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 58E7936B
P 4350 2150
F 0 "R9" V 4430 2150 50  0000 C CNN
F 1 "300" V 4350 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4280 2150 50  0001 C CNN
F 3 "" H 4350 2150 50  0000 C CNN
	1    4350 2150
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GSD Q2
U 1 1 58E7A2EA
P 5100 1850
F 0 "Q2" H 5400 1900 50  0000 R CNN
F 1 "IRLML2502" H 5750 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 5300 1950 50  0001 C CNN
F 3 "" H 5100 1850 50  0000 C CNN
	1    5100 1850
	1    0    0    -1  
$EndComp
$Comp
L D D2
U 1 1 58E7A2F1
P 5200 1400
F 0 "D2" H 5200 1500 50  0000 C CNN
F 1 "BAW16W" H 5200 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 5200 1400 50  0001 C CNN
F 3 "" H 5200 1400 50  0000 C CNN
	1    5200 1400
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 58E7A2F8
P 5050 2150
F 0 "R10" V 5130 2150 50  0000 C CNN
F 1 "300" V 5050 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4980 2150 50  0001 C CNN
F 3 "" H 5050 2150 50  0000 C CNN
	1    5050 2150
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GSD Q3
U 1 1 58E7A3FE
P 5800 1850
F 0 "Q3" H 6100 1900 50  0000 R CNN
F 1 "IRLML2502" H 6450 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 6000 1950 50  0001 C CNN
F 3 "" H 5800 1850 50  0000 C CNN
	1    5800 1850
	1    0    0    -1  
$EndComp
$Comp
L D D3
U 1 1 58E7A405
P 5900 1400
F 0 "D3" H 5900 1500 50  0000 C CNN
F 1 "BAW16W" H 5900 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 5900 1400 50  0001 C CNN
F 3 "" H 5900 1400 50  0000 C CNN
	1    5900 1400
	0    1    1    0   
$EndComp
$Comp
L R R11
U 1 1 58E7A40C
P 5750 2150
F 0 "R11" V 5830 2150 50  0000 C CNN
F 1 "300" V 5750 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5680 2150 50  0001 C CNN
F 3 "" H 5750 2150 50  0000 C CNN
	1    5750 2150
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GSD Q4
U 1 1 58E7A548
P 6500 1850
F 0 "Q4" H 6800 1900 50  0000 R CNN
F 1 "IRLML2502" H 7150 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 6700 1950 50  0001 C CNN
F 3 "" H 6500 1850 50  0000 C CNN
	1    6500 1850
	1    0    0    -1  
$EndComp
$Comp
L D D4
U 1 1 58E7A54F
P 6600 1400
F 0 "D4" H 6600 1500 50  0000 C CNN
F 1 "BAW16W" H 6600 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 6600 1400 50  0001 C CNN
F 3 "" H 6600 1400 50  0000 C CNN
	1    6600 1400
	0    1    1    0   
$EndComp
$Comp
L R R12
U 1 1 58E7A556
P 6450 2150
F 0 "R12" V 6530 2150 50  0000 C CNN
F 1 "300" V 6450 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6380 2150 50  0001 C CNN
F 3 "" H 6450 2150 50  0000 C CNN
	1    6450 2150
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GSD Q5
U 1 1 58E7A62C
P 7200 1850
F 0 "Q5" H 7500 1900 50  0000 R CNN
F 1 "IRLML2502" H 7850 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 7400 1950 50  0001 C CNN
F 3 "" H 7200 1850 50  0000 C CNN
	1    7200 1850
	1    0    0    -1  
$EndComp
$Comp
L D D5
U 1 1 58E7A633
P 7300 1400
F 0 "D5" H 7300 1500 50  0000 C CNN
F 1 "BAW16W" H 7300 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 7300 1400 50  0001 C CNN
F 3 "" H 7300 1400 50  0000 C CNN
	1    7300 1400
	0    1    1    0   
$EndComp
$Comp
L R R13
U 1 1 58E7A63A
P 7150 2150
F 0 "R13" V 7230 2150 50  0000 C CNN
F 1 "300" V 7150 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7080 2150 50  0001 C CNN
F 3 "" H 7150 2150 50  0000 C CNN
	1    7150 2150
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GSD Q6
U 1 1 58E7A762
P 7900 1850
F 0 "Q6" H 8200 1900 50  0000 R CNN
F 1 "IRLML2502" H 8550 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8100 1950 50  0001 C CNN
F 3 "" H 7900 1850 50  0000 C CNN
	1    7900 1850
	1    0    0    -1  
$EndComp
$Comp
L D D6
U 1 1 58E7A769
P 8000 1400
F 0 "D6" H 8000 1500 50  0000 C CNN
F 1 "BAW16W" H 8000 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 8000 1400 50  0001 C CNN
F 3 "" H 8000 1400 50  0000 C CNN
	1    8000 1400
	0    1    1    0   
$EndComp
$Comp
L R R14
U 1 1 58E7A770
P 7850 2150
F 0 "R14" V 7930 2150 50  0000 C CNN
F 1 "300" V 7850 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7780 2150 50  0001 C CNN
F 3 "" H 7850 2150 50  0000 C CNN
	1    7850 2150
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GSD Q7
U 1 1 58E7A88C
P 8600 1850
F 0 "Q7" H 8900 1900 50  0000 R CNN
F 1 "IRLML2502" H 9250 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 8800 1950 50  0001 C CNN
F 3 "" H 8600 1850 50  0000 C CNN
	1    8600 1850
	1    0    0    -1  
$EndComp
$Comp
L D D7
U 1 1 58E7A893
P 8700 1400
F 0 "D7" H 8700 1500 50  0000 C CNN
F 1 "BAW16W" H 8700 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 8700 1400 50  0001 C CNN
F 3 "" H 8700 1400 50  0000 C CNN
	1    8700 1400
	0    1    1    0   
$EndComp
$Comp
L R R15
U 1 1 58E7A89A
P 8550 2150
F 0 "R15" V 8630 2150 50  0000 C CNN
F 1 "300" V 8550 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 8480 2150 50  0001 C CNN
F 3 "" H 8550 2150 50  0000 C CNN
	1    8550 2150
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_GSD Q8
U 1 1 58E7AA42
P 9300 1850
F 0 "Q8" H 9600 1900 50  0000 R CNN
F 1 "IRLML2502" H 9950 1800 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 9500 1950 50  0001 C CNN
F 3 "" H 9300 1850 50  0000 C CNN
	1    9300 1850
	1    0    0    -1  
$EndComp
$Comp
L D D8
U 1 1 58E7AA49
P 9400 1400
F 0 "D8" H 9400 1500 50  0000 C CNN
F 1 "BAW16W" H 9400 1300 50  0000 C CNN
F 2 "Diodes_SMD:D_SOD-123" H 9400 1400 50  0001 C CNN
F 3 "" H 9400 1400 50  0000 C CNN
	1    9400 1400
	0    1    1    0   
$EndComp
$Comp
L R R16
U 1 1 58E7AA50
P 9250 2150
F 0 "R16" V 9330 2150 50  0000 C CNN
F 1 "300" V 9250 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9180 2150 50  0001 C CNN
F 3 "" H 9250 2150 50  0000 C CNN
	1    9250 2150
	0    1    1    0   
$EndComp
$Comp
L CONN_01X08 P5
U 1 1 58E7FF81
P 10000 2750
F 0 "P5" H 10000 3200 50  0000 C CNN
F 1 "CONN_01X08" V 10100 2750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x08_Pitch2.54mm" H 10000 2750 50  0001 C CNN
F 3 "" H 10000 2750 50  0000 C CNN
	1    10000 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 58E831CE
P 6600 2400
F 0 "#PWR02" H 6600 2150 50  0001 C CNN
F 1 "GND" H 6600 2250 50  0000 C CNN
F 2 "" H 6600 2400 50  0000 C CNN
F 3 "" H 6600 2400 50  0000 C CNN
	1    6600 2400
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR03
U 1 1 58E8B0A2
P 2650 1000
F 0 "#PWR03" H 2650 850 50  0001 C CNN
F 1 "+5V" H 2650 1140 50  0000 C CNN
F 2 "" H 2650 1000 50  0000 C CNN
F 3 "" H 2650 1000 50  0000 C CNN
	1    2650 1000
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR04
U 1 1 58E8B589
P 9400 1000
F 0 "#PWR04" H 9400 850 50  0001 C CNN
F 1 "+12V" H 9400 1140 50  0000 C CNN
F 2 "" H 9400 1000 50  0000 C CNN
F 3 "" H 9400 1000 50  0000 C CNN
	1    9400 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR05
U 1 1 58F0CB58
P 2800 3350
F 0 "#PWR05" H 2800 3200 50  0001 C CNN
F 1 "+5V" H 2800 3490 50  0000 C CNN
F 2 "" H 2800 3350 50  0000 C CNN
F 3 "" H 2800 3350 50  0000 C CNN
	1    2800 3350
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR06
U 1 1 58F0CDB5
P 2950 1200
F 0 "#PWR06" H 2950 950 50  0001 C CNN
F 1 "GND" H 2950 1050 50  0000 C CNN
F 2 "" H 2950 1200 50  0000 C CNN
F 3 "" H 2950 1200 50  0000 C CNN
	1    2950 1200
	0    1    1    0   
$EndComp
$Comp
L CONN_01X02 P2
U 1 1 58F0D00B
P 3250 850
F 0 "P2" H 3250 1000 50  0000 C CNN
F 1 "CONN_01X02" V 3350 850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3250 850 50  0001 C CNN
F 3 "" H 3250 850 50  0000 C CNN
	1    3250 850 
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X15 P4
U 1 1 58F11838
P 3700 2950
F 0 "P4" H 3700 3750 50  0000 C CNN
F 1 "CONN_01X15" V 3800 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x15_Pitch2.54mm" H 3700 2950 50  0001 C CNN
F 3 "" H 3700 2950 50  0000 C CNN
	1    3700 2950
	-1   0    0    1   
$EndComp
$Comp
L FILTER FB1
U 1 1 58F11B47
P 2800 2900
F 0 "FB1" H 2800 3050 50  0000 C CNN
F 1 "FILTER" H 2800 2800 50  0000 C CNN
F 2 "Resistors_ThroughHole:R_Axial_Power_L25.0mm_W9.0mm_P30.48mm" H 2800 2900 50  0001 C CNN
F 3 "" H 2800 2900 50  0000 C CNN
	1    2800 2900
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR07
U 1 1 58F11F7E
P 2950 1400
F 0 "#PWR07" H 2950 1250 50  0001 C CNN
F 1 "+5V" H 2950 1540 50  0000 C CNN
F 2 "" H 2950 1400 50  0000 C CNN
F 3 "" H 2950 1400 50  0000 C CNN
	1    2950 1400
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X15 P3
U 1 1 58F14B33
P 3250 1800
F 0 "P3" H 3250 2600 50  0000 C CNN
F 1 "CONN_01X15" V 3350 1800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x15_Pitch2.54mm" H 3250 1800 50  0001 C CNN
F 3 "" H 3250 1800 50  0000 C CNN
	1    3250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1700 1450 1700
Wire Wire Line
	1450 1700 1450 1450
Wire Wire Line
	1150 1900 1650 1900
Wire Wire Line
	1650 1900 1650 1450
Wire Wire Line
	1150 2100 1850 2100
Wire Wire Line
	1850 2100 1850 1450
Wire Wire Line
	1150 2300 2050 2300
Wire Wire Line
	2050 2300 2050 1450
Wire Wire Line
	1150 2500 2250 2500
Wire Wire Line
	2250 2500 2250 1450
Wire Wire Line
	1150 2700 2450 2700
Wire Wire Line
	2450 2700 2450 1450
Wire Wire Line
	1150 2900 2650 2900
Wire Wire Line
	2650 2900 2650 1450
Wire Wire Line
	1150 1600 1250 1600
Wire Wire Line
	1250 1600 1250 3100
Wire Wire Line
	1250 1800 1150 1800
Wire Wire Line
	1250 2000 1150 2000
Connection ~ 1250 1800
Wire Wire Line
	1250 2200 1150 2200
Connection ~ 1250 2000
Wire Wire Line
	1250 2400 1150 2400
Connection ~ 1250 2200
Wire Wire Line
	1250 2600 1150 2600
Connection ~ 1250 2400
Wire Wire Line
	1250 2800 1150 2800
Connection ~ 1250 2600
Wire Wire Line
	1250 3000 1150 3000
Connection ~ 1250 2800
Wire Wire Line
	1250 1150 1250 1050
Wire Wire Line
	1250 1050 2650 1050
Wire Wire Line
	1450 1050 1450 1150
Wire Wire Line
	1650 1050 1650 1150
Connection ~ 1450 1050
Wire Wire Line
	1850 950  1850 1150
Connection ~ 1650 1050
Wire Wire Line
	2050 1050 2050 1150
Connection ~ 1850 1050
Wire Wire Line
	2250 1050 2250 1150
Connection ~ 2050 1050
Wire Wire Line
	2450 1050 2450 1150
Connection ~ 2250 1050
Wire Wire Line
	2650 1000 2650 1150
Connection ~ 2450 1050
Connection ~ 1250 3000
Wire Wire Line
	4500 1550 4500 1650
Wire Wire Line
	4500 2050 4500 2300
Wire Wire Line
	4200 1850 4150 1850
Wire Wire Line
	4150 1850 4150 2350
Wire Wire Line
	4150 2150 4200 2150
Wire Wire Line
	5200 1550 5200 1650
Wire Wire Line
	5200 2050 5200 2300
Wire Wire Line
	4900 1850 4850 1850
Wire Wire Line
	4850 1850 4850 2450
Wire Wire Line
	4850 2150 4900 2150
Wire Wire Line
	5900 1550 5900 1650
Wire Wire Line
	5900 2300 5900 2050
Wire Wire Line
	5600 1850 5550 1850
Wire Wire Line
	5550 1850 5550 2550
Wire Wire Line
	5550 2150 5600 2150
Wire Wire Line
	6600 1550 6600 1650
Wire Wire Line
	6600 2050 6600 2400
Wire Wire Line
	6300 1850 6250 1850
Wire Wire Line
	6250 1850 6250 2650
Wire Wire Line
	6250 2150 6300 2150
Wire Wire Line
	7300 1550 7300 1650
Wire Wire Line
	7300 2050 7300 2400
Wire Wire Line
	7000 1850 6950 1850
Wire Wire Line
	6950 1850 6950 2750
Wire Wire Line
	6950 2150 7000 2150
Wire Wire Line
	8000 1550 8000 1650
Wire Wire Line
	8000 2300 8000 2050
Wire Wire Line
	7700 1850 7650 1850
Wire Wire Line
	7650 1850 7650 2850
Wire Wire Line
	7650 2150 7700 2150
Wire Wire Line
	8700 1550 8700 1650
Wire Wire Line
	8700 2300 8700 2050
Wire Wire Line
	8400 1850 8350 1850
Wire Wire Line
	8350 1850 8350 2950
Wire Wire Line
	8350 2150 8400 2150
Wire Wire Line
	9400 1550 9400 1650
Wire Wire Line
	9400 2300 9400 2050
Wire Wire Line
	9100 1850 9050 1850
Wire Wire Line
	9050 1850 9050 3050
Wire Wire Line
	9050 2150 9100 2150
Wire Wire Line
	4150 2350 3900 2350
Connection ~ 4150 2150
Wire Wire Line
	4850 2450 3900 2450
Connection ~ 4850 2150
Wire Wire Line
	5550 2550 3900 2550
Connection ~ 5550 2150
Wire Wire Line
	6250 2650 3900 2650
Connection ~ 6250 2150
Wire Wire Line
	6950 2750 3900 2750
Connection ~ 6950 2150
Wire Wire Line
	7650 2850 3900 2850
Connection ~ 7650 2150
Wire Wire Line
	8350 2950 3900 2950
Connection ~ 8350 2150
Wire Wire Line
	9050 3050 3900 3050
Connection ~ 9050 2150
Wire Wire Line
	4500 1000 4500 1250
Wire Wire Line
	4500 1050 9400 1050
Wire Wire Line
	5200 1050 5200 1250
Wire Wire Line
	5900 1050 5900 1250
Connection ~ 5200 1050
Wire Wire Line
	6600 1050 6600 1250
Connection ~ 5900 1050
Wire Wire Line
	7300 1050 7300 1250
Connection ~ 6600 1050
Wire Wire Line
	8000 1050 8000 1250
Connection ~ 7300 1050
Wire Wire Line
	8700 900  8700 1250
Connection ~ 8000 1050
Wire Wire Line
	9400 1000 9400 1250
Connection ~ 8700 1050
Wire Wire Line
	9400 1600 9650 1600
Wire Wire Line
	9650 1600 9650 2400
Wire Wire Line
	9650 2400 9800 2400
Connection ~ 9400 1600
Wire Wire Line
	9800 2500 8950 2500
Wire Wire Line
	8950 2500 8950 1600
Wire Wire Line
	8950 1600 8700 1600
Connection ~ 8700 1600
Wire Wire Line
	9800 2600 8250 2600
Wire Wire Line
	8250 2600 8250 1600
Wire Wire Line
	8250 1600 8000 1600
Connection ~ 8000 1600
Wire Wire Line
	9800 2700 7550 2700
Wire Wire Line
	7550 2700 7550 1600
Wire Wire Line
	7550 1600 7300 1600
Connection ~ 7300 1600
Wire Wire Line
	9800 2800 6850 2800
Wire Wire Line
	6850 2800 6850 1600
Wire Wire Line
	6850 1600 6600 1600
Connection ~ 6600 1600
Wire Wire Line
	9800 2900 6150 2900
Wire Wire Line
	6150 2900 6150 1600
Wire Wire Line
	6150 1600 5900 1600
Connection ~ 5900 1600
Wire Wire Line
	9800 3000 5450 3000
Wire Wire Line
	5450 3000 5450 1600
Wire Wire Line
	5450 1600 5200 1600
Connection ~ 5200 1600
Wire Wire Line
	9800 3100 4750 3100
Wire Wire Line
	4750 3100 4750 1600
Wire Wire Line
	4750 1600 4500 1600
Connection ~ 4500 1600
Wire Wire Line
	4500 2300 9400 2300
Connection ~ 5200 2150
Connection ~ 4500 2150
Connection ~ 5900 2150
Connection ~ 5200 2300
Connection ~ 6600 2150
Connection ~ 5900 2300
Connection ~ 7300 2150
Connection ~ 6600 2300
Connection ~ 8000 2150
Connection ~ 7300 2300
Connection ~ 8700 2150
Connection ~ 8000 2300
Connection ~ 9400 2150
Connection ~ 8700 2300
Connection ~ 2650 1050
Connection ~ 9400 1050
Connection ~ 1250 1500
Wire Wire Line
	1250 1500 1250 1450
Connection ~ 1450 1600
Connection ~ 1650 1700
Connection ~ 1850 1800
Connection ~ 2050 1900
Connection ~ 2250 2000
Connection ~ 2450 2100
Connection ~ 2650 2200
Wire Wire Line
	2650 2200 3050 2200
Wire Wire Line
	2450 2100 3050 2100
Wire Wire Line
	2250 2000 3050 2000
Wire Wire Line
	2050 1900 3050 1900
Wire Wire Line
	1850 1800 3050 1800
Wire Wire Line
	1650 1700 3050 1700
Wire Wire Line
	1450 1600 3050 1600
Wire Wire Line
	1150 1500 3050 1500
Wire Wire Line
	3050 2300 2800 2300
Wire Wire Line
	2800 2300 2800 2550
NoConn ~ 3050 2400
NoConn ~ 3050 2500
Wire Wire Line
	2800 3250 2800 3350
Wire Wire Line
	3050 1400 2950 1400
Wire Wire Line
	2950 1200 3050 1200
Wire Wire Line
	3050 900  3050 1100
Wire Wire Line
	3050 800  3000 800 
Wire Wire Line
	3000 800  3000 1200
Connection ~ 3000 1200
NoConn ~ 3050 1300
Wire Wire Line
	4500 1000 3050 1000
Connection ~ 3050 1000
Connection ~ 4500 1050
$Comp
L GND #PWR08
U 1 1 58F1737B
P 4000 3350
F 0 "#PWR08" H 4000 3100 50  0001 C CNN
F 1 "GND" H 4000 3200 50  0000 C CNN
F 2 "" H 4000 3350 50  0000 C CNN
F 3 "" H 4000 3350 50  0000 C CNN
	1    4000 3350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3900 3350 4000 3350
NoConn ~ 3900 3150
NoConn ~ 3900 3250
NoConn ~ 3900 3450
$Comp
L PWR_FLAG #FLG09
U 1 1 58F18C90
P 8700 900
F 0 "#FLG09" H 8700 995 50  0001 C CNN
F 1 "PWR_FLAG" H 8700 1080 50  0000 C CNN
F 2 "" H 8700 900 50  0000 C CNN
F 3 "" H 8700 900 50  0000 C CNN
	1    8700 900 
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG010
U 1 1 58F18DAC
P 1850 950
F 0 "#FLG010" H 1850 1045 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 1130 50  0000 C CNN
F 2 "" H 1850 950 50  0000 C CNN
F 3 "" H 1850 950 50  0000 C CNN
	1    1850 950 
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG011
U 1 1 58F18FF1
P 7300 2400
F 0 "#FLG011" H 7300 2495 50  0001 C CNN
F 1 "PWR_FLAG" H 7300 2580 50  0000 C CNN
F 2 "" H 7300 2400 50  0000 C CNN
F 3 "" H 7300 2400 50  0000 C CNN
	1    7300 2400
	-1   0    0    1   
$EndComp
NoConn ~ 3900 2250
$Comp
L CONN_01X04 P6
U 1 1 5904A7E2
P 4700 3700
F 0 "P6" H 4700 3950 50  0000 C CNN
F 1 "CONN_01X04" V 4800 3700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x04_Pitch2.54mm" H 4700 3700 50  0001 C CNN
F 3 "" H 4700 3700 50  0000 C CNN
	1    4700 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3550 4500 3550
Wire Wire Line
	3900 3650 4500 3650
$Comp
L GND #PWR012
U 1 1 5904ABD2
P 4350 3750
F 0 "#PWR012" H 4350 3500 50  0001 C CNN
F 1 "GND" H 4350 3600 50  0000 C CNN
F 2 "" H 4350 3750 50  0000 C CNN
F 3 "" H 4350 3750 50  0000 C CNN
	1    4350 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 3750 4500 3750
$Comp
L +5V #PWR013
U 1 1 5904B0E8
P 4400 3950
F 0 "#PWR013" H 4400 3800 50  0001 C CNN
F 1 "+5V" H 4400 4090 50  0000 C CNN
F 2 "" H 4400 3950 50  0000 C CNN
F 3 "" H 4400 3950 50  0000 C CNN
	1    4400 3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 3850 4400 3850
Wire Wire Line
	4400 3850 4400 3950
$EndSCHEMATC
