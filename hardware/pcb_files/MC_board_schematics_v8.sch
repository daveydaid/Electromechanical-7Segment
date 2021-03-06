EESchema Schematic File Version 4
EELAYER 30 0
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
L 74xx:74HC595 U1
U 1 1 61CCA076
P 3900 4250
F 0 "U1" H 3900 5031 50  0000 C CNN
F 1 "74HC595" H 3900 4940 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 3900 4250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 3900 4250 50  0001 C CNN
	1    3900 4250
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U2
U 1 1 61CCF844
P 3900 5850
F 0 "U2" H 4400 6350 50  0000 C CNN
F 1 "74HC595" H 4100 6400 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 3900 5850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 3900 5850 50  0001 C CNN
	1    3900 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6650 3900 6550
Wire Wire Line
	3900 3250 3300 3250
Wire Wire Line
	3300 3250 3300 4150
Wire Wire Line
	3300 5250 3900 5250
Wire Wire Line
	3900 3250 3900 3650
Wire Wire Line
	3500 4150 3300 4150
Connection ~ 3300 4150
Wire Wire Line
	3300 4150 3300 5250
Wire Wire Line
	3300 5250 3300 5750
Wire Wire Line
	3300 5750 3500 5750
Connection ~ 3300 5250
Wire Wire Line
	4300 5200 3500 5200
NoConn ~ 4300 6350
NoConn ~ 4300 3850
NoConn ~ 4300 5450
$Comp
L Connector_Generic:Conn_01x05 J2
U 1 1 61D8EF98
P 1900 4050
F 0 "J2" H 1818 4467 50  0000 C CNN
F 1 "Conn_01x05" H 1818 4376 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 1900 4050 50  0001 C CNN
F 3 "~" H 1900 4050 50  0001 C CNN
	1    1900 4050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2700 3850 2700 3250
Connection ~ 3300 3250
Wire Wire Line
	2100 4050 3050 4050
Wire Wire Line
	3050 4050 3050 3850
Wire Wire Line
	3050 3850 3500 3850
Wire Wire Line
	2100 4150 3150 4150
Wire Wire Line
	3150 4150 3150 4050
Wire Wire Line
	3150 4050 3500 4050
Wire Wire Line
	3150 4150 3150 5650
Wire Wire Line
	3150 5650 3500 5650
Connection ~ 3150 4150
Wire Wire Line
	2100 4250 3050 4250
Wire Wire Line
	3050 4250 3050 4350
Wire Wire Line
	3050 4350 3500 4350
Wire Wire Line
	3050 4350 3050 5950
Wire Wire Line
	3050 5950 3500 5950
Connection ~ 3050 4350
Text GLabel 1850 3850 0    50   Input ~ 0
VIO
Text GLabel 1850 3950 0    50   Input ~ 0
GND
Text GLabel 1850 4050 0    50   Input ~ 0
DATA
Text GLabel 1850 4150 0    50   Input ~ 0
CLK_INPUT
Text GLabel 1850 4250 0    50   Input ~ 0
LATCH_CLK
$Comp
L Device:C_Small C1
U 1 1 61DA866E
P 3100 6400
F 0 "C1" H 3192 6446 50  0000 L CNN
F 1 "0.1uF" H 3192 6355 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3100 6400 50  0001 C CNN
F 3 "~" H 3100 6400 50  0001 C CNN
	1    3100 6400
	0    -1   -1   0   
$EndComp
$Comp
L custom_library:TMC2208_bigtree U3
U 1 1 61CE7930
P 6650 1400
F 0 "U3" H 6950 435 50  0000 C CNN
F 1 "TMC2208_bigtree" H 6950 526 50  0000 C CNN
F 2 "TMC2208_SILENTSTEPSTICK_UPDATE:TMC2208_SILENTSTEPSTICK_UPDATE" H 6650 1300 50  0001 C CNN
F 3 "" H 6650 1300 50  0001 C CNN
	1    6650 1400
	-1   0    0    1   
$EndComp
$Comp
L custom_library:TMC2208_bigtree U4
U 1 1 61CEA938
P 6650 2450
F 0 "U4" H 6950 1485 50  0000 C CNN
F 1 "TMC2208_bigtree" H 6950 1576 50  0000 C CNN
F 2 "TMC2208_SILENTSTEPSTICK_UPDATE:TMC2208_SILENTSTEPSTICK_UPDATE" H 6650 2350 50  0001 C CNN
F 3 "" H 6650 2350 50  0001 C CNN
	1    6650 2450
	-1   0    0    1   
$EndComp
$Comp
L custom_library:TMC2208_bigtree U5
U 1 1 61CEB46B
P 6650 3500
F 0 "U5" H 6950 2535 50  0000 C CNN
F 1 "TMC2208_bigtree" H 6950 2626 50  0000 C CNN
F 2 "TMC2208_SILENTSTEPSTICK_UPDATE:TMC2208_SILENTSTEPSTICK_UPDATE" H 6650 3400 50  0001 C CNN
F 3 "" H 6650 3400 50  0001 C CNN
	1    6650 3500
	-1   0    0    1   
$EndComp
$Comp
L custom_library:TMC2208_bigtree U6
U 1 1 61CEC1B9
P 6650 4550
F 0 "U6" H 6950 3585 50  0000 C CNN
F 1 "TMC2208_bigtree" H 6950 3676 50  0000 C CNN
F 2 "TMC2208_SILENTSTEPSTICK_UPDATE:TMC2208_SILENTSTEPSTICK_UPDATE" H 6650 4450 50  0001 C CNN
F 3 "" H 6650 4450 50  0001 C CNN
	1    6650 4550
	-1   0    0    1   
$EndComp
$Comp
L custom_library:TMC2208_bigtree U7
U 1 1 61CECCFB
P 6650 5600
F 0 "U7" H 6950 4635 50  0000 C CNN
F 1 "TMC2208_bigtree" H 6950 4726 50  0000 C CNN
F 2 "TMC2208_SILENTSTEPSTICK_UPDATE:TMC2208_SILENTSTEPSTICK_UPDATE" H 6650 5500 50  0001 C CNN
F 3 "" H 6650 5500 50  0001 C CNN
	1    6650 5600
	-1   0    0    1   
$EndComp
$Comp
L custom_library:TMC2208_bigtree U9
U 1 1 61CF2140
P 6650 7700
F 0 "U9" H 6950 6735 50  0000 C CNN
F 1 "TMC2208_bigtree" H 6950 6826 50  0000 C CNN
F 2 "TMC2208_SILENTSTEPSTICK_UPDATE:TMC2208_SILENTSTEPSTICK_UPDATE" H 6650 7600 50  0001 C CNN
F 3 "" H 6650 7600 50  0001 C CNN
	1    6650 7700
	-1   0    0    1   
$EndComp
$Comp
L custom_library:TMC2208_bigtree U8
U 1 1 61CF19F1
P 6650 6650
F 0 "U8" H 6950 5700 50  0000 C CNN
F 1 "TMC2208_bigtree" H 6950 5800 50  0000 C CNN
F 2 "TMC2208_SILENTSTEPSTICK_UPDATE:TMC2208_SILENTSTEPSTICK_UPDATE" H 6650 6550 50  0001 C CNN
F 3 "" H 6650 6550 50  0001 C CNN
	1    6650 6650
	-1   0    0    1   
$EndComp
NoConn ~ 5950 750 
NoConn ~ 5950 850 
NoConn ~ 5950 1150
NoConn ~ 5950 1800
NoConn ~ 5950 1900
NoConn ~ 5950 2200
NoConn ~ 5950 2850
NoConn ~ 5950 2950
NoConn ~ 5950 3250
NoConn ~ 5950 3900
NoConn ~ 5950 4000
NoConn ~ 5950 4300
NoConn ~ 5950 4950
NoConn ~ 5950 5050
NoConn ~ 5950 5350
NoConn ~ 5950 6000
NoConn ~ 5950 6100
NoConn ~ 5950 6400
NoConn ~ 5950 7050
NoConn ~ 5950 7150
NoConn ~ 5950 7450
Wire Wire Line
	4300 3950 4450 3950
Wire Wire Line
	4450 3950 4450 650 
Wire Wire Line
	4300 4050 4550 4050
Wire Wire Line
	4550 4050 4550 1700
Wire Wire Line
	4300 4150 4650 4150
Wire Wire Line
	4650 4150 4650 2750
Wire Wire Line
	4300 4250 4750 4250
Wire Wire Line
	4750 4250 4750 3800
Wire Wire Line
	4300 4350 4750 4350
Wire Wire Line
	4750 4350 4750 4850
Wire Wire Line
	4300 4450 4650 4450
Wire Wire Line
	4650 4450 4650 5900
Wire Wire Line
	4300 4550 4550 4550
Wire Wire Line
	4550 4550 4550 6950
Wire Wire Line
	4300 5550 4850 5550
Wire Wire Line
	4850 5550 4850 1350
Wire Wire Line
	4300 5650 4950 5650
Wire Wire Line
	4950 5650 4950 2400
Wire Wire Line
	4300 5750 5050 5750
Wire Wire Line
	5050 5750 5050 3450
Wire Wire Line
	4300 5850 5150 5850
Wire Wire Line
	5150 5850 5150 4500
Wire Wire Line
	5700 5950 5700 5550
Wire Wire Line
	5700 5550 5950 5550
Wire Wire Line
	4300 6050 5150 6050
Wire Wire Line
	5150 6050 5150 6600
Wire Wire Line
	4300 6150 5050 6150
Wire Wire Line
	5050 6150 5050 7650
$Comp
L Connector_Generic:Conn_01x09 J1
U 1 1 61E0FF72
P 1900 1850
F 0 "J1" H 1818 2467 50  0000 C CNN
F 1 "Conn_01x09" H 1818 2376 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x09_P2.54mm_Vertical" H 1900 1850 50  0001 C CNN
F 3 "~" H 1900 1850 50  0001 C CNN
	1    1900 1850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2100 1550 4300 1550
Wire Wire Line
	4300 1550 4300 1250
Wire Wire Line
	2100 1650 4300 1650
Wire Wire Line
	4300 1650 4300 2300
Wire Wire Line
	4450 650  5950 650 
Wire Wire Line
	4300 1250 5950 1250
Wire Wire Line
	4850 1350 5950 1350
Wire Wire Line
	4550 1700 5950 1700
Wire Wire Line
	4300 2300 5950 2300
Wire Wire Line
	4650 2750 5950 2750
Wire Wire Line
	5050 3450 5950 3450
Wire Wire Line
	4750 3800 5950 3800
Wire Wire Line
	5150 4500 5950 4500
Wire Wire Line
	4750 4850 5950 4850
Wire Wire Line
	4650 5900 5950 5900
Wire Wire Line
	4300 5950 5700 5950
Wire Wire Line
	5150 6600 5950 6600
Wire Wire Line
	4550 6950 5950 6950
Wire Wire Line
	5050 7650 5950 7650
Wire Wire Line
	4950 2400 5950 2400
Wire Wire Line
	2100 1750 4250 1750
Wire Wire Line
	4250 1750 4250 2500
Wire Wire Line
	4250 2500 5600 2500
Wire Wire Line
	5600 2500 5600 3350
Wire Wire Line
	5600 3350 5950 3350
Wire Wire Line
	2100 1850 4200 1850
Wire Wire Line
	4200 1850 4200 2550
Wire Wire Line
	4200 2550 5550 2550
Wire Wire Line
	5550 2550 5550 4400
Wire Wire Line
	5550 4400 5950 4400
Wire Wire Line
	2100 1950 4150 1950
Wire Wire Line
	4150 1950 4150 2600
Wire Wire Line
	4150 2600 5500 2600
Wire Wire Line
	5500 2600 5500 5450
Wire Wire Line
	5500 5450 5950 5450
Wire Wire Line
	2100 2050 4100 2050
Wire Wire Line
	4100 2050 4100 2650
Wire Wire Line
	4100 2650 5450 2650
Wire Wire Line
	5450 2650 5450 6500
Wire Wire Line
	5450 6500 5950 6500
Wire Wire Line
	2100 2150 4050 2150
Wire Wire Line
	4050 2150 4050 2700
Wire Wire Line
	4050 2700 5400 2700
Wire Wire Line
	5400 2700 5400 7550
Wire Wire Line
	5400 7550 5950 7550
Wire Wire Line
	2100 1450 4250 1450
Wire Wire Line
	4250 1450 4250 1050
Wire Wire Line
	4250 1050 5850 1050
Wire Wire Line
	5850 1050 5850 950 
Wire Wire Line
	5850 950  5950 950 
Connection ~ 5850 1050
Wire Wire Line
	5850 1050 5950 1050
Wire Wire Line
	5850 1050 5850 2000
Wire Wire Line
	5850 2000 5950 2000
Wire Wire Line
	5850 2000 5850 2100
Wire Wire Line
	5850 2100 5950 2100
Connection ~ 5850 2000
Wire Wire Line
	5850 2100 5850 3050
Wire Wire Line
	5850 3050 5950 3050
Connection ~ 5850 2100
Wire Wire Line
	5850 3050 5850 3150
Wire Wire Line
	5850 3150 5950 3150
Connection ~ 5850 3050
Wire Wire Line
	5850 3150 5850 4100
Wire Wire Line
	5850 4100 5950 4100
Connection ~ 5850 3150
Wire Wire Line
	5850 4100 5850 4200
Wire Wire Line
	5850 4200 5950 4200
Connection ~ 5850 4100
Wire Wire Line
	5850 4200 5850 5150
Wire Wire Line
	5850 5150 5950 5150
Connection ~ 5850 4200
Wire Wire Line
	5850 5150 5850 5250
Wire Wire Line
	5850 5250 5950 5250
Connection ~ 5850 5150
Wire Wire Line
	5850 5250 5850 6200
Wire Wire Line
	5850 6200 5950 6200
Connection ~ 5850 5250
Wire Wire Line
	5850 6200 5850 6300
Wire Wire Line
	5850 6300 5950 6300
Connection ~ 5850 6200
Wire Wire Line
	5850 6300 5850 7250
Wire Wire Line
	5850 7250 5950 7250
Connection ~ 5850 6300
Wire Wire Line
	5850 7250 5850 7350
Wire Wire Line
	5850 7350 5950 7350
Connection ~ 5850 7250
Text GLabel 1850 1450 0    50   Input ~ 0
UART_TX
Text GLabel 1850 1550 0    50   Input ~ 0
STEP_MOTORA
Text GLabel 1850 1650 0    50   Input ~ 0
STEP_MOTORB
Text GLabel 1850 1750 0    50   Input ~ 0
STEP_MOTORC
Text GLabel 1850 1850 0    50   Input ~ 0
STEP_MOTORD
Text GLabel 1850 1950 0    50   Input ~ 0
STEP_MOTORE
Text GLabel 1850 2050 0    50   Input ~ 0
STEP_MOTORF
Text GLabel 1850 2150 0    50   Input ~ 0
STEP_MOTORG
Text GLabel 1850 2250 0    50   Input ~ 0
GND
$Comp
L Device:C_Small C2
U 1 1 61EA5D90
P 7300 1350
F 0 "C2" H 7392 1396 50  0000 L CNN
F 1 "0.1uF" H 7392 1305 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 7300 1350 50  0001 C CNN
F 3 "~" H 7300 1350 50  0001 C CNN
	1    7300 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 7650 6750 7650
Wire Wire Line
	6750 2400 6850 2400
Wire Wire Line
	6850 2400 6850 2500
Wire Wire Line
	6750 3450 6850 3450
Wire Wire Line
	6850 3450 6850 3550
Wire Wire Line
	6750 4500 6850 4500
Wire Wire Line
	6750 5550 6850 5550
Wire Wire Line
	6850 5550 6850 5650
Wire Wire Line
	6750 6600 6850 6600
Wire Wire Line
	6850 6600 6850 6700
$Comp
L Device:C_Small C3
U 1 1 61EE0DBA
P 7300 2400
F 0 "C3" H 7392 2446 50  0000 L CNN
F 1 "0.1uF" H 7392 2355 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 7300 2400 50  0001 C CNN
F 3 "~" H 7300 2400 50  0001 C CNN
	1    7300 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 61EE2EBD
P 7250 3450
F 0 "C4" H 7342 3496 50  0000 L CNN
F 1 "0.1uF" H 7342 3405 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 7250 3450 50  0001 C CNN
F 3 "~" H 7250 3450 50  0001 C CNN
	1    7250 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 61EE3484
P 7250 4500
F 0 "C5" H 7342 4546 50  0000 L CNN
F 1 "0.1uF" H 7342 4455 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 7250 4500 50  0001 C CNN
F 3 "~" H 7250 4500 50  0001 C CNN
	1    7250 4500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 61EE3A06
P 7250 5550
F 0 "C6" H 7342 5596 50  0000 L CNN
F 1 "0.1uF" H 7342 5505 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 7250 5550 50  0001 C CNN
F 3 "~" H 7250 5550 50  0001 C CNN
	1    7250 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 61EE4161
P 7250 6600
F 0 "C7" H 7342 6646 50  0000 L CNN
F 1 "0.1uF" H 7342 6555 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 7250 6600 50  0001 C CNN
F 3 "~" H 7250 6600 50  0001 C CNN
	1    7250 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 61EE486D
P 7250 7650
F 0 "C8" H 7342 7696 50  0000 L CNN
F 1 "0.1uF" H 7342 7605 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 7250 7650 50  0001 C CNN
F 3 "~" H 7250 7650 50  0001 C CNN
	1    7250 7650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2300 7100 2300
Wire Wire Line
	6750 3350 7100 3350
Wire Wire Line
	6750 4400 7100 4400
Wire Wire Line
	6850 4500 6850 4600
Wire Wire Line
	6750 5450 7100 5450
Wire Wire Line
	6750 6500 7100 6500
Wire Wire Line
	6750 7550 7100 7550
Wire Wire Line
	6850 7750 6850 7650
$Comp
L Connector:Screw_Terminal_01x05 J9
U 1 1 61F37830
P 8050 7250
F 0 "J9" H 8000 7650 50  0000 L CNN
F 1 "Screw_Terminal_01x05" H 8000 7550 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8050 7250 50  0001 C CNN
F 3 "~" H 8050 7250 50  0001 C CNN
	1    8050 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 7450 6750 7450
Wire Wire Line
	7850 7350 6750 7350
Wire Wire Line
	7850 7250 6750 7250
Wire Wire Line
	7850 7150 6750 7150
Text GLabel 8100 7350 2    50   Output ~ 0
MOTORG_COIL2A
Text GLabel 8100 7450 2    50   Output ~ 0
MOTORG_COIL2B
Text GLabel 8100 7150 2    50   Output ~ 0
MOTORG_COIL1A
Text GLabel 8100 7250 2    50   Output ~ 0
MOTORG_COIL1B
Text GLabel 8100 7050 2    50   UnSpc ~ 0
VMCC
$Comp
L Connector:Screw_Terminal_01x05 J8
U 1 1 61F9E7D7
P 8050 6200
F 0 "J8" H 8000 6600 50  0000 L CNN
F 1 "Screw_Terminal_01x05" H 8000 6500 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8050 6200 50  0001 C CNN
F 3 "~" H 8050 6200 50  0001 C CNN
	1    8050 6200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x05 J7
U 1 1 61F9F8B4
P 8050 5150
F 0 "J7" H 8000 5550 50  0000 L CNN
F 1 "Screw_Terminal_01x05" H 8000 5450 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8050 5150 50  0001 C CNN
F 3 "~" H 8050 5150 50  0001 C CNN
	1    8050 5150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x05 J6
U 1 1 61FA03C6
P 8050 4100
F 0 "J6" H 8000 4500 50  0000 L CNN
F 1 "Screw_Terminal_01x05" H 8000 4400 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8050 4100 50  0001 C CNN
F 3 "~" H 8050 4100 50  0001 C CNN
	1    8050 4100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x05 J5
U 1 1 61FA1173
P 8050 3050
F 0 "J5" H 8000 3450 50  0000 L CNN
F 1 "Screw_Terminal_01x05" H 8000 3350 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8050 3050 50  0001 C CNN
F 3 "~" H 8050 3050 50  0001 C CNN
	1    8050 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x05 J4
U 1 1 61FA1D3D
P 8050 2000
F 0 "J4" H 7900 2350 50  0000 L CNN
F 1 "Screw_Terminal_01x05" H 8000 2300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8050 2000 50  0001 C CNN
F 3 "~" H 8050 2000 50  0001 C CNN
	1    8050 2000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x05 J3
U 1 1 61FA293D
P 8050 950
F 0 "J3" H 8000 1350 50  0000 L CNN
F 1 "Screw_Terminal_01x05" H 8000 1250 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8050 950 50  0001 C CNN
F 3 "~" H 8050 950 50  0001 C CNN
	1    8050 950 
	1    0    0    -1  
$EndComp
Text GLabel 8100 6000 2    50   UnSpc ~ 0
VMCC
Text GLabel 8100 6100 2    50   Output ~ 0
MOTORF_COIL1A
Text GLabel 8100 6200 2    50   Output ~ 0
MOTORF_COIL1B
Text GLabel 8100 6300 2    50   Output ~ 0
MOTORF_COIL2A
Text GLabel 8100 6400 2    50   Output ~ 0
MOTORF_COIL2B
Text GLabel 8100 4950 2    50   UnSpc ~ 0
VMCC
Text GLabel 8100 5050 2    50   Output ~ 0
MOTORE_COIL1A
Text GLabel 8100 5150 2    50   Output ~ 0
MOTORE_COIL1B
Text GLabel 8100 5250 2    50   Output ~ 0
MOTORE_COIL2A
Text GLabel 8100 5350 2    50   Output ~ 0
MOTORE_COIL2B
Text GLabel 8100 3900 2    50   UnSpc ~ 0
VMCC
Text GLabel 8100 4000 2    50   Output ~ 0
MOTORD_COIL1A
Text GLabel 8100 4100 2    50   Output ~ 0
MOTORD_COIL1B
Text GLabel 8100 4200 2    50   Output ~ 0
MOTORD_COIL2A
Text GLabel 8100 4300 2    50   Output ~ 0
MOTORD_COIL2B
Text GLabel 8100 2850 2    50   UnSpc ~ 0
VMCC
Text GLabel 8100 2950 2    50   Output ~ 0
MOTORC_COIL1A
Text GLabel 8100 3050 2    50   Output ~ 0
MOTORC_COIL1B
Text GLabel 8100 3150 2    50   Output ~ 0
MOTORC_COIL2A
Text GLabel 8100 3250 2    50   Output ~ 0
MOTORC_COIL2B
Text GLabel 8100 1800 2    50   UnSpc ~ 0
VMCC
Text GLabel 8100 1900 2    50   Output ~ 0
MOTORB_COIL1A
Text GLabel 8100 2000 2    50   Output ~ 0
MOTORB_COIL1B
Text GLabel 8100 2100 2    50   Output ~ 0
MOTORB_COIL2A
Text GLabel 8100 2200 2    50   Output ~ 0
MOTORB_COIL2B
Text GLabel 8100 750  2    50   UnSpc ~ 0
VMCC
Text GLabel 8100 850  2    50   Output ~ 0
MOTORA_COIL1A
Text GLabel 8100 950  2    50   Output ~ 0
MOTORA_COIL1B
Text GLabel 8100 1050 2    50   Output ~ 0
MOTORA_COIL2A
Text GLabel 8100 1150 2    50   Output ~ 0
MOTORA_COIL2B
Wire Wire Line
	7850 6400 6750 6400
Wire Wire Line
	7850 6300 6750 6300
Wire Wire Line
	7850 6200 6750 6200
Wire Wire Line
	7850 6100 6750 6100
Wire Wire Line
	7850 5350 6750 5350
Wire Wire Line
	7850 5250 6750 5250
Wire Wire Line
	7850 5150 6750 5150
Wire Wire Line
	7850 5050 6750 5050
Wire Wire Line
	7850 4300 6750 4300
Wire Wire Line
	7850 4200 6750 4200
Wire Wire Line
	7850 4100 6750 4100
Wire Wire Line
	7850 4000 6750 4000
Wire Wire Line
	7850 3250 6750 3250
Wire Wire Line
	7850 3150 6750 3150
Wire Wire Line
	7850 3050 6750 3050
Wire Wire Line
	7850 2950 6750 2950
Wire Wire Line
	7850 2850 7700 2850
Wire Wire Line
	7700 2850 7700 2750
Wire Wire Line
	7700 2750 6750 2750
Wire Wire Line
	7850 2200 6750 2200
Wire Wire Line
	7850 2100 6750 2100
Wire Wire Line
	7850 2000 6750 2000
Wire Wire Line
	7850 1900 6750 1900
Wire Wire Line
	7850 1800 7700 1800
Wire Wire Line
	7700 1800 7700 1700
Wire Wire Line
	7700 1700 6750 1700
Wire Wire Line
	7850 1150 6750 1150
Wire Wire Line
	7850 1050 6750 1050
Wire Wire Line
	7850 950  6750 950 
Wire Wire Line
	7850 850  6750 850 
Wire Wire Line
	7850 750  7700 750 
Wire Wire Line
	7700 750  7700 650 
Wire Wire Line
	7700 650  6750 650 
$Comp
L Connector:Screw_Terminal_01x02 J10
U 1 1 621116AC
P 9650 1300
F 0 "J10" H 9600 1500 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 9600 1400 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 9650 1300 50  0001 C CNN
F 3 "~" H 9650 1300 50  0001 C CNN
	1    9650 1300
	1    0    0    -1  
$EndComp
Text GLabel 9700 1300 2    50   Input ~ 0
VMCC
Text GLabel 9700 1400 2    50   Input ~ 0
GND
$Comp
L Device:C_Small C10
U 1 1 62112D20
P 8750 1400
F 0 "C10" H 8842 1446 50  0000 L CNN
F 1 "0.1uF" H 8842 1355 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 8750 1400 50  0001 C CNN
F 3 "~" H 8750 1400 50  0001 C CNN
	1    8750 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C9
U 1 1 621136A1
P 8350 1400
F 0 "C9" H 8442 1446 50  0000 L CNN
F 1 "0.1uF" H 8442 1355 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 8350 1400 50  0001 C CNN
F 3 "~" H 8350 1400 50  0001 C CNN
	1    8350 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C11
U 1 1 62114BFB
P 9200 1450
F 0 "C11" H 9250 1550 50  0000 L CNN
F 1 "100uF" H 9200 1350 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 9238 1300 50  0001 C CNN
F 3 "~" H 9200 1450 50  0001 C CNN
	1    9200 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 1300 9200 1300
Wire Wire Line
	9200 1300 8750 1300
Connection ~ 9200 1300
Wire Wire Line
	8750 1300 8350 1300
Connection ~ 8750 1300
Wire Wire Line
	8350 1300 7700 1300
Wire Wire Line
	7700 1300 7700 750 
Connection ~ 8350 1300
Connection ~ 7700 750 
Wire Wire Line
	7700 1300 7700 1700
Connection ~ 7700 1300
Connection ~ 7700 1700
Wire Wire Line
	7700 1800 7700 2750
Connection ~ 7700 1800
Connection ~ 7700 2750
Wire Wire Line
	7700 2850 7700 3800
Connection ~ 7700 2850
Wire Wire Line
	7700 3800 6750 3800
Wire Wire Line
	7700 3800 7700 3900
Wire Wire Line
	7700 3900 7850 3900
Connection ~ 7700 3800
Wire Wire Line
	7700 3900 7700 4850
Connection ~ 7700 3900
Connection ~ 7700 4850
Wire Wire Line
	7700 4850 6750 4850
Wire Wire Line
	7700 4850 7700 4950
Connection ~ 7700 5900
Wire Wire Line
	7700 5900 6750 5900
Wire Wire Line
	7700 5900 7700 6000
Wire Wire Line
	7700 6950 6750 6950
Wire Wire Line
	7700 4950 7850 4950
Connection ~ 7700 4950
Wire Wire Line
	7700 4950 7700 5900
Wire Wire Line
	7700 6000 7850 6000
Connection ~ 7700 6000
Wire Wire Line
	7700 6000 7700 6950
Wire Wire Line
	7700 6950 7700 7050
Wire Wire Line
	7700 7050 7850 7050
Connection ~ 7700 6950
Wire Wire Line
	9450 1400 9450 1600
Wire Wire Line
	9450 1600 9200 1600
Wire Wire Line
	9200 1600 8750 1600
Wire Wire Line
	8750 1600 8750 1500
Connection ~ 9200 1600
Wire Wire Line
	8750 1600 8350 1600
Wire Wire Line
	8350 1600 8350 1500
Connection ~ 8750 1600
$Comp
L power:GNDREF #PWR03
U 1 1 61D71FD8
P 3350 6650
F 0 "#PWR03" H 3350 6400 50  0001 C CNN
F 1 "GNDREF" H 3355 6477 50  0000 C CNN
F 2 "" H 3350 6650 50  0001 C CNN
F 3 "" H 3350 6650 50  0001 C CNN
	1    3350 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 6050 3350 6400
Wire Wire Line
	3350 6050 3500 6050
Wire Wire Line
	3900 6650 3350 6650
Connection ~ 3350 6650
Wire Wire Line
	2700 3250 2850 3250
$Comp
L power:GNDREF #PWR04
U 1 1 61DA3EC6
P 3500 4950
F 0 "#PWR04" H 3500 4700 50  0001 C CNN
F 1 "GNDREF" H 3505 4777 50  0000 C CNN
F 2 "" H 3500 4950 50  0001 C CNN
F 3 "" H 3500 4950 50  0001 C CNN
	1    3500 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5200 3500 5450
Wire Wire Line
	4300 4750 4300 5200
Wire Wire Line
	3900 4950 3500 4950
Wire Wire Line
	3500 4450 3500 4800
Connection ~ 3500 4950
$Comp
L Device:C_Small C12
U 1 1 61E02363
P 3100 4800
F 0 "C12" H 3192 4846 50  0000 L CNN
F 1 "0.1uF" H 3192 4755 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm" H 3100 4800 50  0001 C CNN
F 3 "~" H 3100 4800 50  0001 C CNN
	1    3100 4800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3200 4800 3500 4800
Connection ~ 3500 4800
Wire Wire Line
	3500 4800 3500 4950
Wire Wire Line
	3000 4800 2850 4800
Wire Wire Line
	2850 4800 2850 3250
Connection ~ 2850 3250
Wire Wire Line
	2850 3250 3300 3250
Wire Wire Line
	3350 6400 3200 6400
Connection ~ 3350 6400
Wire Wire Line
	3350 6400 3350 6650
Wire Wire Line
	3000 6400 2850 6400
Wire Wire Line
	2850 6400 2850 4800
Connection ~ 2850 4800
$Comp
L power:GNDREF #PWR02
U 1 1 61E45235
P 2300 4350
F 0 "#PWR02" H 2300 4100 50  0001 C CNN
F 1 "GNDREF" H 2305 4177 50  0000 C CNN
F 2 "" H 2300 4350 50  0001 C CNN
F 3 "" H 2300 4350 50  0001 C CNN
	1    2300 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3950 2300 3950
Wire Wire Line
	2300 3950 2300 4350
$Comp
L power:GNDREF #PWR01
U 1 1 61E5729B
P 2300 2500
F 0 "#PWR01" H 2300 2250 50  0001 C CNN
F 1 "GNDREF" H 2305 2327 50  0000 C CNN
F 2 "" H 2300 2500 50  0001 C CNN
F 3 "" H 2300 2500 50  0001 C CNN
	1    2300 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 2250 2300 2250
Wire Wire Line
	2300 2250 2300 2500
$Comp
L power:GNDREF #PWR019
U 1 1 61E6AB86
P 9550 1700
F 0 "#PWR019" H 9550 1450 50  0001 C CNN
F 1 "GNDREF" H 9555 1527 50  0000 C CNN
F 2 "" H 9550 1700 50  0001 C CNN
F 3 "" H 9550 1700 50  0001 C CNN
	1    9550 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 1600 9550 1600
Connection ~ 9450 1600
Wire Wire Line
	9550 1600 9550 1700
$Comp
L power:GNDREF #PWR05
U 1 1 61E8D7A5
P 6900 750
F 0 "#PWR05" H 6900 500 50  0001 C CNN
F 1 "GNDREF" V 6905 622 50  0000 R CNN
F 2 "" H 6900 750 50  0001 C CNN
F 3 "" H 6900 750 50  0001 C CNN
	1    6900 750 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6750 750  6900 750 
$Comp
L power:GNDREF #PWR06
U 1 1 61E9DF16
P 6900 1800
F 0 "#PWR06" H 6900 1550 50  0001 C CNN
F 1 "GNDREF" V 6905 1672 50  0000 R CNN
F 2 "" H 6900 1800 50  0001 C CNN
F 3 "" H 6900 1800 50  0001 C CNN
	1    6900 1800
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR07
U 1 1 61E9E75F
P 6900 2850
F 0 "#PWR07" H 6900 2600 50  0001 C CNN
F 1 "GNDREF" V 6905 2722 50  0000 R CNN
F 2 "" H 6900 2850 50  0001 C CNN
F 3 "" H 6900 2850 50  0001 C CNN
	1    6900 2850
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR08
U 1 1 61E9EE00
P 6900 3900
F 0 "#PWR08" H 6900 3650 50  0001 C CNN
F 1 "GNDREF" V 6905 3772 50  0000 R CNN
F 2 "" H 6900 3900 50  0001 C CNN
F 3 "" H 6900 3900 50  0001 C CNN
	1    6900 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR09
U 1 1 61E9F5BE
P 6900 4950
F 0 "#PWR09" H 6900 4700 50  0001 C CNN
F 1 "GNDREF" V 6905 4822 50  0000 R CNN
F 2 "" H 6900 4950 50  0001 C CNN
F 3 "" H 6900 4950 50  0001 C CNN
	1    6900 4950
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR010
U 1 1 61E9FA17
P 6900 6000
F 0 "#PWR010" H 6900 5750 50  0001 C CNN
F 1 "GNDREF" V 6905 5872 50  0000 R CNN
F 2 "" H 6900 6000 50  0001 C CNN
F 3 "" H 6900 6000 50  0001 C CNN
	1    6900 6000
	0    -1   -1   0   
$EndComp
$Comp
L power:GNDREF #PWR011
U 1 1 61E9FE6C
P 6900 7050
F 0 "#PWR011" H 6900 6800 50  0001 C CNN
F 1 "GNDREF" V 6905 6922 50  0000 R CNN
F 2 "" H 6900 7050 50  0001 C CNN
F 3 "" H 6900 7050 50  0001 C CNN
	1    6900 7050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6750 1800 6900 1800
Wire Wire Line
	6750 2850 6900 2850
Wire Wire Line
	6750 3900 6900 3900
Wire Wire Line
	6750 4950 6900 4950
Wire Wire Line
	6750 6000 6900 6000
Wire Wire Line
	6750 7050 6900 7050
$Comp
L power:GNDREF #PWR012
U 1 1 61F0170D
P 6950 1450
F 0 "#PWR012" H 6950 1200 50  0001 C CNN
F 1 "GNDREF" H 6955 1277 50  0000 C CNN
F 2 "" H 6950 1450 50  0001 C CNN
F 3 "" H 6950 1450 50  0001 C CNN
	1    6950 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 1350 6950 1450
Wire Wire Line
	6750 1350 6950 1350
$Comp
L power:GNDREF #PWR013
U 1 1 61F23A06
P 6950 2500
F 0 "#PWR013" H 6950 2250 50  0001 C CNN
F 1 "GNDREF" H 6955 2327 50  0000 C CNN
F 2 "" H 6950 2500 50  0001 C CNN
F 3 "" H 6950 2500 50  0001 C CNN
	1    6950 2500
	1    0    0    -1  
$EndComp
Connection ~ 6950 2500
Wire Wire Line
	6950 2500 6850 2500
$Comp
L power:GNDREF #PWR014
U 1 1 61F24B94
P 6950 3550
F 0 "#PWR014" H 6950 3300 50  0001 C CNN
F 1 "GNDREF" H 6955 3377 50  0000 C CNN
F 2 "" H 6950 3550 50  0001 C CNN
F 3 "" H 6950 3550 50  0001 C CNN
	1    6950 3550
	1    0    0    -1  
$EndComp
Connection ~ 6950 3550
Wire Wire Line
	6950 3550 6850 3550
$Comp
L power:GNDREF #PWR015
U 1 1 61F25047
P 6950 4600
F 0 "#PWR015" H 6950 4350 50  0001 C CNN
F 1 "GNDREF" H 6955 4427 50  0000 C CNN
F 2 "" H 6950 4600 50  0001 C CNN
F 3 "" H 6950 4600 50  0001 C CNN
	1    6950 4600
	1    0    0    -1  
$EndComp
Connection ~ 6950 4600
Wire Wire Line
	6950 4600 6850 4600
$Comp
L power:GNDREF #PWR016
U 1 1 61F2541F
P 6950 5650
F 0 "#PWR016" H 6950 5400 50  0001 C CNN
F 1 "GNDREF" H 6955 5477 50  0000 C CNN
F 2 "" H 6950 5650 50  0001 C CNN
F 3 "" H 6950 5650 50  0001 C CNN
	1    6950 5650
	1    0    0    -1  
$EndComp
Connection ~ 6950 5650
Wire Wire Line
	6950 5650 6850 5650
$Comp
L power:GNDREF #PWR017
U 1 1 61F25867
P 6950 6700
F 0 "#PWR017" H 6950 6450 50  0001 C CNN
F 1 "GNDREF" H 6955 6527 50  0000 C CNN
F 2 "" H 6950 6700 50  0001 C CNN
F 3 "" H 6950 6700 50  0001 C CNN
	1    6950 6700
	1    0    0    -1  
$EndComp
Connection ~ 6950 6700
Wire Wire Line
	6950 6700 6850 6700
$Comp
L power:GNDREF #PWR018
U 1 1 61F25BC2
P 6950 7750
F 0 "#PWR018" H 6950 7500 50  0001 C CNN
F 1 "GNDREF" H 6955 7577 50  0000 C CNN
F 2 "" H 6950 7750 50  0001 C CNN
F 3 "" H 6950 7750 50  0001 C CNN
	1    6950 7750
	1    0    0    -1  
$EndComp
Connection ~ 6950 7750
Wire Wire Line
	6950 7750 6850 7750
Wire Wire Line
	2100 3850 2400 3850
$Comp
L power:VCC #PWR020
U 1 1 61D2CA8A
P 2400 3700
F 0 "#PWR020" H 2400 3550 50  0001 C CNN
F 1 "VCC" H 2415 3873 50  0000 C CNN
F 2 "" H 2400 3700 50  0001 C CNN
F 3 "" H 2400 3700 50  0001 C CNN
	1    2400 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 3700 2400 3850
Connection ~ 2400 3850
Wire Wire Line
	2400 3850 2700 3850
$Comp
L power:VCC #PWR021
U 1 1 61D52951
P 7100 1250
F 0 "#PWR021" H 7100 1100 50  0001 C CNN
F 1 "VCC" H 7115 1423 50  0000 C CNN
F 2 "" H 7100 1250 50  0001 C CNN
F 3 "" H 7100 1250 50  0001 C CNN
	1    7100 1250
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR022
U 1 1 61D8F0C9
P 7100 2300
F 0 "#PWR022" H 7100 2150 50  0001 C CNN
F 1 "VCC" H 7115 2473 50  0000 C CNN
F 2 "" H 7100 2300 50  0001 C CNN
F 3 "" H 7100 2300 50  0001 C CNN
	1    7100 2300
	1    0    0    -1  
$EndComp
Connection ~ 7100 2300
$Comp
L power:VCC #PWR023
U 1 1 61D8F855
P 7100 3350
F 0 "#PWR023" H 7100 3200 50  0001 C CNN
F 1 "VCC" H 7115 3523 50  0000 C CNN
F 2 "" H 7100 3350 50  0001 C CNN
F 3 "" H 7100 3350 50  0001 C CNN
	1    7100 3350
	1    0    0    -1  
$EndComp
Connection ~ 7100 3350
$Comp
L power:VCC #PWR024
U 1 1 61D9004D
P 7100 4400
F 0 "#PWR024" H 7100 4250 50  0001 C CNN
F 1 "VCC" H 7115 4573 50  0000 C CNN
F 2 "" H 7100 4400 50  0001 C CNN
F 3 "" H 7100 4400 50  0001 C CNN
	1    7100 4400
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR025
U 1 1 61D904DD
P 7100 5450
F 0 "#PWR025" H 7100 5300 50  0001 C CNN
F 1 "VCC" H 7115 5623 50  0000 C CNN
F 2 "" H 7100 5450 50  0001 C CNN
F 3 "" H 7100 5450 50  0001 C CNN
	1    7100 5450
	1    0    0    -1  
$EndComp
Connection ~ 7100 5450
$Comp
L power:VCC #PWR026
U 1 1 61D90B3B
P 7100 6500
F 0 "#PWR026" H 7100 6350 50  0001 C CNN
F 1 "VCC" H 7115 6673 50  0000 C CNN
F 2 "" H 7100 6500 50  0001 C CNN
F 3 "" H 7100 6500 50  0001 C CNN
	1    7100 6500
	1    0    0    -1  
$EndComp
Connection ~ 7100 6500
$Comp
L power:VCC #PWR027
U 1 1 61D91022
P 7100 7550
F 0 "#PWR027" H 7100 7400 50  0001 C CNN
F 1 "VCC" H 7115 7723 50  0000 C CNN
F 2 "" H 7100 7550 50  0001 C CNN
F 3 "" H 7100 7550 50  0001 C CNN
	1    7100 7550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 2300 7300 2300
Wire Wire Line
	6950 2500 7300 2500
Wire Wire Line
	6750 1250 7100 1250
Wire Wire Line
	7100 1250 7300 1250
Connection ~ 7100 1250
Wire Wire Line
	7300 1450 6950 1450
Connection ~ 6950 1450
Wire Wire Line
	7100 3350 7250 3350
Wire Wire Line
	6950 3550 7250 3550
Wire Wire Line
	7100 4400 7250 4400
Connection ~ 7100 4400
Wire Wire Line
	6950 4600 7250 4600
Wire Wire Line
	7100 5450 7250 5450
Wire Wire Line
	6950 5650 7250 5650
Wire Wire Line
	7100 6500 7250 6500
Wire Wire Line
	6950 6700 7250 6700
Wire Wire Line
	7100 7550 7250 7550
Connection ~ 7100 7550
Wire Wire Line
	6950 7750 7250 7750
$EndSCHEMATC
