EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "Blinds Control Schematic"
Date "2020-01-01"
Rev "V2"
Comp ""
Comment1 "uses NodeMCU"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ESP8266:NodeMCU_1.0_(ESP-12E) U2
U 1 1 5E0E21D9
P 5000 2950
F 0 "U2" H 5000 1863 60  0000 C CNN
F 1 "NodeMCU_1.0_(ESP-12E)" H 5000 1969 60  0000 C CNN
F 2 "" H 4400 2100 60  0000 C CNN
F 3 "" H 4400 2100 60  0000 C CNN
	1    5000 2950
	-1   0    0    1   
$EndComp
$Comp
L 74xx:74HC595 U4
U 1 1 5E0E7473
P 7250 3700
F 0 "U4" H 7250 4481 50  0000 C CNN
F 1 "74HC595" H 7250 4390 50  0000 C CNN
F 2 "" H 7250 3700 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 7250 3700 50  0001 C CNN
	1    7250 3700
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U3
U 1 1 5E0E792D
P 7250 2000
F 0 "U3" H 7250 2781 50  0000 C CNN
F 1 "74HC595" H 7250 2690 50  0000 C CNN
F 2 "" H 7250 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 7250 2000 50  0001 C CNN
	1    7250 2000
	1    0    0    -1  
$EndComp
$Comp
L Kip_Custom_Lib:JBtekPowerReg PR1
U 1 1 5E0D3CC2
P 3300 1500
F 0 "PR1" H 3083 1615 50  0000 C CNN
F 1 "JBtekPowerReg" H 3083 1524 50  0000 C CNN
F 2 "" H 3300 1500 50  0001 C CNN
F 3 "" H 3300 1500 50  0001 C CNN
	1    3300 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5E0E2039
P 3300 4550
F 0 "#PWR01" H 3300 4300 50  0001 C CNN
F 1 "GND" H 3305 4377 50  0000 C CNN
F 2 "" H 3300 4550 50  0001 C CNN
F 3 "" H 3300 4550 50  0001 C CNN
	1    3300 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1950 3300 2100
Wire Wire Line
	4200 2350 3650 2350
Wire Wire Line
	3650 2350 3650 2100
Wire Wire Line
	3650 2100 3300 2100
Connection ~ 3300 2100
Wire Wire Line
	3300 2100 3300 2650
Wire Wire Line
	3750 1800 3750 2250
Wire Wire Line
	4200 3050 3300 3050
Connection ~ 3300 3050
Wire Wire Line
	3300 3050 3300 3250
$Comp
L Device:R R1
U 1 1 5E0E87FE
P 3600 2650
F 0 "R1" V 3500 2600 50  0000 L CNN
F 1 "10K" V 3600 2600 50  0000 L CNN
F 2 "" V 3530 2650 50  0001 C CNN
F 3 "~" H 3600 2650 50  0001 C CNN
	1    3600 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5E0E9C84
P 3600 2850
F 0 "R2" V 3500 2800 50  0000 L CNN
F 1 "10K" V 3600 2800 50  0000 L CNN
F 2 "" V 3530 2850 50  0001 C CNN
F 3 "~" H 3600 2850 50  0001 C CNN
	1    3600 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5E0E9ECB
P 3600 3650
F 0 "R3" V 3500 3600 50  0000 L CNN
F 1 "10K" V 3600 3600 50  0000 L CNN
F 2 "" V 3530 3650 50  0001 C CNN
F 3 "~" H 3600 3650 50  0001 C CNN
	1    3600 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	3750 2650 3800 2650
Wire Wire Line
	3450 2650 3300 2650
Connection ~ 3300 2650
Wire Wire Line
	3300 2650 3300 2850
Wire Wire Line
	3800 2500 3800 2650
Connection ~ 3800 2650
Wire Wire Line
	3800 2650 4200 2650
Wire Wire Line
	3750 2850 3800 2850
Wire Wire Line
	3450 2850 3300 2850
Connection ~ 3300 2850
Wire Wire Line
	3300 2850 3300 3050
Wire Wire Line
	3800 2950 3800 2850
Connection ~ 3800 2850
Wire Wire Line
	3800 2850 4200 2850
Wire Wire Line
	3450 3650 3300 3650
Connection ~ 3300 3650
Wire Wire Line
	3750 3650 3800 3650
Wire Wire Line
	3800 3450 3800 3650
Connection ~ 3800 3650
Wire Wire Line
	3800 3650 4200 3650
Connection ~ 3300 4400
Wire Wire Line
	3300 4400 3300 4550
Wire Wire Line
	7250 1400 6650 1400
Wire Wire Line
	6650 1400 6650 1900
Wire Wire Line
	6650 1900 6850 1900
Wire Wire Line
	7250 3100 6650 3100
Wire Wire Line
	6650 3100 6650 3600
Wire Wire Line
	6650 3600 6850 3600
Wire Wire Line
	6850 1600 4100 1600
Wire Wire Line
	4100 1600 4100 2750
Wire Wire Line
	4100 2750 4200 2750
Wire Wire Line
	3300 1800 3500 1800
Wire Wire Line
	6650 1400 3750 1400
Wire Wire Line
	3750 1400 3750 1800
Connection ~ 6650 1400
Connection ~ 3750 1800
Text Label 4350 1400 0    50   ~ 0
3V3
Wire Wire Line
	6650 1900 6650 3100
Connection ~ 6650 1900
Connection ~ 6650 3100
Wire Wire Line
	5800 2750 6450 2750
Wire Wire Line
	7250 2750 7250 2700
Wire Wire Line
	6450 2750 6450 3900
Wire Wire Line
	6450 4400 7250 4400
Connection ~ 6450 2750
Wire Wire Line
	6450 2750 7250 2750
Wire Wire Line
	6850 2200 6450 2200
Wire Wire Line
	6450 2200 6450 2750
Wire Wire Line
	6850 3900 6450 3900
Connection ~ 6450 3900
Wire Wire Line
	6450 3900 6450 4400
Wire Wire Line
	4200 3450 4000 3450
Wire Wire Line
	4000 3450 4000 1750
Wire Wire Line
	4000 1750 6200 1750
Wire Wire Line
	6200 1750 6200 2100
Wire Wire Line
	6200 2100 6850 2100
Wire Wire Line
	6850 3800 6200 3800
Wire Wire Line
	6200 3800 6200 2100
Connection ~ 6200 2100
Wire Wire Line
	4200 2950 3900 2950
Wire Wire Line
	3900 2950 3900 1500
Wire Wire Line
	3900 1500 6350 1500
Wire Wire Line
	6450 1500 6450 1800
Wire Wire Line
	6450 1800 6850 1800
Wire Wire Line
	7650 2500 7650 2800
Wire Wire Line
	7650 2800 6550 2800
Wire Wire Line
	6550 2800 6550 3300
Wire Wire Line
	6550 3300 6850 3300
Wire Wire Line
	6850 3500 6350 3500
Wire Wire Line
	6350 3500 6350 1500
Connection ~ 6350 1500
Wire Wire Line
	6350 1500 6450 1500
NoConn ~ 4200 2450
NoConn ~ 4200 2550
NoConn ~ 4200 3250
NoConn ~ 5800 3650
NoConn ~ 5800 3550
NoConn ~ 5800 3450
NoConn ~ 5800 3350
NoConn ~ 5800 3250
NoConn ~ 5800 3150
NoConn ~ 5800 3050
NoConn ~ 5800 2950
NoConn ~ 5800 2850
NoConn ~ 5800 2250
Wire Wire Line
	3750 2250 4200 2250
NoConn ~ 4200 3150
NoConn ~ 5800 2350
NoConn ~ 5800 2450
NoConn ~ 5800 2550
NoConn ~ 5800 2650
NoConn ~ 3300 1650
Wire Wire Line
	2750 1650 2600 1650
Text Label 2350 1650 0    50   ~ 0
12V
NoConn ~ 7650 3700
NoConn ~ 7650 3800
NoConn ~ 7650 3900
NoConn ~ 7650 4000
$Comp
L Connector_Generic:Conn_2Rows-07Pins P?
U 1 1 5E1245C8
P 1500 5600
F 0 "P?" H 1550 5800 50  0000 C CNN
F 1 " " H 1550 5826 50  0000 C CNN
F 2 "" H 1500 5600 50  0001 C CNN
F 3 "~" H 1500 5600 50  0001 C CNN
	1    1500 5600
	-1   0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW?
U 1 1 5E130019
P 2450 5600
F 0 "SW?" H 2450 5885 50  0000 C CNN
F 1 "CLOSE" H 2450 5794 50  0000 C CNN
F 2 "" H 2450 5800 50  0001 C CNN
F 3 "~" H 2450 5800 50  0001 C CNN
	1    2450 5600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW?
U 1 1 5E139219
P 2450 5150
F 0 "SW?" H 2450 5435 50  0000 C CNN
F 1 "OPEN" H 2450 5344 50  0000 C CNN
F 2 "" H 2450 5350 50  0001 C CNN
F 3 "~" H 2450 5350 50  0001 C CNN
	1    2450 5150
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW?
U 1 1 5E139B73
P 2450 6050
F 0 "SW?" H 2450 6335 50  0000 C CNN
F 1 "CAL" H 2450 6244 50  0000 C CNN
F 2 "" H 2450 6250 50  0001 C CNN
F 3 "~" H 2450 6250 50  0001 C CNN
	1    2450 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 5600 2650 5150
Wire Wire Line
	2650 6050 2650 6200
Wire Wire Line
	2650 6200 1200 6200
Wire Wire Line
	1200 6200 1200 5700
Connection ~ 2650 6050
Connection ~ 2650 5600
Wire Wire Line
	2650 5600 2650 6050
Wire Wire Line
	2250 5150 1800 5150
Wire Wire Line
	1800 5150 1800 5500
Wire Wire Line
	1800 5500 1700 5500
Wire Wire Line
	2250 5600 1700 5600
Wire Wire Line
	2250 6050 1100 6050
Wire Wire Line
	1100 6050 1100 5500
Wire Wire Line
	1100 5500 1200 5500
$Comp
L Device:LED D?
U 1 1 5E16A5E8
P 3000 5100
F 0 "D?" V 2900 4850 50  0000 C CNN
F 1 "GREEN" V 3000 4850 50  0000 C CNN
F 2 "" H 3000 5100 50  0001 C CNN
F 3 "~" H 3000 5100 50  0001 C CNN
	1    3000 5100
	0    -1   1    0   
$EndComp
Wire Wire Line
	3000 4950 2750 4950
Wire Wire Line
	1700 6900 1700 5800
Wire Wire Line
	2750 4950 2750 5400
$Comp
L power:GND #PWR?
U 1 1 5E18CBF4
P 2150 7000
F 0 "#PWR?" H 2150 6750 50  0001 C CNN
F 1 "GND" H 2155 6827 50  0000 C CNN
F 2 "" H 2150 7000 50  0001 C CNN
F 3 "" H 2150 7000 50  0001 C CNN
	1    2150 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6900 2150 6900
Wire Wire Line
	2150 7000 2150 6900
$Comp
L Device:R R?
U 1 1 5E1927BF
P 3500 5700
F 0 "R?" V 3400 5650 50  0000 L CNN
F 1 "200K" V 3500 5600 50  0000 L CNN
F 2 "" V 3430 5700 50  0001 C CNN
F 3 "~" H 3500 5700 50  0001 C CNN
	1    3500 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	2150 6900 2500 6900
Connection ~ 2150 6900
$Comp
L Device:LED D?
U 1 1 5E1BBD85
P 3000 5550
F 0 "D?" V 2900 5300 50  0000 C CNN
F 1 "RED" V 3000 5300 50  0000 C CNN
F 2 "" H 3000 5550 50  0001 C CNN
F 3 "~" H 3000 5550 50  0001 C CNN
	1    3000 5550
	0    -1   1    0   
$EndComp
$Comp
L Device:LED D?
U 1 1 5E1BC2BF
P 3000 6000
F 0 "D?" V 2900 5750 50  0000 C CNN
F 1 "BLUE" V 3000 5750 50  0000 C CNN
F 2 "" H 3000 6000 50  0001 C CNN
F 3 "~" H 3000 6000 50  0001 C CNN
	1    3000 6000
	0    -1   1    0   
$EndComp
Wire Wire Line
	3000 5400 2750 5400
Connection ~ 2750 5400
Wire Wire Line
	2750 5400 2750 6900
$Comp
L Device:R R?
U 1 1 5E1DD847
P 3500 5250
F 0 "R?" V 3400 5200 50  0000 L CNN
F 1 "4K3" V 3500 5150 50  0000 L CNN
F 2 "" V 3430 5250 50  0001 C CNN
F 3 "~" H 3500 5250 50  0001 C CNN
	1    3500 5250
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 5E1DE06A
P 3500 6150
F 0 "R?" V 3400 6100 50  0000 L CNN
F 1 "100K" V 3500 6050 50  0000 L CNN
F 2 "" V 3430 6150 50  0001 C CNN
F 3 "~" H 3500 6150 50  0001 C CNN
	1    3500 6150
	0    1    1    0   
$EndComp
Wire Wire Line
	3000 5250 3350 5250
Wire Wire Line
	3000 5700 3350 5700
Wire Wire Line
	3000 6150 3350 6150
Wire Wire Line
	1200 5600 1000 5600
Wire Wire Line
	1000 5600 1000 6300
Wire Wire Line
	1000 6300 3650 6300
Wire Wire Line
	3650 6300 3650 6150
Wire Wire Line
	3650 5700 3650 6150
Connection ~ 3650 6150
Wire Wire Line
	3650 5250 3650 5700
Connection ~ 3650 5700
Text Label 1000 6000 1    50   ~ 0
+12V
$Comp
L Transistor_BJT:2N3904 Q?
U 1 1 5E201517
P 2400 6600
F 0 "Q?" H 2250 6700 50  0000 L CNN
F 1 "2N3904" H 2100 6450 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2600 6525 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 2400 6600 50  0001 L CNN
	1    2400 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 6400 2850 6400
Wire Wire Line
	2850 6400 2850 5850
Wire Wire Line
	2850 5850 3000 5850
Wire Wire Line
	2500 6800 2500 6900
Connection ~ 2500 6900
Wire Wire Line
	2500 6900 2750 6900
$Comp
L Device:R R?
U 1 1 5E26FDB3
P 2050 6600
F 0 "R?" V 1950 6550 50  0000 L CNN
F 1 "200K" V 2050 6500 50  0000 L CNN
F 2 "" V 1980 6600 50  0001 C CNN
F 3 "~" H 2050 6600 50  0001 C CNN
	1    2050 6600
	0    1    1    0   
$EndComp
Wire Wire Line
	1900 6600 1900 5700
Wire Wire Line
	1900 5700 1700 5700
$Comp
L Connector_Generic:Conn_2Rows-07Pins J?
U 1 1 5E27D590
P 1650 3050
F 0 "J?" H 1700 3250 50  0000 C CNN
F 1 " " H 1700 3276 50  0000 C CNN
F 2 "" H 1650 3050 50  0001 C CNN
F 3 "~" H 1650 3050 50  0001 C CNN
	1    1650 3050
	-1   0    0    -1  
$EndComp
Text Label 2150 6200 0    50   ~ 0
3V3
Wire Wire Line
	3000 2500 3800 2500
Wire Wire Line
	1850 2950 3800 2950
Wire Wire Line
	1850 3050 3000 3050
Wire Wire Line
	3000 3050 3000 2500
Wire Wire Line
	1350 2950 1200 2950
Wire Wire Line
	1200 2950 1200 3450
Text Label 2400 2650 1    50   ~ 0
3V3
Wire Wire Line
	2400 2300 2400 3350
Wire Wire Line
	1200 3450 3800 3450
Wire Wire Line
	2400 2300 3500 2300
Wire Wire Line
	3500 2300 3500 1800
Connection ~ 3500 1800
Wire Wire Line
	3500 1800 3750 1800
Wire Wire Line
	1350 3150 1350 3350
Wire Wire Line
	1350 3350 2400 3350
Wire Wire Line
	1350 2100 2600 2100
Wire Wire Line
	2600 2100 2600 1650
Connection ~ 2600 1650
Wire Wire Line
	2600 1650 2000 1650
Wire Wire Line
	1350 2100 1350 3050
Wire Wire Line
	4200 3550 3900 3550
Wire Wire Line
	3900 3550 3900 3150
Wire Wire Line
	3900 3150 1850 3150
Wire Wire Line
	1850 3250 3300 3250
Connection ~ 3300 3250
Wire Wire Line
	3300 3250 3300 3650
Wire Wire Line
	3300 3650 3300 4400
Text Label 1350 2550 1    50   ~ 0
12V
$Comp
L Kip_Custom_Lib:TSOP38238 U?
U 1 1 5E3181D9
P 5500 5750
F 0 "U?" H 5558 6347 60  0000 C CNN
F 1 "TSOP38238" H 5558 6241 60  0000 C CNN
F 2 "" H 5500 5750 50  0001 C CNN
F 3 "" H 5500 5750 50  0001 C CNN
	1    5500 5750
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Male P?
U 1 1 5E331E30
P 4600 5750
F 0 "P?" H 4700 6000 50  0000 C CNN
F 1 " " H 4708 5940 50  0000 C CNN
F 2 "" H 4600 5750 50  0001 C CNN
F 3 "~" H 4600 5750 50  0001 C CNN
	1    4600 5750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 5E1688B2
P 4500 5750
F 0 "J?" H 4350 6000 50  0000 L CNN
F 1 " " H 4528 5685 50  0000 L CNN
F 2 "" H 4500 5750 50  0001 C CNN
F 3 "~" H 4500 5750 50  0001 C CNN
	1    4500 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5650 5000 5650
Wire Wire Line
	5000 5650 5000 5750
Wire Wire Line
	5000 5750 5100 5750
Wire Wire Line
	4800 5750 4950 5750
Wire Wire Line
	4950 5750 4950 5400
Wire Wire Line
	4950 5400 5100 5400
Wire Wire Line
	4800 5850 5000 5850
Wire Wire Line
	5000 5850 5000 6100
Wire Wire Line
	5000 6100 5100 6100
Wire Wire Line
	4200 3350 4100 3350
Wire Wire Line
	4100 3350 4100 5750
Wire Wire Line
	4100 5750 4300 5750
Wire Wire Line
	4300 5650 4000 5650
Wire Wire Line
	4000 5650 4000 4050
Wire Wire Line
	4300 5850 3900 5850
Wire Wire Line
	3900 5850 3900 4400
Wire Wire Line
	3900 4400 3300 4400
Wire Wire Line
	4000 4050 2400 4050
Wire Wire Line
	2400 4050 2400 3350
Connection ~ 2400 3350
$Comp
L Motor:Stepper_Motor_bipolar M?
U 1 1 5E1E386D
P 9550 2300
F 0 "M?" H 9750 2350 50  0000 L CNN
F 1 "Window 1" H 9750 2250 50  0000 L CNN
F 2 "" H 9560 2290 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 9560 2290 50  0001 C CNN
	1    9550 2300
	1    0    0    -1  
$EndComp
$Comp
L ProjectCustom:SBT0811 U?
U 1 1 5E1E7C3C
P 8950 1600
F 0 "U?" H 8925 2165 50  0000 C CNN
F 1 "SBT0811" H 8925 2074 50  0000 C CNN
F 2 "" H 8950 1700 50  0001 C CNN
F 3 "" H 8950 1700 50  0001 C CNN
	1    8950 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 2100 9150 2200
Wire Wire Line
	9150 2200 9250 2200
$Comp
L ProjectCustom:SBT0811 U?
U 1 1 5E22FDCE
P 8950 3100
F 0 "U?" H 8925 3665 50  0000 C CNN
F 1 "SBT0811" H 8925 3574 50  0000 C CNN
F 2 "" H 8950 3200 50  0001 C CNN
F 3 "" H 8950 3200 50  0001 C CNN
	1    8950 3100
	1    0    0    -1  
$EndComp
$Comp
L Motor:Stepper_Motor_bipolar M?
U 1 1 5E23B025
P 9550 3850
F 0 "M?" H 9750 3900 50  0000 L CNN
F 1 "Window 2" H 9750 3800 50  0000 L CNN
F 2 "" H 9560 3840 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 9560 3840 50  0001 C CNN
	1    9550 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 3250 9450 3250
Wire Wire Line
	9450 3250 9450 3550
Wire Wire Line
	9150 3600 9150 3750
Wire Wire Line
	9150 3750 9250 3750
$Comp
L ProjectCustom:SBT0811 U?
U 1 1 5E2BC2AB
P 8950 4700
F 0 "U?" H 8925 5265 50  0000 C CNN
F 1 "SBT0811" H 8925 5174 50  0000 C CNN
F 2 "" H 8950 4800 50  0001 C CNN
F 3 "" H 8950 4800 50  0001 C CNN
	1    8950 4700
	1    0    0    -1  
$EndComp
$Comp
L Motor:Stepper_Motor_bipolar M?
U 1 1 5E2BC7BC
P 9550 5450
F 0 "M?" H 9750 5500 50  0000 L CNN
F 1 "Window 3" H 9750 5400 50  0000 L CNN
F 2 "" H 9560 5440 50  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Application-Note-TLE8110EE_driving_UniPolarStepperMotor_V1.1.pdf?fileId=db3a30431be39b97011be5d0aa0a00b0" H 9560 5440 50  0001 C CNN
	1    9550 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 1850 9450 1850
Wire Wire Line
	9450 1850 9450 2000
Wire Wire Line
	9350 1750 9650 1750
Wire Wire Line
	9650 1750 9650 2000
Wire Wire Line
	9250 2400 9050 2400
Wire Wire Line
	9050 2400 9050 2100
Wire Wire Line
	9350 3350 9650 3350
Wire Wire Line
	9650 3350 9650 3550
Wire Wire Line
	9250 3950 9050 3950
Wire Wire Line
	9050 3950 9050 3600
Wire Wire Line
	9350 4850 9650 4850
Wire Wire Line
	9650 4850 9650 5150
Wire Wire Line
	9350 4950 9450 4950
Wire Wire Line
	9450 4950 9450 5150
Wire Wire Line
	9150 5200 9150 5350
Wire Wire Line
	9150 5350 9250 5350
Wire Wire Line
	9050 5200 9050 5550
Wire Wire Line
	9050 5550 9250 5550
$Comp
L Connector:Conn_01x04_Male P?
U 1 1 5E312279
P 8100 3100
F 0 "P?" H 8250 3350 50  0000 C CNN
F 1 " " H 8208 3290 50  0000 C CNN
F 2 "" H 8100 3100 50  0001 C CNN
F 3 "~" H 8100 3100 50  0001 C CNN
	1    8100 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 5E312BCF
P 8050 1700
F 0 "J?" H 7950 1400 50  0000 L CNN
F 1 " " H 8078 1585 50  0000 L CNN
F 2 "" H 8050 1700 50  0001 C CNN
F 3 "~" H 8050 1700 50  0001 C CNN
	1    8050 1700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 5E314473
P 6900 850
F 0 "J?" H 6928 826 50  0000 L CNN
F 1 "Conn_01x04_Female" H 6928 735 50  0000 L CNN
F 2 "" H 6900 850 50  0001 C CNN
F 3 "~" H 6900 850 50  0001 C CNN
	1    6900 850 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J?
U 1 1 5E3147B1
P 6250 800
F 0 "J?" H 6278 776 50  0000 L CNN
F 1 "Conn_01x04_Female" H 6278 685 50  0000 L CNN
F 2 "" H 6250 800 50  0001 C CNN
F 3 "~" H 6250 800 50  0001 C CNN
	1    6250 800 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male P?
U 1 1 5E35213B
P 8150 1300
F 0 "P?" H 8300 1450 50  0000 C CNN
F 1 " " H 8258 1390 50  0000 C CNN
F 2 "" H 8150 1300 50  0001 C CNN
F 3 "~" H 8150 1300 50  0001 C CNN
	1    8150 1300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 5E35254D
P 10100 1050
F 0 "J?" H 10208 1231 50  0000 C CNN
F 1 "Conn_01x02_Male" H 10208 1140 50  0000 C CNN
F 2 "" H 10100 1050 50  0001 C CNN
F 3 "~" H 10100 1050 50  0001 C CNN
	1    10100 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 5E35286D
P 10100 1500
F 0 "J?" H 10208 1681 50  0000 C CNN
F 1 "Conn_01x02_Male" H 10208 1590 50  0000 C CNN
F 2 "" H 10100 1500 50  0001 C CNN
F 3 "~" H 10100 1500 50  0001 C CNN
	1    10100 1500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5E352F75
P 9550 1050
F 0 "J?" H 9578 1026 50  0000 L CNN
F 1 "Conn_01x02_Female" H 9578 935 50  0000 L CNN
F 2 "" H 9550 1050 50  0001 C CNN
F 3 "~" H 9550 1050 50  0001 C CNN
	1    9550 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5E3533DD
P 9200 750
F 0 "J?" H 9228 726 50  0000 L CNN
F 1 "Conn_01x02_Female" H 9228 635 50  0000 L CNN
F 2 "" H 9200 750 50  0001 C CNN
F 3 "~" H 9200 750 50  0001 C CNN
	1    9200 750 
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5E35378E
P 8050 1300
F 0 "J?" H 8000 1450 50  0000 L CNN
F 1 " " H 8078 1185 50  0000 L CNN
F 2 "" H 8050 1300 50  0001 C CNN
F 3 "~" H 8050 1300 50  0001 C CNN
	1    8050 1300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male P?
U 1 1 5E35E2F6
P 7850 3400
F 0 "P?" H 8000 3650 50  0000 C CNN
F 1 " " H 7958 3590 50  0000 C CNN
F 2 "" H 7850 3400 50  0001 C CNN
F 3 "~" H 7850 3400 50  0001 C CNN
	1    7850 3400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male P?
U 1 1 5E35E72A
P 8150 1700
F 0 "P?" H 8300 1400 50  0000 C CNN
F 1 " " H 8258 1890 50  0000 C CNN
F 2 "" H 8150 1700 50  0001 C CNN
F 3 "~" H 8150 1700 50  0001 C CNN
	1    8150 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 1600 8500 1600
Wire Wire Line
	8350 1700 8500 1700
Wire Wire Line
	8350 1800 8500 1800
Wire Wire Line
	8350 1900 8500 1900
Wire Wire Line
	7650 1600 7850 1600
Wire Wire Line
	7650 1700 7850 1700
Wire Wire Line
	7650 1800 7850 1800
Wire Wire Line
	7650 1900 7850 1900
$EndSCHEMATC
