EESchema Schematic File Version 4
LIBS:CalmFire_board-cache
EELAYER 29 0
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
L power:+3V3 #PWR0101
U 1 1 5CCA8705
P 3700 2600
F 0 "#PWR0101" H 3700 2450 50  0001 C CNN
F 1 "+3V3" H 3715 2773 50  0000 C CNN
F 2 "" H 3700 2600 50  0001 C CNN
F 3 "" H 3700 2600 50  0001 C CNN
	1    3700 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2600 3700 2800
Wire Wire Line
	3700 2800 3950 2800
$Comp
L power:GND #PWR0102
U 1 1 5CCA90DD
P 5750 4850
F 0 "#PWR0102" H 5750 4600 50  0001 C CNN
F 1 "GND" H 5755 4677 50  0000 C CNN
F 2 "" H 5750 4850 50  0001 C CNN
F 3 "" H 5750 4850 50  0001 C CNN
	1    5750 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5CCA9A47
P 3250 4800
F 0 "#PWR0103" H 3250 4550 50  0001 C CNN
F 1 "GND" H 3255 4627 50  0000 C CNN
F 2 "" H 3250 4800 50  0001 C CNN
F 3 "" H 3250 4800 50  0001 C CNN
	1    3250 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 4800 3250 4100
Wire Wire Line
	5750 3400 5550 3400
Wire Wire Line
	5550 2800 5750 2800
Wire Wire Line
	5750 2800 5750 3400
Connection ~ 5750 3400
$Comp
L power:+5V #PWR0104
U 1 1 5CCAA14B
P 3800 4450
F 0 "#PWR0104" H 3800 4300 50  0001 C CNN
F 1 "+5V" H 3815 4623 50  0000 C CNN
F 2 "" H 3800 4450 50  0001 C CNN
F 3 "" H 3800 4450 50  0001 C CNN
	1    3800 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4600 3800 4600
Wire Wire Line
	3800 4600 3800 4450
Wire Wire Line
	3250 4100 3950 4100
$Comp
L Device:Rotary_Encoder_Switch SW1
U 1 1 5CCAB9AD
P 2850 2050
F 0 "SW1" V 2804 2280 50  0000 L CNN
F 1 "Rotary_Encoder_Switch" V 2895 2280 50  0000 L CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm" H 2700 2210 50  0001 C CNN
F 3 "~" H 2850 2310 50  0001 C CNN
	1    2850 2050
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR0105
U 1 1 5CCAD16B
P 3100 2450
F 0 "#PWR0105" H 3100 2300 50  0001 C CNN
F 1 "+3V3" H 3115 2623 50  0000 C CNN
F 2 "" H 3100 2450 50  0001 C CNN
F 3 "" H 3100 2450 50  0001 C CNN
	1    3100 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	3100 2450 2950 2450
Wire Wire Line
	2950 2450 2950 2350
$Comp
L power:GND #PWR0106
U 1 1 5CCAD480
P 2150 2500
F 0 "#PWR0106" H 2150 2250 50  0001 C CNN
F 1 "GND" H 2155 2327 50  0000 C CNN
F 2 "" H 2150 2500 50  0001 C CNN
F 3 "" H 2150 2500 50  0001 C CNN
	1    2150 2500
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0107
U 1 1 5CCADD16
P 3450 1500
F 0 "#PWR0107" H 3450 1350 50  0001 C CNN
F 1 "+3V3" H 3465 1673 50  0000 C CNN
F 2 "" H 3450 1500 50  0001 C CNN
F 3 "" H 3450 1500 50  0001 C CNN
	1    3450 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0108
U 1 1 5CCADFB9
P 2250 1500
F 0 "#PWR0108" H 2250 1350 50  0001 C CNN
F 1 "+3V3" H 2265 1673 50  0000 C CNN
F 2 "" H 2250 1500 50  0001 C CNN
F 3 "" H 2250 1500 50  0001 C CNN
	1    2250 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5CCAEFC8
P 2850 1350
F 0 "#PWR0109" H 2850 1100 50  0001 C CNN
F 1 "GND" H 2855 1177 50  0000 C CNN
F 2 "" H 2850 1350 50  0001 C CNN
F 3 "" H 2850 1350 50  0001 C CNN
	1    2850 1350
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5CCAFA6E
P 3200 1550
F 0 "R5" V 3004 1550 50  0000 C CNN
F 1 "R_Small" V 3095 1550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 3200 1550 50  0001 C CNN
F 3 "~" H 3200 1550 50  0001 C CNN
	1    3200 1550
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5CCB0471
P 2550 1550
F 0 "R2" V 2354 1550 50  0000 C CNN
F 1 "R_Small" V 2445 1550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 2550 1550 50  0001 C CNN
F 3 "~" H 2550 1550 50  0001 C CNN
	1    2550 1550
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5CCB07D3
P 2450 2400
F 0 "R1" V 2254 2400 50  0000 C CNN
F 1 "R_Small" V 2345 2400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 2450 2400 50  0001 C CNN
F 3 "~" H 2450 2400 50  0001 C CNN
	1    2450 2400
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5CCB11FE
P 2150 1900
F 0 "C1" H 2242 1946 50  0000 L CNN
F 1 "C_Small" H 2242 1855 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 2150 1900 50  0001 C CNN
F 3 "~" H 2150 1900 50  0001 C CNN
	1    2150 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5CCB1D85
P 3450 1900
F 0 "C2" H 3542 1946 50  0000 L CNN
F 1 "C_Small" H 3542 1855 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 3450 1900 50  0001 C CNN
F 3 "~" H 3450 1900 50  0001 C CNN
	1    3450 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5CCB2164
P 3450 2200
F 0 "#PWR0110" H 3450 1950 50  0001 C CNN
F 1 "GND" H 3455 2027 50  0000 C CNN
F 2 "" H 3450 2200 50  0001 C CNN
F 3 "" H 3450 2200 50  0001 C CNN
	1    3450 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1350 2850 1750
Wire Wire Line
	3100 1550 3050 1550
Wire Wire Line
	2950 1550 2950 1750
Wire Wire Line
	2750 1750 2750 1700
Wire Wire Line
	2750 1550 2650 1550
Wire Wire Line
	3300 1550 3450 1550
Wire Wire Line
	3450 1550 3450 1500
Wire Wire Line
	3450 1800 3050 1800
Wire Wire Line
	3050 1800 3050 1700
Connection ~ 3050 1550
Wire Wire Line
	3050 1550 2950 1550
Wire Wire Line
	3450 2000 3450 2200
Wire Wire Line
	2450 1550 2250 1550
Wire Wire Line
	2250 1550 2250 1500
Wire Wire Line
	2150 1800 2150 1700
Connection ~ 2750 1700
Wire Wire Line
	2750 1700 2750 1550
Wire Wire Line
	2150 2000 2150 2400
Wire Wire Line
	2350 2400 2150 2400
Connection ~ 2150 2400
Wire Wire Line
	2150 2400 2150 2500
Wire Wire Line
	2550 2400 2750 2400
Wire Wire Line
	2750 2400 2750 2350
Text GLabel 3150 1700 2    50   Input ~ 0
CLK
Wire Wire Line
	3150 1700 3050 1700
Connection ~ 3050 1700
Wire Wire Line
	3050 1700 3050 1550
Text GLabel 1950 1700 0    50   Input ~ 0
DIR
Wire Wire Line
	1950 1700 2150 1700
Connection ~ 2150 1700
Wire Wire Line
	2150 1700 2750 1700
Text GLabel 5850 3300 2    50   Input ~ 0
CLK
Wire Wire Line
	5850 3300 5550 3300
Text GLabel 5850 3500 2    50   Input ~ 0
DIR
Wire Wire Line
	5850 3500 5550 3500
$Comp
L Device:R_Small R4
U 1 1 5CCBD33F
P 2700 3450
F 0 "R4" V 2504 3450 50  0000 C CNN
F 1 "R_Small" V 2595 3450 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 2700 3450 50  0001 C CNN
F 3 "~" H 2700 3450 50  0001 C CNN
	1    2700 3450
	-1   0    0    1   
$EndComp
$Comp
L Device:R_PHOTO R3
U 1 1 5CCBF6E1
P 2700 3100
F 0 "R3" H 2770 3146 50  0000 L CNN
F 1 "R_PHOTO" H 2770 3055 50  0000 L CNN
F 2 "OptoDevice:R_LDR_5.0x4.1mm_P3mm_Vertical" V 2750 2850 50  0001 L CNN
F 3 "~" H 2700 3050 50  0001 C CNN
	1    2700 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0111
U 1 1 5CCBFE73
P 2700 2900
F 0 "#PWR0111" H 2700 2750 50  0001 C CNN
F 1 "+3V3" H 2715 3073 50  0000 C CNN
F 2 "" H 2700 2900 50  0001 C CNN
F 3 "" H 2700 2900 50  0001 C CNN
	1    2700 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5CCC0148
P 2700 3600
F 0 "#PWR0112" H 2700 3350 50  0001 C CNN
F 1 "GND" H 2705 3427 50  0000 C CNN
F 2 "" H 2700 3600 50  0001 C CNN
F 3 "" H 2700 3600 50  0001 C CNN
	1    2700 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 3600 2700 3550
Wire Wire Line
	2700 3350 2700 3300
Wire Wire Line
	2700 2950 2700 2900
Connection ~ 2700 3300
Wire Wire Line
	2700 3300 2700 3250
$Comp
L power:+5V #PWR0114
U 1 1 5CCCACD4
P 7550 2200
F 0 "#PWR0114" H 7550 2050 50  0001 C CNN
F 1 "+5V" H 7565 2373 50  0000 C CNN
F 2 "" H 7550 2200 50  0001 C CNN
F 3 "" H 7550 2200 50  0001 C CNN
	1    7550 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J1
U 1 1 5CCD6637
P 4950 1900
F 0 "J1" H 4978 1876 50  0000 L CNN
F 1 "HT1621_LCD" H 4978 1785 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 4950 1900 50  0001 C CNN
F 3 "~" H 4950 1900 50  0001 C CNN
	1    4950 1900
	1    0    0    -1  
$EndComp
Text GLabel 4600 1700 0    50   Input ~ 0
dispCS
Text GLabel 4600 1800 0    50   Input ~ 0
dispWR
Text GLabel 4600 1900 0    50   Input ~ 0
dispData
Text GLabel 4600 2200 0    50   Input ~ 0
dispLED
Text GLabel 3700 3900 0    50   Input ~ 0
dispCS
Text GLabel 3700 3800 0    50   Input ~ 0
dispWR
Text GLabel 3700 3700 0    50   Input ~ 0
dispData
Text GLabel 3700 4200 0    50   Input ~ 0
dispLED
Wire Wire Line
	4600 1700 4750 1700
Wire Wire Line
	4600 1800 4750 1800
Wire Wire Line
	4600 1900 4750 1900
Wire Wire Line
	4600 2200 4750 2200
Wire Wire Line
	3700 4200 3950 4200
$Comp
L power:+5V #PWR0115
U 1 1 5CCE3E4C
P 4600 2100
F 0 "#PWR0115" H 4600 1950 50  0001 C CNN
F 1 "+5V" V 4615 2273 50  0000 C CNN
F 2 "" H 4600 2100 50  0001 C CNN
F 3 "" H 4600 2100 50  0001 C CNN
	1    4600 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3700 3900 3950 3900
Wire Wire Line
	3700 3700 3950 3700
Wire Wire Line
	3700 3800 3950 3800
Wire Wire Line
	2700 3300 3950 3300
Wire Wire Line
	5750 3400 5750 4850
$Comp
L power:GND #PWR0116
U 1 1 5CDD6061
P 4150 2050
F 0 "#PWR0116" H 4150 1800 50  0001 C CNN
F 1 "GND" H 4155 1877 50  0000 C CNN
F 2 "" H 4150 2050 50  0001 C CNN
F 3 "" H 4150 2050 50  0001 C CNN
	1    4150 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2050 4150 2000
Wire Wire Line
	4150 2000 4750 2000
Wire Wire Line
	4600 2100 4750 2100
$Comp
L Connector:Screw_Terminal_01x02 Power_con1
U 1 1 5CDFE963
P 6350 1550
F 0 "Power_con1" H 6430 1542 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 6430 1451 50  0000 L CNN
F 2 "Connector_Phoenix_MC_HighVoltage:PhoenixContact_MC_1,5_2-G-5.08_1x02_P5.08mm_Horizontal" H 6350 1550 50  0001 C CNN
F 3 "~" H 6350 1550 50  0001 C CNN
	1    6350 1550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0117
U 1 1 5CE00FBA
P 6050 1500
F 0 "#PWR0117" H 6050 1350 50  0001 C CNN
F 1 "+5V" H 6065 1673 50  0000 C CNN
F 2 "" H 6050 1500 50  0001 C CNN
F 3 "" H 6050 1500 50  0001 C CNN
	1    6050 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1500 6050 1550
Wire Wire Line
	6050 1550 6150 1550
$Comp
L power:GND #PWR0118
U 1 1 5CE03096
P 6050 1700
F 0 "#PWR0118" H 6050 1450 50  0001 C CNN
F 1 "GND" H 6055 1527 50  0000 C CNN
F 2 "" H 6050 1700 50  0001 C CNN
F 3 "" H 6050 1700 50  0001 C CNN
	1    6050 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1700 6050 1650
Wire Wire Line
	6050 1650 6150 1650
$Comp
L Transistor_FET:IRF8301M Q1
U 1 1 5CE10137
P 8050 4350
F 0 "Q1" H 8256 4441 50  0000 L CNN
F 1 "IRF8301M" H 8256 4350 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 8256 4259 50  0000 L CIN
F 3 "https://www.infineon.com/dgdl/irf8301mpbf.pdf?fileId=5546d462533600a40153560d0e7a1d58" H 8050 4350 50  0001 L CNN
	1    8050 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4150 8150 4050
$Comp
L power:GND #PWR0119
U 1 1 5CE1C550
P 8150 4600
F 0 "#PWR0119" H 8150 4350 50  0001 C CNN
F 1 "GND" H 8155 4427 50  0000 C CNN
F 2 "" H 8150 4600 50  0001 C CNN
F 3 "" H 8150 4600 50  0001 C CNN
	1    8150 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4600 8150 4550
$Comp
L Device:R_Small R6
U 1 1 5CE23C6B
P 7700 4350
F 0 "R6" V 7504 4350 50  0000 C CNN
F 1 "100R" V 7595 4350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 7700 4350 50  0001 C CNN
F 3 "~" H 7700 4350 50  0001 C CNN
	1    7700 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	7800 4350 7850 4350
Text GLabel 7500 4350 0    50   Input ~ 0
Relay
Wire Wire Line
	7500 4350 7600 4350
Text GLabel 5850 3900 2    50   Input ~ 0
Relay
Wire Wire Line
	5850 3900 5550 3900
$Comp
L power:+5V #PWR0120
U 1 1 5CE2CE33
P 8150 3500
F 0 "#PWR0120" H 8150 3350 50  0001 C CNN
F 1 "+5V" H 8165 3673 50  0000 C CNN
F 2 "" H 8150 3500 50  0001 C CNN
F 3 "" H 8150 3500 50  0001 C CNN
	1    8150 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 3500 8150 3550
$Comp
L Sensor_Temperature:DS18B20 U2
U 1 1 5CE37783
P 7550 2600
F 0 "U2" H 7320 2646 50  0000 R CNN
F 1 "DS18B20" H 7320 2555 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 6550 2350 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 7400 2850 50  0001 C CNN
	1    7550 2600
	1    0    0    -1  
$EndComp
Text GLabel 5850 3800 2    50   Input ~ 0
Temperature
Wire Wire Line
	5850 3800 5550 3800
$Comp
L power:GND #PWR0121
U 1 1 5CE48C43
P 7550 2950
F 0 "#PWR0121" H 7550 2700 50  0001 C CNN
F 1 "GND" H 7555 2777 50  0000 C CNN
F 2 "" H 7550 2950 50  0001 C CNN
F 3 "" H 7550 2950 50  0001 C CNN
	1    7550 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 2950 7550 2900
Text GLabel 8750 3050 2    50   Input ~ 0
Temperature
$Comp
L Connector:Screw_Terminal_01x02 Relay_con1
U 1 1 5CE15286
P 9000 3950
F 0 "Relay_con1" H 9080 3942 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 9080 3851 50  0000 L CNN
F 2 "Connector_Phoenix_MC_HighVoltage:PhoenixContact_MC_1,5_2-G-5.08_1x02_P5.08mm_Horizontal" H 9000 3950 50  0001 C CNN
F 3 "~" H 9000 3950 50  0001 C CNN
	1    9000 3950
	1    0    0    -1  
$EndComp
$Comp
L CalmFire_board:N4100_relay U3
U 1 1 5CE5F614
P 8300 3800
F 0 "U3" H 7972 3846 50  0000 R CNN
F 1 "N4100_relay" H 7972 3755 50  0000 R CNN
F 2 "CalmFire_board:N4100_relay" H 8300 3800 50  0001 C CNN
F 3 "" H 8300 3800 50  0001 C CNN
	1    8300 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 4050 8450 4050
Wire Wire Line
	8700 3800 8700 3950
Wire Wire Line
	8700 3950 8800 3950
$Comp
L Sensor:DHT11 U4
U 1 1 5CE00A60
P 8400 2600
F 0 "U4" H 8156 2646 50  0000 R CNN
F 1 "DHT11" H 8156 2555 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 8400 2200 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 8550 2850 50  0001 C CNN
	1    8400 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 2200 7550 2250
Wire Wire Line
	7550 2250 8400 2250
Wire Wire Line
	8400 2250 8400 2300
Connection ~ 7550 2250
Wire Wire Line
	7550 2250 7550 2300
Wire Wire Line
	8400 2900 7550 2900
Connection ~ 7550 2900
Wire Wire Line
	8750 3050 8700 3050
Wire Wire Line
	8700 3050 8700 2600
Wire Wire Line
	8700 3050 7850 3050
Wire Wire Line
	7850 3050 7850 2600
Connection ~ 8700 3050
$Comp
L ESP32-DEVKITC-32D:ESP32-DEVKITC-32D U1
U 1 1 5CE28294
P 4750 3700
F 0 "U1" H 4750 4867 50  0000 C CNN
F 1 "ESP32-DEVKITC-32D" H 4750 4776 50  0000 C CNN
F 2 "MODULE_ESP32-DEVKITC-32D" H 4750 3700 50  0001 L BNN
F 3 "" H 4750 3700 50  0001 L BNN
F 4 "None" H 4750 3700 50  0001 L BNN "Field4"
F 5 "Espressif Systems" H 4750 3700 50  0001 L BNN "Field5"
F 6 "None" H 4750 3700 50  0001 L BNN "Field6"
F 7 "ESP32-DEVKITC-32D" H 4750 3700 50  0001 L BNN "Field7"
F 8 "Unavailable" H 4750 3700 50  0001 L BNN "Field8"
	1    4750 3700
	1    0    0    -1  
$EndComp
$EndSCHEMATC
