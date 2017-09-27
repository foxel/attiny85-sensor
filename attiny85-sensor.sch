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
LIBS:sensors
LIBS:attiny85-sensor-cache
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
L Earth #PWR01
U 1 1 59CA1187
P 8700 3450
F 0 "#PWR01" H 8700 3200 50  0001 C CNN
F 1 "Earth" H 8700 3300 50  0001 C CNN
F 2 "" H 8700 3450 50  0000 C CNN
F 3 "" H 8700 3450 50  0000 C CNN
	1    8700 3450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR02
U 1 1 59CA11AF
P 8700 2400
F 0 "#PWR02" H 8700 2250 50  0001 C CNN
F 1 "+3.3V" H 8700 2540 50  0000 C CNN
F 2 "" H 8700 2400 50  0000 C CNN
F 3 "" H 8700 2400 50  0000 C CNN
	1    8700 2400
	1    0    0    -1  
$EndComp
$Comp
L ATTINY85-P IC1
U 1 1 59CA0DCA
P 7100 2950
F 0 "IC1" H 5950 3350 50  0000 C CNN
F 1 "ATTINY85-P" H 8100 2550 50  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 8100 2950 50  0000 C CIN
F 3 "" H 7100 2950 50  0000 C CNN
	1    7100 2950
	1    0    0    -1  
$EndComp
$Comp
L DHT22_Temperature_Humidity TH1
U 1 1 59CA1697
P 3400 2900
F 0 "TH1" H 3400 3950 60  0000 C CNN
F 1 "DHT22_Temperature_Humidity" H 3400 3850 60  0000 C CNN
F 2 "Sensors:DHT22_Temperature_Humidity" H 3400 2900 60  0001 C CNN
F 3 "" H 3400 2900 60  0000 C CNN
	1    3400 2900
	1    0    0    -1  
$EndComp
$Comp
L RF_Transmitter_433_MHz RF1
U 1 1 59CA187F
P 3400 4250
F 0 "RF1" H 3400 5050 60  0000 C CNN
F 1 "RF_Transmitter_433_MHz" H 3400 4950 60  0000 C CNN
F 2 "Sensors:RF_Transmitter_433_MHz_4PIN" H 3400 4250 60  0001 C CNN
F 3 "" H 3400 4250 60  0000 C CNN
	1    3400 4250
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR03
U 1 1 59CA19B4
P 3500 4500
F 0 "#PWR03" H 3500 4250 50  0001 C CNN
F 1 "Earth" H 3500 4350 50  0001 C CNN
F 2 "" H 3500 4500 50  0000 C CNN
F 3 "" H 3500 4500 50  0000 C CNN
	1    3500 4500
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR04
U 1 1 59CA1AFA
P 3550 3200
F 0 "#PWR04" H 3550 2950 50  0001 C CNN
F 1 "Earth" H 3550 3050 50  0001 C CNN
F 2 "" H 3550 3200 50  0000 C CNN
F 3 "" H 3550 3200 50  0000 C CNN
	1    3550 3200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 59CA1B31
P 2900 2800
F 0 "#PWR05" H 2900 2650 50  0001 C CNN
F 1 "+3.3V" H 2900 2940 50  0000 C CNN
F 2 "" H 2900 2800 50  0000 C CNN
F 3 "" H 2900 2800 50  0000 C CNN
	1    2900 2800
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P3
U 1 1 59CA1DC3
P 7650 4100
F 0 "P3" H 7650 4250 50  0000 C CNN
F 1 "CONN_01X02" V 7750 4100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 7650 4100 50  0001 C CNN
F 3 "" H 7650 4100 50  0000 C CNN
	1    7650 4100
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR06
U 1 1 59CA1EAA
P 7300 4400
F 0 "#PWR06" H 7300 4150 50  0001 C CNN
F 1 "Earth" H 7300 4250 50  0001 C CNN
F 2 "" H 7300 4400 50  0000 C CNN
F 3 "" H 7300 4400 50  0000 C CNN
	1    7300 4400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 59CA1EF7
P 7300 3850
F 0 "#PWR07" H 7300 3700 50  0001 C CNN
F 1 "+3.3V" H 7300 3990 50  0000 C CNN
F 2 "" H 7300 3850 50  0000 C CNN
F 3 "" H 7300 3850 50  0000 C CNN
	1    7300 3850
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 59CA1D76
P 3250 3250
F 0 "R1" V 3330 3250 50  0000 C CNN
F 1 "10K" V 3250 3250 50  0000 C CNN
F 2 "Discret:R1" V 3180 3250 50  0001 C CNN
F 3 "" H 3250 3250 50  0000 C CNN
	1    3250 3250
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG08
U 1 1 59CA2151
P 2150 2250
F 0 "#FLG08" H 2150 2345 50  0001 C CNN
F 1 "PWR_FLAG" H 2150 2430 50  0000 C CNN
F 2 "" H 2150 2250 50  0000 C CNN
F 3 "" H 2150 2250 50  0000 C CNN
	1    2150 2250
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG09
U 1 1 59CA2175
P 2550 2250
F 0 "#FLG09" H 2550 2345 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 2430 50  0000 C CNN
F 2 "" H 2550 2250 50  0000 C CNN
F 3 "" H 2550 2250 50  0000 C CNN
	1    2550 2250
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR010
U 1 1 59CA218E
P 2550 2450
F 0 "#PWR010" H 2550 2200 50  0001 C CNN
F 1 "Earth" H 2550 2300 50  0001 C CNN
F 2 "" H 2550 2450 50  0000 C CNN
F 3 "" H 2550 2450 50  0000 C CNN
	1    2550 2450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR011
U 1 1 59CA21AE
P 1800 2200
F 0 "#PWR011" H 1800 2050 50  0001 C CNN
F 1 "+3.3V" H 1800 2340 50  0000 C CNN
F 2 "" H 1800 2200 50  0000 C CNN
F 3 "" H 1800 2200 50  0000 C CNN
	1    1800 2200
	1    0    0    -1  
$EndComp
$Comp
L CP C1
U 1 1 59CA2C52
P 6950 4100
F 0 "C1" H 6975 4200 50  0000 L CNN
F 1 "100mF" H 6975 4000 50  0000 L CNN
F 2 "Discret:C1V8" H 6988 3950 50  0000 C CNN
F 3 "" H 6950 4100 50  0000 C CNN
	1    6950 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 2700 8700 2400
Wire Wire Line
	8450 2700 8700 2700
Wire Wire Line
	8700 3200 8700 3450
Wire Wire Line
	8450 3200 8700 3200
Wire Wire Line
	3500 4250 3500 4500
Wire Wire Line
	3550 2900 3550 3200
Wire Wire Line
	2900 2800 2900 3150
Wire Wire Line
	2900 3150 3250 3150
Wire Wire Line
	3250 3150 3250 2900
Wire Wire Line
	3400 4250 3400 4350
Wire Wire Line
	3400 4350 5150 4350
Wire Wire Line
	5000 4350 5000 2800
Wire Wire Line
	3300 4250 3300 4450
Wire Wire Line
	3300 4450 4900 4450
Wire Wire Line
	4900 4450 4900 2700
Wire Wire Line
	7450 4150 7300 4150
Wire Wire Line
	7300 4150 7300 4400
Wire Wire Line
	7450 4050 7300 4050
Wire Wire Line
	7300 4050 7300 3850
Wire Wire Line
	3050 3150 3050 3250
Wire Wire Line
	3050 3250 3100 3250
Connection ~ 3050 3150
Wire Wire Line
	2550 2250 2550 2450
Wire Wire Line
	2150 2250 2150 2300
Wire Wire Line
	2150 2300 1800 2300
Wire Wire Line
	1800 2300 1800 2200
Wire Wire Line
	6950 3950 6950 3900
Wire Wire Line
	6950 3900 7300 3900
Connection ~ 7300 3900
Wire Wire Line
	6950 4250 6950 4300
Wire Wire Line
	6950 4300 7300 4300
Connection ~ 7300 4300
Wire Wire Line
	4900 2700 5750 2700
Wire Wire Line
	5000 2800 5750 2800
Wire Wire Line
	3350 2900 3350 3000
Wire Wire Line
	3350 3000 5750 3000
Wire Wire Line
	3400 3250 3400 3000
Connection ~ 3400 3000
$Comp
L CONN_01X03 P1
U 1 1 59CA7795
P 4250 2300
F 0 "P1" H 4250 2500 50  0000 C CNN
F 1 "PIR" V 4350 2300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 4250 2300 50  0001 C CNN
F 3 "" H 4250 2300 50  0000 C CNN
	1    4250 2300
	-1   0    0    -1  
$EndComp
$Comp
L Earth #PWR012
U 1 1 59CA789B
P 4600 2500
F 0 "#PWR012" H 4600 2250 50  0001 C CNN
F 1 "Earth" H 4600 2350 50  0001 C CNN
F 2 "" H 4600 2500 50  0000 C CNN
F 3 "" H 4600 2500 50  0000 C CNN
	1    4600 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2400 4600 2400
Wire Wire Line
	4600 2400 4600 2500
$Comp
L +3.3V #PWR013
U 1 1 59CA78EC
P 4600 2100
F 0 "#PWR013" H 4600 1950 50  0001 C CNN
F 1 "+3.3V" H 4600 2240 50  0000 C CNN
F 2 "" H 4600 2100 50  0000 C CNN
F 3 "" H 4600 2100 50  0000 C CNN
	1    4600 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2200 4600 2200
Wire Wire Line
	4600 2200 4600 2100
Wire Wire Line
	4450 2300 5250 2300
Wire Wire Line
	5250 2300 5250 2900
Wire Wire Line
	5250 2900 5750 2900
$Comp
L CONN_01X02 P2
U 1 1 59CA7973
P 6100 2200
F 0 "P2" H 6100 2350 50  0000 C CNN
F 1 "BIND_BTN" V 6200 2200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6100 2200 50  0001 C CNN
F 3 "" H 6100 2200 50  0000 C CNN
	1    6100 2200
	1    0    0    1   
$EndComp
Connection ~ 5600 2800
Wire Wire Line
	5900 2250 5600 2250
Wire Wire Line
	5600 2250 5600 2800
$Comp
L +3.3V #PWR014
U 1 1 59CA7A93
P 5700 2050
F 0 "#PWR014" H 5700 1900 50  0001 C CNN
F 1 "+3.3V" H 5700 2190 50  0000 C CNN
F 2 "" H 5700 2050 50  0000 C CNN
F 3 "" H 5700 2050 50  0000 C CNN
	1    5700 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 2150 5900 2150
$Comp
L CONN_01X06 P4
U 1 1 59CA8CD3
P 5450 3800
F 0 "P4" H 5450 4150 50  0000 C CNN
F 1 "CONN_01X06" V 5550 3800 50  0000 C CNN
F 2 "Sensors:SMD_Pads" H 5450 3800 50  0001 C CNN
F 3 "" H 5450 3800 50  0000 C CNN
	1    5450 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 3200 5700 3200
Wire Wire Line
	5600 3600 5600 3000
Connection ~ 5600 3000
Wire Wire Line
	5500 3600 5500 3100
Wire Wire Line
	5500 3100 5750 3100
Wire Wire Line
	5400 3600 5400 2900
Connection ~ 5400 2900
Wire Wire Line
	5300 3600 5300 2800
Connection ~ 5300 2800
Wire Wire Line
	5200 2700 5200 3600
Connection ~ 5200 2700
Wire Wire Line
	5700 2050 5700 2150
Wire Wire Line
	5700 3200 5700 3600
$Comp
L R R2
U 1 1 59CB2772
P 5550 4650
F 0 "R2" V 5630 4650 50  0000 C CNN
F 1 "R" V 5550 4650 50  0000 C CNN
F 2 "Discret:R1" V 5480 4650 50  0001 C CNN
F 3 "" H 5550 4650 50  0000 C CNN
	1    5550 4650
	1    0    0    1   
$EndComp
$Comp
L Earth #PWR015
U 1 1 59CB27C7
P 5550 5000
F 0 "#PWR015" H 5550 4750 50  0001 C CNN
F 1 "Earth" H 5550 4850 50  0001 C CNN
F 2 "" H 5550 5000 50  0000 C CNN
F 3 "" H 5550 5000 50  0000 C CNN
	1    5550 5000
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 59CB27EF
P 5300 4350
F 0 "D1" H 5300 4450 50  0000 C CNN
F 1 "LED" H 5300 4250 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 5300 4350 50  0001 C CNN
F 3 "" H 5300 4350 50  0000 C CNN
	1    5300 4350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5450 4350 5550 4350
Wire Wire Line
	5550 4350 5550 4500
Connection ~ 5000 4350
Wire Wire Line
	5550 4800 5550 5000
$EndSCHEMATC