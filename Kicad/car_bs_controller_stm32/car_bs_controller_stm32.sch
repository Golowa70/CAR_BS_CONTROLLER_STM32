EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Car bs controller STM32"
Date "2021-07-24"
Rev "1"
Comp ""
Comment1 "MCU"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 2700 2750 0    50   Input ~ 0
CS_W25Q32
Text GLabel 2700 2850 0    50   Input ~ 0
RCC_OSC32_IN
Text GLabel 2700 2950 0    50   Input ~ 0
RCC_OSC32_OUT
Text GLabel 2700 2450 0    50   Input ~ 0
RCC_OSC_IN
Text GLabel 2700 2550 0    50   Input ~ 0
RCC_OSC_OUT
Text GLabel 2700 2050 0    50   Input ~ 0
NRST
Text GLabel 2700 2250 0    50   Input ~ 0
BOOT_0
Text GLabel 5050 3150 2    50   Input ~ 0
IGNITION_IN
Text GLabel 5050 3250 2    50   Input ~ 0
PROX_SENSOR_IN
Text GLabel 5050 3350 2    50   Input ~ 0
ONE_WIRE
Text GLabel 5050 3450 2    50   Input ~ 0
SUPPLY_VOLTAGE_AIN
Text GLabel 5050 3550 2    50   Input ~ 0
CS_LCD
Text GLabel 5050 3650 2    50   Input ~ 0
SPI1_SCK
Text GLabel 5050 3750 2    50   Input ~ 0
SPI1_MISO
Text GLabel 5050 3850 2    50   Input ~ 0
SPI1_MOSI
Text GLabel 2700 3150 0    50   Input ~ 0
SENSORS_VOLTAGE_AIN
Text GLabel 2700 3250 0    50   Input ~ 0
RESESTIVE_SENSOR_AIN
Text GLabel 2700 3350 0    50   Input ~ 0
CS_SI4421
Text GLabel 2700 3450 0    50   Input ~ 0
SYS_SWO
Text GLabel 2700 3550 0    50   Input ~ 0
SENS_SUPPLY_EN_OUT
Text GLabel 2700 3650 0    50   Input ~ 0
CONVERTER_OUT
Text GLabel 2700 3750 0    50   Input ~ 0
FRIDGE_OUT
Text GLabel 2700 3850 0    50   Input ~ 0
WATER_PUMP_OUT
Text GLabel 2700 3950 0    50   Input ~ 0
CAN_RX
Text GLabel 2700 4050 0    50   Input ~ 0
CAN_TX
Text GLabel 2700 4150 0    50   Input ~ 0
CS_EXP_MODULE
Text GLabel 2700 4250 0    50   Input ~ 0
BUTTON_ENTER
Text GLabel 2700 4350 0    50   Input ~ 0
DOOR_SWITCH_IN
Text GLabel 2700 4450 0    50   Input ~ 0
SPI2_SCK
Text GLabel 2700 4550 0    50   Input ~ 0
SPI2_MISO
Text GLabel 2700 4650 0    50   Input ~ 0
SPI2_MOSI
Text GLabel 5050 3950 2    50   Input ~ 0
W_WATER_LEVEL_IN
Text GLabel 5050 4050 2    50   Input ~ 0
BUTTON_DOWN
Text GLabel 5050 4150 2    50   Input ~ 0
BUTTON_UP
Text GLabel 5050 4250 2    50   Input ~ 0
USB_DM
Text GLabel 5050 4350 2    50   Input ~ 0
USB_DP
Text GLabel 5050 4450 2    50   Input ~ 0
SYS_SWDIO
Text GLabel 5050 4550 2    50   Input ~ 0
SYS_SWCLK
Text GLabel 5050 4650 2    50   Input ~ 0
MAIN_SUPPLY_EN_OUT
$Comp
L power:GND #PWR?
U 1 1 60FC9A97
P 3900 5400
F 0 "#PWR?" H 3900 5150 50  0001 C CNN
F 1 "GND" H 3905 5227 50  0000 C CNN
F 2 "" H 3900 5400 50  0001 C CNN
F 3 "" H 3900 5400 50  0001 C CNN
	1    3900 5400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 60FCB09C
P 4150 800
F 0 "#PWR?" H 4150 650 50  0001 C CNN
F 1 "+3V3" H 4165 973 50  0000 C CNN
F 2 "" H 4150 800 50  0001 C CNN
F 3 "" H 4150 800 50  0001 C CNN
	1    4150 800 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60FCC502
P 4450 1150
F 0 "#PWR?" H 4450 900 50  0001 C CNN
F 1 "GND" H 4455 977 50  0000 C CNN
F 2 "" H 4450 1150 50  0001 C CNN
F 3 "" H 4450 1150 50  0001 C CNN
	1    4450 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 60FCC9C8
P 3700 850
F 0 "#PWR?" H 3700 700 50  0001 C CNN
F 1 "+BATT" H 3715 1023 50  0000 C CNN
F 2 "" H 3700 850 50  0001 C CNN
F 3 "" H 3700 850 50  0001 C CNN
	1    3700 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 800  4150 850 
$Comp
L Device:C C?
U 1 1 60FCEB3F
P 4450 1000
F 0 "C?" H 4565 1046 50  0000 L CNN
F 1 "C" H 4565 955 50  0000 L CNN
F 2 "" H 4488 850 50  0001 C CNN
F 3 "~" H 4450 1000 50  0001 C CNN
	1    4450 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 850  4150 850 
$Comp
L MCU_ST_STM32F1:STM32F103RBTx U?
U 1 1 60FC691C
P 3900 3450
F 0 "U?" H 3850 3000 50  0000 C CNN
F 1 "STM32F103RBTx" H 3900 2750 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 3300 1750 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 3900 3450 50  0001 C CNN
	1    3900 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5250 3800 5250
Wire Wire Line
	3900 5250 3800 5250
Connection ~ 3800 5250
Wire Wire Line
	4000 5250 3900 5250
Connection ~ 3900 5250
Wire Wire Line
	4100 5250 4000 5250
Connection ~ 4000 5250
Wire Wire Line
	3900 5400 3900 5250
Wire Wire Line
	4150 850  4150 1650
Wire Wire Line
	4150 1650 4100 1650
Connection ~ 4150 850 
Wire Wire Line
	3800 1650 3900 1650
Wire Wire Line
	4000 1650 4100 1650
Connection ~ 4100 1650
Wire Wire Line
	4000 1650 3900 1650
Connection ~ 4000 1650
Connection ~ 3900 1650
Wire Wire Line
	4200 1650 4150 1650
Connection ~ 4150 1650
$EndSCHEMATC