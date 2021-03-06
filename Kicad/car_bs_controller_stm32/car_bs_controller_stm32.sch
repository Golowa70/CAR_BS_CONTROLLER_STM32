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
Text GLabel 2900 5150 0    50   Input ~ 0
RCC_OSC32_IN
Text GLabel 2900 5250 0    50   Input ~ 0
RCC_OSC32_OUT
Text GLabel 2900 3350 0    50   Input ~ 0
RCC_OSC_IN
Text GLabel 2900 3450 0    50   Input ~ 0
RCC_OSC_OUT
Text GLabel 2700 2050 0    50   Input ~ 0
NRST
Text GLabel 2700 2250 0    50   Input ~ 0
BOOT_0
Text GLabel 2900 4750 0    50   Input ~ 0
IGNITION_DIN
Text GLabel 2900 5050 0    50   Input ~ 0
PROX_SENSOR_DIN
Text GLabel 2900 3750 0    50   Input ~ 0
SUPPLY_VOLTAGE_AIN
Text GLabel 4950 2450 2    50   Input ~ 0
LCD_CS
Text GLabel 4950 2550 2    50   Input ~ 0
SPI1_SCK
Text GLabel 4950 2650 2    50   Input ~ 0
SPI1_MISO
Text GLabel 4950 2750 2    50   Input ~ 0
SPI1_MOSI
Text GLabel 2900 3850 0    50   Input ~ 0
SENSORS_VOLTAGE_AIN
Text GLabel 2900 3950 0    50   Input ~ 0
RESESTIVE_SENSOR_AIN
Text GLabel 2900 4150 0    50   Input ~ 0
SI4421_CS
Text GLabel 5000 4050 2    50   Input ~ 0
SYS_SWO
Text GLabel 2900 4850 0    50   Input ~ 0
SENS_SUPPLY_EN_OUT
Text GLabel 2900 3550 0    50   Input ~ 0
CONVERTER_OUT
Text GLabel 5000 4150 2    50   Input ~ 0
FRIDGE_OUT
Text GLabel 5000 4250 2    50   Input ~ 0
WATER_PUMP_OUT
Text GLabel 5000 4550 2    50   Input ~ 0
CAN_RX
Text GLabel 5000 4650 2    50   Input ~ 0
CAN_TX
Text GLabel 2900 4350 0    50   Input ~ 0
BUTTON_ENTER
Text GLabel 4950 2150 2    50   Input ~ 0
DOOR_SWITCH_DIN
Text GLabel 5000 5050 2    50   Input ~ 0
SPI2_SCK
Text GLabel 5000 5150 2    50   Input ~ 0
SPI2_MISO
Text GLabel 5000 5250 2    50   Input ~ 0
SPI2_MOSI
Text GLabel 2900 4450 0    50   Input ~ 0
BUTTON_DOWN
Text GLabel 2900 4550 0    50   Input ~ 0
BUTTON_UP
Text GLabel 4950 3150 2    50   Input ~ 0
USB_DM
Text GLabel 4950 3250 2    50   Input ~ 0
USB_DP
Text GLabel 4950 3350 2    50   Input ~ 0
SYS_SWDIO
Text GLabel 4950 3450 2    50   Input ~ 0
SYS_SWCLK
Text GLabel 2900 4950 0    50   Input ~ 0
MAIN_SUPPLY_EN_OUT
$Comp
L power:GND #PWR?
U 1 1 60FC9A97
P 3900 5550
F 0 "#PWR?" H 3900 5300 50  0001 C CNN
F 1 "GND" H 3905 5377 50  0000 C CNN
F 2 "" H 3900 5550 50  0001 C CNN
F 3 "" H 3900 5550 50  0001 C CNN
	1    3900 5550
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
Wire Wire Line
	4150 850  4150 1650
Connection ~ 4150 850 
Wire Wire Line
	4200 1650 4150 1650
Connection ~ 4150 1650
Wire Wire Line
	3200 2050 2700 2050
Wire Wire Line
	3800 1650 3900 1650
Wire Wire Line
	3200 2250 2700 2250
Wire Wire Line
	3200 3350 2900 3350
Wire Wire Line
	3200 3450 2900 3450
Wire Wire Line
	3200 3750 2900 3750
Wire Wire Line
	3200 3850 2900 3850
Wire Wire Line
	3200 3950 2900 3950
Text GLabel 2900 4050 0    50   Input ~ 0
SENSORS_CURRENT_AIN
Wire Wire Line
	2900 4050 3200 4050
Wire Wire Line
	4950 2050 4600 2050
Wire Wire Line
	4950 2150 4600 2150
Text GLabel 4950 2250 2    50   Input ~ 0
PUMP_CURRENT_AIN
Wire Wire Line
	4950 2250 4600 2250
Text GLabel 4950 2350 2    50   Input ~ 0
FRIDGE_CURRENT_AIN
Wire Wire Line
	4950 2350 4600 2350
Wire Wire Line
	4950 2450 4600 2450
Wire Wire Line
	4950 2550 4600 2550
Wire Wire Line
	4950 2650 4600 2650
Wire Wire Line
	4950 2750 4600 2750
Wire Wire Line
	3200 4150 2900 4150
Text GLabel 2900 4250 0    50   Input ~ 0
LCD_RST
Wire Wire Line
	3200 4250 2900 4250
Text GLabel 5000 3750 2    50   Input ~ 0
LCD_BLA
Wire Wire Line
	5000 3750 4600 3750
Text GLabel 5000 3850 2    50   Input ~ 0
SI4421_IRQ
Wire Wire Line
	5000 3850 4600 3850
Text GLabel 5000 3950 2    50   Input ~ 0
LCD_DC
Wire Wire Line
	5000 3950 4600 3950
Text GLabel 5000 4750 2    50   Input ~ 0
ONE_WIRE
Wire Wire Line
	5000 4750 4600 4750
Text GLabel 5000 4850 2    50   Input ~ 0
EXP_MODULE_CS
Wire Wire Line
	5000 4850 4600 4850
Text GLabel 5000 4950 2    50   Input ~ 0
W25Q32_CS
Wire Wire Line
	5000 4950 4600 4950
Wire Wire Line
	5000 5050 4600 5050
Wire Wire Line
	5000 5150 4600 5150
Wire Wire Line
	5000 5250 4600 5250
Wire Wire Line
	3200 4350 2900 4350
Wire Wire Line
	3200 4450 2900 4450
Wire Wire Line
	3200 4550 2900 4550
Text GLabel 2900 4650 0    50   Input ~ 0
BUTTON_ESC
Wire Wire Line
	3200 4650 2900 4650
Text GLabel 4950 2850 2    50   Input ~ 0
STATUS_LED
Wire Wire Line
	4950 2850 4600 2850
Text GLabel 4950 2950 2    50   Input ~ 0
USART1_TX
Wire Wire Line
	4950 2950 4600 2950
Text GLabel 4950 3050 2    50   Input ~ 0
USART1_RX
Wire Wire Line
	4950 3050 4600 3050
Wire Wire Line
	4950 3150 4600 3150
Wire Wire Line
	4950 3250 4600 3250
Wire Wire Line
	4950 3350 4600 3350
Wire Wire Line
	4950 3450 4600 3450
Text GLabel 4950 3550 2    50   Input ~ 0
BUZZER
Wire Wire Line
	4600 3550 4950 3550
Wire Wire Line
	2900 4850 3200 4850
Wire Wire Line
	2900 4950 3200 4950
Wire Wire Line
	2900 5250 3200 5250
Wire Wire Line
	2900 5150 3200 5150
Wire Wire Line
	2900 5050 3200 5050
Wire Wire Line
	3200 3550 2900 3550
Wire Wire Line
	5000 4050 4600 4050
Wire Wire Line
	5000 4150 4600 4150
Wire Wire Line
	5000 4250 4600 4250
Text GLabel 5000 4350 2    50   Input ~ 0
I2C1_SCL
Wire Wire Line
	5000 4350 4600 4350
Text GLabel 5000 4450 2    50   Input ~ 0
I2C1_SDA
Wire Wire Line
	5000 4450 4600 4450
Wire Wire Line
	5000 4550 4600 4550
Wire Wire Line
	5000 4650 4600 4650
Text GLabel 3700 1700 1    50   Input ~ 0
V_BAT
Wire Wire Line
	3700 1850 3700 1700
Wire Wire Line
	3800 1650 3800 1850
Wire Wire Line
	3900 1850 3900 1650
Connection ~ 3900 1650
Wire Wire Line
	3900 1650 4000 1650
Wire Wire Line
	4000 1850 4000 1650
Connection ~ 4000 1650
Wire Wire Line
	4000 1650 4100 1650
Wire Wire Line
	4100 1850 4100 1650
Connection ~ 4100 1650
Wire Wire Line
	4100 1650 4150 1650
Wire Wire Line
	4200 1650 4200 1850
$Comp
L MCU_ST_STM32F1:STM32F103RBTx U?
U 1 1 60FC691C
P 3900 3650
F 0 "U?" H 3850 3200 50  0000 C CNN
F 1 "STM32F103RBTx" H 3900 2950 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 3300 1950 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 3900 3650 50  0001 C CNN
	1    3900 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 5450 4000 5450
Wire Wire Line
	3900 5450 3800 5450
Wire Wire Line
	3700 5450 3800 5450
Connection ~ 3800 5450
Wire Wire Line
	3900 5450 4000 5450
Connection ~ 3900 5450
Connection ~ 4000 5450
Wire Wire Line
	3900 5550 3900 5450
Wire Wire Line
	3200 4750 2900 4750
Text GLabel 4950 2050 2    50   Input ~ 0
WKUP_DIN
$EndSCHEMATC
