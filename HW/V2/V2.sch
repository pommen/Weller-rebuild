EESchema Schematic File Version 4
LIBS:V2-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 5
Title "Weller WSSD81 moderrnizxation"
Date "2019-02-10"
Rev "B"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 4450 1700 1900 1950
U 5C556ABA
F0 "MCU" 50
F1 "MCU.sch" 50
F2 "SDA" I R 6350 3450 50 
F3 "SCL" I R 6350 3600 50 
F4 "SolderPWM2" I R 6350 3100 50 
F5 "SolderPWM1" I R 6350 3000 50 
F6 "MOSFETtemp" I R 6350 2700 50 
F7 "WMRTresistorDivider" I R 6350 3350 50 
$EndSheet
$Sheet
S 7500 2450 1850 1200
U 5C668421
F0 "SolderCon" 50
F1 "SolderCon.sch" 50
F2 "SolderPWM1" I L 7500 3000 50 
F3 "SolderPWM2" I L 7500 3100 50 
F4 "oneWireTemp" I L 7500 2550 50 
$EndSheet
$Sheet
S 4500 4150 1900 1750
U 5C659DD6
F0 "Temperature mangement" 50
F1 "Temperature.sch" 50
F2 "SDA" I R 6400 4500 50 
F3 "SCL" I R 6400 4350 50 
F4 "WMRTresistorDivider" I R 6400 4700 50 
$EndSheet
Wire Wire Line
	7500 3100 6350 3100
Wire Wire Line
	7500 3000 6350 3000
Wire Wire Line
	6350 3600 6550 3600
Wire Wire Line
	6550 3600 6550 4350
Wire Wire Line
	6550 4350 6400 4350
Wire Wire Line
	6400 4500 6650 4500
Wire Wire Line
	6650 4500 6650 3450
Wire Wire Line
	6650 3450 6350 3450
Wire Wire Line
	7500 2550 7150 2550
Wire Wire Line
	6350 2700 7150 2700
Wire Wire Line
	7150 2700 7150 2550
$Sheet
S 7500 750  1950 1400
U 5C556B0F
F0 "Power" 50
F1 "Power.sch" 50
$EndSheet
Wire Wire Line
	6400 4700 6750 4700
Wire Wire Line
	6750 4700 6750 3350
Wire Wire Line
	6750 3350 6350 3350
$EndSCHEMATC
