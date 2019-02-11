

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ArduinoSort.h>
#include <U8g2lib.h>
#include <EEPROM.h>

//************************************
//
#define DEBUG
//
//************************************
//Define Variables we'll be connecting to
double Setpoint, tiptempRaw, Output;

//Specify the links and initial tuning parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 0.5, consKi = 0.07, consKd = 0.25;
//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
/*I2C adresses:
0x20 PFC8574
0x3C OLED
0x40 INA219
0x48 ADS1000
*/
int adsAddr = 0x48;

float tiptemp = 0.00;
//int tiptempRaw = 0;
long unsigned int sampleTime = 0;
long unsigned int DisplayUpdateTimer = 0;
long unsigned int eepromTimer = 0;

boolean atRest = 1;
boolean EEPROMwriteFlag = 0;
unsigned int EEPROMsetpointAddr = 0;
const unsigned int EEPROMstartAddr = 0;
unsigned int newEEPROMaddrFlag = 0;

//Pins:
const unsigned int solderPWMPin = 5;
const unsigned int pixel = 11;
const unsigned int BTN_UP = 9;
const unsigned int BTN_DN = 8;
const unsigned int HandleTempPin = A7;
//Protos:

//Prints to the screen.
void printOled(int);
//send required readings, returns sorted and averaged adc.
int solderTemp(int samples);
//when we press a button, we go here. return the new setpoint
double changeTemp();
//EEPROM stuff
void updateEEPRoM(double);
//Convert ADC to degrees celcius
double convertTemp(int rawReading);

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
PID myPID(&tiptempRaw, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
void setup()
{
	pinMode(solderPWMPin, OUTPUT);
	pinMode(pixel, OUTPUT);
	pinMode(BTN_UP, INPUT);
	pinMode(BTN_DN, INPUT);
	pinMode(HandleTempPin, INPUT);
	Serial.begin(9600);
	Wire.begin();
	//our last eeprom address, for eeprom cycling
	EEPROM.get(EEPROMstartAddr, EEPROMsetpointAddr);
	//our target temperature
	EEPROM.get(EEPROMsetpointAddr, Setpoint);
#if defined(DEBUG)
	Serial.print("EEPROMstartAddr: ");
	Serial.println(EEPROMstartAddr);
	Serial.print("EEPROMsetpointAddr: ");
	Serial.println(EEPROMsetpointAddr);
	Serial.print("Setpoint: ");
	Serial.println(Setpoint);

#endif // DEBug

	//Setpoint = 1300;
	//turn the PID on
	myPID.SetMode(AUTOMATIC);
	tiptempRaw = solderTemp(5);
	u8g2.begin();
	u8g2.setFlipMode(1);
	u8g2.setFont(u8g2_font_logisoso58_tr); // choose a suitable font

	//printOled(888);
}

void loop()
{

	if (millis() - DisplayUpdateTimer > 300)
		printOled(tiptempRaw);
	Setpoint = changeTemp();
	double gap = abs(Setpoint - tiptempRaw); //distance away from setpoint
	if (gap < 25)
	{ //we're close to setpoint, use conservative tuning parameters
		myPID.SetTunings(consKp, consKi, consKd);
		if (millis() - sampleTime > 20)
		{
			tiptempRaw = solderTemp(5);
			sampleTime = millis();
		}
	}
	else
	{
		if (millis() - sampleTime > 500)
		{
			tiptempRaw = solderTemp(5);
			sampleTime = millis();
		}
		//we're far from setpoint, use aggressive tuning parameters
		myPID.SetTunings(aggKp, aggKi, aggKd);
	}
	myPID.Compute();
	if (!atRest)
		analogWrite(solderPWMPin, Output);

#if defined(DEBUG)

	Serial.print("Output: ");
	Serial.println(Output);
	Serial.print("Setpoint: ");
	Serial.println(Setpoint);
	Serial.print("tiptempRaw: ");
	Serial.println(tiptempRaw);
#endif // DEBUG
}

int solderTemp(int samples)
{

	if (samples < 5 || samples > 20)
		return -0.00;
	byte highbyte, lowbyte;
	int data[samples];
	analogWrite(solderPWMPin, 0);
	delay(25);
	Wire.beginTransmission(adsAddr);
	Wire.write(129); //constant sampling on
	Wire.endTransmission();

	for (int i = 0; i < samples; i++)
	{

		Wire.beginTransmission(adsAddr); // Begin transmission to the Sensor
		Wire.requestFrom(adsAddr, 2);	// Request the transmitted two bytes from the two registers
		Wire.endTransmission();			 // Ends the transmission and transmits the data from the two registers

		while (Wire.available()) // ensure all the data comes in
		{
			highbyte = Wire.read(); // high byte * B11111111
			lowbyte = Wire.read();  // low byte
									//configRegister = Wire.read();
									//lastbyte = Wire.read(); // low byte
		}
		data[i] = ((highbyte << 12) | (lowbyte << 4)) >> 4;

		//data = (highbyte << 8 ) + lowbyte;

		////Serial.print("data BIN:");
		////Serial.println(data,BIN);
		/*   //Serial.print("data DEC:");
    //Serial.println(data[i], DEC);

    //Serial.print("Volt:");
    //Serial.println(data[i] * 0.00245); */
	}
	sortArray(data, samples);
	int sorted = 0;
	for (int i = 2; i < (samples - 2); i++)
	{
		sorted += data[i];
	}
	sorted = sorted / (samples - 4);
	//Serial.print("sorted and mean:");
	//Serial.println(sorted);
	////Serial.println();
	Wire.beginTransmission(adsAddr);
	Wire.write(128); //singel sampling on
	Wire.endTransmission();
	double returnTemperature = convertTemp(sorted);
	if (sorted < 800)
		atRest = 1;
	else
		atRest = 0;
	return returnTemperature;
}

void printOled(int input)
{
	u8g2.clearBuffer(); // clear the internal memory
	char Disp[4];

	//Serial.println(input);
	if (atRest)
	{
		u8g2.drawStr(0, 63, "RST"); // write something to the internal memory
	}
	else if (input > 2040)
	{
		u8g2.drawStr(0, 63, "TIP"); // write something to the internal memory
	}
	else
	{
		strcpy(Disp, u8x8_u8toa(input, 3)); /* convert m to a string with two digits */

		// u8g2.setFont(u8g2_font_logisoso62_tn);
		u8g2.drawStr(0, 63, Disp);
	}
	u8g2.sendBuffer(); // transfer internal memory to the display
	DisplayUpdateTimer = millis();
}

double changeTemp()
{
	double newSetPoint = Setpoint;
	static int counts;
	if (digitalRead(BTN_UP))
		newSetPoint++;
	else if (digitalRead(BTN_DN))
		newSetPoint--;
	if (Setpoint != newSetPoint)
	{
		long unsigned int timeout = millis();
		while (millis() - timeout < 500)
		{
			if (digitalRead(BTN_UP))
			{
				newSetPoint = newSetPoint + 5;
				printOled(newSetPoint);
				timeout = millis();
			}
			else if (digitalRead(BTN_DN))
			{
				newSetPoint = newSetPoint - 5;
				printOled(newSetPoint);
				timeout = millis();
			}
			if (newSetPoint < 100)
			{
				newSetPoint = 100;
			}
			else if (newSetPoint > 450)
			{
				newSetPoint = 450;
			}
		}
		eepromTimer = millis();
		updateEEPRoM(newSetPoint);
		counts++;
		Serial.println(counts);
	}
	return newSetPoint;
}

void updateEEPRoM(double newSetPoint)
{
	if (millis() - eepromTimer > 3000)
	{
		if (newEEPROMaddrFlag == 0) //each time we start the program and save to the EEPROM we cycle the adress. So the EEPROM dosent get worn out.
		{
			EEPROMsetpointAddr = EEPROMsetpointAddr + 10;
			if (EEPROMsetpointAddr > 100)
				EEPROMsetpointAddr = 10;
			EEPROM.put(EEPROMstartAddr, EEPROMsetpointAddr);

			newEEPROMaddrFlag = 1;
		}
		EEPROM.put(EEPROMsetpointAddr, newSetPoint);
	}
}

double convertTemp(int rawReading)
{
	rawReading = rawReading + 3; //guessing that +3 is a good aproximation of the thermal coupling loss when measured with another soldering bolt. i dont want to overshoot.
	double temperature = 0;
	/*
ADC		temp(C)
973 	10
979		12
						1158	20
997		37
1009	57
						--1003	70
1026	80I
1185	200
1258	250
1285	280
1320	300
1360	330
1388	350
1420	370
1465	400
1495	420
1535	450

*/
	if (rawReading < 997)
	{
		temperature = map(rawReading, 950, 996, 10, 36);
	}
	else if (rawReading < 1026)
	{
		temperature = map(rawReading, 997, 1025, 37, 80);
	}
	else if (rawReading < 1185)
	{
		temperature = map(rawReading, 1026, 1185, 80, 200);
	}
	else if (rawReading < 1258)
	{
		temperature = map(rawReading, 1185, 1258, 200, 250);
	}
	else if (rawReading < 1285)
	{
		temperature = map(rawReading, 1258, 1285, 250, 280);
	}
	else if (rawReading < 1320)
	{
		temperature = map(rawReading, 1285, 1320, 280, 300);
	}
	else if (rawReading < 1360)
	{
		temperature = map(rawReading, 1320, 1360, 300, 330);
	}
	else if (rawReading < 1388)
	{
		temperature = map(rawReading, 1360, 1388, 330, 350);
	}
	else if (rawReading < 1420)
	{
		temperature = map(rawReading, 1388, 1420, 350, 370);
	}
	else if (rawReading < 1465)
	{
		temperature = map(rawReading, 1420, 1465, 370, 400);
	}
	else if (rawReading < 1495)
	{
		temperature = map(rawReading, 1465, 1495, 400, 420);
	}
	else if (rawReading < 1500)
	{
		temperature = map(rawReading, 1495, 1535, 420, 450);
	}
	return temperature;
}