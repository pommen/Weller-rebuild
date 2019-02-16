
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <PID_v1.h>

//Pins:
const unsigned int heater1 = PA0;
const unsigned int heater2 = PA1;
const unsigned int BTN_UP = PA3;
const unsigned int BTN_DN = PA2;

//Protos:
//Temperature conversions
int getTemp();

//Globals:
float Setpoint, Input, Output, Kp = 2, Ki = 5, Kd = 1;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
void setup(void)
{
	pinMode(heater1, OUTPUT);
	pinMode(heater2, OUTPUT);
	pinMode(BTN_DN, INPUT);
	pinMode(BTN_UP, INPUT);
	Serial.begin(9600);
	Wire.begin();
	// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
	//ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
	// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
	ads.setGain(GAIN_FOUR); // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
	// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
	// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

	ads.begin();
	myPID.SetMode(AUTOMATIC);
}

void loop(void)
{
	int now = millis();
	getTemp();
	Serial.println(millis() - now);
	delay(500);
}

int getTemp()
{

	/*
Approx 24 ms for checking both Differential channels. 12 ms for one diff channel.

	*/

	int16_t results;

	results = ads.readADC_Differential_0_1();

	if (results < 0)
		results = results * -1;
	/* Serial.println("Diff01:         Diff23:");
	Serial.print(results);
	Serial.print("           "); */
	//delay(10);
	/* results = ads.readADC_Differential_2_3();

	if (results < 0)
		results = results * -1; */
	//Serial.println(results);

	//delay(250);
	//	delay(250);
}