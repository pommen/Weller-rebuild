
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <PID_v1.h>
#include <ArduinoSort.h>

//Pins:
const unsigned int heater1 = PA0;
const unsigned int heater2 = PA1;
const unsigned int BTN_UP = PA3;
const unsigned int BTN_DN = PA2;

//Protos:
//Temperature conversions
void pidAndPWM();
void setpoint();
int ADCRaw(int channel);
//Globals:
//Define the aggressive and conservative Tuning Parameters
float Setpoint, InputHeater1, InputHeater2, OutputHeater1, OutputHeater2;
float aggKp = 4, aggKi = 0.2, aggKd = 1;
float consKp = 1, consKi = 0.05, consKd = 0.25;

//Timers:
long unsigned int pidTimer = 0;

//Flags:

boolean notip1 = 0;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
PID PIDHeater1(&InputHeater1, &OutputHeater2, &Setpoint, consKp, consKi, consKd, DIRECT);
PID PIDHeater2(&InputHeater2, &OutputHeater2, &Setpoint, consKp, consKi, consKd, DIRECT);

#include <Temperature.h>
void setup(void)
{
	pinMode(heater1, OUTPUT);
	pinMode(heater2, OUTPUT);
	pinMode(BTN_DN, INPUT);
	pinMode(BTN_UP, INPUT);
	Serial.begin(9600);
	Wire.begin();
	//ads.setGain(GAIN_TWOTHIRDS); // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
	ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
	// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
	//ads.setGain(GAIN_FOUR); // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
	// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
	// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

	ads.begin();
	//analogWriteResolution()
	PIDHeater1.SetMode(AUTOMATIC);
	//myPID.SetInput
	//myPID.SetOutputLimits(0,65535);
	pidTimer = millis();

	tempSetup();

	while (1)
	{
		getTemp();
		Serial.print("HAndle: ");

		Serial.println((float)temperature_grip / 1000, 3);
		Serial.println();
		delay(1000);
	}

	Serial.println("Setup Done!");
}

void loop(void)
{
}

void pidAndPWM()
{
	InputHeater1 = getTemp();

	//no tip = no pid!
	if (InputHeater1 == 0)
	{
		OutputHeater1 = 0;
		OutputHeater2 = 0;
		return;
	}

	int gap = abs(Setpoint - InputHeater1); //distance away from setpoint
	if (gap < 50)
	{ //we're close to setpoint, use conservative tuning parameters
		PIDHeater1.SetTunings(consKp, consKi, consKd);
	}
	else
	{
		//we're far from setpoint, use aggressive tuning parameters
		PIDHeater1.SetTunings(aggKp, aggKi, aggKd);
	}

	PIDHeater1.Compute();
	analogWrite(heater1, OutputHeater1);
}

void setpoint()
{
}

void display()
{
}
