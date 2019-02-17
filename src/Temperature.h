//Based on  https://github.com/FlyGlas/WMRP

//ADC TO GRIP TEMPERATURE CONVERSION
#define ADC_TO_T_GRIP_A0 289     //                  T[degC] = (U[V] - 1.1073) * 1000/6.91
#define ADC_TO_T_GRIP_A1 -160246 //                     U[V] = adc_value * 4 V/1000 * 1/OPAMP_T_GRIP_GAIN
//                                             10^3 * T[degC] = adc_value * 289.435 - 160246

//ADC TO INPUT VOLTAGE CONVERSION
#define ADC_TO_U_IN_A0 14213 //                 U_adc[V] = U_in[V] * 4700/(4700 + 12000);
//                                                   U_adc[V] = adc_value * 4 V/1000
//                                                    U_in[V] = adc_value * 4 V/1000 *(4700 + 12000)/4700
//                                             10^6 * U_in[V] = adc_value * 14212,77

//ADC TO INPUT CURRENT CONVERSION
#define ADC_TO_I_HEATER_A0 18182 //                U_adc[mV] = UVCC/2[mV] + I_heater[A] * 220 mV/A
//                                                  U_adc[mV] = adc_value * 4 mV
//                                                 UVCC/2[mV] = adc_value(@i=0A) * 4 mV
//                                                I_heater[A] = (adc_value * 4 mV - UVCC/2[mV])/(220 mV/A)
//                                         10^6 * I_heater[A] = (adc_value - adc_value(@i=0A)) * 18182

#define ADC_T_TIP_OFFSET_COMP 21 //offset measured when thermocouple and cold junction are at the same temperature

static float TempC_to_Emf_TypeD[6];
static float Emf_to_TempC_TypeD[7];

long temperature_tip_absolute;
int adc_temperature_tip_relative;
int adc_temperature_tip_relative_old;
int adc_temperature_grip;
long temperature_grip;
int adc_voltage_input;
long voltage_input;

void tempSetup()
{
    //DEFINE COEFFICIENTS
    TempC_to_Emf_TypeD[0] = 0.0000000E+0;
    TempC_to_Emf_TypeD[1] = 9.5685256E-3;
    TempC_to_Emf_TypeD[2] = 2.0592621E-5;
    TempC_to_Emf_TypeD[3] = -1.8464576E-8;
    TempC_to_Emf_TypeD[4] = 7.9498033E-12;
    TempC_to_Emf_TypeD[5] = -1.4240735E-15;

    Emf_to_TempC_TypeD[0] = 7.407745239E-1;
    Emf_to_TempC_TypeD[1] = 9.827743947E+1;
    Emf_to_TempC_TypeD[2] = -1.223274978E+1;
    Emf_to_TempC_TypeD[3] = 1.930483337E+0;
    Emf_to_TempC_TypeD[4] = -1.825482304E-1;
    Emf_to_TempC_TypeD[5] = 9.140368153E-3;
    Emf_to_TempC_TypeD[6] = -1.851682378E-4;
}

float PolyEval(float x, float *Coeffs, unsigned int Degree)
{
    float pol = 0.0;
    for (int i = Degree; i >= 0; i--)
    {
        pol = pol * x + Coeffs[i];
    }
    return pol;
}

long adc_to_t_grip_calc(int adc_value)
{
    long t_grip = (ADC_TO_T_GRIP_A0 * (long)adc_value) + ADC_TO_T_GRIP_A1;
    Serial.println((float)t_grip / 1000, 3);
    return t_grip;
}

long adc_to_u_in_calc(int adc_value)
{
    long u_in = (ADC_TO_U_IN_A0 * (long)adc_value);
    //Serial.println((float)u_in / 1000000, 3);
    return u_in;
}

long adc_to_i_heater_calc(int adc_value, int adc_value_offset)
{
    long i_heater = ADC_TO_I_HEATER_A0 * ((long)adc_value - (long)adc_value_offset);
    //Serial.println((float)i_heater / 1000000, 3);
    return i_heater;
}
float getTemp()
{
    adc_temperature_grip = ADCRaw(1);
    //calculate SI values from the adc value
    temperature_grip = adc_to_t_grip_calc(adc_temperature_grip); //temperature in grad celsius divide with 1000
  //  voltage_input = adc_to_u_in_calc(adc_voltage_input);         //voltage in volt divide with 10^6
    //current_heater = adc_to_i_heater_calc(adc_current_heater, adc_current_heater_offset); //current in volt divide with 10^6

    //calculate real tip temperature as sum of thermo couple voltage and cold junction voltage
    temperature_tip_absolute = 1000 * PolyEval((float)adc_temperature_tip_relative * 4.0 / 431.0 + PolyEval((float)temperature_grip / 1000.0, TempC_to_Emf_TypeD, 5), Emf_to_TempC_TypeD, 6); //temperature in grad celsius divide with 1000

    /*
Approx 24 ms for checking both Differential channels. 12 ms for one diff channel.


todo:

sample handle temperature every 2'nd second or so, to use as cold junction compensation.
translate ADC to temperature.

return temperature in celcius (as a int)

    temperature_tip_absolute = 1000 * PolyEval((float)adc_temperature_tip_relative * 4.0 / 431.0 + PolyEval((float)temperature_grip / 1000.0, TempC_to_Emf_TypeD, 5), Emf_to_TempC_TypeD, 6); //temperature in grad celsius divide with 1000




Handle grader:
adc 25900 	grader 20 	resistans 965 ohm

R=adc/26.8393

kty82/110
C	coefK%	Rmin	Rtyp	Rmax
10  0.83 	874 	886 	898 
20  0.80 	950 	961 	972  7.8 R/C
25  0.79 	990 	1000 	1010 
30  0.78 	1029 	1040 	1051 
40  0.75 	1108 	1122 	1136 8.2 R/C
50  0.73 	1192 	1209 	1225 
60  0.71 	1278 	1299 	1319 
70  0.69 	1369 	1392 	1416 
80  0.67 	1462 	1490	1518 
90  0.65 	1559 	1591 	1623 
100  0.63 	1659 	1696 	1733






tip grader:
notip adc counts = 31110
20 grader = 19062 adc counts
60 grader=19145
75 grader =19225


	analogWrite(heater1, 0);
	analogWrite(heater2, 0);

	delay(12);

	int16_t results;

	results = ads.readADC_Differential_0_1();

	//ensure its always positive
	if (results < 0)
	{
		results = results * -1;
	}
	if (results > 31100)
	{
		notip1 = 1;
		return 0;
	}

	analogWrite(heater1, OUTPUT);
	analogWrite(heater2, OUTPUT);

	if (results < 0)
		results = results * -1;
	//map(results, fromLow, fromHigh, 20, 80);
	return results;



    for (size_t i = 0; i < 5; i++)
		{
			int results = ads.readADC_Differential_2_3();
			delay(12);
			if (results < 0)
			{
				results = results * -1;
			}
			tempReadings[i] = results;
		}

		sortArray(tempReadings, 5);
		int largest = tempReadings[0];
		Serial.print("Largest: ");
		Serial.println(largest);
		int smallest = tempReadings[4];
		Serial.print("smallest: ");
		Serial.println(smallest);
		float avg = 0.0;
		for (size_t i = 0; i < 5; i++)
		{
			avg = avg + tempReadings[i];
		}
		avg = avg / 5;
		Serial.print("Mean: ");
		Serial.println(avg);

		//	samples 0.54

		float resistor = 10100.0;
		float kty82VoltageDivider = ((avg * 0.03125) * 0.541756);
		Serial.print(kty82VoltageDivider);
		Serial.println("mV");
		float resistance = resistor * 1 / (5.00 / (kty82VoltageDivider / 1000) - 1);
		Serial.print("resistance: ");
		Serial.println(resistance);
		delay(2000);


	*/
}

int ADCRaw(int channel)
{

    // channel 0== tip adc, channel 1 == grip adc

    int tempReadings[5];
    if (channel == 0)
    {

        for (size_t i = 0; i < 5; i++)
        {
            int results = ads.readADC_Differential_0_1();
            delay(12);
            if (results < 0)
            {
                results = results * -1;
            }
            tempReadings[i] = results;
        }
    }
    else
    {

        for (size_t i = 0; i < 5; i++)
        {
            int results = ads.readADC_Differential_2_3();
            delay(12);
            if (results < 0)
            {
                results = results * -1;
            }
            tempReadings[i] = results;
        }
    }

    sortArray(tempReadings, 5);
   //     int largest = tempReadings[0];
    // Serial.print("Largest: ");
    // Serial.println(largest);
    //int smallest = tempReadings[4];
    // Serial.print("smallest: ");
    // Serial.println(smallest);
    int avg = 0.0;
    for (size_t i = 0; i < 5; i++)
    {
        avg = avg + tempReadings[i];
    }
    avg = avg / 5;
  //  Serial.print("avg: ");
    //Serial.println(avg);
    int val =map(avg, 0, 32767, 0, 1023);
   // Serial.println(val);
    return val;
}