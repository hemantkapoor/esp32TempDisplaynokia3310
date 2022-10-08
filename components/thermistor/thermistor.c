#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include <driver/adc.h>
#include "thermistor.h"


// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000.0f
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 1
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000.0F
#define VREF_MV (5000.0F)

#define GPIO_THERMISTOR_PIN    GPIO_NUM_32
#define GPIO_THERMISTOR_PIN_SEL (1ULL << GPIO_THERMISTOR_PIN)

#define NUMBER_OF_SAMPLES (20)


static esp_adc_cal_characteristics_t characteristics;


static void configureThermistorPin(void);

void initThermistor(void)
{


    // Configure ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    /* ADC1 8 channels: GPIO32 - GPIO39 */
    /* We have connected thermistor to pin 34 */
    /* ADC1_CHANNEL_6,      ADC1 channel 6 is GPIO34 */
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);

    // Calculate ADC characteristics i.e. gain and offset factors
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,0, &characteristics);

    //Setup Thermistor pin
    configureThermistorPin();

}

float getThermistorValue(void)
{
	float Thermistor_temperature = 0.0f;
	uint32_t currentReading;
	uint32_t reading = 0;
    uint32_t samples = 0;

	gpio_set_level(GPIO_THERMISTOR_PIN, 1);
     esp_err_t status;
    for(samples=0;samples<NUMBER_OF_SAMPLES;++samples)
    {
	    status  = esp_adc_cal_get_voltage(ADC1_CHANNEL_6, &characteristics, &currentReading);
        if(status != ESP_OK) { break; }
        reading += currentReading;
    }
    reading = reading / NUMBER_OF_SAMPLES;
	if(status == ESP_OK)
	{

		float reading_v = reading / 1000.0f;

		printf("Voltage Reading = %f\r\n volts",reading_v);

		//		reading = SERIESRESISTOR / reading;  // 10K / (1023/ADC - 1)
		float Rt = 10.0f * reading_v / (3.3f - reading_v);
		//float Rt = reading_v / 10;
		printf("Resistance calcukated = %d\r\n",reading);


		float tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);

		Thermistor_temperature = tempK - 273.15;
		printf("Temperature = %f\r\n",Thermistor_temperature);

	}
	gpio_set_level(GPIO_THERMISTOR_PIN, 0);


//	long NTC_Resistance = ((1023*Series_Resistance/analog_value) - Series_Resistance);
//	Thermistor_temperature = log(NTC_Resistance);	/* calculate natural log of resistance */
//
//    /* Calculate Temperature using B parameter Equation */
//    /* 1/T = 1/T0 + ((1/B_coefficient)*log(NTC_Resistance/Series_Resistance)) */
//    Thermistor_temperature = (1.0/(Room_temperature + 273.15)) + ((1.0/B_coefficient)*log(NTC_Resistance/Series_Resistance));
//    Thermistor_temperature = (1/Thermistor_temperature) - 273.15;	/* convert kelvin to °C */
//

	 return Thermistor_temperature;

}


/*********** Static function definition here ************************/
static void configureThermistorPin(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_THERMISTOR_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
