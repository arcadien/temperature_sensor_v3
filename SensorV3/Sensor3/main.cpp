/*Begining of Auto generated code by Atmel studio */

/*End of auto generated code by Atmel studio */

#define ARDUINO_AVR_ATTINYX4
#define F_CPU 8000000UL


#include "Arduino.h"
#include "util/delay.h"

// DS18B20
#include "OneWire.h"
#include "DallasTemperature.h"

#include "oregon.h"

#include "avr/sleep.h"
#include "avr/wdt.h"
#include <avr/eeprom.h>

//Beginning of Auto generated function prototypes by Atmel Studio
ISR(WDT_vect );
void init_wdt();
void sleep(uint16_t s);
void SenseAndSave();
void PrintInfo();
uint16_t ADCRead(uint8_t admux);
void ActivatePeripherals();
void DeactivatePeripherals();
void Sleep();
void SendData433();
//End of Auto generated function prototypes by Atmel Studio

// digital
#define LED				3  // PA7
#define TX_433			7  // PA3
#define SENSOR_VCC		0  // PB0
#define PERIPH_GND      2  // PB2

//#define USE_DS18B20 

#ifdef USE_DS18B20
	#define ONE_WIRE_BUS    1  // PB1

	#define TEMPERATURE_PRECISION 12
	OneWire oneWire(ONE_WIRE_BUS);

	// Pass our oneWire reference to Dallas Temperature.
	DallasTemperature sensors(&oneWire);
	// arrays to hold device addresses
	DeviceAddress insideThermometer;

	#define OREGON_ID_DS18B20 0xBC

#endif

// Analog
#define TEMP_SENSOR		A0 // PA0
#define TEMP_SENSOR_REF A1 // PA1
#define BATTERY_VOLTAGE A2 // PA2

#define COMMAND_REPEAT_COUNT 5

// Unique ID of device
#define OREGON_ID_LM35    0xBB


/*
0x0A4D	Inside Temperature
0xEA4C	Outside/Water Temp
0xCA48	Water Temp
0x1A2D	Inside Temp-Hygro
0xFA28	Inside Temp-Hygro
0x*ACC	Outside Temp-Hygro
0xCA2C	Outside Temp-Hygro
0xFAB8	Outside Temp-Hygro
0x1A3D	Outside Temp-Hygro
0x5A5D	Inside Temp-Hygro-Baro
0x5A6D	Inside Temp-Hygro-Baro
0x2A1D	Rain Gauge
0x2A19	Rain Gauge
0x1A99	Anemometer
0x1A89	Anemometer
0x3A0D	Anemometer
0xEA7C	UV sensor
0xDA78	UV sensor
0x*AEC	Date & Time
0xEAC0	Ampere meter
0x[1,2,3]A** Power meter
*/

uint8_t OREGON_TYPE[] = {0x1A,0x2D}; // inside temp

Oregon oregon(LED, TX_433, COMMAND_REPEAT_COUNT);

float lastLm35temp = 0;
Oregon::Message lm35_message;

#ifdef USE_DS18B20
	float lastDs18b20temp = 0;
	bool lastDs18b20Found = false;
	Oregon::Message ds18B20_message;
#endif

uint8_t sleep_interval;

ISR(WATCHDOG_vect)
{
	wdt_reset();
	sleep_interval++;
	// Re-enable WDT interrupt
	_WD_CONTROL_REG |= (1<<WDIE);
}



// Enable watchdog interrupt, set prescaling to 1 sec
void init_wdt()
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	// Disable interrupts
	cli();
	
	MCUSR &= ~(1 << WDRF);
	
#ifdef ARDUINO_AVR_ATTINYX4
	// Start timed sequence
	// Set Watchdog Change Enable bit
	_WD_CONTROL_REG |= (1<<WDCE) | (1<<WDE);

	// Set new prescaler (8 sec), unset reset enable
	_WD_CONTROL_REG = (1<<WDP3)|(1<<WDP0); // 8 sec
	//_WD_CONTROL_REG = (1<<WDP1)|(1<<WDP2); // 1 sec
	
	// enable WDT interrupt
	_WD_CONTROL_REG |= (1 << WDIE);
#else
	#warning "Unknown target: the wadchdog is not configured"
#endif
	sei();
}

// Puts MCU to sleep for specified number of seconds using
// WDT to wake every second and track number of seconds
void sleep(uint16_t s)
{
	s /= 8;
	if(s == 0 ) s = 1;
	sleep_interval = 0;
	while (sleep_interval < s) {
		sleep_mode();
		sleep_disable();
	}
}

void setup()
{
	pinMode(LED, OUTPUT);
	pinMode(PERIPH_GND, OUTPUT);
	pinMode(TX_433, OUTPUT);
	pinMode(SENSOR_VCC, OUTPUT);
	
	pinMode(TEMP_SENSOR, INPUT);
	pinMode(TEMP_SENSOR_REF, INPUT); 
	
	digitalWrite(PERIPH_GND, HIGH);
	
	init_wdt();
 
 #ifdef USE_DS18B20
	// One wire
    sensors.begin();
	if (!sensors.getAddress(insideThermometer, 0))
	{
			digitalWrite(LED, HIGH);
			_delay_ms(200);
			digitalWrite(LED, LOW);
			_delay_ms(200);
			digitalWrite(LED, HIGH);
			_delay_ms(200);
			digitalWrite(LED, LOW);
	}
	else
	{
		lastDs18b20Found = true;
		sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
	}
#endif
}


// return Vcc in Volts (ex. 3.323)
float GetVccUsingInternalRef()
{
	float vcc = 0;
	const uint8_t sampleCount = 16;
	uint16_t accumulator = 0;

	ADMUX = 0;
	_delay_us(4);

	ADMUX = (1 << MUX5) | (1 << MUX0);
	_delay_us(400);

	// stabilization reads, discarded
	// TODO ensure useful
	for(uint8_t i = sampleCount ; i >0; --i)
	{
		ADCSRA |= (1 << ADSC);
		while(ADCSRA & (1 << ADSC));
	}
	for(uint8_t i = sampleCount ; i >0; --i)
	{
		accumulator += ADC;
	}
	vcc = (accumulator >> 4);

	vcc = (1023 * 1.1) / vcc; //1v1 ref should be adjusted

	return vcc;
}

void SenseAndSave()
{

	uint16_t accumulator = 0;
	float sensorVoltage = 0;
	float refVoltage    = 0;
	//float vcc  = GetVccUsingInternalRef();
	float vcc  = 5.0;
		
	// to later divide by 16, just >> 4
	// because 2^4 == 16
	const uint8_t sampleCount = 16;

	analogRead(TEMP_SENSOR);
	accumulator = 0;
	for(uint8_t sampleCounter = sampleCount; sampleCounter > 0; --sampleCounter)
	{
		accumulator += analogRead(TEMP_SENSOR);
	}
	sensorVoltage = (accumulator >> 4);
	
	analogRead(TEMP_SENSOR_REF);
	accumulator = 0;
	for(uint8_t sampleCounter = sampleCount; sampleCounter > 0; --sampleCounter)
	{
	accumulator += analogRead(TEMP_SENSOR_REF);
	}
	refVoltage  = (accumulator >> 4);
	
	float temperature_celcius = (sensorVoltage - refVoltage) * (vcc / 1023.0 * 100.0);
	
	lm35_message.temperature = temperature_celcius;
	
#ifdef USE_DS18B20

	if(lastDs18b20Found)
	{	
		sensors.requestTemperatures();
		ds18B20_message.temperature = sensors.getTempC(insideThermometer);
	}
#endif

}

void PrintInfo(){}


uint16_t ADCRead(uint8_t admux)
{
	ADMUX = admux;
	
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	
	return ADC;
}

void ActivatePeripherals()
{
	
	// ADC Enable
	ADCSRA |= (1<<ADEN);
	
	// http://www.ti.com/lit/ds/symlink/lm35.pdf p.12
	// (LM35 startup response)
	digitalWrite(SENSOR_VCC, HIGH);
	_delay_ms(30);
}

/*!
* Cut power of the peripherals : sensors, 433Mhz emitter
*/
void DeactivatePeripherals()
{
	ADCSRA &= ~(1<<ADEN);
	digitalWrite(PERIPH_GND, LOW);
	digitalWrite(SENSOR_VCC, LOW);
}

void Sleep()
{
	sleep(300); // 300s == 5min
}

/*!
* Send data in internal storage
*/
void SendData433()
{
#ifdef USE_DS18B20
	bool ds18b20tempHasChanged = ds18B20_message.temperature != lastDs18b20temp;
#else
	bool ds18b20tempHasChanged = false;
#endif

	bool lmTempHasChanged = lm35_message.temperature != lastLm35temp;
	bool aTemperatureHasChanged = (lmTempHasChanged || ds18b20tempHasChanged);
	if(aTemperatureHasChanged)
	{
		digitalWrite(PERIPH_GND, HIGH);
		if(lmTempHasChanged)
		{
			lastLm35temp = lm35_message.temperature;
			oregon.Emit(OREGON_TYPE, Oregon::Channel::ONE, OREGON_ID_LM35, lm35_message);
		}

#ifdef USE_DS18B20
	
		if(ds18b20tempHasChanged)
		{
			lastDs18b20temp = ds18B20_message.temperature;
			oregon.Emit(OREGON_TYPE, Oregon::Channel::ONE, OREGON_ID_DS18B20, ds18B20_message);
		}
#endif		
		digitalWrite(PERIPH_GND, LOW);
	}
}


void loop()
{
	digitalWrite(LED, HIGH);
	ActivatePeripherals();
	SenseAndSave();
	SendData433();
	DeactivatePeripherals();
	digitalWrite(LED, LOW);
	Sleep();
}

int main(void)
{
	wdt_disable();
	setup();

	while (1)
	{
		loop();
	}
	return 0;
}
