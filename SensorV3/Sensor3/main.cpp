/*Begining of Auto generated code by Atmel studio */

/*End of auto generated code by Atmel studio */

#define ARDUINO_AVR_ATTINYX4
#define F_CPU 8000000UL

#include "Arduino.h"
#include "util/delay.h"

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
#define EXT_FLAG		1  // PB1
#define PERIPH_GND      2  // PB2

// Analog
#define TEMP_SENSOR		A0 // PA0
#define TEMP_SENSOR_REF A1 // PA1
#define BATTERY_VOLTAGE A2 // PA2

#define TEMP_SENSOR_ID    0x00
#define VOLTAGE_SENSOR_ID 0x01
#define DEVICE_ID		  0x01

#define COMMAND_REPEAT_COUNT 2

// Unique ID of device
#define OREGON_ID 0xBB

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
Oregon::Message message;


uint8_t sleep_interval;

 enum  SENSOR_VALUES
{
	 TEMPERATURE,
	 HUMITIDY,
	 VOLTAGE,
	 SENSOR_VALUES_COUNT
};

float SENSOR_DATA[SENSOR_VALUES_COUNT];

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
	pinMode(LED, OUTPUT);
	pinMode(TX_433, OUTPUT);
	pinMode(SENSOR_VCC, OUTPUT);
	pinMode(EXT_FLAG, OUTPUT);
	
	pinMode(TEMP_SENSOR, INPUT);
	pinMode(TEMP_SENSOR_REF, INPUT);
	
	SENSOR_DATA[TEMPERATURE] = 0;
	SENSOR_DATA[HUMITIDY] = 0;
	SENSOR_DATA[VOLTAGE] = 0;
	 
	
	digitalWrite(PERIPH_GND, HIGH);
	
	init_wdt();

	//Serial.begin(4800);   // set the highest standard baud rate of 115200 bps
}


float GetVccUsingInternalRef()
{
	float vcc = 0;
	
	//// internal 1.1V reference against AVcc
	uint8_t admux = (1 << MUX5) | (1 << MUX0);
	ADCRead(admux); // first discarded read
	for(uint8_t i = 0 ; i < 100; ++i)
	{
	vcc += ADCRead(admux);
	}
	vcc /= 100;
	// Back-calculate AVcc in mV
	vcc = (1023 * 1.1) / vcc; //1v1 ref should be adjusted

	return vcc;
}

void SenseAndSave()
{

	float sensorVoltage = 0;
	float refVoltage    = 0;
	// 	float vcc  = GetVccUsingInternalRef();
	const float vcc = 5.0;

	analogRead(TEMP_SENSOR);
	sensorVoltage = analogRead(TEMP_SENSOR);	

	analogRead(TEMP_SENSOR_REF);
	refVoltage = analogRead(TEMP_SENSOR_REF);
	
	float temperature_celcius = (sensorVoltage - refVoltage) * (vcc / 1023.0 * 100.0);
	
	message.temperature = temperature_celcius;
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
	digitalWrite(PERIPH_GND, HIGH);
	
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
	oregon.Emit(OREGON_TYPE, Oregon::Channel::ONE, OREGON_ID, message);
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
