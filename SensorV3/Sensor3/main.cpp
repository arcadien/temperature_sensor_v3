/*Begining of Auto generated code by Atmel studio */

/*End of auto generated code by Atmel studio */

//#undef F_CPU
//#define F_CPU 8000000UL

#include "Arduino.h"
#include "util/delay.h"

#include "Manchester.h"
#include "protocol.h"

#include "avr/sleep.h"
#include "avr/wdt.h"
#include <avr/eeprom.h>

//Beginning of Auto generated function prototypes by Atmel Studio
ISR(WDT_vect );
void init_wdt();
void sleep(uint8_t s);
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

// #define CALIBRATION true

#ifdef CALIBRATION
#define SENSOR_COUNT	  3
#define	DELAY_EMISSION_ID 0x02
#else
#define SENSOR_COUNT	  2
#endif

#define TEMP_SENSOR_ID    0x00
#define VOLTAGE_SENSOR_ID 0x01
#define DEVICE_ID		  0x01

uint8_t sleep_interval;

uint8_t SENSOR_DATA[SENSOR_COUNT][MESSAGE_SIZE];

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
  
  // Start timed sequence
  // Set Watchdog Change Enable bit
  _WD_CONTROL_REG |= (1<<WDCE) | (1<<WDE);

  // Set new prescaler (1 sec), unset reset enable
  // enable WDT interrupt
  _WD_CONTROL_REG = (1<<WDP3)|(1<<WDP0); // 8 sec
  //_WD_CONTROL_REG = (1<<WDP1)|(1<<WDP2); // 1 sec
  _WD_CONTROL_REG |= (1 << WDIE);
  
  sei();
}

// Puts MCU to sleep for specified number of seconds using
// WDT to wake every second and track number of seconds
void sleep(uint8_t s)
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
  
  man.setupTransmit(TX_433, MAN_4800);
  
  man.delay1  = 93;
  man.delay2  = 98;
  
  digitalWrite(PERIPH_GND, HIGH);
  
  init_wdt();

  //Serial.begin(4800);   // set the highest standard baud rate of 115200 bps (mini pro uses x2)
}


void SenseAndSave()
{

  uint16_t sensorVoltage = 0;
  uint16_t refVoltage	 = 0;
  uint16_t battery_adc   = 0;
  // 1. Read the actual Vcc
  
  //// internal 1.1V reference against AVcc
  uint8_t admux = (1 << MUX5) | (1 << MUX0); 
  _delay_ms(50);
  //
 for(uint8_t i = 0 ; i < 100; ++i)
     ADCRead(admux);	
  float vcc = ADCRead(admux);
  // Back-calculate AVcc in mV
  vcc = 1080 / vcc; // 1023 * internal 1v1 ref (1,05571v)

  analogReference(INTERNAL1V1);
   _delay_ms(50);
  for(uint8_t i = 0 ; i < 100; ++i)
	analogRead(TEMP_SENSOR);
  battery_adc = analogRead(TEMP_SENSOR);

  analogReference(DEFAULT);
  _delay_ms(50);
  for(uint8_t i = 0 ; i < 100; ++i)
	analogRead(TEMP_SENSOR);
  sensorVoltage = analogRead(TEMP_SENSOR);
  
  for(uint8_t i = 0 ; i < 100; ++i)
	analogRead(TEMP_SENSOR_REF);
  refVoltage = analogRead(TEMP_SENSOR_REF);
   
  float temperature_celcius = (sensorVoltage - refVoltage) * (vcc / 1023.0 * 1000);
  
  bool isNegative = (temperature_celcius < 0);

  uint16_t temperature_celcius2 = abs(temperature_celcius);
  uint8_t  integer = temperature_celcius2/10; // ie 26 in this sample
  uint8_t  fractional = temperature_celcius2 % 10; // ie 4 in this sample

  SENSOR_DATA[TEMP_SENSOR_ID][MESSAGE_SIZE_POS] = MESSAGE_SIZE;

  SENSOR_DATA[TEMP_SENSOR_ID][DEV_ID_POS]    = DEVICE_ID;
  SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_ID_POS] = TEMP_SENSOR_ID;

  SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_1_POS] = integer;
  SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_2_POS] = fractional;
  SENSOR_DATA[TEMP_SENSOR_ID][SENSOR_BYTE_3_POS] = (isNegative) ? 1 : 0;

  // get a mesurement in centivolts : ex. 154 for 1.54 v.
  // This format will be suitable for separation of floating point
  // part of the voltage, below
  float battery_voltage_mv = battery_adc * (0.0107421875);// 1.1 / 1023 * 10
  integer = battery_voltage_mv/10; // 
  fractional = temperature_celcius2 % 10;
  
  SENSOR_DATA[VOLTAGE_SENSOR_ID][MESSAGE_SIZE_POS] = MESSAGE_SIZE-1;
  SENSOR_DATA[VOLTAGE_SENSOR_ID][DEV_ID_POS]    = DEVICE_ID;
  SENSOR_DATA[VOLTAGE_SENSOR_ID][SENSOR_ID_POS] = VOLTAGE_SENSOR_ID;
  SENSOR_DATA[VOLTAGE_SENSOR_ID][SENSOR_BYTE_1_POS] = integer;
  SENSOR_DATA[VOLTAGE_SENSOR_ID][SENSOR_BYTE_2_POS] = fractional;
  //SENSOR_DATA[VOLTAGE_SENSOR_ID][SENSOR_BYTE_3_POS] = 0;
  
#ifdef CALIBRATION
   SENSOR_DATA[DELAY_EMISSION_ID][MESSAGE_SIZE_POS] = MESSAGE_SIZE-1;
   SENSOR_DATA[DELAY_EMISSION_ID][DEV_ID_POS]    = DEVICE_ID;
   SENSOR_DATA[DELAY_EMISSION_ID][SENSOR_ID_POS] = DELAY_EMISSION_ID;
   SENSOR_DATA[DELAY_EMISSION_ID][SENSOR_BYTE_1_POS] = man.delay1;
   SENSOR_DATA[DELAY_EMISSION_ID][SENSOR_BYTE_2_POS] = man.delay2;
#endif
}

void PrintInfo(){}


uint16_t ADCRead(uint8_t admux)
{
  ADMUX = admux;
  
  // first discarded read
  ADCSRA |= (1 << ADSC);
  while(ADCSRA & (1 << ADSC));
  
  ADCSRA |= (1 << ADSC);
  while(ADCSRA & (1 << ADSC));
  
  return ADC;
}

void ActivatePeripherals()
{
	digitalWrite(LED, HIGH);

#ifdef CALIBRATION

	if(man.delay1 < 100)
	{
		if(man.delay2 < 100)
		{
			man.delay2++;
		}
		else
		{
			man.delay2 = 87;
			man.delay1++;
		}
	}
	else
	{
		man.delay1 = 87;
	}
#endif

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
	digitalWrite(LED, LOW);
}

void Sleep()
{
	
#ifdef CALIBRATION
  _delay_ms(10);
#else
  sleep(8);
#endif
}

/*!
* Send data in internal storage
*/
void SendData433()
{

  // temperature 
  man.transmitArray(
	SENSOR_DATA[TEMP_SENSOR_ID][MESSAGE_SIZE_POS], 
	SENSOR_DATA[TEMP_SENSOR_ID]);
  _delay_ms(30);
    man.transmitArray(
    SENSOR_DATA[TEMP_SENSOR_ID][MESSAGE_SIZE_POS],
    SENSOR_DATA[TEMP_SENSOR_ID]);
    _delay_ms(30);
	  man.transmitArray(
	  SENSOR_DATA[TEMP_SENSOR_ID][MESSAGE_SIZE_POS],
	  SENSOR_DATA[TEMP_SENSOR_ID]);
	  _delay_ms(30);
	  
  // voltage
    man.transmitArray(
    SENSOR_DATA[VOLTAGE_SENSOR_ID][MESSAGE_SIZE_POS],
    SENSOR_DATA[VOLTAGE_SENSOR_ID]);
    _delay_ms(30);
	  man.transmitArray(
	  SENSOR_DATA[VOLTAGE_SENSOR_ID][MESSAGE_SIZE_POS],
	  SENSOR_DATA[VOLTAGE_SENSOR_ID]);
	  _delay_ms(30);
  man.transmitArray(
  SENSOR_DATA[VOLTAGE_SENSOR_ID][MESSAGE_SIZE_POS],
  SENSOR_DATA[VOLTAGE_SENSOR_ID]);
  _delay_ms(30);  
  
#ifdef CALIBRATION
  // delays
  man.transmitArray(
	SENSOR_DATA[DELAY_EMISSION_ID][MESSAGE_SIZE_POS], 
	SENSOR_DATA[DELAY_EMISSION_ID]);
  _delay_ms(30);
  man.transmitArray(
   SENSOR_DATA[DELAY_EMISSION_ID][MESSAGE_SIZE_POS],
   SENSOR_DATA[DELAY_EMISSION_ID]);
  _delay_ms(30);
  man.transmitArray(
    SENSOR_DATA[DELAY_EMISSION_ID][MESSAGE_SIZE_POS],
    SENSOR_DATA[DELAY_EMISSION_ID]);
  _delay_ms(30);  
#endif
 }


void loop()
{
  ActivatePeripherals();
  SenseAndSave();
  SendData433();
  DeactivatePeripherals();
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
}
