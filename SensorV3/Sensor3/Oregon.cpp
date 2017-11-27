#include "oregon.h"
#include "Arduino.h"

 uint8_t Oregon::PREAMBLE[]={0xFF,0xFF};
 uint8_t Oregon::POSTAMBLE[]={0x00};
	
Oregon::Oregon(uint8_t ledPin, uint8_t txPin, uint8_t repeatCount) :  _ledPin(ledPin),  _txPin(txPin), _repeatCount(repeatCount)
{	
	for(uint8_t counter = 0; counter < 9; ++counter)
	{
			_oregonMessageBuffer[counter] = 0;
	}

}

void Oregon::Emit(uint8_t type[], uint8_t channel, uint8_t id, const Message &message)
{
	
	 setBatteryLevel(_oregonMessageBuffer, message.battery);
	 
	 if(message.temperature != Message::UNSET)
	 {
		setTemperature(_oregonMessageBuffer, message.temperature);
	 }
	 
	 if(message.humidity != Message::UNSET)
	 {
		setTemperature(_oregonMessageBuffer, message.humidity);
	 }
	 
	 setType(_oregonMessageBuffer, type);
	 setChannel(_oregonMessageBuffer, channel);
	 // Calculate the checksum
	 calculateAndSetChecksum(_oregonMessageBuffer);
	
	#ifdef DEBUG 
	// Show the Oregon Message
	for (uint8_t i = 0; i < sizeof(_oregonMessageBuffer); ++i)
	{
	}
	#endif
	
	for(uint8_t emissionCounter = _repeatCount; emissionCounter > 0; --emissionCounter)
	{
		 sendOregon(_oregonMessageBuffer, sizeof(_oregonMessageBuffer));
  		 
		 // pause before new transmission
		 digitalWrite(_txPin, LOW);
		 delayMicroseconds(TWOTIME*8);
	}	
}


/**
 * \brief    Send logical "0" over RF
 * \details  a zero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remember, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void Oregon::sendZero(void) 
{
  digitalWrite(_txPin, HIGH);
  delayMicroseconds(TIME);
  digitalWrite(_txPin, LOW);
  delayMicroseconds(TWOTIME);
  digitalWrite(_txPin, HIGH);
  delayMicroseconds(TIME);
}
 
/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remember, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void Oregon::sendOne(void) 
{
  digitalWrite(_txPin, LOW);
   delayMicroseconds(TIME);
  digitalWrite(_txPin, HIGH);
   delayMicroseconds(TWOTIME);
  digitalWrite(_txPin, LOW);
   delayMicroseconds(TIME);
}
 
/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/
 
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void Oregon::sendQuarterMSB(const uint8_t data) 
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}
 
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void Oregon::sendQuarterLSB(const uint8_t data) 
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}
 
/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void Oregon::sendData(uint8_t *data, uint8_t size)
{
  for(uint8_t i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}
 
/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void Oregon::sendOregon(uint8_t *data, uint8_t size)
{
    sendPreamble();
    //sendSync();
    sendData(data, size);
    sendPostamble();
}
 
/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void Oregon::sendPreamble(void)
{
  sendData(PREAMBLE, 2);
}
 
/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void Oregon::sendPostamble(void)
{
  sendData(POSTAMBLE, 1);  
}
 
/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void Oregon::sendSync(void)
{
  sendQuarterLSB(0xA);
}
  
/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void Oregon::setType(uint8_t *data, uint8_t* type) 
{
  data[0] = type[0];
  data[1] = type[1];
}
 
/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void Oregon::setChannel(uint8_t *data, uint8_t channel) 
{
    data[2] = channel;
}
 
/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void Oregon::setId(uint8_t *data, uint8_t ID) 
{
  data[3] = ID;
}
 
/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void Oregon::setBatteryLevel(uint8_t *data, uint8_t level)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}
 
/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void Oregon::setTemperature(uint8_t *data, float temp) 
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;  
  }
  else
  {
    data[6] = 0x00;
  }
 
  // Determine decimal and float part
  uint16_t decTen = (uint16_t)(temp * 10);
  uint8_t dec = decTen / 10;
  uint8_t flt = decTen % 10;
 data[5] = dec;
 data[4] = flt;
  
  //int tempInt = (int)temp;
  //int td = (int)(tempInt / 10);
  //int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);
 //
  //int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);
  //uint8_t flt_o = tempFloat << 4;
 //
  //uint8_t dec_o = (td << 4);
  //dec_o |=  tf;
 //
  //// Set temperature decimal part
  //data[5] = dec_o;
 //
  //// Set temperature float part
  //data[4] |= flt_o;
}
 
/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void Oregon::setHumidity(uint8_t* data, uint8_t hum)
{
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}
 
/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Oregon::Sum(uint8_t count, const uint8_t* data)
{
  int s = 0;
 
  for(uint8_t i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}
 
/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void Oregon::calculateAndSetChecksum(uint8_t* data)
{
    data[8] = ((Sum(8, data) - 0xa) & 0xFF);
}