/*
 * connectingStuff, Oregon Scientific v2.1 Emitter
 * http://connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/
 *
 * Copyright (C) 2013 olivier.lebrun@gmail.com
 *
 * Refactored to class by Aurélien Labrosse <https://github.com/arcadien> 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#ifndef OREGON_H_
#define OREGON_H_

#include <stdint.h>
#include <limits.h>	

class Oregon
{
	public:
	
	struct Message
	{
		static const float UNSET = -1000;
		Message()
		{
		 temperature  = UNSET;
		 humidity = UNSET;
		 battery = 1;
		}
		float temperature;
		float humidity;
		uint8_t battery; // 0 (low) or 1 (good)
	};
	
	/*!
	* Convenient define for channel codes (1,2,3)
	*/
	 struct Channel
	 {
		static const uint8_t ONE = 0x10;
		static const uint8_t TWO= 0x20;
		static const uint8_t THREE= 0x30;
	};
	
	
	Oregon(uint8_t ledPin = 13, uint8_t txPin = 7,  uint8_t repeatCount = 2);

	
	void Emit(uint8_t type[], uint8_t channel, uint8_t id, const Message &message);

	private:
	const uint8_t _ledPin;
	const uint8_t _txPin;
	const uint8_t _repeatCount;

	static uint8_t PREAMBLE[];
	static uint8_t POSTAMBLE[];

	static const uint16_t TIME = 512;
	static const uint16_t TWOTIME = TIME*2;
	
	// Buffer for Oregon message
	uint8_t _oregonMessageBuffer[9];
	
	/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remember, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendZero(void);

/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remember, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendOne(void);

/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/
 
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const uint8_t data);

 
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const uint8_t data);

/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
inline void sendData(uint8_t *data, uint8_t size);

/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
inline void sendOregon(uint8_t *data, uint8_t size);

/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void);

 
/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void);

/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void);

/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(uint8_t *data, uint8_t* type);

/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(uint8_t *data, uint8_t channel);


/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(uint8_t *data, uint8_t ID) ;

/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
inline void setBatteryLevel(uint8_t *data, uint8_t level);

/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
inline void setTemperature(uint8_t *data, float temp);

/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
inline void setHumidity(uint8_t* data, uint8_t hum);

/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
inline int Sum(uint8_t count, const uint8_t* data);

/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
inline void calculateAndSetChecksum(uint8_t* data);

};

#endif /* OREGON_H_ */