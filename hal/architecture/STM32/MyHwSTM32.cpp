/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "MyHwSTM32.h"
#include <EEPROM.h>
#include <core/MySensorsCore.h> // required for MY_SLEEP_NOT_POSSIBLE

/*
* Pinout STM32F103C8 dev board:
* http://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif
*
* Wiring RFM69 radio / SPI1
* --------------------------------------------------
* CLK	PA5
* MISO	PA6
* MOSI	PA7
* CSN	PA4
* CE	NA
* IRQ	PA3 (default)
*
* Wiring RF24 radio / SPI1
* --------------------------------------------------
* CLK	PA5
* MISO	PA6
* MOSI	PA7
* CSN	PA4
* CE	PB0 (default)
* IRQ	NA
*
*/
bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
	while (!MY_SERIALDEVICE) {}
#endif
#endif
	//NOTE: EEPROM emulation does not require initialization.
	return true; //HW init never fails for this platform.
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *dst = static_cast<uint8_t *>(buf);
	int pos = reinterpret_cast<int>(addr);
	while (length-- > 0) {
		*dst++ = EEPROM.read(pos++);
	}
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *src = static_cast<uint8_t *>(buf);
	int pos = reinterpret_cast<int>(addr);
	while (length-- > 0) {
		EEPROM.write(pos++, *src++);
	}
}

uint8_t hwReadConfig(const int addr)
{
	uint8_t value;
	hwReadConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
	return value;
}

void hwWriteConfig(const int addr, uint8_t value)
{
	hwWriteConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
}

int8_t hwSleep(uint32_t ms)
{
	// TODO: Not supported!
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
	// TODO: Not supported!
	(void)interrupt;
	(void)mode;
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
               const uint8_t mode2,
               uint32_t ms)
{
	// TODO: Not supported!
	(void)interrupt1;
	(void)mode1;
	(void)interrupt2;
	(void)mode2;
	(void)ms;
	return MY_SLEEP_NOT_POSSIBLE;
}


void hwRandomNumberInit(void)
{
    //TODO: Some STMs have an integrated RNG. Use this if available.
	// use internal temperature sensor as noise source

	uint32_t seed = 0;
	uint16_t currentValue = 0;
	uint16_t newValue = 0;

	for (uint8_t i = 0; i < 32; i++) {
		const uint32_t timeout = hwMillis() + 20;
		while (timeout >= hwMillis()) {
			newValue = analogRead(ATEMP);
			if (newValue != currentValue) {
				currentValue = newValue;
				break;
			}
		}
		seed ^= ( (newValue + hwMillis()) & 7) << i;
	}
	randomSeed(seed);
}

bool hwUniqueID(unique_id_t *uniqueID)
{
    //TODO: This function might only be valid for STM32F1xx.
	(void)memcpy((uint8_t *)uniqueID, (uint32_t *)FLASH_SIZE_DATA_REGISTER, 16); // FlashID + ChipID
	return true;
}


#if defined(STM32F1xx)
#define CALX_TEMP 25
#define V25       1430
#define AVG_SLOPE 4300
#define VREFINT   1200
#elif defined(STM32F2xx)
#define CALX_TEMP 25
#define V25       760
#define AVG_SLOPE 2500
#define VREFINT   1210
#endif

#ifndef VREFINT
#warning "Vref value for this MCU is unknown. Look it up in the datasheet and add it to this file."
#define VREFINT 1200
#endif

#if !defined(AVG_SLOPE) && !defined(__LL_ADC_CALC_TEMPERATURE)
#warning "No temperature sensor parameters are available for this MCU. Look them up in the datasheet and add them to this file."
#define CALX_TEMP 25
#define V25       1430
#define AVG_SLOPE 4300
#endif


#if ADC_RESOLUTION == 10
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_10B
#define ADC_RANGE 1024
#else
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096
#endif


uint16_t hwCPUVoltage(void)
{
#ifdef __LL_ADC_CALC_VREFANALOG_VOLTAGE
  return (__LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION));
#else
  return (VREFINT * ADC_RANGE / analogRead(AVREF)); // ADC sample to mV
#endif
}

uint16_t hwCPUFrequency(void)
{
	return F_CPU/100000UL;
}

int8_t hwCPUTemperature(void)
{
uint8_t temp;
#ifdef __LL_ADC_CALC_TEMPERATURE
    // Device has calibration data
    temp = (__LL_ADC_CALC_TEMPERATURE(VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
#elif defined(__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS)
    // Device has no calibration data
    temp =  (__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(AVG_SLOPE, V25, CALX_TEMP, VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
#else
    // Device has not temperature sensor
    return 0;
#endif
    return (temp - MY_STM32_TEMPERATURE_OFFSET) / MY_STM32_TEMPERATURE_GAIN;
}

uint16_t hwFreeMem(void)
{
	//Not yet implemented
	return FUNCTION_NOT_SUPPORTED;
}
