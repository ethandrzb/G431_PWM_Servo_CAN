// Adapted from https://controllerstech.com/using-dht11-sensor-with-stm32/

#include "main.h"
#include <stdbool.h>

// Can be any continuous microsecond timer
#define DELAY_TIMER TIM4

volatile bool readCancelled = false;

typedef struct dht11Data
{
	float temperature;
	float humidity;
	bool isValid;
} dht11Data;

typedef struct dht11DataBytes
{
	uint8_t humidityIntegerByte;
	uint8_t humidityDecimalByte;
	uint8_t temperatureIntegerByte;
	uint8_t temperatureDecimalByte;
	uint8_t checksum;
} dht11DataBytes;

void setPinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void setPinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start(void);
bool DHT11_ReadResponse(void);
uint8_t DHT11_ReadByte(void);
dht11Data DHT11_GetData();
dht11DataBytes DHT11_GetDataBytes();
void DHT11_CancelOperation();
void delayMicroseconds(uint16_t usec);

void setPinOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void setPinInput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull= GPIO_NOPULL;		// Change to GPIO_PULLUP if no data is received
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void DHT11_Start(void)
{
	// Reset cancel flag
	readCancelled = false;

	setPinOutput(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin);
	HAL_GPIO_WritePin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin, GPIO_PIN_RESET);
	delayMicroseconds(18000);
	HAL_GPIO_WritePin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin, GPIO_PIN_SET);
	delayMicroseconds(20);
	setPinInput(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin);
}

// Detects acknowledgment signal from DHT11 sensor
bool DHT11_ReadResponse(void)
{
	// Sensor not present
	bool sensorPresent = false;
	delayMicroseconds(40);
	if(!HAL_GPIO_ReadPin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin))
	{
		delayMicroseconds(80);
		if(HAL_GPIO_ReadPin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin))
		{
			// Sensor present
			sensorPresent = true;
		}
	}

	// Wait for pin to go low
	while((HAL_GPIO_ReadPin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin)) && !readCancelled);

	return sensorPresent;
}

uint8_t DHT11_ReadByte(void)
{
	uint8_t byte = 0;

	for(uint8_t j = 0; j < 8; j++)
	{
		// Wait for pin to go high
		while(!(HAL_GPIO_ReadPin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin)) && !readCancelled);
		delayMicroseconds(40);

		if(!HAL_GPIO_ReadPin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin))
		{
			// Write 0
			byte &= ~(1 << (7 - j));
		}
		else
		{
			// Write 1
			byte |= (1 << (7 - j));
		}

		// Wait for pin to go low
		while((HAL_GPIO_ReadPin(DHT11_SERIAL_GPIO_Port, DHT11_SERIAL_Pin)) && !readCancelled);
	}

	return byte;
}

dht11DataBytes DHT11_GetDataBytes()
{
	dht11DataBytes result;

	DHT11_Start();
	if(DHT11_ReadResponse())
	{
		result.humidityIntegerByte = DHT11_ReadByte();
		result.humidityDecimalByte = DHT11_ReadByte();
		result.temperatureIntegerByte = DHT11_ReadByte();
		result.temperatureDecimalByte = DHT11_ReadByte();
		result.checksum = DHT11_ReadByte();
	}

	return result;
}

dht11Data DHT11_GetData()
{
	dht11Data result;
	dht11DataBytes resultBytes = DHT11_GetDataBytes();

	if(resultBytes.checksum == resultBytes.humidityIntegerByte + resultBytes.humidityDecimalByte + resultBytes.temperatureIntegerByte + resultBytes.temperatureDecimalByte)
	{
		result.temperature = resultBytes.temperatureIntegerByte + (resultBytes.temperatureDecimalByte / 10.0f);
		result.humidity = resultBytes.humidityIntegerByte + (resultBytes.humidityDecimalByte / 10.0f);
		result.isValid = true;
	}

	return result;
}

void DHT11_CancelOperation()
{
	readCancelled = true;
}

void delayMicroseconds(uint16_t usec)
{
	DELAY_TIMER->CNT = 0;
	while(DELAY_TIMER->CNT < usec);
}
