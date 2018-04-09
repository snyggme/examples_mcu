#include "stm32f2xx.h"
#include <stdint.h>
#include <stdbool.h>

#define TXBUFFERSIZE   0x10
#define RXBUFFERSIZE   0x04
#define NBROFADC			 0x08
/* define type of transmitting packets */
#define	ERRORS				 0
#define BUFFER 				 1
#define OK						 2

typedef enum
{
	READY_TO_RECEIVE = 0,
	DATA_RECEIVED,
} RxState;

USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStruct;
uint8_t txBuffer[TXBUFFERSIZE];
uint8_t rxBuffer[RXBUFFERSIZE];
uint8_t nbrOfDataToTransfer = TXBUFFERSIZE;
uint8_t nbrOfDataToRead = RXBUFFERSIZE;
uint8_t txCounter = 0; 
uint8_t rxCounter = 0; 
uint16_t gainCoeffs = 0;
uint16_t adcValues[NBROFADC];
RxState rxState = READY_TO_RECEIVE;
uint8_t txPacketType = BUFFER;

uint8_t crc8(uint8_t *pData, uint8_t len);
void NVIC_Config();
void USART1_IRQHandler();
void setADCConfiguration(uint16_t coeffs, uint8_t scanTime);
bool getMeasStatus();

int main(void) 
{
	NVIC_Config();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
			
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz; 
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
			
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);

	while(1) 
	{
		/* check if measuring of ADCs is ended */
		if (getMeasStatus())
		{
			/* fill in txBuffer with measured ADC values */
			for(int i = 0, j = 0; i < NBROFADC; i++, j += 2)
			{
				txBuffer[j] = (uint8_t)(adcValues[i] & 0xFF);
				txBuffer[j + 1] = (uint8_t)((adcValues[i] >> 8) & 0xFF);
			}
			/* calculate checksum of tx packets */
			txBuffer[TXBUFFERSIZE - 1] = crc8(txBuffer, TXBUFFERSIZE - 1);

			/* enable transmit interrupt */
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			/* wait until USART send txBuffer */
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		}
		
		if (rxState == DATA_RECEIVED)
		{
			__disable_irq();
			/* calculate checksum of received packets */
			uint8_t crc = crc8(rxBuffer, RXBUFFERSIZE - 1);
			
			if (crc == rxBuffer[3])
			{
				/* save all coeffs in one var for further calculations */
				gainCoeffs = (rxBuffer[0] << 8) | rxBuffer[1];
				/* configure adcs with new gain coeffs and new scanning time */
				setADCConfiguration(gainCoeffs, rxBuffer[2]);
				/* set OK packet type to send proper answer */
				txPacketType = OK;
			} else 
			{
				/* set ERROR packet type if no valid checksum detected */
				txPacketType = ERRORS;
			}
			
			__enable_irq();
			
			/* enable transmit interrupt to send answer back to the PC */
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			/* wait until USART send packets */
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			
			rxState = READY_TO_RECEIVE;
			/* enable receive interrupt */
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		}
	}
}

void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* read one byte from the receive data register */
    rxBuffer[rxCounter++] = USART_ReceiveData(USART1);

    if(rxCounter == nbrOfDataToRead)
    {
			/* set data received flag */
			rxState = DATA_RECEIVED;
			rxCounter = 0;
      /* disable receive interrupt */
      USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    }
  }

  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
		switch (txPacketType)
		{
			case BUFFER:
				/* write one byte to the transmit data register */
				USART_SendData(USART1, txBuffer[txCounter++]);

				if(txCounter == nbrOfDataToTransfer)
				{
					txCounter = 0;
					/* disable transmit interrupt */
					USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
				}
				break;
			case ERRORS:
				/* send 0x01 byte if we detected any errors during connection */
				USART_SendData(USART1, 0x01);
				txPacketType = BUFFER;
				/* disable transmit interrupt */
				USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
				break;
			case OK:
				/* send 0x02 byte if received data is valid */
				USART_SendData(USART1, 0x02);
				txPacketType = BUFFER;
				/* disable transmit interrupt */
				USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
				break;
			default:
				break;
		} 
  }
}

uint8_t crc8(uint8_t *pData, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i;

    while (len--)
		{
			crc ^= *pData++;

      for (i = 0; i < 8; i++)
				crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* enable USART Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void setADCConfiguration(uint16_t coeffs, uint8_t scanTime) {}

bool getMeasStatus() {}
