#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"
#include "stm32f30x.h"
#include "stm32f30x_usart.h"
#include <math.h>
#include <string.h>

#define CLOCKWISE 				1
#define COUNTERCLOCKWISE	0

#define REDLED		GPIO_Pin_13
#define MAXSPEED	1
#define MINSPEED	100
#define ONESTEP		1

#define IN1X			GPIO_Pin_8
#define IN2X			GPIO_Pin_9
#define IN3X			GPIO_Pin_10
#define IN4X			GPIO_Pin_11

/*
#define IN1Y			GPIO_Pin_Y
#define IN2Y			GPIO_Pin_Y
#define IN3Y			GPIO_Pin_Y
#define IN4Y			GPIO_Pin_Y

#define IN1Z			GPIO_Pin_Z
#define IN2Z			GPIO_Pin_Z
#define IN3Z			GPIO_Pin_Z
#define IN4Z			GPIO_Pin_Z
*/

#define DUMMY	0

typedef enum {
	
	STOPPED = 0,
	MOVING
	
} MotorStatus;

typedef enum {
	
	PROCESSED = 0,
	PREPARED
	
} DataStatus;

typedef struct 
{
	MotorStatus status;
	
	int32_t position;
	
	GPIO_TypeDef* GPIOx;
	
	uint16_t *pSteps;
	
} Motor;

uint32_t 			delay_count = 0;
int8_t 				j = 0;
DataStatus		dataStatus = PROCESSED;
uint16_t 			stepsX[] = {IN1X | IN3X, IN1X, IN1X | IN4X, IN4X, IN4X | IN2X, IN2X, IN2X | IN3X, IN3X};	// half-step mode, step == 1.8/2 degree
																																																				// 0x500 == 0b0101 0000 0000;
																																																				// set GPIO_Pin_8 and GPIO_Pin_10, reset all remaining
/*
uint16_t 			stepsY[] = {IN1Y | IN3Y, IN1Y, IN1Y | IN4Y, IN4Y, IN4Y | IN2Y, IN2Y, IN2Y | IN3Y, IN3Y}; 
uint16_t 			stepsZ[] = {IN1Z | IN3Z, IN1Z, IN1Z | IN4Z, IN4Z, IN4Z | IN2Z, IN2Z, IN2Z | IN3Z, IN3Z}; 
*/
uint8_t	 			rxBuffer[64];
uint8_t				rxCounter = 0;

void NVICInit(void);
void uartInit(void);
void motorInit(void);
void setCurrentPositionToZero(Motor *motor);
int32_t getPosition(Motor *motor);
void setPosition(Motor *motor, int32_t pos);
MotorStatus getStatus(Motor *motor);
void setStatus(Motor *motor, MotorStatus st);
void sendUartString(uint8_t *buffer, uint16_t size);
Motor *selectMotor(uint8_t motorType);

GPIO_InitTypeDef    GPIO_InitStruct;
USART_InitTypeDef   USART_InitStruct;
NVIC_InitTypeDef		NVIC_InitStructure;

Motor motorX, motorY, motorZ;

void SysTick_Handler(void) {
	
	if (delay_count > 0) delay_count--;
}

void delay_ms(uint32_t ms) {
	
	delay_count = ms;
	while(delay_count){}	
}

void UART4_IRQHandler(void) {	//	uart receiving handler
	
  if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
  {
    if(getStatus(&motorX) == STOPPED /*&& getStatus(&motorY) == STOPPED && getStatus(&motorZ) == STOPPED*/) {
			rxBuffer[rxCounter++] = USART_ReceiveData(UART4);

			if(rxBuffer[rxCounter - 1] == 0x0D) { 	//	0x0D - stop of message
				rxCounter = 0; 												//	start filling buffer from fisrt massive element
				dataStatus = PREPARED;								//	flag that let UART commands to operate one time in main() function
			}
		} else sendUartString((uint8_t *)"MOTOR ARE MOVING...", sizeof("MOTOR ARE MOVING...") - 1);
  }
}

void moveMotor(Motor *motor, int32_t coordinate, uint8_t speed) { //	one step == 1.8 degree, half step == 0.9 degree
	//save next position of the motor
	int32_t newPos = getPosition(motor) + coordinate;
	setPosition(motor, newPos);
	//define current status of the motor
	setStatus(motor, MOVING);
	//define direction of rotating
	uint8_t rotationDirection = CLOCKWISE;
	if (coordinate < 0) {
		rotationDirection = COUNTERCLOCKWISE;
		coordinate *= -1;
	}
	//define formula for converting coordinate into angle of rotation of the motor
	//TODO: calculate formula coefficient: angle = coordinate / COEFF
	uint16_t angle = coordinate / 1;
	
	uint16_t numberOfSteps = (uint16_t)roundf(angle / 0.9); 
	
	for(int i = 0; i < numberOfSteps; i++) {
		//	j range only from 0 to 7 because array step[] have only 8 elements: half-step mode
		if (j == 8 && rotationDirection) j = 0;
		if (j == -1 && !rotationDirection) j = 7;
		//	set or reset 4 GPIO pins to control L298 motor driver
		GPIO_Write(motor->GPIOx, motor->pSteps[j]);
		delay_ms(speed);
		
		if (rotationDirection) {
			j++;
		} else j--;
	}
	GPIO_Write(motor->GPIOx, DUMMY);	//	reset all bits so no current consumption 
	setStatus(motor, STOPPED);
}

void moveMotorRight(Motor *motor) {

	if (getStatus(motor) == STOPPED) {
		moveMotor(motor, ONESTEP, 5);	
	} else {
			GPIO_SetBits(GPIOE, REDLED); // indicates ERROR 
	}
}

void moveMotorLeft(Motor *motor) {
	
	if (getStatus(motor) == STOPPED) {
		moveMotor(motor, -ONESTEP, 5);	
	} else {
			GPIO_SetBits(GPIOE, REDLED); // indicates ERROR 
	}
}

void moveToZero(Motor *motor) {
	
	int32_t newPos = (-1)*getPosition(motor);

	if (getPosition(motor) != 0 && getStatus(motor) == STOPPED) {
		moveMotor(motor, newPos, MAXSPEED);	
		setPosition(motor, 0);
	} else {
			GPIO_SetBits(GPIOE, REDLED); // indicates ERROR 
	}
}

int main() {
	
	SysTick_Config(SystemCoreClock/1000);
	
	__enable_irq();
	NVICInit();
	uartInit();
	motorInit();
	//set up any motor with GPIO_PORT, steps array and position
	motorX.pSteps = stepsX;
	motorX.GPIOx = GPIOB;
	setPosition(&motorX, 0);
	setStatus(&motorX, STOPPED);
	/*	use motorY or motorZ if needed
	motorY.position = 0;
	motorY.pSteps = stepsY;
	motorY.GPIOx = GPIOY;
	
	motorZ.position = 0;
	motorZ.pSteps = stepsZ;
	motorZ.GPIOx = GPIOZ;
	*/
	while(1) {

		if (dataStatus) { // let parsing uart commands and operate motor

			if (strncmp((char const *)rxBuffer, "move ", 5) == 0) {
				//	UART message example from Terminal 1.9b: move X#001#002#013
				//	X - coordinate of movement:	X or Y or Z
				//	#001 - decimal value of COORDINATE
				//	#002 - decimal value of rotating SPEED
				//	#013 - uart end of frame, 13 == 0x0D
	
				moveMotor(selectMotor(rxBuffer[5]), rxBuffer[6], rxBuffer[7]);

			} 
			else if (strncmp((char const *)rxBuffer, "set current position to zero ", 29) == 0) {
				//	UART message example from Terminal 1.9b: set current position to zero X#013
				//	X - settable coordinate:	X or Y or Z
				//	#013 - uart stop of frame, 13 == 0x0D

				setCurrentPositionToZero(selectMotor(rxBuffer[29]));	

			}
			else if (strncmp((char const *)rxBuffer, "step right ", 11) == 0){
				//	UART message example from Terminal 1.9b: move right X#013
				//	X - settable coordinate:	X or Y or Z
				//	#013 - uart stop of frame, 13 == 0x0D				

				moveMotorRight(selectMotor(rxBuffer[11]));	
				
			}
			else if (strncmp((char const *)rxBuffer, "step left ", 10) == 0){
				//	UART message example from Terminal 1.9b: move left X#013
				//	X - settable coordinate:	X or Y or Z
				//	#013 - uart stop of frame, 13 == 0x0D				
				
				moveMotorLeft(selectMotor(rxBuffer[10]));
				
			}
			else if (strncmp((char const *)rxBuffer, "go to zero ", 11) == 0) {
				//	UART message example from Terminal 1.9b: move to zero X#013
				//	X - settable coordinate:	X or Y or Z
				//	#013 - uart stop of frame, 13 == 0x0D		
				
				moveToZero(selectMotor(rxBuffer[11]));
				
			}
			else if (strncmp((char const *)rxBuffer, "get current coordinates ", 24) == 0) {
				//	UART message example from Terminal 1.9b: get current coordinates X#013
				//	X - settable coordinate:	X or Y or Z
				//	#013 - uart stop of frame, 13 == 0x0D					
				
				int32_t coord = getPosition(selectMotor(rxBuffer[24]));
				
				sendUartString((uint8_t *)"Current coordinate of the motor is ", sizeof("Current coordinate of the motor is ") - 1);
				
				while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
				
				USART_SendData(UART4, coord);
			}
			
			dataStatus = PROCESSED;	
		}
	}
}

Motor *selectMotor(uint8_t motorType) {
	
	switch (motorType) {
		case 'X':
			return &motorX;
			break;
		case 'Y':
			return &motorY;
			break;
		case 'Z':
			return &motorZ;
			break;
		default: break;
	}
}

void sendUartString(uint8_t *buffer, uint16_t size) {

	for(int i = 0; i < size; i++) {
		
		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);

		USART_SendData(UART4, buffer[i]);
	}
}

void setCurrentPositionToZero(Motor *motor) {
	motor->position = 0;
	sendUartString((uint8_t *)"POSITION SETTED", sizeof("POSITION SETTED") - 1);
}

int32_t getPosition(Motor *motor) {
	return motor->position;
}

void setPosition(Motor *motor, int32_t pos) {
	motor->position = pos;
}

MotorStatus getStatus(Motor *motor) {
	return motor->status;
}

void setStatus(Motor *motor, MotorStatus st) {
	motor->status = st;
}

void motorInit() {
		
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOE, ENABLE);
	/*
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOY, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOZ, ENABLE);
	*/
 
	GPIO_InitStruct.GPIO_Pin = 		GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Speed = 	GPIO_Speed_Level_2; 
  GPIO_InitStruct.GPIO_Mode = 	GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = 	GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct); 
	
	GPIO_InitStruct.GPIO_Pin = 		GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Speed = 	GPIO_Speed_Level_2; 
  GPIO_InitStruct.GPIO_Mode = 	GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = 	GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStruct); 
	
	/*
	motorY init
	GPIO_InitStruct.GPIO_Pin = 		GPIO_Pin_Y | GPIO_Pin_Y | GPIO_Pin_Y | GPIO_Pin_Y; 
  GPIO_InitStruct.GPIO_Speed = 	GPIO_Speed_Level_2; 
  GPIO_InitStruct.GPIO_Mode = 	GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = 	GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOY, &GPIO_InitStruct); 
	
	motorZ init
	GPIO_InitStruct.GPIO_Pin = 		GPIO_Pin_Z | GPIO_Pin_Z | GPIO_Pin_Z | GPIO_Pin_Z;
  GPIO_InitStruct.GPIO_Speed = 	GPIO_Speed_Level_2; 
  GPIO_InitStruct.GPIO_Mode = 	GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = 	GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOZ, &GPIO_InitStruct); 
	*/
	
	GPIO_ResetBits(GPIOE, REDLED);
}

void uartInit() {
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
 
  GPIO_InitStruct.GPIO_Pin = 		GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Speed = 	GPIO_Speed_50MHz; 
  GPIO_InitStruct.GPIO_Mode = 	GPIO_Mode_AF;
  GPIO_Init(GPIOC, &GPIO_InitStruct); 
 
  GPIO_InitStruct.GPIO_Pin = 		GPIO_Pin_10;
  GPIO_InitStruct.GPIO_Mode  = 	GPIO_Mode_AF; 
  GPIO_Init(GPIOC, &GPIO_InitStruct); 
 
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_5);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_5);
 
  USART_InitStruct.USART_BaudRate = 						115200; 
  USART_InitStruct.USART_WordLength = 					USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = 						USART_StopBits_1; 
  USART_InitStruct.USART_Parity = 							USART_Parity_No ; 
  USART_InitStruct.USART_HardwareFlowControl = 	USART_HardwareFlowControl_None; 
  USART_InitStruct.USART_Mode = 								USART_Mode_Rx | USART_Mode_Tx; 
  USART_Init(UART4, &USART_InitStruct); 

  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  
  USART_Cmd(UART4, ENABLE);
}

void NVICInit() {
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
