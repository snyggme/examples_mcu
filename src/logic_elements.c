#include <stdint.h>
#include "LPC17xx.h"
#include "system_LPC17xx.h"
#include "PIN_LPC17xx.h"
#include "GPIO_LPC17xx.h"
#include "Board_Buttons.h"

#define PIN_COUNT 12

PIN pinsIn[] = {{0,2}, {1,1}, {1,8}, {1,10}, {1,15}, {4,28}, {0,7}, {0,9}, {2,1}, {2,3}, {2,5}, {2,7}};
PIN pinsOut[] = {{0,3}, {1,4}, {1,9}, {1,14}, {4,29}, {0,6}, {0,8}, {2,0}, {2,2}, {2,4}, {2,6}, {2,8}};
PIN powerEnablePin = {0,11};

typedef struct {
	uint8_t* name;
	uint8_t* description;
	uint8_t inputsNumber;
	PIN* pinout;
	uint8_t* funcTable;
	uint8_t poLen;
	uint8_t ftLen;
} LogicElement;

typedef enum {
	NOT_IDENTIFIED = 0,
	IDENTIFIED_AND_INCORRECT_VENTILS = 1,
	IDENTIFIED_AND_CORRECT_VENTILS = 2
} Status;

Status TestLogicElement(LogicElement* elements, uint8_t elementsLen);
void GPIO_init(void);

int main(void)
{
	/*
	************************************************
	*	74HC00 / 74HCT00												
	************************************************
	*/	
	LogicElement HC7400; 
	uint8_t ftHC7400[] =	{ 
													0, 1, 1,
													1, 0, 1,
													1, 1, 0
												};
	PIN pinoutHC7400[] =	{
													pinsIn[0],	pinsIn[1],	pinsOut[2],
													pinsIn[3],	pinsIn[4],	pinsOut[5], 
													pinsIn[7],	pinsIn[8],	pinsOut[6],
													pinsIn[10],	pinsIn[11],	pinsOut[9]
												};	
	HC7400.name = (uint8_t *)"74HC00/74HCT00";
	HC7400.description = (uint8_t *)"Quad 2-input NAND gate";
	HC7400.inputsNumber = 2;
	HC7400.funcTable = ftHC7400;
	HC7400.pinout = pinoutHC7400;
	HC7400.poLen = sizeof(pinoutHC7400) / sizeof(PIN);
	HC7400.ftLen = sizeof(ftHC7400) / sizeof(uint8_t);
	/*
	************************************************
	*	74HC02 / 74HCT02												
	************************************************
	*/				
	LogicElement HC7402;
	uint8_t ftHC7402[] =	{ 
													0, 0, 1,
													0, 1, 0,
													1, 0, 0
												};
	PIN pinoutHC7402[] =	{
													pinsIn[1],	pinsIn[2],	pinsOut[0],
													pinsIn[4],	pinsIn[5],	pinsOut[3], 
													pinsIn[6],	pinsIn[7],	pinsOut[8],
													pinsIn[9],	pinsIn[10],	pinsOut[11]
												};	
	HC7402.name = (uint8_t *)"74HC02/74HCT02";
	HC7402.description = (uint8_t *)"Quad 2-input NOR gate";
	HC7402.inputsNumber = 2;
	HC7402.funcTable = ftHC7402;
	HC7402.pinout = pinoutHC7402;
	HC7402.poLen = sizeof(pinoutHC7402) / sizeof(PIN);
	HC7402.ftLen = sizeof(ftHC7402) / sizeof(uint8_t);
	/*
	************************************************
	*	74HC04 / 74HCT04												
	************************************************
	*/					
	LogicElement HC7404;
	uint8_t ftHC7404[] =	{ 
													0, 1,
													1, 0
												};
	PIN pinoutHC7404[] =	{
													pinsIn[0],	pinsOut[1],
													pinsIn[2],	pinsOut[3], 
													pinsIn[4],	pinsOut[5],
													pinsIn[7],	pinsOut[6],
													pinsIn[9],	pinsOut[8],
													pinsIn[11], pinsOut[10]
												};
	HC7404.name = (uint8_t *)"74HC04/74HCT04";
	HC7404.description = (uint8_t *)"Hex inverter";
	HC7404.inputsNumber = 1;
	HC7404.funcTable = ftHC7404;
	HC7404.pinout = pinoutHC7404;
	HC7404.poLen = sizeof(pinoutHC7404) / sizeof(PIN);
	HC7404.ftLen = sizeof(ftHC7404) / sizeof(uint8_t);
	/*
	************************************************
	*	74HC08 / 74HCT08												
	************************************************
	*/													
	LogicElement HC7408; 
	uint8_t ftHC7408[] =	{ 
													0, 0, 0,
													0, 1, 0,
													1, 0, 0,
													1, 1, 1
												};	
	PIN pinoutHC7408[] =	{
													pinsIn[0],	pinsIn[1],	pinsOut[2],
													pinsIn[3],	pinsIn[4],	pinsOut[5], 
													pinsIn[7],	pinsIn[8],	pinsOut[6],
													pinsIn[10],	pinsIn[11],	pinsOut[9]
												};	
	HC7408.name = (uint8_t *)"74HC08/74HCT08";
	HC7408.description = (uint8_t *)"Quad 2-input AND gate";
	HC7408.inputsNumber = 2;
	HC7408.funcTable = ftHC7408;
	HC7408.pinout = pinoutHC7408;
	HC7408.poLen = sizeof(pinoutHC7408) / sizeof(PIN);
	HC7408.ftLen = sizeof(ftHC7408) / sizeof(uint8_t);															
	/*
	************************************************
	*	74HC10 / 74HCT10												
	************************************************
	*/													
	LogicElement HC7410; 
	uint8_t ftHC7410[] =	{ 
													0, 1, 1, 1,
													1, 0, 1, 1,
													1, 1, 0, 1,
													1, 1, 1, 0
												};	
	PIN pinoutHC7410[] =	{
													pinsIn[0],	pinsIn[1],	pinsIn[11],	pinsOut[10],
													pinsIn[2],	pinsIn[3],	pinsIn[4],	pinsOut[5],
													pinsIn[7],	pinsIn[8],	pinsIn[9],	pinsOut[5]
												};	
	HC7410.name = (uint8_t *)"74HC10/74HCT10";
	HC7410.description = (uint8_t *)"Triple 3-input NAND gate";
	HC7410.inputsNumber = 3;
	HC7410.funcTable = ftHC7410;
	HC7410.pinout = pinoutHC7410;
	HC7410.poLen = sizeof(pinoutHC7410) / sizeof(PIN);
	HC7410.ftLen = sizeof(ftHC7410) / sizeof(uint8_t);	
	/*
	************************************************
	*	74HC11 / 74HCT11												
	************************************************
	*/													
	LogicElement HC7411; 
	uint8_t ftHC7411[] = 	{ 
													0, 1, 1, 0,
													1, 0, 1, 0,
													1, 1, 0, 0,
													1, 1, 1, 1
												};	
	PIN pinoutHC7411[] =	{
													pinsIn[0],	pinsIn[1],	pinsIn[11],	pinsOut[10],
													pinsIn[2],	pinsIn[3],	pinsIn[4],	pinsOut[5],
													pinsIn[7],	pinsIn[8],	pinsIn[9],	pinsOut[6]
												};	
	HC7411.name = (uint8_t *)"74HC11/74HCT11";
	HC7411.description = (uint8_t *)"Triple 3-input AND gate";
	HC7411.inputsNumber = 3;
	HC7411.funcTable = ftHC7411;
	HC7411.pinout = pinoutHC7411;
	HC7411.poLen = sizeof(pinoutHC7411) / sizeof(PIN);
	HC7411.ftLen = sizeof(ftHC7411) / sizeof(uint8_t);																	
	/*
	************************************************
	*	74HC14 / 74HCT14												
	************************************************
	*/													
	LogicElement HC7414; 
	uint8_t ftHC7414[] =	{ 
													0, 1,
													1, 0
												};	
	PIN pinoutHC7414[] = 	{
													pinsIn[0],	pinsOut[1],
													pinsIn[2],	pinsOut[3], 
													pinsIn[4],	pinsOut[5],
													pinsIn[7],	pinsOut[6],
													pinsIn[9],	pinsOut[8],
													pinsIn[11], pinsOut[10]
												};	
	HC7414.name = (uint8_t *)"74HC14/74HCT14";
	HC7414.description = (uint8_t *)"Hex inverting Schmitt trigger";
	HC7414.inputsNumber = 1;
	HC7414.funcTable = ftHC7414;
	HC7414.pinout = pinoutHC7414;	
	HC7414.poLen = sizeof(pinoutHC7414) / sizeof(PIN);
	HC7414.ftLen = sizeof(ftHC7414) / sizeof(uint8_t);																	
	/*
	************************************************
	*	74HC86 / 74HCT86												
	************************************************
	*/													
	LogicElement HC7486; 
	uint8_t ftHC7486[] =	{ 
													0, 0, 0,
													0, 1, 1,
													1, 0, 1,
													1, 1, 0
												};	
	PIN pinoutHC7486[] = 	{
													pinsIn[0],	pinsIn[1],	pinsOut[2],
													pinsIn[3],	pinsIn[4],	pinsOut[5], 
													pinsIn[7],	pinsIn[8],	pinsOut[6],
													pinsIn[10],	pinsIn[11],	pinsOut[9]
												};	
	HC7486.name = (uint8_t *)"74HC86/74HCT86";
	HC7486.description = (uint8_t *)"Quad 2-input EXCLUSIVE-OR gate";
	HC7486.inputsNumber = 1;
	HC7486.funcTable = ftHC7486;
	HC7486.pinout = pinoutHC7486;																	
	HC7486.poLen = sizeof(pinoutHC7486) / sizeof(PIN);
	HC7486.ftLen = sizeof(ftHC7486) / sizeof(uint8_t);			
																
	LogicElement logicElements[] = {HC7400, HC7402, HC7404, HC7408, HC7410, HC7411, HC7414, HC7486};
	uint8_t logicElementsLen = sizeof(logicElements) / sizeof(LogicElement);
	
	SystemCoreClockUpdate(); 
	GPIO_init();
	
	while(1)
	{
		if (Buttons_GetState() != 0){
			/*	Provide power to tested logic element	*/
			GPIO_PinWrite(powerEnablePin.Portnum, powerEnablePin.Pinnum, 1);
			/*	start testing and identifying process	*/
			Status status = TestLogicElement(logicElements, logicElementsLen);
		}	
	}
}

Status TestLogicElement(LogicElement* elements, uint8_t elementsLen) {
	
	uint8_t correctFunc = 0;
	uint8_t correctVentils = 0;

	for (int k = 0; k < elementsLen; k++) {
		/*
		* check every ventil in logic element
		*/
		for (int i = 0;  i < elements[k].poLen; i =  i + 1 + elements[k].inputsNumber) {
			/*
			* check every combination in function table
			*/
			for (int l = 0; l < elements[k].ftLen; l = l + 1 + elements[k].inputsNumber) {
				/*
				* set input pins according to the pinout and functional table associated with logic element
				*/
				for (int j = i; j < i + elements[k].inputsNumber; j++) {
					
					GPIO_PinWrite(elements[k].pinout[j].Portnum,
												elements[k].pinout[j].Pinnum,
												elements[k].funcTable[j - i + l]);
				}
				/*
				* elements[k].inputsNumber - is a position of an output pin of tested logic element
				* check output value of according output pin of logic element
				*/
				uint8_t outputValue = GPIO_PinRead(elements[k].pinout[l + elements[k].inputsNumber].Portnum,
																					 elements[k].pinout[l + elements[k].inputsNumber].Pinnum);
				
				if (outputValue == elements[k].funcTable[l + elements[k].inputsNumber])
					correctFunc++;
			}
			/*
			* if all combinations in function table matched measured values then this ventil is working correct
			*/
			if (correctFunc == (elements[k].ftLen / (elements[k].inputsNumber + 1)))
				correctVentils++;
		}	

		if (correctVentils == (elements[k].poLen / (elements[k].inputsNumber + 1))) 
			return IDENTIFIED_AND_CORRECT_VENTILS;
		else if (correctVentils != 0)
			return IDENTIFIED_AND_INCORRECT_VENTILS;
	}
	
	return NOT_IDENTIFIED;
}

void GPIO_init() {

  GPIO_PortClock(1);

  for (int i = 0; i < PIN_COUNT; i++) {
    PIN_Configure(pinsOut[i].Portnum, pinsIn[i].Pinnum, PIN_FUNC_0, PIN_PINMODE_PULLDOWN, PIN_PINMODE_NORMAL);
    GPIO_SetDir(pinsOut[i].Portnum, pinsIn[i].Pinnum, GPIO_DIR_OUTPUT);
  }	
	
	for (int i = 0; i < PIN_COUNT; i++) {
		PIN_Configure(pinsIn[i].Portnum, pinsIn[i].Pinnum, PIN_FUNC_0, PIN_PINMODE_PULLDOWN, PIN_PINMODE_NORMAL);
    GPIO_SetDir(pinsIn[i].Portnum, pinsIn[i].Pinnum, GPIO_DIR_INPUT);
  }
	
	PIN_Configure(powerEnablePin.Portnum, powerEnablePin.Pinnum, PIN_FUNC_0, PIN_PINMODE_PULLDOWN, PIN_PINMODE_NORMAL);
  GPIO_SetDir(powerEnablePin.Portnum, powerEnablePin.Pinnum, GPIO_DIR_INPUT);	
}
