#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include "main.h"
#include "stm32_utils.h"

///===CONFIG===
#define CAN_UTIL_LED_EN False
#define CAN_UTIL_LED_Port
#define CAN_UTIL_LED_Pin

extern CAN_HandleTypeDef hcan;

typedef struct CAN_Message{
	uint8_t data[8];
	uint8_t length;
	uint32_t id;
}CAN_Message;

typedef enum CAN_TX{
	CAN_TX_STATE,
	CAN_TX_N
}CAN_TX;

extern CAN_Message CAN_Messages[CAN_TX_N];

#if CAN_UTIL_LED_EN == True
	#define CAN_UTIL_LED_Code(code) code
#else
	#define CAN_UTIL_LED_Code(code)
#endif

void CAN_RX_Handler();

void CAN_UTIL_Transmit(CAN_HandleTypeDef *can, CAN_TX message);
void CAN_UTIL_SetID(CAN_TX message, uint32_t id);
void CAN_UTIL_SetLength(CAN_TX message, uint8_t length);
void CAN_UTIL_Setup(CAN_TX message, uint32_t id, uint8_t length);
void CAN_UTIL_SetByte(CAN_TX message, uint8_t index, uint8_t data);
void CAN_UTIL_SetData(CAN_TX message, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7);
uint8_t CAN_UTIL_GetByte(CAN_TX message, uint8_t index);


#endif
