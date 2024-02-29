#include "can_utils.h"
#include "control.h"

CAN_Message CAN_RX_Queue[32];
uint8_t CAN_RX_Queue_Count;

CAN_RxHeaderTypeDef CAN_RxHeader;
uint8_t CAN_Rx_Message[8];

void CAN_RX_Handler(){
	if(CAN_RX_Queue_Count==0){
		return;
	}
	CAN_RX_Queue_Count--;

    switch(CAN_RX_Queue[CAN_RX_Queue_Count].id){
	case 0x18FF50E5:
		DevCon_Charger(True);
		break;
    default:
    	break;
    }
}
CAN_Message CAN_Messages[CAN_TX_N];

void CAN_UTIL_Transmit(CAN_HandleTypeDef *hcan, CAN_TX message){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Tx_Mailbox;

	TxHeader.DLC = CAN_Messages[message].length;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.ExtId = CAN_Messages[message].id;

	HAL_CAN_AddTxMessage(hcan, &TxHeader, CAN_Messages[message].data, &Tx_Mailbox);
	CAN_UTIL_LED_Code(HAL_GPIO_TogglePin(CAN_UTIL_LED_Port, CAN_UTIL_LED_Pin));
}

void CAN_UTIL_SetID(CAN_TX message, uint32_t id){
	CAN_Messages[message].id = id;
}
void CAN_UTIL_SetLength(CAN_TX message, uint8_t length){
	CAN_Messages[message].length = length;
}

void CAN_UTIL_Setup(CAN_TX message, uint32_t id, uint8_t length){
	CAN_UTIL_SetID(message, id);
	CAN_UTIL_SetLength(message, length);
}

void CAN_UTIL_SetByte(CAN_TX message, uint8_t index, uint8_t data){
	if(index>7){return;}
	CAN_Messages[message].data[index]=data;
}
void CAN_UTIL_SetData(CAN_TX message, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7){
	CAN_Messages[message].data[0]=data_0;
	CAN_Messages[message].data[1]=data_1;
	CAN_Messages[message].data[2]=data_2;
	CAN_Messages[message].data[3]=data_3;
	CAN_Messages[message].data[4]=data_4;
	CAN_Messages[message].data[5]=data_5;
	CAN_Messages[message].data[6]=data_6;
	CAN_Messages[message].data[7]=data_7;
}
uint8_t CAN_UTIL_GetByte(CAN_TX message, uint8_t index){
	if(index>7){return 0;}
	return CAN_Messages[message].data[index];
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef * hcan){

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * can){
    HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &CAN_RxHeader, CAN_Rx_Message);
	if(can == &hcan){
	    CAN_RX_Queue[CAN_RX_Queue_Count].id = CAN_RxHeader.ExtId;
	    CAN_RX_Queue[CAN_RX_Queue_Count].length = CAN_RxHeader.DLC;
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[0] = CAN_Rx_Message[0];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[1] = CAN_Rx_Message[1];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[2] = CAN_Rx_Message[2];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[3] = CAN_Rx_Message[3];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[4] = CAN_Rx_Message[4];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[5] = CAN_Rx_Message[5];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[6] = CAN_Rx_Message[6];
	    CAN_RX_Queue[CAN_RX_Queue_Count].data[7] = CAN_Rx_Message[7];
	    CAN_RX_Queue_Count++;

	}

}
