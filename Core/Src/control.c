/*
 * control.c
 *
 *  Created on: Feb 7, 2023
 *      Author: TheDa
 */

#include "control.h"
#include "stm32_utils.h"
#include "can_utils.h"

RCB_HANDLE hrcb;

void Err_Handler(){
	Kill();
	while(1){
		IWDG_Delay(500);
		HV_Check();
		CAN_Transmit_State();
	}
}

void Input_Check(){
	HV_Check();
	if(HAL_GPIO_ReadPin(IND_60V_GPIO_Port, IND_60V_Pin) == GPIO_PIN_SET) {
		//60V detected on startup.
		ERR(ERR_60V_ENG_STARTUP)
	}
	if(HAL_GPIO_ReadPin(RELAY_AUX_AIRPOS_GPIO_Port, RELAY_AUX_AIRPOS_Pin) == GPIO_PIN_SET) {
		//AIRPOS engaged on startup.
		ERR(ERR_RELAY_AIRPOS_ENG_STARTUP)
	}
	if(HAL_GPIO_ReadPin(RELAY_AUX_AIRNEG_GPIO_Port, RELAY_AUX_AIRNEG_Pin) == GPIO_PIN_SET) {
		//AIRNEG engaged on startup.
		ERR(ERR_RELAY_AIRNEG_ENG_STARTUP)
	}
	if(HAL_GPIO_ReadPin(RELAY_AUX_PC_GPIO_Port, RELAY_AUX_PC_Pin) == GPIO_PIN_SET) {
		//Pre-Charge relay engaged on startup.
		ERR(ERR_RELAY_PC_ENG_STARTUP)
	}
	if(HAL_GPIO_ReadPin(IND_PWR_IMD_GPIO_Port, IND_PWR_IMD_Pin) == GPIO_PIN_RESET){
		//No power detected for IMD on startup.
		ERR(ERR_NO_PWR_IMD_STARTUP)
	}
	if(HAL_GPIO_ReadPin(IND_PWR_FAN1_GPIO_Port, IND_PWR_FAN1_Pin) == GPIO_PIN_RESET){
		//No power detected for fan power 1 on startup.
		ERR(ERR_NO_PWR_FAN1_STARTUP)
	}
	if(HAL_GPIO_ReadPin(IND_PWR_FAN2_GPIO_Port, IND_PWR_FAN2_Pin) == GPIO_PIN_RESET){
		//No power detected for fan power 2 on startup.
		ERR(ERR_NO_PWR_FAN2_STARTUP)
	}
	if(HAL_GPIO_ReadPin(REQUEST_TS_GPIO_Port, REQUEST_TS_Pin) == GPIO_PIN_SET){
		//TS activated on start-up.
		ERR(ERR_TS_ACTIVE_ONSTART)
	}
}

void Relay_AIRPOS_SetState(RELAY_STATE state){
	BITSET(hrcb.RELAY_IT_IGNORE, RI_AIRPOS);	//Ignore AUX change interrupt as this isn't an accidental trigger.
	HAL_GPIO_WritePin(RELAY_TRIGGER_AIRPOS_GPIO_Port, RELAY_TRIGGER_AIRPOS_Pin, (uint8_t)state);	//Trigger relay.

	IWDG_Delay(RELAY_CHECK_DELAY);	//Delay to let relays change state before checking transition.

	Relay_AIRPOS_Check();	//Checks to confirm relay has changed state as intented.
	BITRESET(hrcb.RELAY_IT_IGNORE,RI_AIRPOS);	//Let AUX trigger interrupts again as any changes here on will be unintended.
	CAN_Transmit_State();
}
void Relay_AIRNEG_SetState(RELAY_STATE state){
	BITSET(hrcb.RELAY_IT_IGNORE, RI_AIRNEG);	//Ignore AUX change interrupt as this isn't an accidental trigger.
	HAL_GPIO_WritePin(RELAY_TRIGGER_AIRNEG_GPIO_Port, RELAY_TRIGGER_AIRNEG_Pin, (uint8_t)state);	//Trigger relay.

	IWDG_Delay(RELAY_CHECK_DELAY);	//Delay to let relays change state before checking transition.

	Relay_AIRNEG_Check();	//Checks to confirm relay has changed state as intented.
	BITRESET(hrcb.RELAY_IT_IGNORE,RI_AIRNEG);	//Let AUX trigger interrupts again as any changes here on will be unintended.
	CAN_Transmit_State();
}
void Relay_PC_SetState(RELAY_STATE state){
	BITSET(hrcb.RELAY_IT_IGNORE, RI_PC);	//Ignore AUX change interrupt as this isn't an accidental trigger.
	HAL_GPIO_WritePin(RELAY_TRIGGER_PC_GPIO_Port, RELAY_TRIGGER_PC_Pin, (uint8_t)state);	//Trigger relay.

	IWDG_Delay(RELAY_CHECK_DELAY);	//Delay to let relays change state before checking transition.

	Relay_PC_Check();	//Checks to confirm relay has changed state as intented.
	BITRESET(hrcb.RELAY_IT_IGNORE,RI_PC);	//Let AUX trigger interrupts again as any changes here on will be unintended.
	CAN_Transmit_State();
}


void Relay_AIRPOS_Check(){
	//Compares the AUX reading and the Trigger pin Status

	//If AUX detects relay open...
	if (HAL_GPIO_ReadPin(RELAY_AUX_AIRPOS_GPIO_Port, RELAY_AUX_AIRPOS_Pin) == GPIO_PIN_RESET) {
		//If trigger is open...
		if (HAL_GPIO_ReadPin(RELAY_TRIGGER_AIRPOS_GPIO_Port, RELAY_TRIGGER_AIRPOS_Pin) == GPIO_PIN_RESET) {
			BITRESET(hrcb.RELAY_STATES,RS_AIRPOS);
		}
		//Else if trigger is closed...
		else {
			if(BITCHECK(hrcb.RELAY_IT_IGNORE,RI_AIRPOS)){
				ERR(ERR_RELAY_AIRPOS_ENG_FAIL);
			}else{
				ERR(ERR_RELAY_AIRPOS_DISENG);
			}
		}
	}
	//Else if AUX detects relay closed...
	else {
		//If trigger is closed...
		if (HAL_GPIO_ReadPin(RELAY_TRIGGER_AIRPOS_GPIO_Port, RELAY_TRIGGER_AIRPOS_Pin) == GPIO_PIN_SET) {
			BITSET(hrcb.RELAY_STATES,RS_AIRPOS);
		}
		//Else if trigger is open...
		else {
			if(BITCHECK(hrcb.RELAY_IT_IGNORE,RI_AIRPOS)){
				ERR(ERR_RELAY_AIRPOS_DISENG_FAIL);
			}else{
				ERR(ERR_RELAY_AIRPOS_ENG);
			}
		}
	}
}
void Relay_AIRNEG_Check(){
	//Compares the AUX reading and the Trigger pin Status

	//If AUX detects relay open...
	if (HAL_GPIO_ReadPin(RELAY_AUX_AIRNEG_GPIO_Port, RELAY_AUX_AIRNEG_Pin) == GPIO_PIN_RESET) {
		//If trigger is open...
		if (HAL_GPIO_ReadPin(RELAY_TRIGGER_AIRNEG_GPIO_Port, RELAY_TRIGGER_AIRNEG_Pin) == GPIO_PIN_RESET) {
			BITRESET(hrcb.RELAY_STATES,RS_AIRNEG);
		}
		//Else if tigger is closed...
		else {
			if(BITCHECK(hrcb.RELAY_IT_IGNORE,RI_AIRNEG)){
				ERR(ERR_RELAY_AIRNEG_ENG_FAIL);
			}else{
				ERR(ERR_RELAY_AIRNEG_DISENG);
			}

		}
	}
	//Else if AUX detects relay closed...
	else {
		//If trigger is closed...
		if (HAL_GPIO_ReadPin(RELAY_TRIGGER_AIRNEG_GPIO_Port, RELAY_TRIGGER_AIRNEG_Pin) == GPIO_PIN_SET) {
			BITSET(hrcb.RELAY_STATES,RS_AIRNEG);
		}
		//Else if trigger is open...
		else {
			if(BITCHECK(hrcb.RELAY_IT_IGNORE,RI_AIRNEG)){
				ERR(ERR_RELAY_AIRNEG_DISENG_FAIL);
			}else{
				ERR(ERR_RELAY_AIRNEG_ENG);
			}
		}
	}
}
void Relay_PC_Check(){
	//Compares the AUX reading and the Trigger pin Status

	//If AUX detects relay open...
	if (HAL_GPIO_ReadPin(RELAY_AUX_PC_GPIO_Port, RELAY_AUX_PC_Pin) == GPIO_PIN_RESET) {
		//If trigger is open...
		if (HAL_GPIO_ReadPin(RELAY_TRIGGER_PC_GPIO_Port, RELAY_TRIGGER_PC_Pin) == GPIO_PIN_RESET) {
			BITRESET(hrcb.RELAY_STATES,RS_PC);
		}
		//Else if tigger is closed...
		else {
			if(BITCHECK(hrcb.RELAY_IT_IGNORE,RI_PC)){
				ERR(ERR_RELAY_PC_ENG_FAIL);
			}else{
				ERR(ERR_RELAY_PC_DISENG);
			}
		}
	}
	//Else if AUX detects relay closed...
	else {
		//If trigger is closed...
		if (HAL_GPIO_ReadPin(RELAY_TRIGGER_PC_GPIO_Port, RELAY_TRIGGER_PC_Pin) == GPIO_PIN_SET) {
			BITSET(hrcb.RELAY_STATES,RS_PC);
		}
		//Else if trigger is open...
		else {
			if(BITCHECK(hrcb.RELAY_IT_IGNORE,RI_AIRNEG)){
				ERR(ERR_RELAY_PC_DISENG_FAIL);
			}else{
				ERR(ERR_RELAY_PC_ENG);
			}
		}
	}
}
void HV_Check(){
	//If 60V detected...
	if (HAL_GPIO_ReadPin(IND_60V_GPIO_Port, IND_60V_Pin) == GPIO_PIN_SET) {
		BITSET(hrcb.RELAY_STATES,RS_60V);
	}
	//Else if 60V not detected...
	else {
		BITRESET(hrcb.RELAY_STATES,RS_60V);
	}
}

void PC_Routine_Charge_Start(){
	Status_Set(STATUS_PC_Charge);		//Enter charger pre-charge state.
	Relay_AIRNEG_SetState(CLOSE);	//Close both AIRNEG and PC relays
	Relay_PC_SetState(CLOSE);

	IWDG_Delay(PRECHARGE_DELAY);	//Wait for pre-charge to happen with fixed delay.

	PC_Routine_Charge_Complete();	//Finish pre-charge.
}
void PC_Routine_Charge_Complete(){
	if(!Status_Check(STATUS_PC_Charge)){
		ERR(ERR_PC_CHARGE_COMPLETE_INVALID_ATTEMPT);
	}
	HV_Check();							//Gets current state of 60V indicator.
	if(!BITCHECK(hrcb.RELAY_STATES,RS_60V)){	//If no 60V detected...
		//ERR(ERR_60V_DISENG_CHARGE);
	}

	Relay_AIRPOS_SetState(CLOSE);	//Close AIRPOS, bringing TS live.
	Relay_PC_SetState(OPEN);
	hrcb.TS_ACTIVE = True;				//Set TS_ACTIVE flag high as TS is now on.
	Status_Set(STATUS_Charge);			//Enter charging state.
}

void PC_Routine_Discharge_Start(){
	HV_Check();
	if(BITCHECK(hrcb.RELAY_STATES,RS_60V)){
		ERR(ERR_60V_ENG)
	}
	Status_Set(STATUS_PC_Discharge);
	Relay_AIRNEG_SetState(CLOSE);
	HAL_TIM_Base_Start_IT(&PC_TIMER_TYPE);
	Relay_PC_SetState(CLOSE);

}
void PC_Routine_Discharge_Complete(){
	if(!Status_Check(STATUS_PC_Discharge)){
		ERR(ERR_PC_DISCHARGE_COMPLETE_INVALID_ATTEMPT);
	}
	HAL_TIM_Base_Stop_IT(&PC_TIMER_TYPE);
	hrcb.PC_DURATION = __HAL_TIM_GET_COUNTER(&PC_TIMER_TYPE);
	if(hrcb.PC_DURATION<PC_MINIMUM_DURATION){
		ERR(ERR_PC_FAST)
	}else if(hrcb.PC_DURATION>PC_MAXIMUM_DURATION){
		ERR(ERR_PC_SLOW)
	}

	__HAL_TIM_SET_COUNTER(&PC_TIMER_TYPE,0x00);

	IWDG_Delay(200);

	HV_Check();
	if(!BITCHECK(hrcb.RELAY_STATES, RS_60V)){
		//ERR(ERR_60V_DISENG);
	}

	Relay_AIRPOS_SetState(CLOSE);
	Relay_PC_SetState(OPEN);

	hrcb.TS_ACTIVE = True;
	Status_Set(STATUS_Discharge);
}

void PC_Overflow(){
	if(HAL_GPIO_ReadPin(IND_PC_GPIO_Port, IND_PC_Pin)==GPIO_PIN_RESET){
		ERR(ERR_PC_OVERFLOW_1)
	}else{
		ERR(ERR_PC_OVERFLOW_2)
	}
}

void Kill(){
	HAL_TIM_Base_Stop_IT(&SOFTCLK_TIMER_TYPE);
	HAL_TIM_Base_Stop_IT(&PC_TIMER_TYPE);
	BITSET(hrcb.RELAY_IT_IGNORE, RI_AIRNEG | RI_AIRPOS | RI_PC);

	HAL_GPIO_WritePin(RELAY_TRIGGER_AIRPOS_GPIO_Port, RELAY_TRIGGER_AIRPOS_Pin, GPIO_PIN_RESET);
	BITRESET(hrcb.RELAY_STATES,RS_AIRPOS);
	HAL_GPIO_WritePin(RELAY_TRIGGER_AIRNEG_GPIO_Port, RELAY_TRIGGER_AIRNEG_Pin, GPIO_PIN_RESET);
	BITRESET(hrcb.RELAY_STATES,RS_AIRNEG);
	HAL_GPIO_WritePin(RELAY_TRIGGER_PC_GPIO_Port, RELAY_TRIGGER_PC_Pin, GPIO_PIN_RESET);
	BITRESET(hrcb.RELAY_STATES,RS_PC);
	hrcb.TS_ACTIVE = False;

	BITRESET(hrcb.RELAY_IT_IGNORE, RI_AIRNEG | RI_AIRPOS | RI_PC);
}
void Shutdown(){
	BITSET(hrcb.RELAY_IT_IGNORE, RI_AIRNEG | RI_AIRPOS | RI_PC);

	Relay_AIRPOS_SetState(OPEN);
	Relay_PC_SetState(OPEN);
	Relay_AIRNEG_SetState(OPEN);
	hrcb.TS_ACTIVE = False;

	BITRESET(hrcb.RELAY_IT_IGNORE, RI_AIRNEG | RI_AIRPOS | RI_PC);

	Status_Set(STATUS_Idle);
}

void TS_Request(uint8_t request){
	if(request == hrcb.TS_ACTIVE){
		if(request){
			ERR(ERR_TS_ENGAGE_DOUBLE_REQUEST);
		}else{
			ERR(ERR_TS_DISENGAGE_DOUBLE_REQUEST);
		}
	}

	if(request){
		if(Status_Check(STATUS_Idle)){
			PC_Routine_Discharge_Start();
		}
	}else{
		Shutdown();
	}
}

uint8_t Status_Check(uint8_t status){
	if(hrcb.STATUS == status){
		return True;
	}else{
		return False;
	}
}
uint8_t Status_Set(uint8_t status){
	if(Status_Check(STATUS_Error)){
		return 1;
	}
	switch(status){
	case STATUS_Startup:
		if(hrcb.STATUS!=STATUS_Startup){
			ERR(ERR_STARTUP_REINIT);
		}
		break;
	case STATUS_Idle:
		if(Status_Check(STATUS_Charge) || Status_Check(STATUS_Discharge) || Status_Check(STATUS_Startup)){
			hrcb.STATUS = status;
		}else{
			ERR(ERR_STATUS_IDLE_ATTEMPT);
		}
		break;
	case STATUS_PC_Charge:
		if(Status_Check(STATUS_Idle)){
			hrcb.STATUS = status;
		}else{
			ERR(ERR_STATUS_PC_CHARGE_ATTEMPT);
		}
		break;
	case STATUS_PC_Discharge:
		if(Status_Check(STATUS_Idle)){
			hrcb.STATUS = status;
		}else{
			ERR(ERR_STATUS_PC_DISCHARGE_ATTEMPT);
		}
		break;
	case STATUS_Charge:
		if(Status_Check(STATUS_PC_Charge)){
			hrcb.STATUS = status;
		}else{
			ERR(ERR_STATUS_CHARGE_ATTEMPT);
		}
		break;
	case STATUS_Discharge:
		if(Status_Check(STATUS_PC_Discharge)){
			hrcb.STATUS = status;
		}else{
			ERR(ERR_STATUS_DISCHARGE_ATTEMPT);
		}
		break;
	case STATUS_Override:
		if(OVERRIDE_EN){
			hrcb.STATUS = status;
		}else{
			ERR(ERR_STATUS_OVERRIDE_ATTEMPT);
		}
		break;
	case STATUS_Error:
		ERR(ERR_STATUS_INVALID_ERROR_USAGE);	//ERRORS SHOULD ONLY BE CALLED VIA ERR() MACRO.
		break;
	default:
		ERR(ERR_STATUS_INVALID_ATTEMPT);
		break;
	}
	return 0;
}

void CAN_Transmit_State(){
	CAN_UTIL_SetData(	CAN_TX_STATE,
						hrcb.TS_ACTIVE*0x55,
						hrcb.RELAY_STATES,
						hrcb.STATUS,
						BYTE1(hrcb.PC_DURATION),
						BYTE0(hrcb.PC_DURATION),
						hrcb.ERR_STATE,
						0,
						0);
	CAN_UTIL_Transmit(&hcan, CAN_TX_STATE);
}

void Startup(){
	HAL_GPIO_WritePin(RELAY_TRIGGER_AIRNEG_GPIO_Port, RELAY_TRIGGER_AIRNEG_Pin, GPIO_PIN_RESET);	//Trigger relay.
	HAL_GPIO_WritePin(RELAY_TRIGGER_AIRPOS_GPIO_Port, RELAY_TRIGGER_AIRPOS_Pin, GPIO_PIN_RESET);	//Trigger relay.
	HAL_GPIO_WritePin(RELAY_TRIGGER_PC_GPIO_Port, RELAY_TRIGGER_PC_Pin, GPIO_PIN_RESET);	//Trigger relay.

	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
	IWDG_Delay(200);
	HAL_GPIO_WritePin(LED_INDICATOR_GPIO_Port, LED_INDICATOR_Pin, GPIO_PIN_SET);
	IWDG_Delay(200);
	HAL_GPIO_WritePin(LED_OKAY_GPIO_Port, LED_OKAY_Pin, GPIO_PIN_SET);
	IWDG_Delay(400);
	HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_INDICATOR_GPIO_Port, LED_INDICATOR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_OKAY_GPIO_Port, LED_OKAY_Pin, GPIO_PIN_RESET);
}

void Debounce_Power(){
//	if(HAL_GPIO_ReadPin(IND_PWR_FAN1_GPIO_Port, IND_PWR_FAN1_Pin)==GPIO_PIN_RESET){
//		ERR(ERR_NO_PWR_FAN1);
//	}
//	if(HAL_GPIO_ReadPin(IND_PWR_FAN2_GPIO_Port, IND_PWR_FAN2_Pin)==GPIO_PIN_RESET){
//		ERR(ERR_NO_PWR_FAN2);
//	}
	if(HAL_GPIO_ReadPin(IND_PWR_IMD_GPIO_Port, IND_PWR_IMD_Pin)==GPIO_PIN_RESET){
		ERR(ERR_NO_PWR_IMD);
	}
}
void Debounce_TS(){
	uint8_t request = HAL_GPIO_ReadPin(REQUEST_TS_GPIO_Port, REQUEST_TS_Pin);
	if(hrcb.TS_ACTIVE && (!request)){
		TS_Request(False);
	}else if((!hrcb.TS_ACTIVE) && request){
		TS_Request(True);
	}
}
void Debounce_PC(){
	if(HAL_GPIO_ReadPin(IND_PC_GPIO_Port, IND_PC_Pin)==GPIO_PIN_RESET){
		PC_Routine_Discharge_Complete();
	}
}
void Debounce_Relays(){
	Relay_AIRNEG_Check();
	Relay_AIRPOS_Check();
	Relay_PC_Check();
	HV_Check();
}

void DevCon_Charger(uint8_t state){
	if(state){
		hrcb.DEVCON_CHARGER = True;
		ScheduleTask(SCH_DEVCON_CHARGER, 3000, 0, 0);
	}else{
		ERR(ERR_DEVCON_CHARGER_LOST);
	}
}

void DevCon_Override(uint8_t state){
	if(state){
		hrcb.DEVCON_OVERRIDE = True;
		ScheduleTask(SCH_DEVCON_OVERRIDE, 1000, 0, 0);
	}else{
		ERR(ERR_DEVCON_OVERRIDE_LOST);
	}
}
