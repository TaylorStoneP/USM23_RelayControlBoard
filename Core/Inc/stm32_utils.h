#ifndef INC_STM32_UTILS_H_
#define INC_STM32_UTILS_H_

#include "main.h"

#define True 1
#define False 0

#define SOFTCLK_TIMER_TYPE htim1

extern TIM_HandleTypeDef SOFTCLK_TIMER_TYPE;

#define IWDG_EN False
#define IWDG_DELAY_PERIOD 10

#if IWDG_EN == True
extern IWDG_HandleTypeDef hiwdg;
#define IWDG_RESET() HAL_IWDG_Refresh(&hiwdg);
#else
#define IWDG_RESET()
#endif

#define OUTPUT_SET(pin,port,value) 	HAL_GPIO_WritePin(port,pin, value)
#define OUTPUT_TOGGLE(pin,port) 	HAL_GPIO_TogglePin(port,pin)
#define INPUT_READ(pin,port) 		HAL_GPIO_ReadPin(port,pin)

enum{
	BIT1 = 0x1,
	BIT2 = 0x2,
	BIT3 = 0x4,
	BIT4 = 0x8,
	BIT5 = 0x10,
	BIT6 = 0x20,
	BIT7 = 0x40,
	BIT8 = 0x80,
	BIT9 = 0x100,
	BIT10 = 0x200,
	BIT11 = 0x400,
	BIT12 = 0x800,
	BIT13 = 0x1000,
	BIT14 = 0x2000,
	BIT15 = 0x4000,
	BIT16 = 0x8000,
	BIT17 = 0x10000,
	BIT18 = 0x20000,
	BIT19 = 0x40000,
	BIT20 = 0x80000,
	BIT21 = 0x100000,
	BIT22 = 0x200000,
	BIT23 = 0x400000,
	BIT24 = 0x800000,
	BIT25 = 0x1000000,
	BIT26 = 0x2000000,
	BIT27 = 0x4000000,
	BIT28 = 0x8000000,
	BIT29 = 0x10000000,
	BIT30 = 0x20000000,
	BIT31 = 0x40000000,
	BIT32 = 0x80000000

};

#define BITCHECK(reg,bit) 	((reg&(bit)) == (bit))		//Checks if bit is set.
#define BITSHIFTCHECK(reg,shift) ((reg>>(shift))&0x1)	//Checks bit at shift level is set. Shift 0->BIT0, 1->BIT1, 2->BIT2 etc.
#define BITSET(reg,bit)		(reg |= (bit))				//Sets bit in register.
#define BITRESET(reg,bit) 	(reg &= ~(bit))				//Clears bit in register.
#define BITTOGGLE(reg,bit) 	(reg ^= (bit))				//Toggles bit in register.

//Increments a counter that resets on n_loops -> Never reaches n_loops.
#define INCLOOP(counter, n_loops) counter=(counter+1)%n_loops;if(counter==0)

#define BYTE3(reg)	((reg>>24)&0xFF)
#define BYTE2(reg)	((reg>>16)&0xFF)
#define BYTE1(reg)	((reg>>8)&0xFF)
#define BYTE0(reg) 	(reg&0xFF)

typedef struct{
	uint32_t time_ms;
	uint8_t flag;
	uint8_t auto_schedule;
	uint32_t period;
} Schedule;
extern Schedule CurrentTime;

typedef enum SCHEDULES{
	SCH_PC_CHARGE_HV_CHECK,
	SCH_LED_FAULT,
	SCH_LED_INDICATOR,
	SCH_LED_OKAY,
	SCH_CAN_STATE,
	SCH_DEBOUNCE_RELAYS,
	SCH_DEBOUNCE_PC,
	SCH_DEBOUNCE_TS_REQUEST,
	SCH_DEBOUNCE_PWR,
	SCH_DEVCON_CHARGER,
	SCH_DEVCON_OVERRIDE,
	SCH_CHARGER_CHECK,
	SCH_COMPLETE_STARTUP,
	SCH_PC_OVERFLOW,
	SCHEDULE_N
}SCHEDULES;
extern Schedule ScheduleQueue[SCHEDULE_N];

#define SCHEDULE_HANDLE(task) 	if(ScheduleQueue[task].flag){ \
		ScheduleQueue[task].flag=0;

void ScheduleTask(SCHEDULES schedule, uint32_t period, uint8_t auto_re, uint32_t offset);
void ScheduleTaskStop(SCHEDULES schedule);
void TaskScheduleSoftClock();
void TaskScheduleHandler();
void TaskScheduleSoftClock_FlagSet();

void IWDG_Delay(uint32_t delay);

#endif /* INC_STM32_UTILS_H_ */
