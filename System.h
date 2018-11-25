#ifndef __SYSTEM_H_
#define __SYSTEM_H_

#include <inttypes.h>


#define F_MASTER        24000000UL
#define UART_BRR_H	300000UL
#define UART_BRR_L	9600UL

extern char REG_CALL[2];
static char REG_ID[2];

static char button = 0;

extern char OutBuff[40];

#define AudioDataP              TIM2_CCR2L
#define AudioDataN              TIM2_CCR1L

#define IWDG_ReloadCounter()    IWDG_KR=0xAA
#define Tim16(x)                TIM1_CR1_CEN = (x);

#define Timer4_Div              0x07
#define Timer4_ARR              0xFF

#define Timer2_Div              0x00
#define Timer2_ARR              0xFF

#define Timer1_Div              640
#define Timer1_ARR              1667    //5000

extern char Sec;
extern char ms;
static char ADCValue;

#define DirectCan(x)						PC_ODR_bit.ODR5 = (x)
#define CallButton()						PB_IDR_bit.IDR4
#define LED(x)							PC_ODR_bit.ODR6 = !(x)

#define ADC_Enable()						ADC_CR1_ADON = 1
#define ADC_Disable()						ADC_CR1_ADON = 0
#define DAC_Enable()						TIM2_CR1_CEN = 1
#define DAC_Disable()						TIM2_CCR1L=0; TIM2_CCR2L=0; TIM2_CR1_CEN = 0

#define MyID                                                    (const char *)("00")
void SendCAN_String (char *str);
void Clock_Config (void);
void ADC_Config (void);
void PWM_Config (void);
void GPIO_Config (void);
void UART_Config (uint32_t BRR);
void DAC_Data (unsigned char value);
void Timer16_Init (void);
void Timer3_Init_10ms  (void);
void WatchDogInit (void);
#define ENABLE          1
#define DISABLE         !ENABLE

extern unsigned char EventMessedg;


#define REG_CALL_Bit_Button		0
#define REG_CALL_Bit_Calling	        1

#define REG_State_Bit_SPK		0
#define REG_State_Bit_MIC		1
#define REG_State_Bit_UART_BRR	        2
#define REG_State_Bit_RESET		3

#define GET		("GET")
#define SETT		("SET")
#define REGISTER_TERM	("R_TERM")
#define REGISTER_HUMI	("R_HUMI")
#define REGISTER_CALL	("GET R_CALL")
#define REGISTER_STATE	("R_STAT")
#define REGISTER_ID	("R_S_ID")
#define REGISTER_ALLST	("SET R_PARAM")


void InitMCU (void);
void REG_Proc (void);
void BEEP (void);
#endif /*	__SYSTEM_H_		*/
