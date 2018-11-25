
#include "iostm8s003f3.h"
#include "System.h"
#include <string.h>
#include "intrinsics.h"

unsigned char EventMessedg = 0;


#define UART_BRR_H	300000UL
#define UART_BRR_L	9600UL

char USART_BUFF[40];
char OutBuff[40];
char REG_State[4];

char REG_CALL[2]={0,0};
void WatchDogInit (void)
{
    IWDG_KR=0x55;
    IWDG_PR=0x06;
    IWDG_RLR=0xFF;
    IWDG_KR=0xAA;
}

char Sec = 56, ms = 0;
char Unloked_DHT = 0;
void Clock_Config (void)
{
    CLK_ECKR_bit.HSEEN = 1;           
    CLK_SWCR_bit.SWEN=1;              
    while(CLK_ECKR_bit.HSERDY != 1) {}
    CLK_CKDIVR = 0;                   
    CLK_SWR = 0xB4;                   
    while (CLK_SWCR_bit.SWIF != 1){}  
}

void GPIO_Config (void)
{
  //    Call
  PB_DDR_bit.DDR4= 0;
  PB_CR1_bit.C14 = 0;
  PB_CR2_bit.C24 = 0;
  
  //Led 1
  PC_DDR_bit.DDR3= 1;
  PC_CR1_bit.C13 = 1;
  PC_CR2_bit.C23 = 1;
  PC_ODR_bit.ODR3= 0;
  
    //Led 1
  PC_DDR_bit.DDR6= 1;
  PC_CR1_bit.C16 = 1;
  PC_CR2_bit.C26 = 1;
  PC_ODR_bit.ODR6= 0;
  
  //PWM P       Tim2 CH1
  PD_DDR_bit.DDR4= 1;
  PD_CR1_bit.C14 = 1;
  PD_CR2_bit.C24 = 1;
  PD_ODR_bit.ODR4= 0; 
  
  // PWM N       Tim2 CH2
  PD_DDR_bit.DDR3= 1;
  PD_CR1_bit.C13 = 1;
  PD_CR2_bit.C23 = 1;
  PD_ODR_bit.ODR3= 0;
  
  // RS-485 Direct_Data
  PC_DDR_bit.DDR5= 1;
  PC_CR1_bit.C15 = 1;
  PC_CR2_bit.C25 = 1;
  PC_ODR_bit.ODR5= 0;

  // ADC        CH3
  PD_DDR_bit.DDR2= 0;
  PD_CR1_bit.C12 = 0;
  PD_CR2_bit.C22 = 0;
  
  // UART       TX
  PD_DDR_bit.DDR5= 1;
  PD_CR1_bit.C15 = 1;
  PD_CR2_bit.C25 = 0;  
  PD_ODR_bit.ODR5= 1;  
  
  // UART       RX  
  PD_DDR_bit.DDR6= 0;  
  PD_CR1_bit.C16 = 0;
  PD_CR2_bit.C26 = 0;
  
  DirectCan(0);
}

void ADC_Config(void)
{
    CLK_PCKENR2 = 0xff;                                                         
    ADC_CR1_ADON = 1;       //  Turn ADC on, note a second set is required to start the conversion.
    ADC_CSR_CH = 0x03;      //  ADC on AIN4 only.
    ADC_CR3_DBUF = 0;
    ADC_CR2_ALIGN = 1;      //  Data is right aligned.
    ADC_CSR_EOCIE = 0;      //  Enable the interrupt after conversion completed.

    ADC_CR1_ADON = 1;       //  Turn ADC on, note a second set is required to start the conversion.
    ADC_CR1_ADON = 1;       //  Turn ADC on, note a second set is required to start the conversion.
    while (!ADC_CSR_EOC);
    ADC_CSR_EOC = 0;
    ADC_CR1_ADON = 1; 
}

void PWM_Config(void)
{
	 TIM2_PSCR = (unsigned char)(Timer2_Div);	
  TIM2_ARRH = (unsigned char)(Timer2_ARR >> 8);
  TIM2_ARRL = (unsigned char)(Timer2_ARR);
  
  // Enable and settigs CH_2
   TIM2_CCER1_CC2P = 0;
   TIM2_CCER1_CC2E = 1;
   TIM2_CCMR2_OC2M = 6;
   TIM2_CCR2L = 0;
   
   TIM2_CCER1_CC1P = 0;
   TIM2_CCER1_CC1E = 1;
   TIM2_CCMR1_OC1M = 6;
   TIM2_CCR1L = 0;
   
   TIM2_IER_UIE = 0;            //  Disable interrupts.
   TIM2_SR1_UIF = 0;            //  Reset the interrupt otherwise it will fire again straight away.  
   TIM2_CR1_CEN = 1;            //  Enable Timer 2    
}

void DAC_Data (unsigned char value)
{
	AudioDataP = value;
        AudioDataN = 0xFF - value;
}
unsigned char CountConnect = 0;


void Timer16_Init  (void)
{
      TIM1_PSCRH = (Timer1_Div&0xFF00)>>8;     //  Prescaler = 16.
    TIM1_PSCRL = (Timer1_Div&0xFF);
    TIM1_ARRH = (Timer1_ARR&0xFF00)>>8;     //  Prescaler = 16.
    TIM1_ARRL = (Timer1_ARR&0xFF);
      TIM1_IER_UIE = 1;           //  Enable the update interrupts.
    TIM1_CR1_CEN = 1;           //  Finally enable the timer.
}

#pragma vector = TIM1_OVR_UIF_vector
__interrupt void TIM1_UP_OVF_IRQHandler(void)
{    
  TIM1_SR1_UIF = 0;       //  Reset the interrupt otherwise it will fire again straight away.  

            CountConnect++;
           // GPIO_WriteBit(GPIOA, GPIO_Pin_0, !(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0)));
            if (CountConnect > 3)
            {
            	UART_Config (UART_BRR_L);
				Tim16(DISABLE);

				REG_State[REG_State_Bit_MIC] = '0';
				REG_State[REG_State_Bit_RESET] = '1';
				REG_State[REG_State_Bit_UART_BRR] = '3';
				REG_CALL[REG_CALL_Bit_Calling] = '0';
            }
}

void Timer3_Init_10ms  (void)
{
    TIM4_PSCR = Timer4_Div;     //  Prescaler = 16.
    TIM4_ARR  = Timer4_ARR;     //  Set Timer4 value
    TIM4_IER_UIE = 1;           //  Enable the update interrupts.
    TIM4_CR1_CEN = 1;           //  Finally enable the timer.
}
unsigned char temp_dt = 0;
#pragma vector = TIM4_OVR_UIF_vector
__interrupt void TIM4_UPD_OVF_IRQHandler(void)
{    
             TIM4_SR_UIF = 0;       //  Reset the interrupt otherwise it will fire again straight away.  

}

void UART_Config (uint32_t BRR)
{
  unsigned long b_rate;
  //????????? ???????? ????????
  b_rate = F_MASTER/BRR;
  UART1_BRR2 = b_rate & 0x000F;
  UART1_BRR2 |= b_rate >> 12;
  UART1_BRR1 = (b_rate >> 4) & 0x00FF;
  //???????? ?????????
  UART1_CR1_PIEN = 0;
  //???????? ???????? ????????
  UART1_CR1_PCEN = 0;
  //8-?????? ?????
  UART1_CR1_M = 0;
  //???????? UART
  UART1_CR1_UART0 = 0;
  //????????? ?????????? ?? ??????????? ????????. ????????
  UART1_CR2_TIEN = 0;
  //????????? ?????????? ?? ?????????? ????????
  UART1_CR2_TCIEN = 0;
  //????????? ?????????? ?? ?????????? ????????? ????????
  UART1_CR2_RIEN = 1;
  UART1_CR2_ILIEN = 0;
  UART1_CR2_TEN = 1;
  UART1_CR2_REN = 1;
  UART1_CR2_SBK = 0;
  UART1_CR3_STOP = 0;
}


unsigned char count = 0;
char temp = 0;
void Send_R_TERM (void)
{
	count = 0;
	DirectCan(1);
	//for (count = 0; count < 6; count++)
	UART1_DR = ' ';
        while(!UART1_SR_TC);
	/*while (REGISTER_TERM[count] != 0)
	{
		USART1->TDR = REGISTER_TERM[count];
		count++;
		while (!USART_GetFlagStatus(USART1, USART_ISR_TC));
	}
	*/
			UART1_DR = 'R';
    while(!UART1_SR_TC);     
			UART1_DR = '_';
    while(!UART1_SR_TC);     
			UART1_DR = 'T';
    while(!UART1_SR_TC);     
			UART1_DR = 'E';
    while(!UART1_SR_TC);     
			UART1_DR = 'R';
    while(!UART1_SR_TC);     
			UART1_DR = 'M';
    while(!UART1_SR_TC);     
                             
			UART1_DR = ' ';
    while(!UART1_SR_TC);     
			UART1_DR = '\n';
    while(!UART1_SR_TC);
	DirectCan(0);

}
void SearchSTR (void)
{
	EventMessedg = 1;

		if ((temp == '\n')||(count > 39))
			{
				for (unsigned int dt = 0; dt < 50; dt++);
						if (strstr(USART_BUFF, REGISTER_CALL) != 0)
							{
                                                            if (REG_CALL[REG_CALL_Bit_Button] == '0')
                                                            {
                                                              SendCAN_String("R_CALL 00\n");
                                                            }
                                                            else if (REG_CALL[REG_CALL_Bit_Button] == '1')
                                                            {
                                                              SendCAN_String("R_CALL 10\n");
                                                              button = 0;
                                                            }
							}
							else if (strstr(USART_BUFF, REGISTER_ALLST) != 0)
								{
								count = 0;
									while ((USART_BUFF[count] != 'R')&&(count < 40))
										count++;
									while ((USART_BUFF[count] != ' ')&&(count < 40))
										count++;
									unsigned char n = ++count;
									while ((count - n)<2)
									{
										REG_CALL[count-n] = USART_BUFF[count];
										count++;
									}
									n+=2;
									while ((count - n)<4)
									{
										REG_State[count-n] = USART_BUFF[count];
										count++;
									}
									SendCAN_String(REGISTER_CALL);
									SendCAN_String(REG_CALL);
									SendCAN_String(REGISTER_STATE);
									SendCAN_String(REG_State);
									SendCAN_String(" \n");
								}
				}
				for (count = 0; count < 40; count++)
				{
					USART_BUFF[count] = 0;
				}
				count = 0;
                            UART1_CR2_RIEN = 1;
}

#pragma vector=UART1_R_RXNE_vector 
__interrupt void uart_rx_interrupt(void)
{
		temp = UART1_DR;
                UART1_SR_RXNE = 0;
		if (REG_State[REG_State_Bit_UART_BRR] == '2')
			{
				CountConnect = 0;
                                if (REG_State[REG_State_Bit_SPK] == '1')
                                {
                                  DAC_Data(temp);
                                }
				if (REG_State[REG_State_Bit_MIC] == '1')
				{
					DirectCan(1);
                                        unsigned char i= ADC_DRH, tempTX;
                                        tempTX =(unsigned char)(((((ADC_DRL + (i << 8))>>2)))&0xFF); 
					UART1_DR = tempTX;
                                        ADC_Enable();
                                        while(!UART1_SR_TC);
                                        DirectCan(0);
				}
			}
		else if ((temp != '\n')&&(count < 40))
		{
			USART_BUFF[count] = temp;
			count++;
		}
		else SearchSTR();
	
}

void SendCAN_String (char *str)
{
	unsigned char count = 0;
	DirectCan(1);

	//for (count = 0; count < 6; count++)
	UART1_DR = ' ';
        while(!UART1_SR_TC);
	while (str[count] != 0)
	{
		UART1_DR = str[count];
		count++;
		    while(!UART1_SR_TC);
	}
	DirectCan(0);
}



void InitMCU (void)
{
	Clock_Config();
	GPIO_Config();
	ADC_Config();
	PWM_Config();
	Timer16_Init();
	UART_Config(9600);
	Timer3_Init_10ms();
	//Timer16_Init();
	REG_ID[0] = '0';
	REG_ID[1] = '0';
	REG_CALL[0] = '0';
	REG_CALL[1] = '0';
	WatchDogInit();
            __enable_interrupt();
}



void REG_Proc (void)
{
	if (REG_CALL[REG_CALL_Bit_Button] == '1')
	LED(1);
	else LED(0);
	if (CallButton())
	{
		while (CallButton());
                button = 1;
		REG_CALL[REG_CALL_Bit_Button] = '1';
	}
	if (REG_CALL[REG_CALL_Bit_Calling]=='1')
	{
		REG_CALL[REG_CALL_Bit_Button] = '0';
		Tim16(ENABLE);
	}
	if (EventMessedg == 1)
	{
		EventMessedg = 0;
		if (REG_State[REG_State_Bit_SPK] == '1')
		{
			DAC_Enable();
		}
		else
		{
			DAC_Disable();
		}
		if (REG_State[REG_State_Bit_MIC] == '1')
		{
			ADC_Enable();
		}
		else
		{
			ADC_Disable();
		}
		if (REG_State[REG_State_Bit_UART_BRR] == '1')
		{
			UART_Config (UART_BRR_H);
			Tim16(ENABLE);
			REG_State[REG_State_Bit_UART_BRR] = '2';
		}
		if (REG_State[REG_State_Bit_RESET] == '1')
		{
                   // while (1);
		}
	}
}


unsigned char BEEP_Data = 0;
void BEEP (void)
{
  DAC_Enable();

	if (BEEP_Data)
		BEEP_Data = 0x7F;
	else BEEP_Data = 0;
		DAC_Data (BEEP_Data);
}
