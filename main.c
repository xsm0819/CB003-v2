#include "System.h"
#include "Device.h"
#include "iostm8s003f3.h"

int main(void)
{
	InitMCU();
    while(1)
    {
    	IWDG_ReloadCounter();
    	REG_Proc();
    	
    }
}
