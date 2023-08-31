#include "user_fcn.h"
#include "usart.h"
#include "arm_math.h"
#include "tim.h" 

extern u8 State;
extern float Speed_Fdk;
extern float Posi_Ref;
extern float Posi_Fdk;
extern float Speed_Ref;
extern CURRENT_DQ_DEF Current_Idq; 




extern float Posi_Ref ;
void control_cmd(void)
{


}

float muti_error;
void muti_switch(void)
{


	
}






void LED_DIR(u32 ms)
{
	static u32 cnt=0;
	cnt++;
	if(cnt%ms==0)
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	
	if(cnt>=U32_MAX)
		cnt=0;
	
}


