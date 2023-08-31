#include "tzmx.h"
#include "math.h"
#include "arm_math.h"
#include "motor.h"
/*ref signal deal*/
float ref_in=0.0f;
float ref_out;
float low_pass_fator=20.0f;//low pass fator
float speed_ref_temp1;
float speed_ref_temp2,speed_ref_temp3,speed_ref_temp3_1;

float Kpp=0.02f;
	
extern CURRENT_DQ_DEF Current_Idq; 
extern VOLTAGE_DQ_DEF Voltage_DQ;

void ref_deal_core(void)
{
	speed_ref_temp1 = ref_in-speed_ref_temp3_1;
	speed_ref_temp2 = speed_ref_temp1 / low_pass_fator;
	speed_ref_temp3 = speed_ref_temp2 + speed_ref_temp3_1;
	ref_out = speed_ref_temp3;
	speed_ref_temp3_1 = speed_ref_temp3;
}


float Uq;

void tzmx_c_Outputs_wrapper(const float *iqk,
                            const float *n_error,
                            const float *nk,
                            float *uqk)
{
/* Output_BEGIN */

    
/* Output_END */
}


