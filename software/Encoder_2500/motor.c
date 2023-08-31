#include "motor.h"
#include "arm_math.h"
#include "tzmx.h"
#include "user_fcn.h"


float ID_REF = 0.0F;
float IQ_REF = 0.0F;


float D_PI_I = 550.0F; 
float D_PI_KB = 50.0F;
float D_PI_LOW_LIMIT = -V_Limit;
float D_PI_P = 1.5F; 
float D_PI_UP_LIMIT = V_Limit;

float Q_PI_I = 550.0F;
float Q_PI_KB = 50.0F;
float Q_PI_LOW_LIMIT = -V_Limit;
float Q_PI_P = 1.5F;
float Q_PI_UP_LIMIT = V_Limit;

float SPEED_PI_I = 10.0F;
float SPEED_PI_KB = 0.015F;
float SPEED_PI_LOW_LIMIT = -7.0F;
float SPEED_PI_P = 0.002F;
float SPEED_PI_UP_LIMIT = 7.0F;

 u32 hPhaseA_OffSet;//2050
 u32 hPhaseB_OffSet;//2016
 u32 hPhaseC_OffSet;//2047
int32_T sector;
u8 State;
CURRENT_ABC_DEF Current_Iabc;
CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;
VOLTAGE_ALPHA_BETA_DEF Voltage_Alpha_Beta;
TRANSF_COS_SIN_DEF Transf_Cos_Sin;
CURRENT_DQ_DEF Current_Idq; 
VOLTAGE_DQ_DEF Voltage_DQ;
CURRENT_PID_DEF Current_D_PID;
CURRENT_PID_DEF Current_Q_PID;
SPEED_PID_DEF Speed_Pid;
u8 PWM4Direction=PWM2_MODE;
float Speed_Ref = 0.0F;
float Speed_Pid_Out;  
float Speed_Fdk;

float adc_p_speed;

int32_t circle_num=0;

/* extern */
extern float ref_in;
extern float ref_out;
extern float Uq;
extern float Kpp;

//encoder
s16 hPrevious_angle, hSpeed_Buffer[SPEED_BUFFER_SIZE], hRot_Speed;
static u8 bSpeed_Buffer_Index = 0;
static volatile u16 hEncoder_Timer_Overflow;
static bool bIs_First_Measurement = TRUE;
static bool bError_Speed_Measurement = FALSE;
static u16 hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
volatile u8 bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
float cnt;
volatile u8 ref_Signal_tim_500us = Ref_Signal_Time;

void board_config(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,PWM_PERIOD-1);

	HAL_ADCEx_InjectedStart_IT(&hadc1);

	HAL_TIM_Encoder_Start_IT(&ENC_TIM,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&ENC_TIM);
	HAL_TIM_Base_Start_IT(&htim4);
 
	PID_Init();	
	
	State=START;
	

	
	
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(State==START)
		Start_Up();
	else if(State==RUN)
		foc_algorithm_step();	 
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	if(htim->Instance == TIM2)//Encoder
		{
      if (hEncoder_Timer_Overflow != U16_MAX)  
      {
         hEncoder_Timer_Overflow++;
      }	
			if(State==RUN)
			{
				circle_num++;
			}
			if(circle_num>=U32_MAX)
				circle_num=0;  			
		}
		
	if(htim->Instance == TIM4)//5ms
	{
//		get_DC_BUS();
//		temper_trans();
		
	}

		
		
}

void HAL_SYSTICK_Callback(void)
{
	
	//500us if update, write HAL_InitTick()  1000U->2000U (1ms->500us)
	 if(hSpeedMeas_Timebase_500us !=0)  
  {								
    hSpeedMeas_Timebase_500us--;	  
  }									  
  else
  {
    hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
		ENC_Calc_Average_Speed();	
	}

	if (bPID_Speed_Sampling_Time_500us != 0 ) 
  {
    bPID_Speed_Sampling_Time_500us --;
  }
	else
  { 
    bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
		Speed_Fdk = (float)ENC_Get_Mechanical_Speed()*6.0f;	// feedback is rpm(0.1HZ)
    Speed_Pid_Calc(Speed_Ref,Speed_Fdk,&Speed_Pid_Out,&Speed_Pid);      
  }
	/*get Sine Wave*/
	if(State==RUN)
	{
		if(ref_Signal_tim_500us !=0)
		{
			ref_Signal_tim_500us--;
		}
		else
		{
			ref_Signal_tim_500us=Ref_Signal_Time;
			ref_deal_core();
			LED_DIR(1000);
		}
	}
	else if(State==FAULT)
			LED_DIR(80);
	
	/*PID Speed-loop Ref*/
//	ref_in=get_anlog_in(); 
	Speed_Ref = ref_out;
	
  
}



/*FOC Core part 10Khz*/
void foc_algorithm_step(void) 
{

	
	Clarke_Transf(get_Iab(),&Current_Ialpha_beta);
	Angle_To_Cos_Sin(ENC_Get_Electrical_Angle(),&Transf_Cos_Sin); //when use openloop, angle=cnt;ENC_Get_Electrical_Angle()
	Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);
	
	/*PID Control part*/
	IQ_REF = Speed_Pid_Out;
	Current_PID_Calc(ID_REF,Current_Idq.Id,&Voltage_DQ.Vd,&Current_D_PID);   
  Current_PID_Calc(IQ_REF,Current_Idq.Iq,&Voltage_DQ.Vq,&Current_Q_PID); 

	/*MPIC-MATLAB Auto Code  Control part*/

//	Voltage_DQ.Vd=2.0;
//	Voltage_DQ.Vq=0.0f;
//	Angle_To_Cos_Sin(cnt,&Transf_Cos_Sin); 
//	cnt+=0.01f;
	/*TZMX Control part*/	

	
	Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta); 
	SVPWM_Calc(Voltage_Alpha_Beta,Udc,PWM_TIM_PULSE_TPWM);

	
	
}
/*Motor Start & Encoder Align part*/
void Start_Up(void)
{
	static u32 wTimebase=0;
	wTimebase++;
	if(wTimebase<=T_ALIGNMENT_PWM_STEPS)
	{
		Clarke_Transf(get_Iab(),&Current_Ialpha_beta);
		Angle_To_Cos_Sin((float)ALIGNMENT_ANGLE/360.0f *2.0f*PI,&Transf_Cos_Sin); //when use openloop, angle=cnt;
		Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);
		IQ_REF = 0.0;
		ID_REF = I_ALIGNMENT * (float)wTimebase / (float)T_ALIGNMENT_PWM_STEPS;  
		Current_PID_Calc(ID_REF,Current_Idq.Id,&Voltage_DQ.Vd,&Current_D_PID);   
		Current_PID_Calc(IQ_REF,Current_Idq.Iq,&Voltage_DQ.Vq,&Current_Q_PID); 
		Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta); 
		SVPWM_Calc(Voltage_Alpha_Beta,Udc,PWM_TIM_PULSE_TPWM);
	}
	else
	{
		wTimebase = 0;  	
		Voltage_DQ.Vd =Voltage_DQ.Vq=0.0f;
		IQ_REF = 0.0F;
		ID_REF = 0.0F;
		ENC_ResetEncoder(); 
		__HAL_TIM_SET_COUNTER(&htim2,0);
		ENC_Clear_Speed_Buffer(); 
		State=RUN;
	}
	
	

}



float get_DC_BUS(void)
{
	static float temp,out_k,out_k1;
	float a=0.1;
	temp = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_4);
	temp = temp*SAMPLE_VOL_CON_FACTOR - 0.2f;
	
	out_k = a*temp+(1-a)*out_k1;
	out_k1 = out_k;

//	if(temp<Udc*0.8 || temp>Udc*1.2 )
//	{
//		shut_pwm();
//		State=FAULT;
//	}
	return out_k;
}

void shut_pwm(void)
{
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
}

float temper_trans(void)
{
	float Rt = 0.0f;// R now
	float R = 10000.0f;//10k
	float T0 = 237.15f+25.0f;//trans K
	float B = 3380.0f;
	float K = 273.15f;
	float VR=0.0f;
	uint16_t adc_v[2];
	float temp;
	static float out_k,out_k1;
	float a=0.1;
	


	for(int i=0;i<2;i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,50);
		adc_v[i]=HAL_ADC_GetValue(&hadc1);
	}

	
	
	VR = (float)adc_v[0] / 4096.0f * 3.3f;
	Rt = (3.3-VR) * 3300.0f/VR;
	temp = 1.0/(1.0/T0 + log(Rt/R)/B)-K+33.4;//33.4 is offset
	
	out_k = a*temp+(1-a)*out_k1;
	out_k1 = out_k;
	
//	if(temp>TEMPER_SET)
//	{
//		shut_pwm();
//		State=FAULT;
//	}
	
	return out_k;
	
}


float get_anlog_in(void)
{
	static float temp,out_k,out_k1;
	float a=0.1;
	uint16_t adc_v[2];
	for(int i=0;i<2;i++)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,50);
		adc_v[i]=HAL_ADC_GetValue(&hadc1);
	}
	
	adc_p_speed=adc_v[1];
	adc_p_speed = adc_p_speed-1060;  //0--2200
	if(adc_p_speed<=0)
		adc_p_speed=0;
	adc_p_speed=adc_p_speed/2200.0f;
	temp=adc_p_speed;
	out_k = a*temp+(1-a)*out_k1;
	out_k1 = out_k;
	return out_k;
}



void ENC_Clear_Speed_Buffer(void)
{   
  u32 i;

  for (i=0;i<SPEED_BUFFER_SIZE;i++)
  {
    hSpeed_Buffer[i] = 0;
  }
  bIs_First_Measurement = TRUE;
}
	void ENC_ResetEncoder(void)
{
  TIM2->CNT = COUNTER_RESET;
}
	 

/**********************************************************************************************************
get Theta
**********************************************************************************************************/ 
float ENC_Get_Electrical_Angle(void)
{
	return ((float)(ENC_Get_Electrical_Angle_S16()) / 32768.0f*PI );
}


float ENC_Get_Mechanical_Angle_X(void)
{
	float angle;
//	angle = (float)(__HAL_TIM_GET_COUNTER(&ENC_TIM)) / ((float)(4*ENCODER_PPR)) * 2.0f*PI + (circle_num)*2.0f*PI;
	if ( (TIM2->CR1 & TIM_COUNTERMODE_DOWN) != TIM_COUNTERMODE_DOWN)  
	{
		angle = (float)(__HAL_TIM_GET_COUNTER(&ENC_TIM)) / ((float)(4*ENCODER_PPR)) * 2.0f*PI + (circle_num)*2.0f*PI;
	}
	else
	{
		angle = (float)(__HAL_TIM_GET_COUNTER(&ENC_TIM)) / ((float)(4*ENCODER_PPR)) * 2.0f*PI - (circle_num)*2.0f*PI;
	}
	
	return angle; 
}


s16 ENC_Get_Electrical_Angle_S16(void)
{
  s32 temp;
  
  temp = (s32)(__HAL_TIM_GET_COUNTER(&ENC_TIM)) * (s32)(U32_MAX / (4*ENCODER_PPR));         
  temp *= (POLE_PAIR_NUM);  
  return((s16)(temp/65536 ));   //s16 result
}


/**********************************************************************************************************
get Iab
**********************************************************************************************************/ 
CURRENT_ABC_DEF get_Iab(void)
{
	CURRENT_ABC_DEF I_temp;
	float Ia_temp,Ib_temp,Ic_temp;
	Ia_temp = (s16)((s16)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1)-(s16)hPhaseA_OffSet)* SAMPLE_CURR_CON_FACTOR;
	Ib_temp = (s16)((s16)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2)-(s16)hPhaseB_OffSet)* SAMPLE_CURR_CON_FACTOR;
	Ic_temp = (s16)((s16)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3)-(s16)hPhaseC_OffSet)* SAMPLE_CURR_CON_FACTOR;
switch(sector)
	{
		case 4:
		case 5:
			I_temp.Ia = Ia_temp;
			I_temp.Ib = Ib_temp;
			break;
		case 6:
		case 1:
			I_temp.Ib = Ib_temp;
			I_temp.Ia = -Ic_temp-Ib_temp ;
			break;
		case 2:
		case 3:
			I_temp.Ia = Ia_temp;
			I_temp.Ib = -Ic_temp-Ia_temp ;
			break;
		default:
			I_temp.Ia = Ia_temp;
			I_temp.Ib = Ib_temp;
			break;			
	}
	
		I_temp.Ic = -I_temp.Ia - I_temp.Ib;
	return (I_temp);
}



void PID_Init(void)
{
	// Id- loop
	Current_D_PID.P_Gain = D_PI_P;
  Current_D_PID.I_Gain = D_PI_I;
  Current_D_PID.B_Gain = D_PI_KB;
  Current_D_PID.Max_Output = D_PI_UP_LIMIT;
  Current_D_PID.Min_Output = D_PI_LOW_LIMIT;
  Current_D_PID.I_Sum = 0.0f;
  //Iq- loop
  Current_Q_PID.P_Gain = Q_PI_P;
  Current_Q_PID.I_Gain = Q_PI_I;
  Current_Q_PID.B_Gain = Q_PI_KB;
  Current_Q_PID.Max_Output = Q_PI_UP_LIMIT;
  Current_Q_PID.Min_Output = Q_PI_LOW_LIMIT;
  Current_Q_PID.I_Sum = 0.0f;
	//Speed loop
	Speed_Pid.P_Gain = SPEED_PI_P;
  Speed_Pid.I_Gain = SPEED_PI_I;
  Speed_Pid.B_Gain = SPEED_PI_KB;
  Speed_Pid.Max_Output = SPEED_PI_UP_LIMIT;
  Speed_Pid.Min_Output = SPEED_PI_LOW_LIMIT;
  Speed_Pid.I_Sum = 0.0f;
	
}

/***************************************
current PID 
B is Integral windup gain ,
usually, it is about I gain 
***************************************/
void Current_PID_Calc(float ref_temp,float fdb_temp,float* out_temp,CURRENT_PID_DEF* current_pid_temp)
{
  float error;
  float temp;
  error = ref_temp - fdb_temp;
  temp = current_pid_temp->P_Gain * error + current_pid_temp->I_Sum;
  if (temp > current_pid_temp->Max_Output) 
  {
    *out_temp = current_pid_temp->Max_Output;
  } 
  else if (temp < current_pid_temp->Min_Output) 
  {
    *out_temp = current_pid_temp->Min_Output;
  } 
  else 
  {
    *out_temp = temp;
  }
  current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain * error) *FOC_PERIOD;
}

void Speed_Pid_Calc(float ref_temp,float fdb_temp,float* out_temp,SPEED_PID_DEF* current_pid_temp)
{

  float error;
  float temp;

  error =  ref_temp - fdb_temp;             //2*pi的作用是 单位转换   Hz转换为rad/s

  temp = (error + current_pid_temp->I_Sum) * current_pid_temp->P_Gain;

 
  if (temp > current_pid_temp->Max_Output) {
    *out_temp = current_pid_temp->Max_Output;
  } else if (temp < current_pid_temp->Min_Output) {
    *out_temp = current_pid_temp->Min_Output;
  } else {
    *out_temp = temp;
  }
  current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain* error) * SPEED_PID_PERIOD;
}



s16 ENC_Get_Mechanical_Speed(void)
{
  return(hRot_Speed);
}


s16 ENC_Calc_Rot_Speed(void)
{   
  s32 wDelta_angle;
  u16 hEnc_Timer_Overflow_sample_one, hEnc_Timer_Overflow_sample_two;
  u16 hCurrent_angle_sample_one, hCurrent_angle_sample_two;
  signed long long temp;
  s16 haux;
  
  if (!bIs_First_Measurement)
  {
    // 1st reading of overflow counter    
    hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow; 
    // 1st reading of encoder timer counter
    hCurrent_angle_sample_one = TIM2->CNT;
    // 2nd reading of overflow counter
    hEnc_Timer_Overflow_sample_two = hEncoder_Timer_Overflow;  
    // 2nd reading of encoder timer counter
    hCurrent_angle_sample_two = TIM2->CNT;      

    // Reset hEncoder_Timer_Overflow and read the counter value for the next
    // measurement
    hEncoder_Timer_Overflow = 0;
    haux = TIM2->CNT;   
    
    if (hEncoder_Timer_Overflow != 0) 
    {
      haux = TIM2->CNT; 
      hEncoder_Timer_Overflow = 0;            
    }
     
    if (hEnc_Timer_Overflow_sample_one != hEnc_Timer_Overflow_sample_two)
    { //Compare sample 1 & 2 and check if an overflow has been generated right 
      //after the reading of encoder timer. If yes, copy sample 2 result in 
      //sample 1 for next process 
      hCurrent_angle_sample_one = hCurrent_angle_sample_two;
      hEnc_Timer_Overflow_sample_one = hEnc_Timer_Overflow_sample_two;
    }
    
    if ( (TIM2->CR1 & TIM_COUNTERMODE_DOWN) == TIM_COUNTERMODE_DOWN)  
    {// encoder timer down-counting
      wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle - 
                    (hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
    }
    else  
    {//encoder timer up-counting
      wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle + 
                    (hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
    }
    
    // speed computation as delta angle * 1/(speed sempling time)
    temp = (signed long long)(wDelta_angle * SPEED_SAMPLING_FREQ);                                                                
    temp *= 10;  // 0.1 Hz resolution
    temp /= (4*ENCODER_PPR);
        
  } //is first measurement, discard it
  else
  {
    bIs_First_Measurement = FALSE;
    temp = 0;
    hEncoder_Timer_Overflow = 0;
    haux = TIM2->CNT;       
    // Check if Encoder_Timer_Overflow is still zero. In case an overflow IT 
    // occured it resets overflow counter and wPWM_Counter_Angular_Velocity
    if (hEncoder_Timer_Overflow != 0) 
    {
      haux = TIM2->CNT; 
      hEncoder_Timer_Overflow = 0;            
    }
  }
  
  hPrevious_angle = haux;  
 
  return((s16) temp);
}


void ENC_Calc_Average_Speed(void)
{   
  s32 wtemp;
  u16 hAbstemp;
  u32 i;
  u8 static bError_counter;
  float L_K=0.9f;
	static float Y_k,U_k,Y_k_1;
  wtemp = ENC_Calc_Rot_Speed();
  hAbstemp = ( wtemp < 0 ? - wtemp :  wtemp);

/* Checks for speed measurement errors when in RUN State and saturates if 
                                                                    necessary*/  
  if (State == RUN)
  {    
    if(hAbstemp < MINIMUM_MECHANICAL_SPEED)
    { 
      if (wtemp < 0)
      {
        wtemp = -(s32)(MINIMUM_MECHANICAL_SPEED);
      }
      else
      {
        wtemp = MINIMUM_MECHANICAL_SPEED;
      }
      bError_counter++;
    }
    else  if (hAbstemp > MAXIMUM_MECHANICAL_SPEED) 
          {
            if (wtemp < 0)
            {
              wtemp = -(s32)(MAXIMUM_MECHANICAL_SPEED);
            }
            else
            {
              wtemp = MAXIMUM_MECHANICAL_SPEED;
            }
            bError_counter++;
          }
          else
          { 
            bError_counter = 0;
          }
  
    if (bError_counter >= MAXIMUM_ERROR_NUMBER)
    {
     bError_Speed_Measurement = TRUE;
    }
    else
    {
     bError_Speed_Measurement = FALSE;
    }
  }
  else
  {
    bError_Speed_Measurement = FALSE;
    bError_counter = 0;
  }
  
/* Compute the average of the read speeds */
  
  hSpeed_Buffer[bSpeed_Buffer_Index] = (s16)wtemp;
  bSpeed_Buffer_Index++;
  
  if (bSpeed_Buffer_Index == SPEED_BUFFER_SIZE) 
  {
    bSpeed_Buffer_Index = 0;
  }

  wtemp=0;

  for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
    wtemp += hSpeed_Buffer[i];
    }
  wtemp /= SPEED_BUFFER_SIZE;
  
  hRot_Speed = ((s16)(wtemp));
		

		
		
}





void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp)
{
  Current_alpha_beta_temp->Ialpha = (Current_abc_temp.Ia - (Current_abc_temp.Ib + Current_abc_temp.Ic) * 0.5F) * 2.0F / 3.0F;
  Current_alpha_beta_temp->Ibeta = (Current_abc_temp.Ib - Current_abc_temp.Ic) * 0.866025388F * 2.0F / 3.0F;
}

void Angle_To_Cos_Sin(float angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp)
{
  cos_sin_temp->Cos = arm_cos_f32(angle_temp);
  cos_sin_temp->Sin = arm_sin_f32(angle_temp);
}

void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp)
{
  current_dq_temp->Id = current_alpha_beta_temp.Ialpha * cos_sin_temp.Cos + current_alpha_beta_temp.Ibeta * cos_sin_temp.Sin;
  current_dq_temp->Iq = -current_alpha_beta_temp.Ialpha * cos_sin_temp.Sin + current_alpha_beta_temp.Ibeta * cos_sin_temp.Cos;
}

void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp)
{
  v_alpha_beta_temp->Valpha = cos_sin_temp.Cos * v_dq_temp.Vd - cos_sin_temp.Sin * v_dq_temp.Vq;
  v_alpha_beta_temp->Vbeta  = cos_sin_temp.Sin * v_dq_temp.Vd + cos_sin_temp.Cos * v_dq_temp.Vq;
}



void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,float Udc_temp,float Tpwm_temp)
{
  
  float Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;
	u16 Tcmp4 ;
	float  hDeltaDuty;
  sector = 0;
  Tcmp1 = 0.0F;
  Tcmp2 = 0.0F;
  Tcmp3 = 0.0F;
	Tcmp4 = 0;//计算采样时机
  if (v_alpha_beta_temp.Vbeta > 0.0F) {
    sector = 1;
  }
  
  if ((1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 2;
  }
  
  if ((-1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 4;
  }
  
  switch (sector) {
  case 1:
    Tx = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 2:
    Tx = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    break;
    
  case 3:
    Tx = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    break;
    
  case 4:
    Tx = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    Ty = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 5:
    Tx = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    Ty = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
    
  default:
    Tx = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
  }
  
  f_temp = Tx + Ty;
  if (f_temp > Tpwm_temp) {
    Tx /= f_temp;
    Ty /= (Tx + Ty);
  }
  
  Ta = (Tpwm_temp - (Tx + Ty)) / 4.0F;
  Tb = Tx / 2.0F + Ta;
  Tc = Ty / 2.0F + Tb;
  switch (sector) {
  case 1:
    Tcmp1 = Tb;
    Tcmp2 = Ta;
    Tcmp3 = Tc;
	
		if((u16)(PWM_PERIOD-(u16)Tcmp1)>TW_AFTER)
		{
			Tcmp4=PWM_PERIOD-1;
		}
		else
		{
			hDeltaDuty = (u16)(Tcmp1 - Tcmp2);
			if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp1)*2) 
			{
					Tcmp4 = (u16)Tcmp1 - TW_BEFORE; // Ts before Phase A 
			}
			else
			{
				Tcmp4 = (u16)Tcmp1 + TW_BEFORE;
				if (Tcmp4 >= PWM_PERIOD)
				{        
					PWM4Direction=PWM1_MODE;
					Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
				}
			}
		}
		
    break;
    
  case 2:
    Tcmp1 = Ta;
    Tcmp2 = Tc;
    Tcmp3 = Tb;
	
		if((u16)(PWM_PERIOD-(u16)Tcmp2)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp2 - Tcmp1);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp2)*2) 
				{
						Tcmp4 = (u16)Tcmp2 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp2 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}
		
    break;
    
  case 3:
    Tcmp1 = Ta;
    Tcmp2 = Tb;
    Tcmp3 = Tc;
	
		if((u16)(PWM_PERIOD-(u16)Tcmp2)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp2 - Tcmp3);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp2)*2) 
				{
						Tcmp4 = (u16)Tcmp2 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp2 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}
		
    break;
    
  case 4:
    Tcmp1 = Tc;
    Tcmp2 = Tb;
    Tcmp3 = Ta;
		if((u16)(PWM_PERIOD-(u16)Tcmp3)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp3 - Tcmp2);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp3)*2) 
				{
						Tcmp4 = (u16)Tcmp3 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp3 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}	
    break;
    
  case 5:
    Tcmp1 = Tc;
    Tcmp2 = Ta;
    Tcmp3 = Tb;
		if((u16)(PWM_PERIOD-(u16)Tcmp3)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp3 - Tcmp1);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp3)*2) 
				{
						Tcmp4 = (u16)Tcmp3 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp3 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}	
    break;
    
  case 6:
    Tcmp1 = Tb;
    Tcmp2 = Tc;
    Tcmp3 = Ta;
			if((u16)(PWM_PERIOD-(u16)Tcmp1)>TW_AFTER)
			{
				Tcmp4=PWM_PERIOD-1;
			}
			else
			{
				hDeltaDuty = (u16)(Tcmp1 - Tcmp3);
				if (hDeltaDuty > (u16)(PWM_PERIOD-(u16)Tcmp1)*2) 
				{
						Tcmp4 = (u16)Tcmp1 - TW_BEFORE; // Ts before Phase A 
				}
				else
				{
					Tcmp4 = (u16)Tcmp1 + TW_BEFORE;
					if (Tcmp4 >= PWM_PERIOD)
					{        
						PWM4Direction=PWM1_MODE;
						Tcmp4 = (2 * PWM_PERIOD) - Tcmp4-1;
					}
				}
			}
    break;
  }
	
	if (PWM4Direction == PWM2_MODE)
  {
    //Set Polarity of CC4 High
    TIM1->CCER &= 0xDFFF;    
  }
  else
  {
    //Set Polarity of CC4 Low
    TIM1->CCER |= 0x2000;
  }
	
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(u16)Tcmp1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(u16)Tcmp2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(u16)Tcmp3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,(u16)Tcmp4); 

}




