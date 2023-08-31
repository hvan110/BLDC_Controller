#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "stdio.h"
#include "tim.h"

typedef int int32_T;
typedef double real64_T;

#define MAX_SPEED 3600.0f
#define V_Limit  20.21f
#define Angle_OffSet 0   //6050
#define PWM_TIM_PULSE_TPWM  (CKTIM/(PWM_FREQ))
#define ENC_TIM htim2
#define Udc 35.0f
#define PI					3.14159265358979f
#define ADC_REF_V                   (float)(3.3f)
#define SAMPLE_RES                  (double)(0.01f)
#define AMP_GAIN                    (double)(15.0f)
#define SAMPLE_CURR_CON_FACTOR      (double)(ADC_REF_V/4095.0f/AMP_GAIN/SAMPLE_RES)
#define FOC_PERIOD (1.0f / (float)PWM_FREQ)
#define SPEED_PID_PERIOD (1.0f*(float)(PID_SPEED_SAMPLING_TIME+1)/2000.0f)


#define R_HIGH  200.0F
#define R_LOW		10.0F
#define SAMPLE_VOL_CON_FACTOR 	 ADC_REF_V /4095.0F / (R_LOW/(R_LOW+R_HIGH))

#define TEMPER_SET 60.0f


#define PID_SPEED_SAMPLING_500us      0     // min 500usec
#define PID_SPEED_SAMPLING_1ms        1
#define PID_SPEED_SAMPLING_2ms        3     // (3+1)*500usec = 2msec
#define PID_SPEED_SAMPLING_5ms        9		// (9+1)*500usec = 5msec		
#define PID_SPEED_SAMPLING_10ms       19	// (19+1)*500usec = 10msec
#define PID_SPEED_SAMPLING_20ms       39	// (39+1)*500usec = 20msec
#define PID_SPEED_SAMPLING_127ms      255   // max (255-1)*500us = 127 ms
#define SPEED_BUFFER_SIZE   8

#define SPEED_SAMPLING_TIME   (u8)(PID_SPEED_SAMPLING_1ms)
#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))
#define MAXIMUM_ERROR_NUMBER (u8)100
#define MINIMUM_MECHANICAL_SPEED_RPM  (u32)0   //rpm
#define MAXIMUM_MECHANICAL_SPEED_RPM  (u32)5000 //rpm
#define MINIMUM_MECHANICAL_SPEED  (u16)(MINIMUM_MECHANICAL_SPEED_RPM/6)
#define MAXIMUM_MECHANICAL_SPEED  (u16)(MAXIMUM_MECHANICAL_SPEED_RPM/6)
#define PID_SPEED_SAMPLING_TIME   (u8)(PID_SPEED_SAMPLING_2ms)
#define Ref_Signal_Time PID_SPEED_SAMPLING_1ms
#define SAMPLING_FREQ   ((u16)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz
#define T_ALIGNMENT           (u16) 100    // Alignment time in ms
#define I_ALIGNMENT           (float) 10.0F//定位时使用额定电流，这时Iq = 0,可以适当加大启动扭矩。 
#define T_ALIGNMENT_PWM_STEPS     (u32) ((T_ALIGNMENT * SAMPLING_FREQ)/1000) 

#define PWM2_MODE 0
#define PWM1_MODE 1
#define TNOISE_NS 1550  
#define TRISE_NS 1550     
#define SAMPLING_TIME_NS   700  
#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS) * 168uL)/1000uL) 
#define TNOISE (u16)((((u16)(TNOISE_NS)) * 168uL)/1000uL)
#define TRISE (u16)((((u16)(TRISE_NS)) * 168uL)/1000uL)
#define TDEAD (u16)((DEADTIME_NS * 168uL)/1000uL)
#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif
#define TW_AFTER ((u16)(((DEADTIME_NS+MAX_TNTR_NS)*168uL)/1000ul))
#define TW_BEFORE (((u16)(((((u16)(SAMPLING_TIME_NS)))*168uL)/1000ul))+1)



typedef struct
{
  float Ia;
  float Ib;
  float Ic;
}CURRENT_ABC_DEF;

typedef struct
{
  float Ialpha;
  float Ibeta;
}CURRENT_ALPHA_BETA_DEF;

typedef struct
{
  float Valpha;
  float Vbeta;
}VOLTAGE_ALPHA_BETA_DEF;

typedef struct
{
  float Cos;
  float Sin;
}TRANSF_COS_SIN_DEF;

typedef struct
{
  float Id;
  float Iq;
}CURRENT_DQ_DEF;

typedef struct
{
  float Vd;
  float Vq;
}VOLTAGE_DQ_DEF;

typedef struct
{
  float P_Gain;
  float I_Gain;
  float D_Gain;
  float B_Gain;
  float Max_Output;
  float Min_Output;
  float I_Sum;
}CURRENT_PID_DEF;

typedef struct
{
  float P_Gain;
  float I_Gain;
  float D_Gain;
  float B_Gain;
  float Max_Output;
  float Min_Output;
  float I_Sum;
}SPEED_PID_DEF;

typedef enum 
{
IDLE, INIT, START, RUN, STOP, BRAKE, WAIT, FAULT
} SystStatus_t;



float get_DC_BUS(void);
float temper_trans(void);
void ENC_ResetEncoder(void);
void ENC_Clear_Speed_Buffer(void);
void board_config(void);
void foc_algorithm_step(void);
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp);
void Angle_To_Cos_Sin(float angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp);
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp);
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp);
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,float Udc_temp,float Tpwm_temp);
float ENC_Get_Electrical_Angle(void);
s16 ENC_Get_Mechanical_Angle(void);
s16 ENC_Get_Electrical_Angle_S16(void);
CURRENT_ABC_DEF get_Iab(void);
void Current_PID_Calc(float ref_temp,float fdb_temp,float* out_temp,CURRENT_PID_DEF* current_pid_temp);
void Speed_Pid_Calc(float ref_temp,float fdb_temp,float* out_temp,SPEED_PID_DEF* current_pid_temp);
void PID_Init(void);
void ENC_Calc_Average_Speed(void);
float ENC_Calc_Rot_Speed1(void);
s16 ENC_Calc_Rot_Speed(void);
s16 ENC_Get_Mechanical_Speed(void);
void Start_Up(void);
float ENC_Get_Mechanical_Angle_X(void);
float get_anlog_in(void);

#endif


