/****************************************************************************
 *
 * File Name:
 *  
 *    Author:
 *  
 *      Date:
 * 
 * Descriptions:
 * 
 *
 ******************************************************************************/
 
/*----------------------------------------------------------------------------*
**                             Dependencies                                   *
**----------------------------------------------------------------------------*/
#include "app.h"
#include <rtthread.h>

#include "tim.h"
#include "adc.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "foc.h"

/**---------------------------------------------------------------------------*
 **                            Debugging Flag                                 *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
**                             Compiler Flag                                  *
**----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern   "C"
{
#endif

/*----------------------------------------------------------------------------*
**                             Mcaro Definitions                              *
**----------------------------------------------------------------------------*/

#define THREAD_PRIORITY         25
#define THREAD_STACK_SIZE       512
#define THREAD_TIMESLICE        5

#define position_cycle 6 * 3.14159265358979 
#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define rad60 deg2rad(60)

/*----------------------------------------------------------------------------*
**                             Data Structures                                *
**----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
**                             Local Vars                                     *
**----------------------------------------------------------------------------*/
static arm_pid_instance_f32 pid_position;
static arm_pid_instance_f32 pid_speed;
static arm_pid_instance_f32 pid_torque_d;
static arm_pid_instance_f32 pid_torque_q;

extern 	float result;



////中值滤波////
#define lenth 3
float data_mon[lenth];
float lvbo_data[lenth];

float angle_diff;
float motor_speed;

float speed_diff,Uq_set,tor_set;
float tor_diff,speed_set;
float postion_diff;

unsigned int ADC_tim_count=0;
float vote_BUS,tempture=0;

float motor_i_u,motor_i_v,motor_i_d,motor_i_q;
float u_1, u_2;

float FOC_speed_pid(float speed,float _motor_speed);
float V_Q_pid=0;

float angle_mon_last=0;


float motor_speed_set=1,motor_tor_set=0,motor_pos_set=0;

/*----------------------------------------------------------------------------*
**                             Extern Function                                *
**----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
**                             Local Function                                 *
**----------------------------------------------------------------------------*/
/////////速度环////////////////

static float speed_loop(float speed_rad)
{
    float diff = speed_rad-motor_speed;
    return arm_pid_f32(&pid_speed, diff);
}
void lib_speed_control(float speed)
{
    float d = 0;
    float q = speed_loop(speed);
    SVPWM_SET_OUT(angle_Multi*7,0,q);
}

/////////位置环/////////////
static float position_loop(float rad)
{
    float diff = cycle_diff(rad -angle_Multi, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}

void lib_position_control(float rad)
{
    float d = 0;
    float q = position_loop(rad);
    SVPWM_SET_OUT(angle_Multi*7,0,q);
}

/*----------------------------------------------------------------------------*
**                             Public Function                                *
**----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
**                             Function Define                                *
**----------------------------------------------------------------------------*/
/*************************************************
* Function:
* Description:
* Author:
* Returns: 
* Parameter:
* History:
*************************************************/
static char thread2_stack[1024];
static struct rt_thread thread2;
/* 线程 2 入口 */
static void thread2_entry(void *param)
{
//    rt_uint32_t count = 0;

//    /* 线程 2 拥有较高的优先级，以抢占线程 1 而获得执行 */
//    for (count = 0; count < 10 ; count++)
//    {
//        /* 线程 2 打印计数值 */
//        rt_kprintf("thread2 count: %d\n", count);
//    }
//    rt_kprintf("thread2 exit\n");
    /* 线程 2 运行结束后也将自动被系统脱离 */
		// LOG_P("Angle: %f radians\r\n", angle);
	
}


/* 线程示例 */
int thread_sample(void)
{
    /* 初始化线程 2，名称是 thread2，入口是 thread2_entry */
    rt_thread_init(&thread2,
                   "thread2",
                   thread2_entry,
                   RT_NULL,
                   &thread2_stack[0],
                   sizeof(thread2_stack),
                   THREAD_PRIORITY - 1, THREAD_TIMESLICE);
    rt_thread_startup(&thread2);

    return 0;
}

/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(thread_sample, thread sample);

/*************************************************
* Function:
* Description:
* Author:
* Returns: 
* Parameter:
* History:
*************************************************/
int pwm_start(void)
{
		/////PWM init//////
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start (&htim1);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_1 );
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_2 );
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_3 );
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	

	
		set_motor_pid(          ///独立模式
      2.990,0.0000301 , 0,///位置环
      0.010010, 0.0010010, 0,///速度环
      0, 0, 0,///力矩环d
      0.0054, 0.001, 0);////力矩环q
	
	return 0;
}

/*************************************************
* Function:
* Description:
* Author:
* Returns: 
* Parameter:
* History:
*************************************************/
int adc_inject_it(void)
{
//  HAL_ADC_Start(&hadc1);  // 启动ADC  
//  HAL_ADCEx_InjectedStart_IT(&hadc1);
	  HAL_ADC_Start(&hadc2);  // 启动ADC  
  HAL_ADCEx_InjectedStart_IT(&hadc2);
	  HAL_ADC_Start(&hadc3);  // 启动ADC  
  HAL_ADCEx_InjectedStart_IT(&hadc3);
	
	return 0;
}

/*************************************************
* Function:
* Description:
* Author:
* Returns: 
* Parameter:
* History:
*************************************************/
void set_motor_pid(  //////PID参数   位置  速度  力矩 PID调节
float position_p, float position_i, float position_d,
float speed_p, float speed_i, float speed_d,
float torque_d_p, float torque_d_i, float torque_d_d,
float torque_q_p, float torque_q_i, float torque_q_d)
{
    pid_position.Kp = position_p;
    pid_position.Ki = position_i;
    pid_position.Kd = position_d;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i;
    pid_speed.Kd = speed_d;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i;
    pid_torque_d.Kd = torque_d_d;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i;
    pid_torque_q.Kd = torque_q_d;
    arm_pid_init_f32(&pid_position, false);//false代表清空内部增量数据
    arm_pid_init_f32(&pid_speed, false);
    arm_pid_init_f32(&pid_torque_d, false);
    arm_pid_init_f32(&pid_torque_q, false);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) //电流计算-clarke变换-Park变换-PID-SVPWM-PWM
{
  if (hadc->Instance == ADC2)
  {
//   u_1 = 3.3f * ((float)hadc->Instance->JDR1/ ((1 << 12) - 1) - 0.5);
     u_1 = 3.3f * ((float)hadc->Instance->JDR1/ ((1 << 12) - 1)-0.5);
  }
	if (hadc->Instance == ADC3)
	{
//		u_2 = 3.3f * ((float)hadc->Instance->JDR1 / ((1 << 12) - 1) - 0.5); 
		u_2 = 3.3f * ((float)hadc->Instance->JDR1 / ((1 << 12) - 1)-0.5); 
		float i_1= u_1 / 0.0005f / 20;
    float i_2 = u_2 / 0.0005f / 20;
	  motor_i_v= i_1;
    motor_i_u = i_2;
		float i_alpha = 0;
    float i_beta = 0;
    arm_clarke_f32(motor_i_u, motor_i_v, &i_alpha, &i_beta);
    float sin_value = arm_sin_f32(angle_Multi);
    float cos_value = arm_cos_f32(angle_Multi);
    arm_park_f32(i_alpha, i_beta, &motor_i_d, &motor_i_q, sin_value, cos_value);

    motor_i_d = Low_pass_filter(motor_i_d,0.1); 
    motor_i_q = Low_pass_filter(motor_i_q,0.1); 
		
				SVPWM_SET_OUT(angle * 7 ,0,result);	
//			lib_position_control(deg2rad(motor_pos_set));
//			lib_speed_control(motor_speed_set);	
	}

}  
/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif
// End of xxx.c

