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
#include "foc.h"

#include "main.h"
#include "arm_math.h"
#include <stdbool.h>
#include "math.h"
#include "tim.h"

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

#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#define rad60 deg2rad(60)
#define SQRT3 1.73205080756887729353///����3
#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
/*----------------------------------------------------------------------------*
**                             Data Structures                                *
**----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
**                             Local Vars                                     *
**----------------------------------------------------------------------------*/
float D_U,D_V,D_W;//ռ�ձ�
/*----------------------------------------------------------------------------*
**                             Extern Function                                *
**----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
**                             Local Function                                 *
**----------------------------------------------------------------------------*/

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
float limit_max(float input,float lim)//�޷����ֵ
{
	if(input>lim)
		return lim;
	else
		return input;
}
float limit_min(float input,float lim)//�޷���Сֵ
{
	if(input<lim)
		return lim;
	else
		return input;
}
float Normalization(float input,float Norm)//��һ������
{
	float data=fmod(input,Norm);
	return (data>=0)?data:(data+Norm);
}
float cycle_diff(float diff, float cycle) //�������������ݵ���С��ֵ
{
    if (diff > (cycle / 2))
        diff -= cycle;
    else if (diff < (-cycle / 2))
        diff += cycle;
    return diff;
}

float Low_pass_filter(float input,float prop)//��ͨ�˲�
{
	static float last_data;
	last_data=(input*prop+last_data*(1-prop));
	return last_data;
}

//���յ���ĽǶȣ�phi�� ���е�Ƕȹ�ϵ��phi = pole_pairs * ��
//ֱ��/�����ѹ������d, q��  �޷���Χ[-1, 1]
//alpha, beta ��ֹ����ϵ��ѹ���� ͨ����Park�任�õ�
//sector �ο�ʸ������������1~6�� ��alpha��beta�ļ��ι�ϵ����
//�������PWMռ�ձȣ�d_u, d_v, d_w��

void SVPWM(float phi, float d, float q, float *d_u, float *d_v, float *d_w)  
{
    d = min(d, 1);
    d = max(d, -1);
    q = min(q, 1);
    q = max(q, -1);
    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};
    float sin_phi = arm_sin_f32(phi);
    float cos_phi = arm_cos_f32(phi);
    float alpha = 0;
    float beta = 0;
    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

    bool A = beta > 0;
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;

    int K = 4 * A + 2 * B + C;
    int sector = K_to_sector[K];

    float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
    float t_n = beta * arm_cos_f32(sector * rad60 - rad60) - alpha * arm_sin_f32(sector * rad60 - rad60);
    float t_0 = 1 - t_m - t_n;

    *d_u = t_m * v[sector - 1][0] + t_n * v[sector % 6][0] + t_0 / 2;
    *d_v = t_m * v[sector - 1][1] + t_n * v[sector % 6][1] + t_0 / 2;
    *d_w = t_m * v[sector - 1][2] + t_n * v[sector % 6][2] + t_0 / 2;
}

void SVPWM_SET_OUT(float angle_el,float Uq,float Ud)   //��Ƕ� d  q
{
	SVPWM(angle_el, Ud, Uq, &D_U, &D_V, &D_W);
	limit_max(D_U,0.90);
	limit_max(D_V,0.90);
	limit_max(D_W,0.90);
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1 ,(int)(TIM_1_8_PERIOD_CLOCKS*D_U));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2 ,(int)(TIM_1_8_PERIOD_CLOCKS*D_V));
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3 ,(int)(TIM_1_8_PERIOD_CLOCKS*D_W));
}

/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif
// End of xxx.c

