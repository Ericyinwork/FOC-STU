/****************************************************************************
 *
 * File Name:   as5047p.c
 *  
 * Author:  
 *  
 * Date:            
 * 
 * Descriptions:            
 * 
 *
 ******************************************************************************/
/*----------------------------------------------------------------------------*
**                             Dependencies                                   *
**----------------------------------------------------------------------------*/
#include "as5047p.h"
#include "dma.h"
#include "utils.h"

/**---------------------------------------------------------------------------*
 **                            Debugging Flag                                 *
 **---------------------------------------------------------------------------*/
 
//#define DBG_TAG               "as5047p"    //dbg�ı�ͷ
//#ifdef RT_AS5047P_DEBUG                    //dbg�궨��
//#define DBG_LVL               DBG_LOG
//#else
//#define DBG_LVL               DBG_ERROR
//#endif
//#include <rtdbg.h>						 //dbgͷ�ļ�

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
#define Polar 7     //������
#define PI 3.14159265358979f
#define _2PI 6.28318530717958f
extern 	float result;


#define rotor_phy_angle (angle - angle_start_mon)     // ת������Ƕ�
#define rotor_logic_angle rotor_phy_angle * Polar          // ת�Ӷ�Ȧ�Ƕ�  ������
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define abs(x) ((x)>0?(x):-(x))



/*----------------------------------------------------------------------------*
**                             Data Structures                                *
**----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
**                             Local Vars                                     *
**----------------------------------------------------------------------------*/
float angle;
uint16_t as5047_rx_data;
float angle_add,angle_Multi,angle_mon;////��Ȧ�Ƕȱ���
unsigned char angle_mon_flag,angle_start_mon;///��ʼ�Ƕȼ�¼


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

void as5047_start(void)
{
	AS5047P_CS_H;
	HAL_Delay(1);
	AS5047P_CS_L;  // ����CS�͵�ƽ��ʼͨ��
	HAL_Delay(10);
  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)0x7ffff,(uint8_t *)&as5047_rx_data,2);  // ����SPI����

}


/*************************************************
* Function:
* Description:
* Author:
* Returns: 
* Parameter:
* History:
*************************************************/
//** callback function **//
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // Transmission complete callback
  if (hspi->Instance == SPI3)
  {
    // Handle SPI3 transmission complete
    AS5047P_CS_H;  // Set CS high to end transmission
	//	LOG_D("SPI3 Transmission Complete!\r\n");
   as5047_rx_data = as5047_rx_data & 0x3FFF;  // Mask to get 14-bit angle data
   angle = _2PI * as5047_rx_data / 0x3FFF;  // Convert to angle in radians 
   if (angle_mon_flag == 1)  // If angle monitoring is enabled
   {
     angle_start_mon = angle;  // Record the initial angle
     angle_mon_flag = 0;  // Reset the flag
	//	 LOG_D("Initial angle recorded: %d radians\n", angle_start_mon);    
   }
   float angle_deff = (float)as5047_rx_data - angle_mon;  // Calculate angle difference
   if(abs(angle_deff) > (0.8*16383))  // If angle difference exceeds threshold
   {
     angle_add += (angle_deff > 0) ? -_2PI : _2PI;  // Adjust angle_add based on direction
   } 
   angle_mon = as5047_rx_data;
   angle_Multi = angle_add + angle;  // Calculate the multi-turn angle
	// LOG_D("Angle: %f radians, Multi-turn angle: %d radians\r\n", angle);


   HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)0x7fff,(uint8_t *)&as5047_rx_data,2);  // ����SPI����
	 
	 AS5047P_CS_L;  // Set CS low to start next transmission


	// LOG_D("Next spi Transmit!\r\n");
	}
}


/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif
// End of xxx.c

