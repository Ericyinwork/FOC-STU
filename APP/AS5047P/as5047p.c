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
#define Polar 7     //极对数
#define PI 3.14159265358979f
#define _2PI 6.28318530717958f


uint16_t as5047_rx_data;
unsigned char angle_mon_flag,angle_start_mon;///初始角度记录
#define rotor_phy_angle (angle - angle_start_mon)     // 转子物理角度
#define rotor_logic_angle rotor_phy_angle * Polar          // 转子多圈角度  极对数
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float angle;
uint16_t as5047_rx_data;
float angle_add,angle_Multi,angle_mon;////多圈角度变量
unsigned char angle_mon_flag;

/*----------------------------------------------------------------------------*
**                             Data Structures                                *
**----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*
**                             Local Vars                                     *
**----------------------------------------------------------------------------*/




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
	rt_thread_mdelay(1);
	AS5047P_CS_L;  // 设置CS低电平开始通信
  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)0x7fff,(uint8_t *)&as5047_rx_data,2);  // 启动SPI接收
  angle_mon_flag=1;
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
  //  printf("SPI3 Transmission Complete\n");
   as5047_rx_data = as5047_rx_data & 0x3FFF;  // Mask to get 14-bit angle data
   angle = _2PI * as5047_rx_data / 0x3FFF;  // Convert to angle in radians 
   if (angle_mon_flag == 1)  // If angle monitoring is enabled
   {
     angle_start_mon = angle;  // Record the initial angle
     angle_mon_flag = 0;  // Reset the flag
   //  printf("Initial angle recorded: %f radians\n", angle_start_mon);        
   }
   float angle_deff = (float)as5047_rx_data - angle_mon;  // Calculate angle difference
   if(abs(angle_deff) > (0.8*16383))  // If angle difference exceeds threshold
   {
     angle_add += (angle_deff > 0) ? -_2PI : _2PI;  // Adjust angle_add based on direction
   } 
   angle_mon = as5047_rx_data;
   angle_Multi = angle_add + angle;  // Calculate the multi-turn angle
 //  printf("Angle: %f radians, Multi-turn angle: %f radians\n", angle);

   AS5047P_CS_L;  // Set CS low to start next transmission
   HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)0x7fff,(uint8_t *)&as5047_rx_data,2);  // 启动SPI接收
  // printf("Next spi Transmit\n");
	}
}


/**---------------------------------------------------------------------------*
 **                         Compiler Flag                                     *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif
// End of xxx.c

