/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   drivers/drvic/drv8301/src/32b/f28x/f2806x/drv8301.c
//! \brief  Contains the various functions related to the DRV8301 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "assert.h"
#include <math.h>

// drivers
#include "drv8301.h"
//#include "rtthread.h"
//#include "board.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals

DRV8301_Obj hdrv8301;
DRV_SPI_8301_Vars_t Vars;

// **************************************************************************
// the function prototypes handle
 int init_motor_control(void)
		{
			SPI_HandleTypeDef *hspi;
			DRV8301_SetUp(&hdrv8301,&Vars);
			DRV8301_writeData(&hdrv8301,&Vars);
//			DRV8301_readData(&hdrv8301,&Vars);

		}
		
//INIT_APP_EXPORT(init_motor_control);

void DRV8301_SetUp(DRV8301_Obj *handle,DRV_SPI_8301_Vars_t *vars)
{

	handle->spiHandle = &hspi3;
//	handle->EngpioHandle  = EN_GATE_GPIO_Port;
//	handle->EngpioNumber  = EN_GATE_Pin;
	handle->nCSgpioHandle = M0_nCS_GPIO_Port;
	handle->nCSgpioNumber = M0_nCS_Pin;
	
	HAL_GPIO_WritePin(EN_GATE_GPIO_Port,EN_GATE_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	    // Make sure the Fault bit is not set during startup
    while ((DRV8301_readSpi(handle,DRV8301_RegName_Status_1) & DRV8301_STATUS1_FAULT_BITS) != 0);

    // Wait for the DRV8301 registers to update
    HAL_Delay(1);
	
//	vars->Ctrl_Reg_1.PWM_MODE        = DRV8301_PwmMode_Six_Inputs;
//	vars->Ctrl_Reg_1.DRV8301_CURRENT = DRV8301_PeakCurrent_1p70_A;
//	vars->Ctrl_Reg_1.DRV8301_RESET   = DRV8301_Reset_Normal;
////	vars->Ctrl_Reg_1.OC_ADJ_SET      = DRV8301_VdsLevel_1p043_V;
//	vars->Ctrl_Reg_1.OC_ADJ_SET      = DRV8301_VdsLevel_0p730_V;	
////	vars->Ctrl_Reg_1.OC_MODE 				 = DRV8301_OcMode_CurrentLimit;
//	vars->Ctrl_Reg_1.OC_MODE 				 = DRV8301_OcMode_LatchShutDown; //DRV8301_OcMode_LatchShutDown  
//	
//	vars->Ctrl_Reg_2.DC_CAL_CH1p2 = DRV8301_DcCalMode_Ch1_Load | DRV8301_DcCalMode_Ch2_Load;
//	vars->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_20VpV;
//	vars->Ctrl_Reg_2.OCTW_SET = DRV8301_OcTwMode_Both;
//	vars->Ctrl_Reg_2.OC_TOFF = DRV8301_OcOffTimeMode_Normal;

		DRV8301_RegName_e  drvRegName;
    uint16_t drvDataNew;
		// Update Status Register 1
    drvRegName = DRV8301_RegName_Status_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
    vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
    vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
    vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
    vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
    vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
    vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
    vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
    vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
    vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
    vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);

    // Update Status Register 2
    drvRegName = DRV8301_RegName_Status_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
    vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);

    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
    vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
    vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
    vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
    vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);

    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
    vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
    vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
    vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);
		
		vars->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    vars->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A
    vars->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_20VpV;
}

bool DRV8301_isFault(DRV8301_Obj *handle)
{
  DRV8301_Word_t readWord;
  bool status=false;
	

  // read the data
  readWord = DRV8301_readSpi(handle,DRV8301_RegName_Status_1);

  if(readWord & DRV8301_STATUS1_FAULT_BITS)
    {
      status = true;
    }

  return(status);
} // end of DRV8301_isFault() function

uint16_t DRV8301_readSpi(DRV8301_Obj *handle, const DRV8301_RegName_e regName)
{

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  HAL_Delay(1);

  // Do blocking read
  uint16_t zerobuff = 0;
  uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, regName, 0);
  uint16_t recbuff = 0xbeef;
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);
	HAL_Delay(1);
  // Datasheet says you don't have to pulse the nCS between transfers, (16 clocks should commit the transfer)
  // but for some reason you actually need to pulse it.
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
	HAL_Delay(1);
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
	//HAL_Delay(1);
  HAL_SPI_TransmitReceive(handle->spiHandle, (uint8_t*)(&zerobuff), (uint8_t*)(&recbuff), 1, 1000);
 // HAL_Delay(1);
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
	//HAL_Delay(1);
	assert(recbuff != 0xbeef);
  return(recbuff & DRV8301_DATA_MASK);
}  // end of DRV8301_readSpi() function

void DRV8301_writeSpi(DRV8301_Obj *handle, const DRV8301_RegName_e regName,const uint16_t data)
{
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  HAL_Delay(1);

  // Do blocking write
  uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);
  HAL_Delay(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  HAL_Delay(1);
	
  return;
}  // end of DRV8301_writeSpi() function


void DRV8301_writeData(DRV8301_Obj *handle, DRV_SPI_8301_Vars_t *vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;
	if(hspi3.State == HAL_SPI_STATE_READY)
		vars->SndCmd = true;

  if(vars->SndCmd)
  {
    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = vars->Ctrl_Reg_1.DRV8301_CURRENT |  \
                 vars->Ctrl_Reg_1.DRV8301_RESET   |  \
                 vars->Ctrl_Reg_1.PWM_MODE     |  \
                 vars->Ctrl_Reg_1.OC_MODE      |  \
                 vars->Ctrl_Reg_1.OC_ADJ_SET;
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);

    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = vars->Ctrl_Reg_2.OCTW_SET      |  \
                 vars->Ctrl_Reg_2.GAIN          |  \
                 vars->Ctrl_Reg_2.DC_CAL_CH1p2  |  \
                 vars->Ctrl_Reg_2.OC_TOFF;
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);

    vars->SndCmd = false;
  }

  return;
}  // end of DRV8301_writeData() function


void DRV8301_readData(DRV8301_Obj *handle, DRV_SPI_8301_Vars_t *vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;

	if(hspi3.State == HAL_SPI_STATE_READY)
		vars->RcvCmd = true;
  if(vars->RcvCmd)
  {
    // Update Status Register 10.
    drvRegName = DRV8301_RegName_Status_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Stat_Reg_1.FAULT = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
    vars->Stat_Reg_1.GVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
    vars->Stat_Reg_1.PVDD_UV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
    vars->Stat_Reg_1.OTSD = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
    vars->Stat_Reg_1.OTW = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_OTW_BITS);
    vars->Stat_Reg_1.FETHA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
    vars->Stat_Reg_1.FETLA_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
    vars->Stat_Reg_1.FETHB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
    vars->Stat_Reg_1.FETLB_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
    vars->Stat_Reg_1.FETHC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
    vars->Stat_Reg_1.FETLC_OC = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);
    vars->Stat_Reg_1_Value = drvDataNew;

    // Update Status Register 2
    drvRegName = DRV8301_RegName_Status_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Stat_Reg_2.GVDD_OV = (bool)(drvDataNew & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
    vars->Stat_Reg_2.DeviceID = (uint16_t)(drvDataNew & (uint16_t)DRV8301_STATUS2_ID_BITS);
    vars->Stat_Reg_2_Value = drvDataNew;

    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Ctrl_Reg_1.DRV8301_CURRENT = (DRV8301_PeakCurrent_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
    vars->Ctrl_Reg_1.DRV8301_RESET = (DRV8301_Reset_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
    vars->Ctrl_Reg_1.PWM_MODE = (DRV8301_PwmMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
    vars->Ctrl_Reg_1.OC_MODE = (DRV8301_OcMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
    vars->Ctrl_Reg_1.OC_ADJ_SET = (DRV8301_VdsLevel_e)(drvDataNew & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);
    vars->Ctrl_Reg_1_Value = drvDataNew;

    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    vars->Ctrl_Reg_2.OCTW_SET = (DRV8301_OcTwMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
    vars->Ctrl_Reg_2.GAIN = (DRV8301_ShuntAmpGain_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
    vars->Ctrl_Reg_2.DC_CAL_CH1p2 = (DRV8301_DcCalMode_e)(drvDataNew & (uint16_t)(DRV8301_CTRL2_DC_CAL_1_BITS | DRV8301_CTRL2_DC_CAL_2_BITS));
    vars->Ctrl_Reg_2.OC_TOFF = (DRV8301_OcOffTimeMode_e)(drvDataNew & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);
    vars->Ctrl_Reg_2_Value = drvDataNew;
    vars->RcvCmd = false;
  }

  return;
}  // end of DRV8301_readData() function

// end of file
