/*
 * This file is part of the MicroPython project, http://micropython.org/
 */

/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/usbd_conf.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    26-February-2014
  * @brief   This file implements the USB Device library callbacks and MSP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "usbd_core.h"
#include "py/obj.h"
#include "py/mphal.h"
#include "irq.h"
#include "usb.h"

#if MICROPY_HW_USB_FS || MICROPY_HW_USB_HS

#if MICROPY_HW_USB_FS
PCD_HandleTypeDef pcd_fs_handle;
#endif
#if MICROPY_HW_USB_HS
PCD_HandleTypeDef pcd_hs_handle;
#endif


#define hpcd_USB_FS pcd_fs_handle 

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);


/*******************************************************************************
                       PCD BSP Routines
*******************************************************************************/

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
	if(hpcd->Instance==USB)
	{
		/* USER CODE BEGIN USB_MspInit 0 */

		/* USER CODE END USB_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_USB_CLK_ENABLE();

		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
		/* USER CODE BEGIN USB_MspInit 1 */

		/* USER CODE END USB_MspInit 1 */
	}
}

/**
  * @brief  DeInitializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
	if(hpcd->Instance==USB)
	{
		/* USER CODE BEGIN USB_MspDeInit 0 */

		/* USER CODE END USB_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USB_CLK_DISABLE();

		/* Peripheral interrupt Deinit*/
		HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);

		HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

		/* USER CODE BEGIN USB_MspDeInit 1 */

		/* USER CODE END USB_MspDeInit 1 */
	}
}

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/

/**
  * @brief  Setup stage callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
}

/**
  * @brief  Data Out stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
  * @brief  Data In stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
/*
This is now handled by the USB CDC interface.
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF(hpcd->pData);
}
*/

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
	USBD_SpeedTypeDef speed = USBD_SPEED_FULL;

	if ( hpcd->Init.speed != PCD_SPEED_FULL)
	{
		//Error_Handler();
	}
	/* Set Speed. */
	USBD_LL_SetSpeed((USBD_HandleTypeDef*)hpcd->pData, speed);

	/* Reset Device. */
	USBD_LL_Reset((USBD_HandleTypeDef*)hpcd->pData);
}

/**
  * @brief  Suspend callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
	/* Inform USB library that core enters in suspend Mode. */
	USBD_LL_Suspend((USBD_HandleTypeDef*)hpcd->pData);
	/* Enter in STOP mode. */
	/* USER CODE BEGIN 2 */
	if (hpcd->Init.low_power_enable)
	{
		/* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
		SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
	}
	/* USER CODE END 2 */
}

/**
  * @brief  Resume callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_Resume(hpcd->pData);
}

/**
  * @brief  ISOC Out Incomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

/**
  * @brief  ISOC In Incomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

/**
  * @brief  Connect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_DevConnected(hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
    USBD_LL_DevDisconnected(hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev, int high_speed) {
	/* Init USB Ip. */
	/* Link the driver to the stack. */
	hpcd_USB_FS.pData = pdev;
	pdev->pData = &hpcd_USB_FS;

	hpcd_USB_FS.Instance = USB;
	hpcd_USB_FS.Init.dev_endpoints = 8;
	hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
	{
		//Error_Handler( );
	}
	//#define MSC_IN_EP     (0x81)
	//#define MSC_OUT_EP    (0x01)
	//#define CDC_IN_EP     (0x83)
	//#define CDC_OUT_EP    (0x03)
	//#define CDC_CMD_EP    (0x82)
	//#define CDC2_IN_EP    (0x85)
	//#define CDC2_OUT_EP   (0x05)
	//#define CDC2_CMD_EP   (0x84)
	//#define HID_IN_EP_WITH_CDC (0x81)
	//#define HID_OUT_EP_WITH_CDC (0x01)
	//#define HID_IN_EP_WITH_MSC (0x83)
	//#define HID_OUT_EP_WITH_MSC (0x03)
	
	/* USER CODE BEGIN EndPoint_Configuration */
	uint16_t addr = 0;
	addr = 64 ;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x00 , PCD_SNG_BUF, addr);
	addr += 64;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x80 , PCD_SNG_BUF, addr);
	addr += 64;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x01 , PCD_SNG_BUF, addr);
	addr += 64;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x81 , PCD_SNG_BUF, addr);
	addr += 64;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x02 , PCD_SNG_BUF, addr);
	addr += 64;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x82 , PCD_SNG_BUF, addr);
	addr += 64;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x83 , PCD_SNG_BUF, addr);
	addr += 64;
	HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x03 , PCD_SNG_BUF, addr);
	
	//HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x84 , PCD_SNG_BUF, 0x118);
	//HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x04 , PCD_SNG_BUF, 0x158);
	/* USER CODE END EndPoint_Configuration_MSC */
	return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_DeInit(pdev->pData);

	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status; 
}

/**
  * @brief  Starts the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_Start(pdev->pData);
	 
	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_Stop(pdev->pData);

	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);

	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_EP_Close(pdev->pData, ep_addr);
	  
	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;  
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_EP_Flush(pdev->pData, ep_addr);
	  
	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;  
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_EP_SetStall(pdev->pData, ep_addr);

	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;  
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);  
	 
	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status; 
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
    PCD_HandleTypeDef *hpcd = pdev->pData;

    if ((ep_addr & 0x80) == 0x80) {
        return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
    } else {
        return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
    }
}

/**
  * @brief  Assigns an USB address to the device
  * @param  pdev: Device handle
  * @param  dev_addr: USB address
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_SetAddress(pdev->pData, dev_addr);
	 
	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;  
}

/**
  * @brief  Transmits data over an endpoint
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size)
{
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
	 
	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status;    
}

/**
  * @brief  Prepares an endpoint for reception
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf:pointer to data to be received
  * @param  size: data size
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size)
{
	HAL_StatusTypeDef hal_status = HAL_OK;
	USBD_StatusTypeDef usb_status = USBD_OK;

	hal_status = HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
	 
	usb_status =  USBD_Get_USB_Status(hal_status);

	return usb_status; 
}

/**
  * @brief  Returns the last transfered packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t  ep_addr) {
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/**
  * @brief  Delay routine for the USB Device Library
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay) {
    HAL_Delay(Delay);
}

//void *USBD_static_malloc(uint32_t size)
//{
//	static uint32_t mem[(sizeof(USBD_MSC_BOT_HandleTypeDef)/4)+1];/* On 32-bit boundary */
//	return mem;
//}
//
///**
//  * @brief  Dummy memory free
//  * @param  p: Pointer to allocated  memory address
//  * @retval None
//  */
//void USBD_static_free(void *p)
//{
//
//}

USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status)
{
	USBD_StatusTypeDef usb_status = USBD_OK;

	switch (hal_status)
	{
	case HAL_OK :
	  usb_status = USBD_OK;
	break;
	case HAL_ERROR :
	  usb_status = USBD_FAIL;
	break;
	case HAL_BUSY :
	  usb_status = USBD_BUSY;
	break;
	case HAL_TIMEOUT :
	  usb_status = USBD_FAIL;
	break;
	default :
	  usb_status = USBD_FAIL;
	break;
	}
	return usb_status;
}


#endif // MICROPY_HW_USB_FS || MICROPY_HW_USB_HS

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
