/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "usbd_cdc_if.h"
#include <stdbool.h>
#ifdef __STDC_NO_ATOMICS__ // since gcc 4.9
#error "Do not support C11 atomic"
#else
#include <stdatomic.h>
#endif
#include "swd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*cdc_writeFirm)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
LL_USART_InitTypeDef USART_InitStruct = {0};

#define UART_TX_SIZE 2048
#define UART_RX_SIZE 4096

uint8_t TX_USART_Buffer[UART_TX_SIZE];
atomic_uint TxInPosition;
atomic_uint TxOutPosition;
uint8_t RX_USART_Buffer[UART_RX_SIZE];
atomic_uint RxOutPosition;

atomic_bool bCDCTXGoing;
atomic_bool bTXGoing;
bool bWorking;

uint8_t Firmware_Buffer[10240];
atomic_uint uFirmInPos;
uint32_t uFirmOutPos;

atomic_uint CDCRxCallback;

uint32_t uFirmWritePos;
uint32_t uFirmSize;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// USB to UART Begin
void CheckRxDataAndSendToCDC(void)
{
  uint32_t ulRxOutPosition = atomic_load(&RxOutPosition);
  uint16_t RxInPosition = sizeof(RX_USART_Buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
  int16_t iMinus = RxInPosition - ulRxOutPosition;
  atomic_store(&bCDCTXGoing, true);
  if (iMinus == 0)
    atomic_store(&bCDCTXGoing, false);
  else if (iMinus > 0)
  {
    CDC_Transmit_FS(RX_USART_Buffer + ulRxOutPosition, iMinus);
    ulRxOutPosition += iMinus;
  }
  else
  {
    // Send data close to tail first
    uint16_t ReceivedData = sizeof(RX_USART_Buffer) - ulRxOutPosition;
    CDC_Transmit_FS(RX_USART_Buffer + ulRxOutPosition, ReceivedData);
    ulRxOutPosition = 0;
  }
  atomic_store(&RxOutPosition, ulRxOutPosition);
}

void LL_UART_Tx(uint8_t *pBuffer, uint16_t uLen)
{
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, uLen);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)pBuffer);

  LL_DMA_ClearFlag_TC4(DMA1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

void CheckTxDataAndSendToTarget(void)
{
  uint32_t ulTxOutPosition = atomic_load(&TxOutPosition);
  int32_t iMinus = atomic_load(&TxInPosition) - ulTxOutPosition;
  atomic_store(&bTXGoing, true);
  if (iMinus == 0)
    atomic_store(&bTXGoing, false);
  else if (iMinus > 0)
  {
    LL_UART_Tx(TX_USART_Buffer + ulTxOutPosition, iMinus);
    ulTxOutPosition += iMinus;
  }
  else
  {
    uint16_t uSizeTx = sizeof(TX_USART_Buffer) - ulTxOutPosition;
    LL_UART_Tx(TX_USART_Buffer + ulTxOutPosition, uSizeTx);
    ulTxOutPosition = 0;
  }
  atomic_store(&TxOutPosition, ulTxOutPosition);
}

void DMA1_Channel2_IRQHandler(void)
{
  if (LL_DMA_IsActiveFlag_TC2(DMA1))
  {
    LL_DMA_ClearFlag_TC2(DMA1);
    CheckTxDataAndSendToTarget();
  }
}

void TickRefresh(void)
{
  if (bWorking)
  {
    if (!atomic_load(&bCDCTXGoing))
      CheckRxDataAndSendToCDC();
    if (!atomic_load(&bTXGoing))
      CheckTxDataAndSendToTarget();
  }
}

void CDCTxCmplt(void)
{
  CheckRxDataAndSendToCDC();
}

void TxCmplt(void)
{
  CheckTxDataAndSendToTarget();
}

void CDCRxCmplt(uint8_t *pBuffer, uint16_t uLen)
{
  uint32_t ulTxInPosition = atomic_load(&TxInPosition);
  if (ulTxInPosition + uLen < sizeof(TX_USART_Buffer))
  {
    memcpy(TX_USART_Buffer + ulTxInPosition, pBuffer, uLen);
    ulTxInPosition += uLen;
  }
  else
  {
    uint16_t uFirstPart = sizeof(TX_USART_Buffer) - ulTxInPosition;
    memcpy(TX_USART_Buffer + ulTxInPosition, pBuffer, uFirstPart);
    memcpy(TX_USART_Buffer, pBuffer + uFirstPart, uLen - uFirstPart);
    ulTxInPosition = uLen - uFirstPart;
  }
  atomic_store(&TxInPosition, ulTxInPosition);
}

void StartBridge(void)
{
  bWorking = true;
  atomic_store(&TxInPosition, 0);
  atomic_store(&TxOutPosition, 0);
  atomic_store(&RxOutPosition, 0);
  atomic_store(&bCDCTXGoing, false);
  atomic_store(&bTXGoing, false);

  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableDMAReq_TX(USART3);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)RX_USART_Buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, sizeof(RX_USART_Buffer));
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
}

void StopBridge(void)
{
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_USART_DisableDMAReq_RX(USART3);
  LL_USART_DisableDMAReq_TX(USART3);
  NVIC_DisableIRQ(DMA1_Channel2_IRQn);
  bWorking = false;
}
// USB to UART End

uint32_t uintotal;
void CDCFirmWriteRxCmplt(uint8_t *pBuffer, uint16_t uLen)
{
  uint32_t uluFirmInPos = atomic_load(&uFirmInPos);
  if (uluFirmInPos + uLen < sizeof(Firmware_Buffer))
  {
    memcpy(Firmware_Buffer + uluFirmInPos, pBuffer, uLen);
    uluFirmInPos += uLen;
  }
  else
  {
    uint16_t uFirstPart = sizeof(Firmware_Buffer) - uluFirmInPos;
    memcpy(Firmware_Buffer + uluFirmInPos, pBuffer, uFirstPart);
    memcpy(Firmware_Buffer, pBuffer + uFirstPart, uLen - uFirstPart);
    uluFirmInPos = uLen - uFirstPart;
  }
  uintotal += uLen;
  atomic_store(&uFirmInPos, uluFirmInPos);
}

bool CDCGotReceiveData(uint8_t *dest, const uint32_t size, const bool bBlocking)
{
  if (size == 0)
    return true;
  // if (size > 150)
  //   printf("error");
  uint32_t ReceivedData;
  do
  {
    uint32_t uInPos = atomic_load(&uFirmInPos);
    if (uInPos >= uFirmOutPos)
      ReceivedData = uInPos - uFirmOutPos;
    else
      ReceivedData = uInPos + (sizeof(Firmware_Buffer) - uFirmOutPos);
    if (ReceivedData >= size)
    {
      if (uInPos > uFirmOutPos)
      {
        memcpy(dest, Firmware_Buffer + uFirmOutPos, size);
        uFirmOutPos += size;
      }
      else
      {
        uint32_t uFirstPart = sizeof(Firmware_Buffer) - uFirmOutPos;
        memcpy(dest, Firmware_Buffer + uFirmOutPos, uFirstPart);
        // printf("Ring buffer %08X", uFirstPart, size - uFirstPart);
        memcpy(dest + uFirstPart, Firmware_Buffer, size - uFirstPart);
        uFirmOutPos = size - uFirstPart;
      }
      return true;
    }
  } while (bBlocking);
  return false;
}

uint32_t uBuffer[64];
void OnFirmRxForOngoing(void)
{
  uint32_t uFirmWritten = 0;
  do
  {
    uint32_t uRemain = uFirmSize - uFirmWritten;
    uint32_t uGotThisTime = uRemain > 64 ? 64 : uRemain;
    CDCGotReceiveData((uint8_t *)uBuffer, uGotThisTime, true);

    // printf("Write Firm %08X %d\n", uFirmWritePos + uFirmWritten, uGotThisTime);
    if (!SW_WriteMem(uFirmWritePos + uFirmWritten, uBuffer, uGotThisTime))
      printf("Failed to write\n");

    // SW_WriteDP(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);
    uFirmWritten += uGotThisTime;
  } while (uFirmWritten < uFirmSize);

  // printf("Write Finish\n");

  uint32_t uMSP, uReset;
  if (!SW_ReadData(uFirmWritePos, &uMSP))
    printf("Failed to get MSP\n");
  if (!SW_ReadData(uFirmWritePos + 4, &uReset))
    printf("Failed to get Reset\n");
  uReset &= 0xFFFFFFFE;
  if (!SW_WriteCoreReg(13, uMSP))
    printf("Failed to Write REG 13\n");
  if (!SW_WriteCoreReg(15, uReset))
    printf("Failed to Write REG 15\n");

  if (!SW_WriteData(0xE000ED08, uFirmWritePos))
    printf("Failed to SET VTOR\n");

  printf("VTOR SP PC %08X %08X %08X\n", uFirmWritePos, uMSP, uReset);

  if (!SW_RestoreCore())
    printf("Failed to Go SWD\n");
  SendIdle();

  atomic_store(&CDCRxCallback, 0);
}

void OnFirmRxForNewRecv(void)
{
  CDCGotReceiveData((uint8_t *)uBuffer, 3 * 4, true);

  if (!SW_InitDebug())
    printf("Failed to Init SWD\n");
    
  HAL_Delay(10);
  if (!SW_HaltCore())
    printf("Failed to halt\n");

  uFirmWritePos = *uBuffer;
  uFirmSize = *(uBuffer + 1);

  uint32_t uRegSet = *(uBuffer + 2);

  // printf("Addr %08X size %08X Regs: %d\n", uFirmWritePos, uFirmSize, uRegSet);
  CDCGotReceiveData((uint8_t *)uBuffer, uRegSet * 8, true);
  for (uint8_t i = 0; i < uRegSet; i++)
  {
    uint32_t uData = *(uBuffer + 2 * i);
    uint32_t uAddr = *(uBuffer + 1 + 2 * i);
    // printf("Set Reg %08X %08X\n", uAddr, uData);
    if (!SW_WriteData(uAddr, uData))
      printf("Failed to SET Reg Data\n");
    uint32_t uGetData;
    if (!SW_ReadData(uAddr, &uGetData) && uGetData != uData)
      printf("SET Wrong Reg Data\n");
    // printf("Write %08X -> %08X\n", uData, uAddr);
    // SW_WriteDP(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);
  }
  atomic_store(&CDCRxCallback, (uint32_t)OnFirmRxForOngoing);
}

void OnBaudrateChange(uint32_t uBandrate)
{
  if (uBandrate > 2500)
    RegisterTxRxCallback(CDCTxCmplt, CDCRxCmplt);
  else
  {
    atomic_store(&CDCRxCallback, (uint32_t)OnFirmRxForNewRecv);
    RegisterTxRxCallback(NULL, CDCFirmWriteRxCmplt);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // printf("Start!\n");
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  atomic_init(&TxInPosition, 0);
  atomic_init(&TxOutPosition, 0);
  atomic_init(&RxOutPosition, 0);
  atomic_init(&bCDCTXGoing, false);
  atomic_init(&bTXGoing, false);
  atomic_init(&CDCRxCallback, 0);
  atomic_init(&uFirmInPos, 0);
  uFirmOutPos = 0;
  uFirmWritePos = 0;
  uFirmSize = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  LL_TIM_EnableCounter(TIM6);
  SetBaudRateChangeCallback(OnBaudrateChange);
  StartBridge();

  // if (!SW_InitDebug())
  //   printf("Failed to Init SWD\n");
  // else
  // {
  //   HAL_Delay(10);
  //   if (!SW_HaltCore())
  //     printf("Failed to halt\n");

  //   uint32_t *pCode = (uint32_t *)0x0801E000;
  //   if (!SW_WriteMem(0x20000000, pCode, 0x1058))
  //     printf("Failed to write\n");

  //   if (!SW_WriteCoreReg(13, *pCode))
  //     printf("Failed to Write REG 13\n");
  //   if (!SW_WriteCoreReg(15, *(pCode + 1)))
  //     printf("Failed to Write REG 15\n");

  //   if (!SW_WriteData(0xE000ED08, 0x20000000))
  //     printf("Failed to SET VTOR\n");

  //   uint32_t uBuffer[16];
  //   if (!SW_ReadData(0xE000ED08, uBuffer) || uBuffer[0] != 0x20000000)
  //     printf("Wrong VTOR\n");

  //   if (!SW_ReadCoreReg(13, uBuffer))
  //     printf("Failed to REG 13\n");
  //   if (!SW_ReadCoreReg(15, uBuffer + 1))
  //     printf("Failed to REG 15\n");

  //   if (!SW_ReadData(DBG_ICSR, uBuffer + 2))
  //     printf("Failed to ICSR\n");

  //   printf("SP PC %08X %08X ICSR %08X\n", uBuffer[0], uBuffer[1], uBuffer[2]);

  //   if (!SW_RestoreCore())
  //     printf("Failed to Go SWD\n");
  //   SendIdle();

  //   while (1)
  //   {
  //     HAL_Delay(1000);
  //     if (!SW_ReadCoreReg(13, uBuffer))
  //       printf("Failed to REG 13\n");
  //     if (!SW_ReadCoreReg(15, uBuffer + 1))
  //       printf("Failed to REG 15\n");

  //     if (!SW_ReadData(DBG_ICSR, uBuffer + 2))
  //       printf("Failed to ICSR\n");

  //     printf("SP PC %08X %08X ICSR %08X\n", uBuffer[0], uBuffer[1], uBuffer[2]);
  //   }
  // }
  // LL_GPIO_SetPinMode(SWDIO_GPIO_Port, SWDIO_Pin, LL_GPIO_MODE_INPUT);
  // LL_GPIO_SetPinMode(SWCLK_GPIO_Port, SWCLK_Pin, LL_GPIO_MODE_INPUT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t uCallback = atomic_load(&CDCRxCallback);
    if (uCallback != 0)
    {
      cdc_writeFirm pCallback = (cdc_writeFirm)uCallback;
      atomic_store(&CDCRxCallback, 0);
      pCallback();
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_HSI48_Enable();

  /* Wait till HSI48 is ready */
  while (LL_RCC_HSI48_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 10, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1)
  {
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(80000000);

  /* Update the time base */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 1 - 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  // LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration
  PC4   ------> USART3_TX
  PC5   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART3 DMA Init */

  /* USART3_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART3_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_2);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

  /**/
  LL_GPIO_ResetOutputPin(SWCLK_GPIO_Port, SWCLK_Pin);

  /**/
  LL_GPIO_SetOutputPin(SWDIO_GPIO_Port, SWDIO_Pin);

  /**/
  GPIO_InitStruct.Pin = SWCLK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SWCLK_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SWDIO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SWDIO_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
