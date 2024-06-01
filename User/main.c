/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "main.h"
#include "py32f0xx_bsp_printf.h"
#include "base64.h"
#include <string.h>

static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);




void
log_send_audio(const int16_t *samples, int length, uint32_t sequence_no)
{
    // OPT: iterate over chunks of the samples, encode them gradually, reduce size of buffer
    static unsigned char buffer[256];
    size_t written;

    const int status = \
        base64_encode(buffer, 256, &written, (const uint8_t *)samples, 2*length);
    
    if (status != BASE64_OK) {
        printf("log-send-audio-error status=%d\r\n", status);
        return;
    }

    printf("audio-block seq=%ld ", (long)sequence_no);

    printf("data=");
    for (int i=0; i<written; i++) {
        BSP_UART_TxChar((char )buffer[i]);
    }

    printf(" a=b \r\n");
}

#define SAMPLES_LENGTH 64
#define SAMPLERATE 8000

int main(void)
{
  APP_SystemClockConfig();
  APP_GPIOConfig();

  BSP_USART_Config(921600);
  LL_mDelay(100);
  printf("PY32F0xx GPIO Example\r\nClock: %ld\r\n", SystemCoreClock);
  int counter = 0;

  const int sample_chunk_ms = (SAMPLES_LENGTH * 1000) / SAMPLERATE;
  static int16_t audio_buffer[SAMPLES_LENGTH];
  memset(audio_buffer, 0, 2*SAMPLES_LENGTH);

  while (1)
  {
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_0);

    log_send_audio(audio_buffer, SAMPLES_LENGTH, counter);

    LL_mDelay(sample_chunk_ms);
    counter += 1;
  }
}

static void APP_SystemClockConfig(void)
{
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(24000000);
  LL_SetSystemCoreClock(24000000);
}


static void APP_GPIOConfig(void)
{
  // PA0
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1);
}
#endif /* USE_FULL_ASSERT */
