/***
 * Demo: LED Toggle
 * 
 * PA0   ------> LED+
 * GND   ------> LED-
 */
#include "py32f0xx_bsp_printf.h"

#include "dsp/transform_functions.h"

#define FFT_LENGTH 128
#define N_MELS 40

#include "audio.h"

void test_fft()
{

    arm_rfft_instance_q15 rfft;

    static volatile q15_t dma_buffer[FFT_LENGTH] = {1,};

    static q15_t samples[FFT_LENGTH] = {0,};

    memcpy(samples, dma_buffer, sizeof(q15_t)*FFT_LENGTH);

    static q15_t out[FFT_LENGTH*2] = {0, }; 
    static q31_t mels[N_MELS] = {0, }; 

    mel_filters(samples, out, mels);

    //const arm_status init_status = arm_rfft_init_q15(&rfft, FFT_LENGTH, 1, 1);
    //const arm_status init_status = arm_rfft_init_64_q15(&rfft, 1, 1);

    // NOTE: output is scaled differently based on FFT_LENGTH
    //arm_rfft_q15(&rfft, samples, out);

}

static void APP_GPIO_Config(void);

int main(void)
{
  HAL_Init();                                 
  APP_GPIO_Config();
  BSP_USART_Config();
  printf("PY32F0xx LED Toggle Demo\r\nSystem Clock: %ld\r\n", SystemCoreClock);
  
    test_fft();

  while (1)
  {
    HAL_Delay(1000);                            
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    printf("echo\r\n");
  }
}

static void APP_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  // PA0
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void APP_ErrorHandler(void)
{
  while (1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Export assert error source and line number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1);
}
#endif /* USE_FULL_ASSERT */
