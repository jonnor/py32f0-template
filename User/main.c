/***
 * Demo: ADC With DMA Triggered by TIM1
 * 
 * PY32          
 * PA4      ------> Input voltage between 0V ~ 3.3V
 * 
 * PA2(TX)  ------> RX
 * PA3(RX)  ------> TX
 */
#include "main.h"
#include "systick.h"
#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_bsp_printf.h"

#include "base64.h"
#include <string.h>

#if 0
#include "dsp/transform_functions.h"
#define FFT_LENGTH 128
#define N_MELS 40

#include "audio.h"
#endif

static uint32_t ADCxConvertedDatas;

static void APP_ADCConfig(void);
static void APP_TimerInit(void);
static void APP_DMAConfig(void);
static void APP_GPIO_Config(void);
static void APP_SystemClockConfig(void);

const int BLINK_RATE = 1000;

#define SAMPLES_LENGTH 64
#define SAMPLERATE 8000

uint32_t GetTick(void) {
  return systick_GetTick();
}




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




#if 0
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
#endif

int main(void)
{
  APP_SystemClockConfig();

  BSP_USART_Config(921600);
  LL_mDelay(100);

  printf("app-start clock=%ld \r\n", SystemCoreClock);

  APP_DMAConfig();
  APP_ADCConfig();
  APP_GPIO_Config();
  // Start ADC regular conversion and wait for next external trigger
  LL_ADC_REG_StartConversion(ADC1);

  APP_TimerInit();

  // Status LED
  uint32_t previous_blink = 0;


  // Dummy audio data. FIXME: fill with a sinewave or similar
  const int sample_chunk_ms = (SAMPLES_LENGTH * 1000) / SAMPLERATE;
  static int16_t audio_buffer[SAMPLES_LENGTH];
  memset(audio_buffer, 0, 2*SAMPLES_LENGTH);
  int counter = 0;
  uint32_t previous_audio_chunk = 0;

//   test_fft();

  while (1)
  {
    // FIXME: check and process input data buffers

    const uint32_t tick = GetTick();

    // Blink the status LED
    if (tick > (previous_blink + BLINK_RATE)) {

      LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
      printf("blink tick=%lld\r\n", (long long)tick);
      previous_blink = tick;
    }

    // Send audio chunks
    if (tick >= (previous_audio_chunk + sample_chunk_ms)) {

      log_send_audio(audio_buffer, SAMPLES_LENGTH, counter);
      previous_blink = tick;
      counter += 1;
    }

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

static void APP_GPIO_Config(void)
{
  // PB5 as liveness indicator. Blink/toggle
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}


static void APP_TimerInit(void)
{
  // FIXME: set samplerate

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  LL_TIM_SetPrescaler(TIM1, (SystemCoreClock / 6000) - 1);
  LL_TIM_SetAutoReload(TIM1, 6000 - 1);
  /* Triggered by update */
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);

  LL_TIM_EnableCounter(TIM1);
}

static void APP_ADCConfig(void)
{
  __IO uint32_t backup_setting_adc_dma_transfer = 0;

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  LL_ADC_Reset(ADC1);
  // Calibrate start
  if (LL_ADC_IsEnabled(ADC1) == 0)
  {
    /* Backup current settings */
    backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
    /* Turn off DMA when calibrating */
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_StartCalibration(ADC1);

    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);

    /* Delay 1ms(>= 4 ADC clocks) before re-enable ADC */
    LL_mDelay(1);
    /* Apply saved settings */
    LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
  }
  // Calibrate end

  /* PA0 as ADC input */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
  /* Set ADC channel and clock source when ADEN=0, set other configurations when ADSTART=0 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetResolution(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_41CYCLES_5);

  /* Set TIM1 as trigger source */
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);

  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
  /* Enable: each conversions in the sequence need to be triggerred separately */
  LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
  /* Can be multiple channels */
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4);

  LL_ADC_Enable(ADC1);
}

static void APP_DMAConfig(void)
{
  // TODO: use a larger DMA buffer, to reduce intrrrupt rate

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  // Remap ADC to LL_DMA_CHANNEL_1
  LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_ADC);
  // Transfer from peripheral to memory
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  // Set priority
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);
  // Circular mode
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  // Peripheral address no increment
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  // Memory address no increment
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_NOINCREMENT);
  // Peripheral data alignment : Word
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
  // Memory data alignment : Word
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);
  // Data length
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);
  // Sorce and target address
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR, (uint32_t)&ADCxConvertedDatas, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
  // Enable DMA channel 1
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  // Enable transfer-complete interrupt
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void APP_TransferCompleteCallback(void)
{
  // FIXME: Copy data to our processing buffer

  printf("adc-transfer-complete tick=%lld", (long long)GetTick());
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
