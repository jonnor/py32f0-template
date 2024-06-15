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

#include <fvad.h>
#include <fvad.c>

#include "queue.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832
#endif

#if 0
#include "dsp/transform_functions.h"
#define FFT_LENGTH 128
#define N_MELS 40

#include "audio.h"
#endif

#define AUDIO_SAMPLERATE 8000
// for libfvad, the frame size must be 10/20/30ms
#define AUDIO_BUFFER_SIZE 80
#define DMA_BUFFER_SIZE (AUDIO_BUFFER_SIZE)


struct audio_msg {
    int16_t data[AUDIO_BUFFER_SIZE];
};

QUEUE_DECLARATION(audio_msg_queue, struct audio_msg, 2);
QUEUE_DEFINITION(audio_msg_queue, struct audio_msg);

struct audio_msg_queue audio_queue;
__IO uint16_t dma_buffer[AUDIO_BUFFER_SIZE];

Fvad vad_instance;

static void APP_ADCConfig(void);
static void APP_TimerInit(void);
static void APP_DMAConfig(void);
static void APP_GPIO_Config(void);
static void APP_SystemClockConfig(void);

const int BLINK_RATE = 1000;


uint32_t GetTick(void) {
  return systick_GetTick();
}

void dc_filter(int16_t *samples, int length)
{
    static float xm1 = 0.0f;
    static float ym1 = 0.0f;
    const float pole = 0.995;

    for (int i=0; i<AUDIO_BUFFER_SIZE; i++) {
        const float x = samples[i];
        const float y = x - xm1 + pole * ym1;
        xm1 = x;
        ym1 = y;
        samples[i] = y;
    }
}


void
log_send_audio(const int16_t *samples, int length, uint32_t sequence_no)
{
    // OPT: iterate over chunks of the samples, encode them gradually, reduce size of buffer
    static unsigned char buffer[256];
    size_t written = 0;

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

void
sinewave_fill(int16_t *audio, size_t length, int sr, float freq, int amplitude)
{
    for (int i=0; i<length; i++) {
        const float s = sinf(2.0*M_PI*(freq/sr)*i);
        audio[i] = amplitude * s;
    }
}

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

  audio_msg_queue_init(&audio_queue);

    // Setup VAD
    fvad_reset(&vad_instance);
    const int VAD_MODE = 3;
    const int mode_status = fvad_set_mode(&vad_instance, VAD_MODE);
    const int samplerate_status = fvad_set_sample_rate(&vad_instance, AUDIO_SAMPLERATE);
    printf("vad-init mode=%d samplerate=%d \r\n", mode_status, samplerate_status);

  // Status LED
  uint32_t previous_blink = 0;


#if 0
  // Dummy audio data. FIXME: fill with a sinewave or similar, push in via queue
  const int sample_chunk_ms = (SAMPLES_LENGTH * 1000) / SAMPLERATE;
  static int16_t audio_buffer[SAMPLES_LENGTH];
  memset(audio_buffer, 0, 2*SAMPLES_LENGTH);
  sinewave_fill(audio_buffer, SAMPLES_LENGTH, SAMPLERATE, 100.0, 10000);
#endif

  int audio_counter = 0;

//   test_fft();

  struct audio_msg audio_chunk;

  while (1)
  {

    const uint32_t tick = GetTick();

    // Blink the status LED
    if (tick >= (previous_blink + BLINK_RATE)) {

      LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
      printf("blink tick=%ld\r\n", (long)tick);
      previous_blink = tick;
    }

    // Process audio chunks
    const enum dequeue_result res = audio_msg_queue_dequeue(&audio_queue, &audio_chunk);
    if (res == DEQUEUE_RESULT_SUCCESS) {

      // Remove DC offset
      dc_filter(audio_chunk.data, AUDIO_BUFFER_SIZE);

      // Process the audio
      const int result = fvad_process(&vad_instance, audio_chunk.data, AUDIO_BUFFER_SIZE);
      printf("vad-processed res=%d \r\n", result);

      // Send audio over serial
      log_send_audio(audio_chunk.data, AUDIO_BUFFER_SIZE, audio_counter);
      audio_counter += 1;
     
      const uint32_t duration = GetTick() - tick;
      printf("audio-chunk-processed duration=%ld \r\n", (long)duration);
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

  LL_SYSTICK_EnableIT();
}

static void APP_GPIO_Config(void)
{
  // PB5 as liveness indicator. Blink/toggle
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
}


static void APP_TimerInit(void)
{

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
  // Configure timer for 8000 Hz
  LL_TIM_SetPrescaler(TIM1, 24);
  LL_TIM_SetAutoReload(TIM1, 125);
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
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
  /* Set ADC channel and clock source when ADEN=0, set other configurations when ADSTART=0 */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);

  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
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
  LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_1);

  LL_ADC_Enable(ADC1);
}

static void APP_DMAConfig(void)
{

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
  // Memory address increment
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  // Peripheral data alignment : Word
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  // Memory data alignment : Word
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
  // Data length
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, DMA_BUFFER_SIZE);
  // Sorce and target address
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR,
        (uint32_t)dma_buffer,
        LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
  // Enable DMA channel 1
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  // Enable transfer-complete interrupt
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void APP_TransferCompleteCallback(void)
{
  // Push data into processing queue
  struct audio_msg new_msg;
  memcpy(new_msg.data, (void *)dma_buffer, sizeof(int16_t)*DMA_BUFFER_SIZE);

  const enum enqueue_result result = audio_msg_queue_enqueue(&audio_queue, &new_msg);
  if (result != ENQUEUE_RESULT_SUCCESS) {
    printf("audio-queue-overflow tick=%ld \r\n", (long)GetTick());
  }
  printf("adc-transfer-complete tick=%ld \r\n", (long)GetTick());

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
