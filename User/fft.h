
// FFT implementation using CMSIS-DSP arm_rfft q15

#ifndef FFT_LENGTH
#error "FFT_LENGTH not specified"
#endif

#include "dsp/statistics_functions.h"
#include "dsp/transform_functions.h"
#include "arm_const_structs.h"

#include "fft_tables.h"

arm_status
rfft_init_q15_64(arm_rfft_instance_q15 * S,
    uint32_t ifftFlagR,                                           
    uint32_t bitReverseFlag )                                     
{                                                                                    
    /*  Initialize the Flag for selection of RFFT or RIFFT */
    S->ifftFlagR = (uint8_t) ifftFlagR;
    S->bitReverseFlagR = (uint8_t) bitReverseFlag;
    S->twidCoefRModifier = 1; // twiddle table matches length

    S->fftLenReal = (uint16_t)64;
    S->pTwiddleAReal = fft_table_q15_a_64;
    S->pTwiddleBReal = fft_table_q15_b_64;
    S->pCfft = &arm_cfft_sR_q15_len64;                     

    return (ARM_MATH_SUCCESS);
}

arm_status
rfft_init_q15_128(arm_rfft_instance_q15 * S,
    uint32_t ifftFlagR,                                           
    uint32_t bitReverseFlag )                                     
{                                                                                    
    /*  Initialize the Flag for selection of RFFT or RIFFT */
    S->ifftFlagR = (uint8_t) ifftFlagR;
    S->bitReverseFlagR = (uint8_t) bitReverseFlag;
    S->twidCoefRModifier = 1; // twiddle table matches length

    S->fftLenReal = (uint16_t)128;
    S->pTwiddleAReal = fft_table_q15_a_128;
    S->pTwiddleBReal = fft_table_q15_b_128;
    S->pCfft = &arm_cfft_sR_q15_len128;

    return (ARM_MATH_SUCCESS);
}

// Initialize FFT
arm_status
rfft_init(arm_rfft_instance_q15 * rfft)
{
#if FFT_LENGTH==64
    const arm_status init_status = rfft_init_q15_64(rfft, 0, 1);
#elif FFT_LENGTH==128
    const arm_status init_status = rfft_init_q15_128(rfft, 0, 1);
#else
#error "Unsupported FFT length"
#endif
    return init_status;
}

// Reduce the FFT by computing the mean over consequtive bins
void
fft_summarize_mean(q15_t *fft, int length,
                    q15_t *out, int out_length)
{
    const int bin_width = length / out_length;
    for (int i=0; i<out_length; i++) {
        const int start = i*bin_width;
        q15_t mean = 0;
        arm_mean_q15(fft+start, bin_width, &mean);
        out[i] = mean;
    }
}
