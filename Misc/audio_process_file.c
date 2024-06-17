
/*
Audio processing code that runs on host

Primarily to be able to test and validate the audio pipeline on data.


To build/run

gcc -o process Misc/audio_process_file.c -I./Libraries/miniaudio/ -I./User -I./Libraries/CMSIS-DSP/Include/ -I./Libraries/CMSIS-DSP/Source/ -I./Libraries/CMSIS/Core/Include/ -I./Libraries/CMSIS-DSP/PrivateInclude -lm && ./process youtube-speech.wav

*/

#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"

#define FFT_LENGTH 128
#define SPECTRUM_LENGTH 32
#include "fft.h"
#include "audio.h"

// CMSIS-DSP
#include "TransformFunctions/TransformFunctions.c"
#include "StatisticsFunctions/StatisticsFunctions.c"
#include "CommonTables/CommonTables.c"
#include "BasicMathFunctions/BasicMathFunctions.c"
#include "SupportFunctions/SupportFunctions.c"
#include "FastMathFunctions/FastMathFunctions.c"
#include "ComplexMathFunctions/ComplexMathFunctions.c"
#include "MatrixFunctions/MatrixFunctions.c"

#include <stdio.h>
#include <stdint.h>

int
main(int argc, const char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Incorrect number of arguments\n");
        return -1;
    }

    const char *path = argv[1];


    // Audio preprocessing setup
    arm_rfft_instance_q15 rfft = {0, };
    static q15_t fft_out[FFT_LENGTH*2] = {0, }; 
    static q15_t spectrum[SPECTRUM_LENGTH] = {0, }; 
    rfft_init(&rfft);

    const int samplerate = 8000;
    ma_decoder_config config = ma_decoder_config_init(ma_format_s16, 1, samplerate);

    // TODO: add support for opus using custom decoders, example in miniaudio repo
    ma_decoder decoder;
    const ma_result init = ma_decoder_init_file(path, &config, &decoder);
    if (init != MA_SUCCESS) {
        fprintf(stderr, "Failed to load decoder\n");
        return -1;   // An error occurred.
    }

    const int chunk_size = FFT_LENGTH;
    int16_t *frames[chunk_size];
    ma_uint64 frames_read = chunk_size;
    
    uint64_t sample_no = 0;
    while (frames_read == chunk_size) {
        const ma_result read = \
            ma_decoder_read_pcm_frames(&decoder, frames, chunk_size, &frames_read);
        if (read != MA_SUCCESS) {
            fprintf(stderr, "Read failed\n");
            return -1;   // An error occurred.
        }
        sample_no += frames_read;
        const float t = sample_no / (float)samplerate;
        printf("frame-read sample=%d t=%.3fs \n", (int)sample_no, t);

        // Process the audio
        audio_dc_filter(frames, chunk_size);

        arm_rfft_q15(&rfft, frames, fft_out);
        fft_summarize_mean(fft_out, FFT_LENGTH, spectrum, SPECTRUM_LENGTH);

        // FIXME: write to a .npy file, or similar        
    }


    ma_decoder_uninit(&decoder);
}
