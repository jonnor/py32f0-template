

#include <stdint.h>

// Very basic DC blocking filter. Effectively a high pass with very low cutoff
// PERF: would ideally be implemented with fixed-point
void
audio_dc_filter(int16_t *samples, int length)
{
    static float xm1 = 0.0f;
    static float ym1 = 0.0f;
    const float pole = 0.995;

    for (int i=0; i<length; i++) {
        const float x = samples[i];
        const float y = x - xm1 + pole * ym1;
        xm1 = x;
        ym1 = y;
        samples[i] = y;
    }
}

