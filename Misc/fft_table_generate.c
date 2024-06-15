
/*
FFT twiddle table generator for CMSIS-DSP

This is a workaround for CMSIS-DSP not having different tables for different lengths for q15/q31 RFFT.
Meaning that it always always uses a table 4096 long FFT - which takes 32 kB of program memory.
This wastes a lot of program space for short FFTs: 64/128/256/512 etc.

Ref: https://github.com/ARM-software/CMSIS-DSP/issues/25

Usage:

    gcc -Wall -o fft_table_generate Misc/fft_table_generate.c -lm
    ./fft_table_generate > User/fft_tables.h
*/



#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/*
Original CMSIS-DSP documentation on generating the tables

  @par
  Generation fixed-point realCoefAQ15 array in Q15 format:
  @par
  n = 4096
  <pre>for (i = 0; i < n; i++)
  {
     pATable[2 * i]     = 0.5 * ( 1.0 - sin (2 * PI / (double) (2 * n) * (double) i));
     pATable[2 * i + 1] = 0.5 * (-1.0 * cos (2 * PI / (double) (2 * n) * (double) i));
  }</pre>
  @par
  Convert to fixed point Q15 format
        round(pATable[i] * pow(2, 15))


  @par
  Generation of real_CoefB array:
  @par
  n = 4096
  <pre>for (i = 0; i < n; i++)
  {
     pBTable[2 * i]     = 0.5 * (1.0 + sin (2 * PI / (double) (2 * n) * (double) i));
     pBTable[2 * i + 1] = 0.5 * (1.0 * cos (2 * PI / (double) (2 * n) * (double) i));
  }</pre>
  @par
  Convert to fixed point Q15 format
        round(pBTable[i] * pow(2, 15))
*/

int
fill_table_a(double *table, int table_length, int fft_length)
{
    if (table_length != 2*fft_length) {
        return -1;
    }

    const int n = fft_length;
    for (int i = 0; i < n; i++) {
        table[2 * i]     = 0.5 * ( 1.0 - sin (2 * M_PI / (double) (2 * n) * (double) i));
        table[2 * i + 1] = 0.5 * (-1.0 * cos (2 * M_PI / (double) (2 * n) * (double) i));
    }
    return 0;
}

int
fill_table_b(double *table, int table_length, int fft_length)
{
    if (table_length != 2*fft_length) {
        return -1;
    }

    const int n = fft_length;
    for (int i = 0; i < n; i++) {
         table[2 * i]     = 0.5 * (1.0 + sin (2 * M_PI / (double) (2 * n) * (double) i));
         table[2 * i + 1] = 0.5 * (1.0 * cos (2 * M_PI / (double) (2 * n) * (double) i));
    }
    return 0;
}

int
print_table(const double *table, int table_length,
        const int fft_length,
        const char *prefix,
        const char *specifiers,
        const char *ctype,
        const int fixed_bits,
        const int width
        )
{
    if (table_length != 2*fft_length) {
        return -1;
    }
    /*
    Prints table out as a C variable definition:

    const q15_t __ALIGNED(4) realCoefBQ15[8192] = {
        (q15_t)0x4000, ...
    };
    */

    printf("const %s %s %s%d = {\n", ctype, specifiers, prefix, fft_length);

    for (int i=0; i<table_length; i++) {
        const double value = table[i];
        const uint16_t fixed = round(value * pow(2, fixed_bits));
        const bool is_end = (i == table_length-1);
        const bool is_column_end = ((i+1) % width == 0);
        printf("(%s)0x%04x", ctype, fixed);
        if (!is_end) {
            printf(", ");
        }
        if (is_column_end) {
            printf("\n");
        }
    }

    printf("};\n");

    return 0;
}

int main() {

    // TODO: support FFT length as input
    // FIXME: q31 will probably need minor fixes
    const int fft_length = 8192/2;
    const int fixedpoint_bits = 15;
    const char *ctype = "q15_t";
    const char *specifiers = "__ALIGNED(4)";
    const int print_width = 8;

    const int table_length = fft_length * 2;
    double *table = (double *)malloc(sizeof(double)*table_length);

    fprintf(stderr, "init\n");

    // A coefficients
    fill_table_a(table, table_length, fft_length);
    print_table(table, table_length, fft_length,
        "fft_table_q15_a_", specifiers, ctype, fixedpoint_bits, print_width);

    // B coefficients
    fill_table_b(table, table_length, fft_length);
    print_table(table, table_length, fft_length,
        "fft_table_q15_b_", specifiers, ctype, fixedpoint_bits, print_width);

    fprintf(stderr, "done\n");

    // FIXME: check against CMSIS-DSP for FFT length = 8192
}


