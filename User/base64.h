
#define MBEDTLS_CONFIG_FILE "mbedtls_config.h"

//#define MBEDTLS_BASE64_C
#include <mbedtls/base64.h>
//#include <base64.c>

#define BASE64_OK 0

int base64_encode( unsigned char *dst, size_t dlen, size_t *olen,
                   const unsigned char *src, size_t slen )
{
    return mbedtls_base64_encode(dst, dlen, olen, src, slen);

}

int
base64_decode( unsigned char *dst, size_t dlen, size_t *olen,
                   const unsigned char *src, size_t slen )
{
    return mbedtls_base64_decode(dst, dlen, olen, src, slen);
}

