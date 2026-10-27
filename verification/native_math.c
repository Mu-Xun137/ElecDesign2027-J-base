/* The legacy Windows TinyCC CRT lacks these C99 exports.
 * Firmware uses ARM's native single-precision math library; this is host-only. */
#include <math.h>
float sinf(float x) { return (float)sin((double)x); }
float cosf(float x) { return (float)cos((double)x); }
float tanf(float x) { return (float)tan((double)x); }
float sqrtf(float x) { return (float)sqrt((double)x); }
