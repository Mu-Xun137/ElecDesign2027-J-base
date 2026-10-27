#ifndef J_KEYS_H
#define J_KEYS_H
#include <stdint.h>
typedef struct {
    uint32_t changed_ms[4];
    uint8_t raw, stable, released, count[4], initialized,validated;
} JKeys;
uint8_t JKeys_Step(JKeys *keys, uint8_t raw_pressed, uint32_t now);
#endif
