#pragma once

#define APP_BLOCK_FUNC()          \
    do {                          \
        __asm volatile("b    ."); \
    } while (0)

typedef enum {
    APP_STATE_ALLON,
} APP_STATE_E;
