#include "time_calculation.h"
#include "WE2_core.h"
#include <stdint.h>


uint64_t get_timestamp_us()
{
    uint32_t loop_cnt, systick;
    SystemGetTick(&systick, &loop_cnt);
    uint64_t ticks = loop_cnt;
    ticks <<= 24;
    ticks += 0xFFFFFF - (systick & 0xFFFFFF);
    return ticks / (SystemCoreClock / 1000000);
}

uint32_t get_timestamp_ms()
{
    return (uint32_t)(get_timestamp_us() / 1000);
}