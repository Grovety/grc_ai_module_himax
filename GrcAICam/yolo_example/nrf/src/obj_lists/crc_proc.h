/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CRC32_PLATE_LISTS_H
#define CRC32_PLATE_LISTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t crc32_plists(const uint8_t* data, unsigned len); // CRC-32/ISO-HDLC

#ifdef __cplusplus
}
#endif

#endif // CRC32_PLATE_LISTS_H
