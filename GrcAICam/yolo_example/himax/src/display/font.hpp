/**
 * ---------------------------------------------------------------+
 * @desc        FONTs
 * ---------------------------------------------------------------+
 *              Copyright (C) 2020 Marian Hrinko.
 *              Written by Marian Hrinko (mato.hrinko@gmail.com)
 *
 * @author      Marian Hrinko
 * @datum       07.10.2020
 * @file        font.h
 * @tested      AVR Atmega16
 *
 * @depend
 * ---------------------------------------------------------------+
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// number of columns for chars
#define CHARS_COLS_LENGTH 5
// @const Characters
extern const unsigned char FONTS[][CHARS_COLS_LENGTH];

#ifdef __cplusplus
}
#endif