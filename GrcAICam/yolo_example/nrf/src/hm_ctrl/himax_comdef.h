/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HIMAX_COMDEF_H
#define HIMAX_COMDEF_H

#define HIMAX_SPI_FREQUENCY 10000000

#define SPI_HDR_SIZE 8
#define SPI_BUF_PLEN 16
#define SPI_BUF_NUMB 60
#define SPI_PAYLOAD_LEN (SPI_BUF_PLEN*SPI_BUF_NUMB) // 960
#define SPI_TRANS_LEN SPI_HDR_SIZE+SPI_PAYLOAD_LEN

#define MAX_JPG_SIZE 12480
#define MAX_JPG_ITER (MAX_JPG_SIZE/SPI_PAYLOAD_LEN) // 13

#define SPI_JPG_PAR(last, index, size) (((last & 1)<< 15) | ((index & 0x1f) << 10) | (size & 0x3ff))
#define SPI_JPG_LAST(param) ((param >> 15) & 1)
#define SPI_JPG_INDX(param) ((param >> 10) & 0x1f)
#define SPI_JPG_SIZE(param) (param & 0x3ff)

typedef enum {
    HIMAX_SPI_DATA_OBJINFO = 0,
    HIMAX_SPI_DATA_IMAGE
} himax_data_type_t;

#define HIMAX_CTL_CONT_WORK 0x01
#define HIMAX_CTL_PWR_DOWN 0x02
#define HIMAX_CTL_DPWR_DOWN 0x04
#define HIMAX_CTL_READ_IMAGE 0x08
//#define HIMAX_CTL_RDIMG_CMPL 0x10
#define HIMAX_CTL_IMIT_ON 0x20
#define HIMAX_CTL_IMIT_OFF 0x40

#endif // HIMAX_COMDEF_H
