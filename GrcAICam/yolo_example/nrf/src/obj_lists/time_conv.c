/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <date_time.h>

#include "time_conv.h"

const char* time_to_str(const time_t* t)
{  // "yyyy-mm-dd hh:mm:ss"
	static char tm_buf[64];

	if (t == NULL) {
		strcpy(tm_buf, "xxxx-xx-xx xx:xx:xx");
	}
	else {
		struct tm* tm = localtime(t);
		sprintf(tm_buf,
				"%04d-%02d-%02d %02d:%02d:%02d",
				tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	}
	return tm_buf;
} 

static bool conv_2b(const char* p, int len, int min, int max, int* out)
{
	char temp[5] = { 0 }; // len only 2 or 4
	temp[0] = *p;
	temp[1] = *(p + 1);

	if (len == 4) {
		temp[2] = *(p + 2);
		temp[3] = *(p + 3);
	}

	int res = atoi(temp);
	if ((res < min) || (res > max)) {
		return false;
	}

	*out = res;
	return true;
}

bool str_to_time(const char* buf, time_t* t)
{ // "yyyy-mm-dd hh:mm:ss"
  //  0    5  8  b  e  11
	if (strlen(buf) != 19) {
		return 0;
	}

	struct tm tm = { 0 };
	int ret = conv_2b(buf + 17, 2, 0, 61, &tm.tm_sec);		// 0 - 61
	ret = ret && *(buf + 16) == ':';
	ret = ret && conv_2b(buf + 14, 2, 0, 59, &tm.tm_min);	// 0 - 59
	ret = ret && *(buf + 13) == ':';
	ret = ret && conv_2b(buf + 11, 2, 0, 23, &tm.tm_hour);	// 0 - 23
	ret = ret && *(buf + 10) == ' ';
	ret = ret && conv_2b(buf + 8, 2, 1, 31, &tm.tm_mday);	// 1 - 31
	ret = ret && *(buf + 7) == '-';
	ret = ret && conv_2b(buf + 5, 2, 1, 12, &tm.tm_mon);	// 1 - 12
	ret = ret && *(buf + 4) == '-';
	ret = ret && conv_2b(buf, 4, 1970, 2100, &tm.tm_year);	// 1970 - 2100

	if (ret) {
		tm.tm_mon -= 1;
		tm.tm_year -= 1900;
		*t = mktime(&tm);
	}

	return ret;
}

int set_time(const time_t* new_time)
{
	struct tm* tm = localtime(new_time);
	return date_time_set(tm);
}

void init_time()
{ // 2025-01-01 00:00:00
	struct tm t;
  	t.tm_year = 2025-1900;
  	t.tm_mon = 0;
  	t.tm_mday = 1;
  	t.tm_hour = 0;
  	t.tm_min = 0;
  	t.tm_sec = 1;
  	t.tm_isdst = 0;
	time_t set_tm = mktime(&t);
  	set_time(&set_tm);
}
