/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <string.h>

#include "crc_proc.h"
#include "objects_validator.h"

bool check_object_format(const char* object)
{
    if (strlen(object) > (OBJECTS_INFO_MAX_LENGTH-1)) {
        return false;
    }
/* add a validation procedure here, it may be helpful in case 
   when the object names have a special format like car plates */
    return true;
}
