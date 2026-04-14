
#ifndef MP3_NE_SEC_H
#define MP3_NE_SEC_H

#include <stdint.h>
#include "cJSON.h"

/* random secret key, 4 bytes aligned */
uint8_t * ne_create_secret_key(int size);
cJSON* weapi(cJSON* json);
cJSON* eapi(const char* url, cJSON* json);

#endif
