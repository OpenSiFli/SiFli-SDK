
#ifndef __INTENT_INT_H__
#define __INTENT_INT_H__

#if defined(_MSC_VER)
    typedef signed __int8 int8_t;
    typedef unsigned __int8 uint8_t;
    typedef signed __int16 int16_t;
    typedef unsigned __int16 uint16_t;
    typedef signed __int32 int32_t;
    typedef unsigned __int32 uint32_t;
    typedef signed __int64 int64_t;
    typedef unsigned __int64 uint64_t;
#endif

#define INTENT_MAX_LEN 128
#define INTENT_SEPARATER '\0'

typedef struct
{
    char content[INTENT_MAX_LEN];
    uint32_t content_len;
} _intent, *p_intent;

#endif  /* __INTENT_INT_H__ */

