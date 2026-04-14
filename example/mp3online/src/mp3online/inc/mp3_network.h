
#ifndef __MP3_NETWORK_H__
#define __MP3_NETWORK_H__

#include <stdint.h>

#ifndef MIN
#define MIN(a,b) ((a)>(b)?(b):(a))
#endif

#define POST_URL_LEN_MAX               256
#define POST_RESP_BUFSZ                2048
#define POST_HEADER_BUFSZ              2048

#define MP3_HOST_NAME           "downloads.sifli.com"

typedef int (*mp3_nw_rsp_data_callback)(uint8_t *data, size_t len);

typedef enum
{
    MP3_NW_CMD_GET,
    MP3_NW_CMD_GET_PART,    //for large file, only get part
    MP3_NW_CMD_GET_PART_CONTINUE,
    MP3_NW_CMD_GET_PART_CANCEL,
} mp3_nw_cmd_t;

typedef struct
{
    mp3_nw_cmd_t cmd;
    const char *url;
    uint8_t *user_data; // post data or get data buffer
    size_t data_len;   // post data length when cmd is POST; get data length when cmd is GET_PART
    mp3_nw_rsp_data_callback callback;
} mp3_nw_msg_t;

int mp3_network_get(const char *url, mp3_nw_rsp_data_callback callback);
int mp3_network_get_part(const char *url, size_t size, mp3_nw_rsp_data_callback callback);
int mp3_network_get_part_continue(size_t size, mp3_nw_rsp_data_callback callback);
int mp3_network_get_part_cancel(void);

#endif