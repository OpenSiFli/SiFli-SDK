#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "os_adaptor.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
#include "audio_mp3ctrl.h"

#undef audio_mem_malloc
#undef audio_mem_free
#undef audio_mem_calloc
#define audio_mem_malloc    rt_malloc
#define audio_mem_free      rt_free
#define audio_mem_calloc    rt_calloc

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

#if !PKG_USING_LIBHELIX
static int mp3_getinfo(const char *filename, mp3_id3_info_t *info)
{
    RT_ASSERT(0);
    return -1;
}
#else

#include "mp3dec.h"

#if PKG_USING_ZLIB
    #define  is_has_zlib    0 //1
#else
    #define  is_has_zlib    0
#endif

#define ID3v2_HEADER_SIZE 10

/**
 * Default magic bytes for ID3v2 header: "ID3"
 */
#define ID3v2_DEFAULT_MAGIC "ID3"

#define ID3v2_FLAG_DATALEN     0x0001
#define ID3v2_FLAG_UNSYNCH     0x0002
#define ID3v2_FLAG_ENCRYPTION  0x0004
#define ID3v2_FLAG_COMPRESSION 0x0008

enum ID3v2Encoding
{
    ID3v2_ENCODING_ISO8859  = 0,
    ID3v2_ENCODING_UTF16BOM = 1,
    ID3v2_ENCODING_UTF16BE  = 2,
    ID3v2_ENCODING_UTF8     = 3,
};

typedef struct
{
    uint8_t *buffer;
    int     offset;
    int     buffer_len;
} id3_io_t;


typedef struct
{
    uint8_t header[3];     // 必须是“ID3”，49 44 33，否则认为标签不存在
    uint8_t ver;           // 版本号,ID3V2.3 记录03, ID3V2.4 记录04
    uint8_t revision;      // 副版本号，此版本记录00
    uint8_t flag;          // 存放标志的字节，这个版本只定义三位
    uint8_t size[4];       // 标签大小，包括标签帧和扩展标签头。（不包括标签头的10个字节）
} head_tag;

static inline uint8_t avio_r8(id3_io_t *fd)
{
    uint8_t data = fd->buffer[fd->offset++];
    return data;
}
static inline unsigned int avio_rb16(id3_io_t *fd)
{
    unsigned int val;
    val = avio_r8(fd) << 8;
    val |= avio_r8(fd);
    return val;
}

static inline unsigned int avio_rb24(id3_io_t *fd)
{
    unsigned int val;
    val = avio_rb16(fd) << 8;
    val |= avio_r8(fd);
    return val;
}
static inline unsigned int avio_rb32(id3_io_t *fd)
{
    unsigned int val;
    val = avio_rb16(fd) << 16;
    val |= avio_rb16(fd);
    return val;
}
static unsigned int avio_rl16(id3_io_t *s)
{
    unsigned int val;
    val = avio_r8(s);
    val |= avio_r8(s) << 8;
    return val;
}

static int ff_id3v2_tag_len(const uint8_t *buf)
{
    int len = ((buf[6] & 0x7f) << 21) +
              ((buf[7] & 0x7f) << 14) +
              ((buf[8] & 0x7f) << 7) +
              (buf[9] & 0x7f) +
              ID3v2_HEADER_SIZE;
    if (buf[5] & 0x10)
        len += ID3v2_HEADER_SIZE;
    return len;
}
static uint8_t file_read_r8(int fd)
{
    uint8_t data = 0;
    read(fd, &data, 1);
    return data;
}

static inline unsigned int file_read_rb16(int fd)
{
    unsigned int val;
    val = file_read_r8(fd) << 8;
    val |= file_read_r8(fd);
    return val;
}

static inline unsigned int file_read_rb24(int fd)
{
    unsigned int val;
    val = file_read_rb16(fd) << 8;
    val |= file_read_r8(fd);
    return val;
}
static inline unsigned int file_read_rb32(int fd)
{
    unsigned int val;
    val = file_read_rb16(fd) << 16;
    val |= file_read_rb16(fd);
    return val;
}
static unsigned int get_size(int fd, int len)
{
    int v = 0;
    while (len--)
        v = (v << 7) + (file_read_r8(fd) & 0x7F);
    return v;
}

static unsigned int size_to_syncsafe(unsigned int size)
{
    return (((size) & (0x7f << 0)) >> 0) +
           (((size) & (0x7f << 8)) >> 1) +
           (((size) & (0x7f << 16)) >> 2) +
           (((size) & (0x7f << 24)) >> 3);
}

/* No real verification, only check that the tag consists of
 * a combination of capital alpha-numerical characters */
static int is_tag(const char *buf, unsigned int len)
{
    if (!len)
        return 0;

    while (len--)
        if ((buf[len] < 'A' ||
                buf[len] > 'Z') &&
                (buf[len] < '0' ||
                 buf[len] > '9'))
            return 0;

    return 1;
}

static inline int f_tell(int fd)
{
#if 0
    int offset = 0;
    struct dfs_fd *d = fd_get(fd);
    if (d)
        offset = d->size
                 return offset;
#else
    return lseek(fd, 0, SEEK_CUR);
#endif
}

/**
* Return 1 if the tag of length len at the given offset is valid, 0 if not, -1 on error
*/
static int check_tag(int fd, int offset, unsigned int len)
{
    char tag[4];

    if (len > 4 ||
            lseek(fd, offset, SEEK_SET) < 0 ||
            read(fd, tag, len) < (int)len)
        return -1;
    else if (/*!avio_rb32(tag) || */is_tag(tag, len))
        return 1;

    return 0;
}

static const uint8_t ff_log2_tab[256] =
{
    0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7
};

static int av_log2(unsigned int v)
{
    int n = 0;
    if (v & 0xffff0000)
    {
        v >>= 16;
        n += 16;
    }
    if (v & 0xff00)
    {
        v >>= 8;
        n += 8;
    }
    n += ff_log2_tab[v];

    return n;
}

#define PUT_UTF8(val, tmp)\
    {\
        int bytes, shift;\
        uint32_t in = val;\
        if (in < 0x80) {\
            tmp = in;\
            *dst++ = tmp; \
        } else {\
            bytes = (av_log2(in) + 4) / 5;\
            shift = (bytes - 1) * 6;\
            tmp = (256 - (256 >> bytes)) | (in >> shift);\
            *dst++ = tmp; \
            while (shift >= 6) {\
                shift -= 6;\
                tmp = 0x80 | ((in >> shift) & 0x3f);\
                *dst++ = tmp; \
            }\
        }\
    }

static uint32_t GET_16BIT(int left, int is_big, id3_io_t *fd)
{
    if (left < 0)
    {
        return 0;
    }
    if (is_big)
    {
        return avio_rb16(fd);
    }
    return avio_rl16(fd);
}
static int decode_str(id3_io_t *fd, int encoding, uint8_t *dst, int *maxread)
{
    uint8_t tmp;
    uint32_t ch = 1;
    int left = *maxread;
    uint8_t is_big = 1; //avio_rl16
    switch (encoding)
    {
    case ID3v2_ENCODING_ISO8859:
        while (left && ch)
        {
            ch = avio_r8(fd);
            PUT_UTF8(ch, tmp)
            left--;
        }
        break;

    case ID3v2_ENCODING_UTF16BOM:
        if ((left -= 2) < 0)
        {
            LOG_I("Cannot read BOM value, input too short");
            return -1;
        }
        switch (avio_rb16(fd))
        {
        case 0xfffe:
            is_big = 0;
        case 0xfeff:
            break;
        default:
            LOG_I("Incorrect BOM value");
            *maxread = left;
            return -1;
        }
    // fall-through

    case ID3v2_ENCODING_UTF16BE:
        while ((left > 1) && ch)
        {
            left -= 2;
            ch = GET_16BIT(left, is_big, fd);
            {
                unsigned int hi = ch - 0xD800;
                if (hi < 0x800)
                {
                    left -= 2;
                    ch = GET_16BIT(left, is_big, fd) - 0xDC00;
                    if (ch > 0x3FFU || hi > 0x3FFU)
                        break;
                    ch += (hi << 10) + 0x10000;
                }
            }
            PUT_UTF8(ch, tmp)
        }
        if (left < 0)
            left += 2;  /* did not read last char from pb */
        break;

    case ID3v2_ENCODING_UTF8:
        while (left && ch)
        {
            ch = avio_r8(fd);
            *dst++ = ch;
            left--;
        }
        break;
    default:
        LOG_I("Unknown encoding");
    }

    if (ch)
        *dst = 0;


    *maxread = left;

    return 0;
}

/**
 * Parse a text tag.
 */
static void read_ttag(id3_io_t *fd, uint8_t *dst, int taglen, const char *key)
{
    int encoding;

    if (taglen < 1)
        return;

    encoding = avio_r8(fd);
    taglen--; /* account for encoding type byte */

    if (decode_str(fd, encoding, dst, &taglen) < 0)
    {
        LOG_I("Error reading frame %s, skipped\n", key);
        return;
    }
}



static void id3v2_parse(int fd, int len, uint8_t version, uint8_t flags, mp3_id3_info_t *info)
{
    int isv34, unsync;
    uint32_t tlen;
    char tag[5];
    int64_t next, end = lseek(fd, 0, SEEK_CUR) + len;
    int taghdrlen;
    const char *reason = NULL;
    uint8_t *decode_buf = NULL;
    uint8_t *buffer = NULL;


    unsigned char *uncompressed_buffer = NULL;
    int uncompressed_buffer_size = 0;

    LOG_I("id3v2 ver:%d flags:%02X len:%d\n", version, flags, len);

    switch (version)
    {
    case 2:
        if (flags & 0x40)
        {
            reason = "compression";
            goto error;
        }
        isv34 = 0;
        taghdrlen = 6;
        break;

    case 3:
    case 4:
        isv34 = 1;
        taghdrlen = 10;
        break;

    default:
        reason = "version";
        goto error;
    }

    unsync = flags & 0x80;

    if (isv34 && flags & 0x40)   /* Extended header present, just skip over it */
    {
        int extlen = get_size(fd, 4);
        if (version == 4)
            /* In v2.4 the length includes the length field we just read. */
            extlen -= 4;

        if (extlen < 0)
        {
            reason = "invalid extended header length";
            goto error;
        }
        lseek(fd, extlen, SEEK_CUR);
        len -= extlen + 4;
        if (len < 0)
        {
            reason = "extended header too long.";
            goto error;
        }
    }

    while (len >= taghdrlen)
    {
        unsigned int tflags = 0;
        int tunsync = 0;
        int tcomp = 0;
        int tencr = 0;
        unsigned long  dlen;
        if (info->album && info->artist && info->title)
        {
            break; //no need look up others now
        }
        if (isv34)
        {
            if (read(fd, tag, 4) < 4)
                break;
            tag[4] = 0;
            if (version == 3)
            {
                tlen = file_read_rb32(fd);
            }
            else
            {
                LOG_I("id3 version not support now");
                break;
#if 0
                /* some encoders incorrectly uses v3 sizes instead of syncsafe ones
                 * so check the next tag to see which one to use */
                tlen = avio_rb32(fd);
                if (tlen > 0x7f)
                {
                    if (tlen < len)
                    {
                        int64_t cur = f_tell(fd);

                        if (ffio_ensure_seekback(fd, 2 /* tflags */ + tlen + 4 /* next tag */))
                            break;

                        if (check_tag(fd, cur + 2 + size_to_syncsafe(tlen), 4) == 1)
                            tlen = size_to_syncsafe(tlen);
                        else if (check_tag(fd, cur + 2 + tlen, 4) != 1)
                            break;
                        lseek(fd, cur, SEEK_SET);
                    }
                    else
                        tlen = size_to_syncsafe(tlen);
                }
#endif
            }
            tflags = file_read_rb16(fd);
            tunsync = tflags & ID3v2_FLAG_UNSYNCH;
        }
        else
        {
            if (read(fd, tag, 3) < 3)
                break;
            tag[3] = 0;
            tlen = file_read_rb24(fd);
        }
        if (tlen > (1 << 28))
            break;
        len -= taghdrlen + tlen;

        if (len < 0)
            break;

        next = f_tell(fd) + tlen;
        if (!tlen)
        {
            if (tag[0])
                LOG_I("Invalid empty frame %s, skipping.\n", tag);
            continue;
        }

        if (tflags & ID3v2_FLAG_DATALEN)
        {
            if (tlen < 4)
                break;
            dlen = file_read_rb32(fd);
            tlen -= 4;
        }
        else
            dlen = tlen;

        tcomp = tflags & ID3v2_FLAG_COMPRESSION;
        tencr = tflags & ID3v2_FLAG_ENCRYPTION;

        /* skip encrypted tags and, if no zlib, compressed tags */
        if (tencr || (!is_has_zlib && tcomp))
        {
            const char *type;
            if (!tcomp)
                type = "encrypted";
            else if (!tencr)
                type = "compressed";
            else
                type = "encrypted and compressed";

            LOG_I("Skipping %s ID3v2 frame %s", type, tag);
            lseek(fd, tlen, SEEK_CUR);
            /* check for text tag or supported special meta tag */
        }
        else if (!memcmp("TIT2", tag, 4) || !memcmp("TPE1", tag, 4) || !memcmp("TALB", tag, 4) || !memcmp("TYER", tag, 4))
        {
            id3_io_t id3_io = {0};
            if (!memcmp("TIT2", tag, 4))
            {
                if (info->title)
                {
                    audio_mem_free(info->title);
                }
                info->title = (char *)audio_mem_calloc(tlen * 3 + 1, 1); // for unicode to utf8
                if (!info->title)
                {
                    goto seek;
                }
                decode_buf = (uint8_t *)info->title;
            }
            else if (!memcmp("TPE1", tag, 4))
            {
                if (info->artist)
                {
                    audio_mem_free(info->artist);
                }
                info->artist = (char *)audio_mem_calloc(tlen * 3 + 1, 1); // for unicode to utf8
                if (!info->artist)
                {
                    goto seek;
                }
                decode_buf = (uint8_t *)info->artist;
            }
            else if (!memcmp("TALB", tag, 4))
            {
                if (info->album)
                {
                    audio_mem_free(info->album);
                }
                info->album = (char *)audio_mem_calloc(tlen * 3 + 1, 1); // for unicode to utf8
                if (!info->album)
                {
                    goto seek;
                }
                decode_buf = (uint8_t *)info->album;
            }
            else if (!memcmp("TYER", tag, 4))
            {
                decode_buf = (uint8_t *)&info->year[0];
            }
            else
            {
                goto seek;
            }
            if (buffer)
                audio_mem_free(buffer);

            buffer = (uint8_t *)audio_mem_calloc(tlen, 1);
            if (!buffer)
            {
                LOG_I("Failed to alloc %d bytes\n", tlen);
                goto seek;
            }
            if (unsync || tunsync)
            {
                int64_t end = lseek(fd, 0, SEEK_CUR) + tlen;
                uint8_t *b;

                b = buffer;
                while (lseek(fd, 0, SEEK_CUR) < end && b - buffer < tlen)
                {
                    *b++ = file_read_r8(fd);
                    if (*(b - 1) == 0xff && f_tell(fd) < end - 1 && b - buffer < tlen)
                    {
                        uint8_t val = file_read_r8(fd);
                        *b++ = val ? val : file_read_r8(fd);
                    }
                }
                tlen = b - buffer;
            }
            else if (!tcomp)
            {
                if (read(fd, buffer, tlen) < 0)
                {
                    goto seek;
                }
            }

#if is_has_zlib
            if (tcomp)
            {
                int err;

                LOG_I("Compresssed frame %s tlen=%d dlen=%ld", tag, tlen, dlen);
                if (uncompressed_buffer)
                    audio_mem_free(uncompressed_buffer);

                uncompressed_buffer = audio_mem_calloc(dlen, 1);
                if (!uncompressed_buffer)
                {
                    LOG_I("Failed to alloc %ld bytes", dlen);
                    goto seek;
                }

                if (!(unsync || tunsync))
                {
                    err = read(fd, buffer, tlen);
                    if (err < 0)
                    {
                        LOG_I("Failed to read compressed tag");
                        audio_mem_free(uncompressed_buffer);
                        uncompressed_buffer = NULL;
                        goto seek;
                    }
                    tlen = err;
                }

                err = uncompress(uncompressed_buffer, &dlen, buffer, tlen);
                if (err != Z_OK)
                {
                    LOG_I("Failed to uncompress tag: %d", err);
                    audio_mem_free(uncompressed_buffer);
                    uncompressed_buffer = NULL;
                    goto seek;
                }

                tlen = dlen;

                id3_io.buffer = uncompressed_buffer;
                id3_io.offset = 0;
                id3_io.buffer_len = tlen;
            }
            else
#endif
            {
                id3_io.buffer = buffer;
                id3_io.offset = 0;
                id3_io.buffer_len = tlen;
            }
            if (tag[0] == 'T')  /* parse text tag */
            {
                if (!memcmp("TYER", tag, 4))
                {
                    read_ttag(&id3_io, decode_buf, 5, tag);
                }
                else
                {
                    read_ttag(&id3_io, decode_buf, tlen, tag);
                }
            }
        }
        else if (!tag[0])
        {
            if (tag[1])
                LOG_I("invalid frame id, assuming padding\n");
            lseek(fd, tlen, SEEK_CUR);
            break;
        }
        /* Skip to end of tag */
seek:
        lseek(fd, next, SEEK_SET);
    }

    /* Footer preset, always 10 bytes, skip over it */
    if (version == 4 && flags & 0x10)
        end += 10;

error:
    if (reason)
        LOG_I("ID3v2.%d tag skipped, cannot handle %s\n", version, reason);
    lseek(fd, end, SEEK_SET);
    if (buffer)
        audio_mem_free(buffer);
    if (uncompressed_buffer)
    {
        audio_mem_free(uncompressed_buffer);
    }

    return;
}

static int is_id3v2_match(const uint8_t *buf, const char *magic)
{
    return  buf[0] == magic[0] &&
            buf[1] == magic[1] &&
            buf[2] == magic[2] &&
            buf[3] != 0xff &&
            buf[4] != 0xff &&
            (buf[6] & 0x80) == 0 &&
            (buf[7] & 0x80) == 0 &&
            (buf[8] & 0x80) == 0 &&
            (buf[9] & 0x80) == 0;
}
void mp3_get_id3_end(mp3_id3_info_t *info)
{
    if (info)
    {
        if (info->title)
            audio_mem_free(info->title);
        if (info->artist)
            audio_mem_free(info->artist);
        if (info->album)
            audio_mem_free(info->album);
    }
}
/*only support id3V2.3*/
int mp3_get_id3_start(const char *filename, mp3_id3_info_t *info)
{
    int fd;
    int ret = 0;

    uint32_t     file_size;
    uint32_t     tag_len;
    head_tag     id3;
    RT_ASSERT(info);
    memset(info, 0, sizeof(mp3_id3_info_t));

    if (!filename || !info)
    {
        LOG_E("mp3 parameter error");
        return -1;
    }

    LOG_I("mp3_getinfo %s", filename);
    {
        LOG_I("mp3 open %s", filename);
        struct stat stat_buf;
        stat(filename, &stat_buf);
        file_size = stat_buf.st_size;
        fd = open(filename, O_RDONLY | O_BINARY);
        if (fd < 0)
        {
            LOG_E("mp3 open %s error", filename);
            return -1;
        }
    }

    read(fd, (char *)&id3, sizeof(id3));
    if (!is_id3v2_match((const uint8_t *)&id3, "ID3"))
    {
        LOG_I("no ID3");
        ret = -1;
        //goto file_end_id3v1;
        goto Exit;
    }

    tag_len = ff_id3v2_tag_len((const uint8_t *)&id3);

    LOG_I("ID3 len=%d fsize=%d %d", tag_len, file_size, sizeof(id3));

    if (tag_len >= file_size)
    {
        ret = -2;
        goto Exit;
    }

    id3v2_parse(fd, tag_len, id3.ver, id3.flag, info);

Exit:
    close(fd);
    return ret;
}
#endif