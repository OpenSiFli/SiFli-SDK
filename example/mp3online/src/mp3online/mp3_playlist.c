
#include <rtthread.h>
#include <string.h>
#include "mp3_playlist.h"

typedef struct
{
    const char *id;
    const char *name;
    const char *artist;
    const char *pic_url;
    const char *stream_url;
} mp3_playlist_entry_t;

static const mp3_playlist_entry_t g_static_playlist[] =
{
    {
        "beautiful-dream",
        "Beautiful Dream",
        "Sifli Music Lab",
        "http://downloads.sifli.com/music/mp3_0.jpg",
        "http://downloads.sifli.com/music/beautiful-dream.mp3"
    },
};

static bool g_playlist_initialized = false;

static int mp3_playlist_entry_count(void)
{
    return sizeof(g_static_playlist) / sizeof(g_static_playlist[0]);
}

static bool mp3_playlist_index_valid(int index)
{
    if (!g_playlist_initialized)
    {
        return false;
    }
    return (index >= 0) && (index < mp3_playlist_entry_count());
}

int mp3_playlist_get(const char *playlist_id)
{
    if (!g_playlist_initialized) 
    {
        g_playlist_initialized = true;
        rt_kprintf("Playlist initialized with %d entries\n", mp3_playlist_entry_count());
        for(int i = 0; i < mp3_playlist_entry_count(); ++i) {
            rt_kprintf("  Entry %d pic_url: %s\n", i, g_static_playlist[i].pic_url ? g_static_playlist[i].pic_url : "(null)");
        }
    }
    return 0;
}

int mp3_playlist_get_count(void)
{
    if (!g_playlist_initialized)
    {
        return 0;
    }
    return mp3_playlist_entry_count();
}

const char *mp3_playlist_get_song_title(int index)
{
    if (!mp3_playlist_index_valid(index))
    {
        return NULL;
    }
    return g_static_playlist[index].name;
}

const char *mp3_playlist_get_song_artist(int index)
{
    if (!mp3_playlist_index_valid(index))
    {
        return NULL;
    }
    return g_static_playlist[index].artist;
}

void mp3_playlist_get_song_id(int index, char *id, size_t id_len)
{
    if ((id == NULL) || (id_len == 0))
    {
        return;
    }
    if (!mp3_playlist_index_valid(index))
    {
        id[0] = '\0';
        return;
    }
    rt_snprintf(id, id_len, "%s", g_static_playlist[index].id);
}

void mp3_playlist_get_pic_url(int index, char *url, size_t url_len)
{
    if ((url == NULL) || (url_len == 0))
    {
        return;
    }
    if (!mp3_playlist_index_valid(index))
    {
        url[0] = '\0';
        return;
    }
    const char *src = g_static_playlist[index].pic_url ? g_static_playlist[index].pic_url : "";
    rt_snprintf(url, url_len, "%s", src);
    rt_kprintf("mp3_playlist_get_pic_url: %s\n", url);
    rt_kprintf("%s %d: pic_url=%s\n", __func__, __LINE__, url);
}

void mp3_playlist_get_song_url(int index, char *url, size_t url_len)
{
    if ((url == NULL) || (url_len == 0))
    {
        return;
    }
    if (!mp3_playlist_index_valid(index))
    {
        url[0] = '\0';
        return;
    }
    const char *src = g_static_playlist[index].stream_url ? g_static_playlist[index].stream_url : "";
    rt_snprintf(url, url_len, "%s", src);
}
