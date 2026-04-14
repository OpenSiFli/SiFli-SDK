
#ifndef __MP3_PLAYLIST_H__
#define __MP3_PLAYLIST_H__

#include <stddef.h>
#include <stdint.h>

#ifndef bool
#define bool int
#endif

#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif

int mp3_playlist_get(const char *playlist_id);
int mp3_playlist_get_count(void);
const char *mp3_playlist_get_song_title(int index);
const char *mp3_playlist_get_song_artist(int index);
void mp3_playlist_get_song_id(int index, char *id, size_t id_len);
void mp3_playlist_get_pic_url(int index, char *url, size_t url_len);
void mp3_playlist_get_song_url(int index, char *url, size_t url_len);

#endif
