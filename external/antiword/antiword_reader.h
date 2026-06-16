/*
 * antiword_reader.h
 * Minimal reader integration wrapper for Antiword.
 * DOC reader path uses antiword_decode_with_callback() to avoid temporary text files.
 */

#ifndef __ANTIWORD_READER_H__
#define __ANTIWORD_READER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#ifndef ANTIWORD_TEXT_CB_T_DEFINED
#define ANTIWORD_TEXT_CB_T_DEFINED
typedef int (*antiword_text_cb_t)(const char *text, size_t len, void *user);
#endif

int antiword_decode_with_callback(const char *doc_path, antiword_text_cb_t cb, void *user);

#ifdef __cplusplus
}
#endif

#endif /* __ANTIWORD_READER_H__ */
