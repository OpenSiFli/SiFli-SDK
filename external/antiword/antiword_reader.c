/*
 * antiword_reader.c
 * Minimal DOC-to-text wrapper for reader integration.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#include <wchar.h>
#endif
#if !defined(DEBUG) && !defined(NDEBUG)
#define NDEBUG
#endif
#include <rtthread.h>
#include "antiword.h"
#include "antiword_reader.h"

#if defined(_WIN32) || defined(WIN32)
extern char *dfs_win32_dirdup(char *path);

static BOOL antiword_reader_utf8_to_wide(const char *src, wchar_t *dst, int dst_len)
{
    int ret;

    if (!src || !dst || dst_len <= 0) return FALSE;

    /*
     * The simulator may pass DFS paths in either UTF-8 or the local ANSI code
     * page.  A non-strict CP_UTF8 conversion can accept invalid byte sequences
     * and produce a wrong wide path, so use MB_ERR_INVALID_CHARS first and only
     * fall back to CP_ACP when the path is not valid UTF-8.
     */
    ret = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, src, -1, dst, dst_len);
    if (ret <= 0)
        ret = MultiByteToWideChar(CP_ACP, 0, src, -1, dst, dst_len);
    return ret > 0;
}

static FILE *antiword_reader_wfopen_utf8(const char *path, const char *mode)
{
    wchar_t wpath[PATH_MAX];
    wchar_t wmode[16];

    if (!antiword_reader_utf8_to_wide(path, wpath, (int)(sizeof(wpath) / sizeof(wpath[0]))))
        return NULL;
    if (!antiword_reader_utf8_to_wide(mode, wmode, (int)(sizeof(wmode) / sizeof(wmode[0]))))
        return NULL;
    return _wfopen(wpath, wmode);
}

static void antiword_reader_slash_to_backslash(char *path)
{
    size_t i;

    if (!path) return;
    for (i = 0; path[i]; i++)
    {
        if (path[i] == '/') path[i] = '\\';
    }
}

static BOOL antiword_reader_join_disk_path(const char *base, const char *doc_path, char *buf, size_t buf_len)
{
    size_t base_len;
    size_t path_len;

    if (!base || !base[0] || !doc_path || doc_path[0] != '/' || !buf || buf_len == 0) return FALSE;

    base_len = strlen(base);
    path_len = strlen(doc_path);
    if (base_len + 5 + path_len + 1 > buf_len) return FALSE;

    memcpy(buf, base, base_len);
    if (base_len && buf[base_len - 1] != '\\' && buf[base_len - 1] != '/')
        buf[base_len++] = '\\';
    memcpy(buf + base_len, "disk", 4);
    base_len += 4;
    memcpy(buf + base_len, doc_path, path_len + 1);
    antiword_reader_slash_to_backslash(buf);
    return TRUE;
}

static BOOL antiword_reader_make_win32_disk_path(const char *doc_path, char *buf, size_t buf_len)
{
#if defined(WIN32_DIRDISK_ROOT)
    const char *root = WIN32_DIRDISK_ROOT;
#else
    const char *root = ".\\disk";
#endif
    size_t root_len;
    size_t path_len;

    if (!doc_path || doc_path[0] != '/' || !buf || buf_len == 0) return FALSE;

    root_len = strlen(root);
    path_len = strlen(doc_path);
    if (root_len + path_len + 1 > buf_len) return FALSE;

    memcpy(buf, root, root_len);
    memcpy(buf + root_len, doc_path, path_len + 1);
    antiword_reader_slash_to_backslash(buf);
    return TRUE;
}

static BOOL antiword_reader_make_win32_disk_path_from_dir(char *dir, const char *doc_path, char *buf, size_t buf_len)
{
    int depth;

    if (!dir || !dir[0]) return FALSE;

    for (depth = 0; depth < 10; depth++)
    {
        char *slash;

        if (antiword_reader_join_disk_path(dir, doc_path, buf, buf_len))
        {
            DWORD attr;
            wchar_t wpath[PATH_MAX];

            if (antiword_reader_utf8_to_wide(buf, wpath, (int)(sizeof(wpath) / sizeof(wpath[0]))))
            {
                attr = GetFileAttributesW(wpath);
                if (attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY))
                    return TRUE;
            }
        }

        slash = strrchr(dir, '\\');
        if (!slash) break;
        *slash = 0;
        if (!dir[0]) break;
    }

    return FALSE;
}

static BOOL antiword_reader_path_exists(const char *path)
{
    DWORD attr;
    wchar_t wpath[PATH_MAX];

    if (!path || !path[0]) return FALSE;
    if (!antiword_reader_utf8_to_wide(path, wpath, (int)(sizeof(wpath) / sizeof(wpath[0])))) return FALSE;
    attr = GetFileAttributesW(wpath);
    return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

static void antiword_reader_wide_to_utf8_path(const wchar_t *wpath, char *path, size_t path_len)
{
    int ret;

    if (!path || path_len == 0) return;
    path[0] = 0;
    if (!wpath) return;
    ret = WideCharToMultiByte(CP_UTF8, 0, wpath, -1, path, (int)path_len, NULL, NULL);
    if (ret <= 0)
        WideCharToMultiByte(CP_ACP, 0, wpath, -1, path, (int)path_len, NULL, NULL);
    path[path_len - 1] = 0;
}

static FILE *antiword_reader_try_open_sibling_doc(const char *doc_path, const char *mapped_path,
                                                  char *opened_path, size_t opened_path_len)
{
    char dir_path[PATH_MAX];
    wchar_t wdir[PATH_MAX];
    wchar_t wpattern[PATH_MAX];
    WIN32_FIND_DATAW find_data;
    HANDLE find_handle;
    char *slash1;
    char *slash2;
    size_t dir_len;
    FILE *fp = NULL;

    if (!mapped_path || !mapped_path[0]) return NULL;

    strncpy(dir_path, mapped_path, sizeof(dir_path) - 1);
    dir_path[sizeof(dir_path) - 1] = 0;
    slash1 = strrchr(dir_path, '\\');
    slash2 = strrchr(dir_path, '/');
    if (!slash1 || (slash2 && slash2 > slash1)) slash1 = slash2;
    if (!slash1) return NULL;
    *slash1 = 0;

    if (!antiword_reader_utf8_to_wide(dir_path, wdir, (int)(sizeof(wdir) / sizeof(wdir[0])))) return NULL;

    dir_len = wcslen(wdir);
    if (dir_len + 3 > sizeof(wpattern) / sizeof(wpattern[0])) return NULL;
    wcscpy(wpattern, wdir);
    if (dir_len && wpattern[dir_len - 1] != L'\\' && wpattern[dir_len - 1] != L'/')
        wpattern[dir_len++] = L'\\';
    wpattern[dir_len++] = L'*';
    wpattern[dir_len] = 0;

    find_handle = FindFirstFileW(wpattern, &find_data);
    if (find_handle == INVALID_HANDLE_VALUE)
    {
        rt_kprintf("antiword_reader: enum sibling failed dfs=%s dir=%s winerr=%lu\n",
                   doc_path, dir_path, (unsigned long)GetLastError());
        return NULL;
    }

    do
    {
        const wchar_t *ext;
        wchar_t wfull[PATH_MAX];
        size_t name_len;
        size_t full_len;

        if (find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        ext = wcsrchr(find_data.cFileName, L'.');
        if (!ext || lstrcmpiW(ext, L".doc") != 0) continue;

        name_len = wcslen(find_data.cFileName);
        full_len = wcslen(wdir);
        if (full_len + 1 + name_len + 1 > sizeof(wfull) / sizeof(wfull[0])) continue;
        wcscpy(wfull, wdir);
        if (full_len && wfull[full_len - 1] != L'\\' && wfull[full_len - 1] != L'/')
            wfull[full_len++] = L'\\';
        wcscpy(wfull + full_len, find_data.cFileName);

        errno = 0;
        fp = _wfopen(wfull, L"rb");
        if (fp)
        {
            if (opened_path && opened_path_len)
                antiword_reader_wide_to_utf8_path(wfull, opened_path, opened_path_len);
            rt_kprintf("antiword_reader: wfopen sibling doc ok dfs=%s dir=%s\n", doc_path, dir_path);
            break;
        }
        rt_kprintf("antiword_reader: wfopen sibling doc failed dfs=%s dir=%s errno=%d winerr=%lu\n",
                   doc_path, dir_path, errno, (unsigned long)GetLastError());
    } while (FindNextFileW(find_handle, &find_data));

    FindClose(find_handle);
    return fp;
}

static FILE *antiword_reader_try_open_mapped(const char *doc_path, const char *mapped_path,
                                             char *opened_path, size_t opened_path_len)
{
    FILE *fp;
    DWORD last_error;

    if (!mapped_path || !mapped_path[0]) return NULL;

    SetLastError(0);
    errno = 0;
    fp = antiword_reader_wfopen_utf8(mapped_path, "rb");
    if (fp)
    {
        if (opened_path && opened_path_len)
        {
            strncpy(opened_path, mapped_path, opened_path_len - 1);
            opened_path[opened_path_len - 1] = 0;
        }
        rt_kprintf("antiword_reader: wfopen mapped ok dfs=%s host=%s\n", doc_path, mapped_path);
        return fp;
    }
    last_error = GetLastError();
    rt_kprintf("antiword_reader: wfopen mapped failed dfs=%s host=%s exists=%d errno=%d winerr=%lu\n",
               doc_path, mapped_path, antiword_reader_path_exists(mapped_path), errno, (unsigned long)last_error);

    fp = antiword_reader_try_open_sibling_doc(doc_path, mapped_path, opened_path, opened_path_len);
    if (fp) return fp;

    return NULL;
}

static BOOL antiword_reader_search_disk_tree(const char *base, const char *doc_path, char *buf, size_t buf_len, int depth)
{
    char pattern[PATH_MAX];
    WIN32_FIND_DATAA find_data;
    HANDLE find_handle;
    size_t base_len;
    size_t pattern_len;

    if (!base || !base[0] || !doc_path || doc_path[0] != '/' || !buf || buf_len == 0 || depth <= 0) return FALSE;

    if (antiword_reader_join_disk_path(base, doc_path, buf, buf_len) && antiword_reader_path_exists(buf))
        return TRUE;

    base_len = strlen(base);
    pattern_len = base_len;
    if (pattern_len + 3 > sizeof(pattern)) return FALSE;
    memcpy(pattern, base, base_len);
    if (pattern_len && pattern[pattern_len - 1] != '\\' && pattern[pattern_len - 1] != '/')
        pattern[pattern_len++] = '\\';
    pattern[pattern_len++] = '*';
    pattern[pattern_len] = 0;

    find_handle = FindFirstFileA(pattern, &find_data);
    if (find_handle == INVALID_HANDLE_VALUE) return FALSE;

    do
    {
        char child[PATH_MAX];
        size_t child_len;
        size_t name_len;

        if (!(find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) continue;
        if (strcmp(find_data.cFileName, ".") == 0 || strcmp(find_data.cFileName, "..") == 0) continue;

        child_len = base_len;
        name_len = strlen(find_data.cFileName);
        if (child_len + 1 + name_len + 1 > sizeof(child)) continue;
        memcpy(child, base, child_len);
        if (child_len && child[child_len - 1] != '\\' && child[child_len - 1] != '/')
            child[child_len++] = '\\';
        memcpy(child + child_len, find_data.cFileName, name_len + 1);

        if (antiword_reader_search_disk_tree(child, doc_path, buf, buf_len, depth - 1))
        {
            FindClose(find_handle);
            return TRUE;
        }
    } while (FindNextFileA(find_handle, &find_data));

    FindClose(find_handle);
    return FALSE;
}

static FILE *antiword_reader_open_doc_file(const char *doc_path, char *opened_path, size_t opened_path_len)
{
    FILE *fp;
    char mapped_path[PATH_MAX];
    char dir_path[PATH_MAX];
    char *dfs_mapped_path;
    DWORD len;

    if (opened_path && opened_path_len)
    {
        strncpy(opened_path, doc_path, opened_path_len - 1);
        opened_path[opened_path_len - 1] = 0;
    }

    errno = 0;
    fp = antiword_reader_wfopen_utf8(doc_path, "rb");
    if (fp)
    {
        rt_kprintf("antiword_reader: wfopen ok path=%s\n", doc_path);
        return fp;
    }
    rt_kprintf("antiword_reader: wfopen failed path=%s errno=%d\n", doc_path, errno);

    dfs_mapped_path = dfs_win32_dirdup((char *)doc_path);
    if (dfs_mapped_path)
    {
        fp = antiword_reader_try_open_mapped(doc_path, dfs_mapped_path, opened_path, opened_path_len);
        rt_free(dfs_mapped_path);
        if (fp) return fp;
    }

    if (antiword_reader_make_win32_disk_path(doc_path, mapped_path, sizeof(mapped_path)))
    {
        fp = antiword_reader_try_open_mapped(doc_path, mapped_path, opened_path, opened_path_len);
        if (fp) return fp;
    }

    len = GetCurrentDirectoryA(sizeof(dir_path), dir_path);
    if (len > 0 && len < sizeof(dir_path))
    {
        rt_kprintf("antiword_reader: cwd=%s\n", dir_path);
        if (antiword_reader_make_win32_disk_path_from_dir(dir_path, doc_path, mapped_path, sizeof(mapped_path)))
        {
            fp = antiword_reader_try_open_mapped(doc_path, mapped_path, opened_path, opened_path_len);
            if (fp) return fp;
        }
        if (antiword_reader_search_disk_tree(dir_path, doc_path, mapped_path, sizeof(mapped_path), 6))
        {
            rt_kprintf("antiword_reader: found disk tree from cwd host=%s\n", mapped_path);
            fp = antiword_reader_try_open_mapped(doc_path, mapped_path, opened_path, opened_path_len);
            if (fp) return fp;
        }
    }

    len = GetModuleFileNameA(NULL, dir_path, sizeof(dir_path));
    if (len > 0 && len < sizeof(dir_path))
    {
        char *slash = strrchr(dir_path, '\\');
        if (slash) *slash = 0;
        rt_kprintf("antiword_reader: exe_dir=%s\n", dir_path);
        if (antiword_reader_make_win32_disk_path_from_dir(dir_path, doc_path, mapped_path, sizeof(mapped_path)))
        {
            fp = antiword_reader_try_open_mapped(doc_path, mapped_path, opened_path, opened_path_len);
            if (fp) return fp;
        }
        if (antiword_reader_search_disk_tree(dir_path, doc_path, mapped_path, sizeof(mapped_path), 6))
        {
            rt_kprintf("antiword_reader: found disk tree from exe host=%s\n", mapped_path);
            fp = antiword_reader_try_open_mapped(doc_path, mapped_path, opened_path, opened_path_len);
            if (fp) return fp;
        }
    }

    return NULL;
}
#else
static FILE *antiword_reader_open_doc_file(const char *doc_path, char *opened_path, size_t opened_path_len)
{
    if (opened_path && opened_path_len)
    {
        strncpy(opened_path, doc_path, opened_path_len - 1);
        opened_path[opened_path_len - 1] = 0;
    }
    errno = 0;
    return fopen(doc_path, "rb");
}
#endif

static long antiword_reader_get_file_size(FILE *fp)
{
    long cur;
    long size;

    if (!fp) return -1;
    cur = ftell(fp);
    if (cur < 0) cur = 0;
    if (fseek(fp, 0, SEEK_END) != 0) return -1;
    size = ftell(fp);
    if (fseek(fp, cur, SEEK_SET) != 0) return -1;
    return size;
}

static int antiword_reader_init_options(void)
{
    static int inited = 0;
    int opt_ret;
    char *argv[] = { "antiword", "-m", "UTF-8.txt", "dummy.doc" };

    if (inited)
    {
        rt_kprintf("antiword_reader: options already initialized\n");
        return 0;
    }

    rt_kprintf("antiword_reader: init options argc=4 map=UTF-8.txt\n");
    opt_ret = iReadOptions(4, argv);
    rt_kprintf("antiword_reader: iReadOptions ret=%d\n", opt_ret);
    if (opt_ret <= 0)
        return -1;

    inited = 1;
    return 0;
}

int antiword_decode_with_callback(const char *doc_path, antiword_text_cb_t cb, void *user)
{
    FILE *in_file;
    diagram_type *diag;
    char opened_path[PATH_MAX];
    long file_size;
    int word_version;
    BOOL result;

    rt_kprintf("antiword_reader: decode start path=%s cb=%p user=%p\n",
               doc_path ? doc_path : "NULL", cb, user);

    if (!doc_path || !doc_path[0] || !cb)
    {
        rt_kprintf("antiword_reader: invalid args path=%s cb=%p\n", doc_path ? doc_path : "NULL", cb);
        return -1;
    }

    if (antiword_reader_init_options() != 0)
    {
        rt_kprintf("antiword_reader: options init failed path=%s\n", doc_path);
        return -1;
    }

    in_file = antiword_reader_open_doc_file(doc_path, opened_path, sizeof(opened_path));
    if (!in_file)
    {
        rt_kprintf("antiword_reader: fopen failed path=%s errno=%d\n", doc_path, errno);
        return -1;
    }
    rt_kprintf("antiword_reader: fopen ok path=%s opened=%s\n", doc_path, opened_path);

    errno = 0;
    file_size = antiword_reader_get_file_size(in_file);
    rt_kprintf("antiword_reader: filesize path=%s opened=%s size=%ld errno=%d\n", doc_path, opened_path, file_size, errno);
    if (file_size < 0)
    {
        fclose(in_file);
        return -1;
    }
    rewind(in_file);

    word_version = iGuessVersionNumber(in_file, file_size);
    rt_kprintf("antiword_reader: guess version=%d size=%ld\n", word_version, file_size);
    if (word_version < 0 || word_version == 3)
    {
        rt_kprintf("antiword_reader: unsupported word version=%d path=%s\n", word_version, doc_path);
        fclose(in_file);
        return -1;
    }
    rewind(in_file);

    diag = pCreateDiagram("antiword", opened_path);
    rt_kprintf("antiword_reader: create diagram diag=%p path=%s opened=%s\n", diag, doc_path, opened_path);
    if (!diag)
    {
        fclose(in_file);
        return -1;
    }

    rt_kprintf("antiword_reader: decrypt begin path=%s size=%ld version=%d\n", doc_path, file_size, word_version);
    vSetTextOutputCallback(cb, user);
    result = bWordDecryptor(in_file, file_size, diag);
    vSetTextOutputCallback(NULL, NULL);
    rt_kprintf("antiword_reader: decrypt end result=%d path=%s\n", result, doc_path);
    vDestroyDiagram(diag);
    fclose(in_file);

    rt_kprintf("antiword_reader: decode done ret=%d path=%s\n", result ? 0 : -1, doc_path);
    return result ? 0 : -1;
}

