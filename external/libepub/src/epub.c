/**
 * @file epub.c
 * @brief implementation of the epub library
 */

#include "../inc/epub.h"
#include <string.h>

#ifndef LIBEPUB_USE_APP_MEM
#define LIBEPUB_USE_APP_MEM
#endif

#ifdef LIBEPUB_USE_APP_MEM

#include "app_mem.h"

void *epub_alloc(size_t size)
{
    SIMULATOR_MEM_LEAKAGE_MALLOC(LEAK_EPUB, size);
    
    void *ret = NULL;
#if defined (USING_BLOCK_MEM)
    ret = bmem_alloc(size);
#endif
    if (!ret)
        ret = app_anim_alloc(size);
    RET_ADDR_TRACE(ret);
    return ret;
}

void *epub_calloc(rt_size_t count, rt_size_t size)
{
    SIMULATOR_MEM_LEAKAGE_CALLOC(LEAK_EPUB, count, size);
    
    void *ret = epub_alloc(count * size);
    if (ret)
        app_memset(ret, 0x00, count * size);
    RET_ADDR_TRACE(ret);
    return ret;
}

void *epub_realloc(void *ptr, size_t nbytes)
{
    SIMULATOR_MEM_LEAKAGE_REALLOC(LEAK_EPUB, ptr, nbytes);

    void *ret = NULL;
    if (0 < nbytes)
    {
        if (!ptr)
        {
            ret = epub_alloc(nbytes);
            goto end;
        }

        size_t old_size = app_mem_get_size(ptr);
        if (nbytes <= old_size)
            return ptr;

        ret = epub_alloc(nbytes);
        if (ret)
            app_memcpy(ret, ptr, old_size);
    }
    epub_free(ptr);
end:
    RET_ADDR_TRACE(ret);
    return ret;
}

void epub_free(void *ptr)
{
    SIMULATOR_MEM_LEAKAGE_FREE(LEAK_EPUB, ptr);
    
    if (ptr)
    {
#if defined (USING_BLOCK_MEM)
        if (0 == bmem_free(ptr))return;
#endif
        app_anim_free(ptr);
    }
}

char *epub_strdup(const char *s)
{
    char *ptr = NULL;
    if (s) 
    {
        ptr = epub_alloc(strlen(s) + 1);
        if (ptr)
            strcpy(ptr, s);
    }
    RET_ADDR_TRACE(ptr);
    return ptr;
}

void *epub_hash_alloc(size_t size)
{
    SIMULATOR_MEM_LEAKAGE_MALLOC(LEAK_EPUB, size);
    
    void *ret = NULL;
#if defined (USING_BLOCK_MEM)
    ret = bmem_alloc(size);
#endif
    if (!ret)
        ret = app_cache_alloc(size, CACHE_PSRAM);
    RET_ADDR_TRACE(ret);
    return ret;
}

void epub_hash_free(void *ptr)
{
    SIMULATOR_MEM_LEAKAGE_FREE(LEAK_EPUB, ptr);
    
    if (ptr)
    {
#if defined (USING_BLOCK_MEM)
        if (0 == bmem_free(ptr))return;
#endif
        app_anim_free(ptr);
    }
}

#else

void *epub_alloc(size_t size)
{
    return rt_malloc(size);
}

void *epub_calloc(rt_size_t count, rt_size_t size)
{
    return rt_calloc(count, size);
}

void *epub_realloc(void *ptr, size_t nbytes)
{
    return rt_realloc(ptr, nbytes);
}

void epub_free(void *ptr)
{
    rt_free(ptr);
}

char *epub_strdup(const char *s)
{
    char *ptr = NULL;
    if (s) 
    {
        ptr = epub_alloc(strlen(s) + 1);
        if (ptr)
            strcpy(ptr, s);
    }
    RET_ADDR_TRACE(ptr);
    return ptr;
}

void *epub_hash_alloc(size_t size)
{
    return rt_malloc(size);
}

void epub_hash_free(void *ptr)
{
    rt_free(ptr);
}

#endif

char *epub_txt_strdup(const char *s)
{
    char *ptr = NULL;
#ifdef SOLUTION_USING_TYPESETTING_RULES
    const char *fullwidth_space = "\xe3\x80\x80";
#else
    const char *fullwidth_space = "";
#endif
    if (s)
    {
        ptr = epub_alloc(strlen(s) + 7);
        uint32_t i = 0;
        for (; i < strlen(s); )
        {
            if (' ' == s[i] || '\t' == s[i]) 
            {
                i++;
                continue;
            }
            else if (0 == strcmp(&s[i], fullwidth_space))
            {
                i += 3;
                continue;
            }
            break;
        }
        if (ptr)
        {
            sprintf(ptr, "%s%s%s", fullwidth_space, fullwidth_space, &s[i]);
            //strcpy(ptr, s);
        }
    }
    RET_ADDR_TRACE(ptr);
    return ptr;
}

/* internal functions - not to be exposed to the user */
static long int _get_fsize(_zip_t *z, const char *fname)
{
    struct zip_stat zs;
    for (int i = 0; i < zip_get_num_entries(z, ZIP_FL_UNCHANGED); i++) {
        if (zip_stat_index(z, i, 0, &zs) == 0)
            if (strncmp(zs.name, fname, strlen(fname)) == 0)
                return zs.size;
    }
    return 0;
}

static void epub_dump_zip_read_error(_zip_t *z, struct zip_file *zfile,
                                     const char *fname, long fsize,
                                     zip_int64_t read_ret)
{
    int zerr = -1;
    int serr = -1;
    const char *file_err = "(no zfile)";
    const char *zip_err = "(no zip)";
    struct zip_stat zs;
    rt_uint32_t total = 0;
    rt_uint32_t used = 0;
    rt_uint32_t max_used = 0;

    if (zfile)
    {
        zip_error_t *err = zip_file_get_error(zfile);
        zerr = zip_error_code_zip(err);
        serr = zip_error_code_system(err);
        file_err = zip_file_strerror(zfile);
    }
    else if (z)
    {
        zip_error_t *err = zip_get_error(z);
        zerr = zip_error_code_zip(err);
        serr = zip_error_code_system(err);
    }

    if (z) zip_err = zip_strerror(z);

    rt_memory_info(&total, &used, &max_used);
    rt_kprintf("[reader][zip-read-error] fname=%s fsize=%d read_ret=%d zfile=%p zerr=%d serr=%d file_err=%s zip_err=%s heap=%u/%u max=%u entries=%d\n",
               fname ? fname : "(null)",
               (int)fsize,
               (int)read_ret,
               zfile,
               zerr,
               serr,
               file_err ? file_err : "(null)",
               zip_err ? zip_err : "(null)",
               used,
               total,
               max_used,
               z ? (int)zip_get_num_entries(z, ZIP_FL_UNCHANGED) : -1);

    if (z && fname)
    {
        zip_stat_init(&zs);
        if (zip_stat(z, fname, ZIP_FL_UNCHANGED, &zs) == 0)
        {
            rt_kprintf("[reader][zip-read-error] stat name=%s size=%d comp_size=%d comp_method=%d crc=%08x valid=%08x\n",
                       zs.name ? zs.name : "(null)",
                       (int)zs.size,
                       (int)zs.comp_size,
                       (int)zs.comp_method,
                       (unsigned int)zs.crc,
                       (unsigned int)zs.valid);
        }
        else
        {
            rt_kprintf("[reader][zip-read-error] stat failed fname=%s zip_err=%s\n",
                       fname,
                       zip_strerror(z));
        }
    }
}

static char *_parse_data(char *sstr, char *container_dat, char delim,
                char *search_term)
{
   /* if container data is not present, return NULL immediately */
   if (!container_dat)
        return NULL;

   /* increment till the part where the index is pointing to the last of
    * the search pattern - ROOT_FILE */
   char *index = strstr(container_dat, search_term);
   if (!index)
       return NULL;
   for (unsigned long c = 0; c < strlen(search_term); c++, index++);

   /* ascertain the number of bytes to be read */
   unsigned long i = 0;
   for (; index[i] != delim && i < strlen(index); i++);
   sstr = epub_calloc(i + 1, sizeof(char));
   RT_ASSERT(sstr);
   /* now process the substring */
   strncpy(sstr, index, i);

   /* return the value */
   return sstr;
}

char *read_zfile_with_size(_zip_t *z, const char *fname, uint32_t size)
{
    /* this function will read the contents of the fname and pass the data
     * to the calling function */

    /* first get the size of the file */
    long fsize = _get_fsize(z, fname);

    if (fsize > (long) size) fsize = (long) size;

    /* allocate the necessary memory to the buffer */
    char *buf = epub_calloc(fsize + 1, sizeof(char));
    RT_ASSERT(buf);
    /* open the file in an instance of the struct zip_file */
    struct zip_file *zfile = zip_fopen(z, fname, ZIP_FL_UNCHANGED);

    /* now read the file contents */
    if (zip_fread(zfile, buf, fsize) == EOF)
    {
        epub_free(buf);
        return NULL;
    }
    
    /* close zfile */
    if (zfile)
        zip_fclose(zfile);

    /* return the result */
    return buf;
}


char *read_zfile(_zip_t *z, const char *fname, uint32_t *size)
{
    /* this function will read the contents of the fname and pass the data
     * to the calling function */

     if (size) *size = 0;

    /* first get the size of the file */
    long fsize = _get_fsize(z, fname);

    /* allocate the necessary memory to the buffer */
    char *buf = epub_calloc(fsize + 1, sizeof(char));
    if (!buf) 
    {
        rt_kprintf("read_zfile failed: fname %s size %d\n", fname, fsize + 1);
        return NULL;
    }
    /* open the file in an instance of the struct zip_file */
    struct zip_file *zfile = zip_fopen(z, fname, ZIP_FL_UNCHANGED);

    /* now read the file contents */
    zip_int64_t read_ret = zfile ? zip_fread(zfile, buf, fsize) : -1;
    if (!zfile || read_ret == EOF)
    {
        if (size) *size = 0;
        epub_free(buf);
        rt_kprintf("%s: fname %s size %d zfile %p\n", __func__, fname, fsize, zfile);
        epub_dump_zip_read_error(z, zfile, fname, fsize, read_ret);
        RT_ASSERT(0);
        return NULL;
    }
    
    /* close zfile */
    if (zfile)
        zip_fclose(zfile);

    if (size) *size = fsize;
    
    /* return the result */
    return buf;
}

/*
 * @brief get the document from the XML content passed to this function
 */
xmlDocPtr prepare_doc(char *content)
{
    xmlDocPtr doc = xmlParseDoc((const unsigned char *)content);
    return doc ? doc : NULL;
}

/*
 * @brief get the desired node from the xml data passed
 */
xmlNodePtr get_root_node(xmlDocPtr doc)
{
    /* necessary variables */
    xmlNodePtr cur = NULL;

    /* get the root element node - what happens in case */
    cur = xmlDocGetRootElement(doc);

    return cur;
}

/*
 * @brief init function for the epub module
 */
int epub_init(epub_t *epub, const char* filepath)
{
    /* open the zip file in RD_ONLY mode */
    int err = 0;
    epub->zipfile = zip_open(filepath, ZIP_RDONLY, &err);
    epub->rf_path = NULL;
    if (epub->zipfile)
    {
        rt_kprintf("[reader][epub-init] path=%s zip=%p entries=%d\n",
                   filepath ? filepath : "(null)",
                   epub->zipfile,
                   (int)zip_get_num_entries(epub->zipfile, ZIP_FL_UNCHANGED));
    }
    else
    {
        rt_kprintf("[reader][epub-init] path=%s zip=%p err=%d\n",
                   filepath ? filepath : "(null)", epub->zipfile, err);
    }
    return epub->zipfile ? 1 : 0;
}

/**
 * @brief get the fullpath of the root file
 */
char *get_root_file(epub_t *epub)
{
    /* read the contents of the container file */
    char *cbuf = read_zfile(epub->zipfile, CONTAINER, NULL);
    if (!cbuf)
    {
        rt_kprintf("[reader][root-file] read container failed epub=%p zip=%p\n",
                   epub, epub ? epub->zipfile : NULL);
        return NULL;
    }
    rt_kprintf("[reader][root-file] container head=%.*s\n", 96, cbuf);
    /* now get the specified substring */
    epub->rf_path = _parse_data(epub->rf_path, cbuf, '"', ROOT_FILE);
    rt_kprintf("[reader][root-file] rf=%s\n", epub->rf_path ? epub->rf_path : "(null)");
    epub_free(cbuf);
    return epub->rf_path;
}

/*
 * @brief function to get the desired node
 */
xmlNodePtr get_node(xmlNodePtr cur, const char *node_name)
{
    cur = cur->xmlChildrenNode;
    while (cur) {
        if ((!xmlStrcmp(cur->name, (const unsigned char *)node_name)))
                return cur;
        cur = cur->next;
    }
    return NULL;
}

// Helper function: Check if a character is a delimiter
// This function iterates through the delimiter string 'delim'
// and checks if the given character 'c' matches any of the delimiters.
// If a match is found, it returns true; otherwise, it returns false.
static bool is_delim(char c, const char *delim) 
{
    while (*delim) 
    {
        if (c == *delim) 
        {
            return true;
        }
        delim++;
    }
    return false;
}

// Simulate the strtok function
// This function splits a string 'str' into tokens based on the delimiter string 'delim'.
// It uses a static variable 'last' to keep track of the position in the string across multiple calls.
// On the first call, 'str' should be a valid string. Subsequent calls should pass NULL as 'str'.
char *epub_strtok(char *str, const char *delim) 
{
    static char *last;  // Used to record the position of the last processed string
    if (str != NULL) 
    {
        last = str;  // For the first call, initialize 'last' to 'str'
    } else if (last == NULL) 
    {
        return NULL;  // No more content to split
    }

    // Skip leading delimiters
    while (*last && is_delim(*last, delim)) 
    {
        last++;
    }

    if (*last == '\0') 
    {
        last = NULL;  // No more content to split
        return NULL;
    }

    char *token = last;  // Record the start position of the currently split substring

    // Find the next delimiter
    while (*last && !is_delim(*last, delim))
    {
        last++;
    }

    if (*last) 
    {
        *last = '\0';  // Replace the delimiter with '\0'
        last++;  // Move to the next character
    } 
    else 
    {
        last = NULL;  // No more content to split
    }

    return token;
}

/*
 * @brief function to get the specified property value for the supplied node with namespace
 */
unsigned char *get_node_prop_with_namespace(xmlNodePtr cur, const char *attr_name) 
{
    if (!cur || !attr_name) {
        return NULL;
    }

    // Check if the attribute has a namespace
    if (strstr(attr_name, ":")) 
    {
        char *temp = epub_alloc(strlen(attr_name) + 1);
        RT_ASSERT(temp);
        strcpy(temp, attr_name);
        char *prefix = strtok((char *)temp, ":");
        char *local_name = strtok(NULL, ":");

        xmlNsPtr ns = xmlSearchNs(cur->doc, cur, (const xmlChar *)prefix);
        if (ns) 
        {
            xmlAttrPtr attr = xmlHasNsProp(cur, (const xmlChar *)local_name, ns->href);
            if (attr) 
            {
                unsigned char *ret =  (unsigned char *)xmlGetProp(cur, (const xmlChar *)local_name);
                epub_free(temp);
                return ret;
            }
        }
        epub_free(temp);
    } 
    else 
    {
        xmlAttrPtr attr = xmlHasProp(cur, (const xmlChar *)attr_name);
        if (attr) 
        {
            return (unsigned char *)xmlGetProp(cur, (const xmlChar *)attr_name);
        }
    }

    return NULL;
}


/*
 * @brief function to get the specified property value for the supplied node
 */
unsigned char *get_node_prop(xmlNodePtr cur, const char *attr_name)
{
    /* need to get all the properties under the current node */
    unsigned char *v = xmlGetProp(cur, (const unsigned char *)attr_name);
    return v ? v : NULL;
}

/*
 * @brief destroy the resources used for parsing the epub file
 */
void epub_destroy(epub_t *epub)
{
    if (epub->zipfile)
        zip_close(epub->zipfile);
    epub_free(epub->rf_path);
    epub_free(epub);
}

static int _epub_decoder_init(void)
{
    epub_decoder_init();
    return 0;
}
INIT_APP_EXPORT(_epub_decoder_init);
