/**
 * @file epub.h
 * @brief header file for parsing epub files
 */


#ifndef _EPUB_H
#define _EPUB_H

#include <zip.h>
#include <stdlib.h>
#include <libxml/parser.h>
#include "rtthread.h"
#include "lvsf_font.h"


/**
 * @brief first file to be read in order to get the filepath of the root file
 */
#define CONTAINER "META-INF/container.xml"

/**
 * @brief root file search term from the first container file
 */
#define ROOT_FILE "full-path=\""

typedef struct zip _zip_t;


/* Text alignment enumeration */
typedef enum 
{
    ALIGN_LEFT,
    ALIGN_CENTER,
    ALIGN_RIGHT,
    ALIGN_JUSTIFY
} item_alignment_t;

/* Media type enumeration */
typedef enum 
{
    TYPE_NONE,
    TYPE_TEXT,
    TYPE_IMAGE,
    TYPE_AUDIO,
    TYPE_SPAN,
    TYPE_EM
} item_type_t;

/* CSS style structure */
typedef struct 
{
    lv_font_t       *font;
    char            *font_name;
#if LV_COLOR_DEPTH == 16
    uint16_t padding1;
#endif
    lv_color_t       color;                   // Text color
#if LV_COLOR_DEPTH == 16
    uint16_t padding2;
#endif
    lv_color_t       background_color;        // Background color
    float            font_size;               // Font pointer
    float            line_spacing;            // Line spacing value
    int16_t          margin[4];               // Padding: top, right, bottom, left
    uint32_t         alignment          : 2;  // Text alignment
    uint32_t         letter_spacing     : 8;  // Character spacing
    uint32_t         indent             : 7;  // First line indentation
    uint32_t         e_color            : 1;
    uint32_t         e_font_name        : 1;
    uint32_t         e_font_size        : 1;
    uint32_t         e_font_size_em     : 1;
    uint32_t         e_indent           : 1;
    uint32_t         e_line_spacing     : 1;
    uint32_t         e_line_spacing_em  : 1;
    uint32_t         e_letter_spacing   : 1;
    uint32_t         e_alignment        : 1;
    uint32_t         e_margin           : 1;
    uint32_t         e_margin_em        : 1;
    uint32_t         e_background_color : 1;
    uint32_t         h_level            : 3;
    char            *id;
} epub_css_t;

/* Structure representing a single line of HTML text with metadata */
typedef struct 
{
    rt_list_t    list;
    uint16_t     is_title       : 1;          /* Whether it's a title (1=yes, 0=no) */
    uint16_t     in_line        : 1;
    uint16_t     is_block       : 1;
    uint16_t     type           : 4;
    uint16_t     null_style     : 1;
    uint16_t     h_level        : 3;          /* Stable heading level */
    uint16_t     line_idx;
    char        *tag_name;
    char        *value;                 /* Content value (text/image path/audio path) */
    char        *href;                  /* href */
    char        *id;                    /* Stable element id */
    epub_css_t  *style;                 /* Item-specific style */
} 
html_line_t;

/*
 * @brief structure holding content href
 */
 
typedef struct 
{
    rt_list_t    list;
    char        *href;          /* href */
} content_href_t;


/* Main parser structure maintaining state */
typedef struct 
{
    rt_list_t    line_list;             /* list of text lines */
    /* CSS stylesheet cache */
    char       **css_files;             /* Array of CSS file paths */
    char       **css_contents;          /* Array of CSS file contents */
    int          css_count;             /* Number of CSS files */
    void        *css_cache_head;        /* Opaque pointer to CSS match cache list */
    /* Custom file reader function */
    char      *(*zfile_reader)(struct zip *z, const char *fname, uint32_t *size);
} 
html_parser_t;

/**
 * @brief structure holding important information about the epub file
 */
typedef struct 
{
    struct zip      *zipfile;           /* Epub file to be processed */
    char            *rf_path;           /* root filepath */
    char            *html_path;         /* root filepath */
    html_line_t     *cur_line;          /* current line ptr of parser */
    rt_list_t       *p_line_list;       /* pointer to current_chapter line_list */
    rt_list_t        id_list;
} epub_t;

/**
 *Structure for storing the cover path and title of an EPUB book
 */
typedef struct 
{
    char            *cover_path;        // Cover image path (to be freed by the caller)
    char            *title;             // Book title (to be freed by the caller)
} epub_meta_t;

/**
 * @brief function to initialise the epub module
 * @params [epub_str] pointer to the structure created
 * @params [filepath] path of the epub file to be opened
 */
int epub_init(epub_t *spub_str, const char* filepath);

/**
 * @brief get the fullpath of the root file
 */
char *get_root_file(epub_t *epub_str);

/*
 * @brief function to read a file from the zip file provided and get the
 * contents
 * @params [z] zip file to be read
 * @params [fname] name of the file to be read - file should be present in the
 * zip file
 */
char *read_zfile(_zip_t *z, const char *fname, uint32_t *size);
char *read_zfile_with_size(_zip_t *z, const char *fname, uint32_t size);

/**
 * @brief function to get the document for parsing - XML data
 * @params [epub_str] epub_t structure holding the important details
 * @params [content] content to be passed for getting the XML document
 */
xmlDocPtr prepare_doc(char *content);

/**
 * @brief function to parse required xml file and load the attributes and
 * filepath pair
 */
xmlNodePtr get_root_node(xmlDocPtr doc);

/**
 * @brief function to get the desired node
 */
xmlNodePtr get_node(xmlNodePtr cur, const char *node_name);

/*
 * @brief function to get the specified property value for the supplied node with namespace
 */
unsigned char *get_node_prop_with_namespace(xmlNodePtr cur, const char *attr_name);

/**
 * @brief function to get the xml node properties
 */
unsigned char *get_node_prop(xmlNodePtr cur, const char *attr_name);

/**
 * @brief function to clean up the epub module
 */
void         epub_destroy(epub_t *epub_str);

/* @brief This function splits a string 'str' into tokens based on the delimiter string 'delim'.
 *        It uses a static variable 'last' to keep track of the position in the string across multiple calls.
 *        On the first call, 'str' should be a valid string. Subsequent calls should pass NULL as 'str'.
 */
char        *epub_strtok(char *str, const char *delim);

/**
 * @brief Set Font Base Size 
 */
void         epub_decode_set_font_base_size(uint16_t base_size);

void        *epub_alloc(size_t size);
void        *epub_calloc(rt_size_t count, rt_size_t size);
void        *epub_realloc(void *ptr, size_t nbytes);
void         epub_free(void *ptr);
void         epub_line_free(html_line_t *line);
char        *epub_strdup(const char *s);

lv_res_t     epub_decode_get_img_info(_zip_t *zip, char *src, lv_img_header_t *header, char *zip_path);
char        *epub_decode_set_img_src(_zip_t *zip, const char *src);
char        *epub_decode_set_cover_src(const char *zip_name, const char *src);
char        *epub_decode_get_cover_cache(const char *zip_name, const char *src);
int          epub_decode_cover(const char *epub_file, epub_meta_t *cover);
int          epub_decode_next_xhtml(epub_t *epub, char *href);
epub_t      *epub_decode_init(const char *epub_file);
void         epub_decode_deinit(epub_t *epub);
int          epbub_decoder_img_init(void);
void         epub_decode_clean_hash_cache(void);
void         epub_parser_free(html_parser_t *parser);
int          epub_decoder_init(void);
#endif
