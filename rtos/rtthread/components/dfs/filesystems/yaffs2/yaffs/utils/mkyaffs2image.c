/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2018 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 * Nick Bane modifications flagged NCBw
 * Endian handling patches by James Ng.
 * mkyaffs2image hacks by NCB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * makeyaffs2image.c
 *
 * Makes a YAFFS2 file system image that can be used to load up a file system.
 * Uses default Linux MTD layout - search for "NAND LAYOUT" to change.
 */

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <windows.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <dirent.h>
#include <string.h>
//#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include "yaffs_ecc.h"
#include "yaffs_guts.h"
#include "yaffs_endian.h"

#include "yaffs_packedtags2.h"

unsigned yaffs_trace_mask = 0;

#define MAX_OBJECTS 10000

#define USE_INBAND_TAGS

#define ARGV_DIR_POS       (0)
#define ARGV_IMGFILE_POS   (1)
#define ARGV_IMG_SIZE_POS  (2)
#define ARGV_CONVERT_POS   (3)


// Adjust these to match your NAND LAYOUT:
#define chunkSize 2048
#define spareSize 64
#define pagesPerBlock 64

typedef struct
{
    dev_t dev;
    ino_t ino;
    int   obj;
} objItem;

struct yaffs_dev dummy_dev = {.swap_endian = 0,
           .param.n_reserved_blocks = 5,
#ifdef USE_INBAND_TAGS
           .param.no_tags_ecc = 1,
           .param.inband_tags = 1
#endif /* USE_INBAND_TAGS */
};


static objItem obj_list[MAX_OBJECTS];
static int obj_id = YAFFS_NOBJECT_BUCKETS + 1;

static int n_obj, nDirectories, nPages;
static int total_block_num;
static int reserved_size;
static int remaining_size;

//static int outFile;
static HANDLE outFile;

static int error;
static int savedErrno;

static int convert_endian = 0;

/* Provide our own endian swap that does nothing. */
void yaffs_do_endian_packed_tags2(struct yaffs_dev *dev,
                                  struct yaffs_packed_tags2_tags_only *ptt)
{
#if 0
    if (!dev->swap_endian)
        return;
    ptt->seq_number = swap_u32(ptt->seq_number);
    ptt->obj_id = swap_u32(ptt->obj_id);
    ptt->chunk_id = swap_u32(ptt->chunk_id);
    ptt->n_bytes = swap_u32(ptt->n_bytes);
#endif
}

static void fatal(const char *fn)
{
    perror(fn);
    error |= 1;
    exit(error);
}

static int warn(const char *fn)
{
    savedErrno = errno;
    perror(fn);
    error |= 2;
    return error;
}

static int obj_compare(const void *a, const void *b)
{
    objItem *oa, *ob;

    oa = (objItem *)a;
    ob = (objItem *)b;

    if (oa->dev < ob->dev) return -1;
    if (oa->dev > ob->dev) return 1;
    if (oa->ino < ob->ino) return -1;
    if (oa->ino > ob->ino) return 1;

    return 0;
}


static void add_obj_to_list(dev_t dev, ino_t ino, int obj)
{
    if (n_obj < MAX_OBJECTS)
    {
        obj_list[n_obj].dev = dev;
        obj_list[n_obj].ino = ino;
        obj_list[n_obj].obj = obj;
        n_obj++;
        qsort(obj_list, n_obj, sizeof(objItem), obj_compare);

    }
    else
    {
        // oops! not enough space in the object array
        fprintf(stderr, "Not enough space in object array\n");
        exit(1);
    }
}


static int find_obj_in_list(dev_t dev, ino_t ino)
{
    objItem *i = NULL;
    objItem test;

    test.dev = dev;
    test.ino = ino;

    if (n_obj > 0)
    {
        i = bsearch(&test, obj_list, n_obj, sizeof(objItem), obj_compare);
    }

    if (i)
    {
        return i->obj;
    }
    return -1;
}

/* This little function converts a little endian tag to a big endian tag.
 * NOTE: The tag is not usable after this other than calculating the CRC
 * with.
 */
static void little_to_big_endian(struct yaffs_ext_tags *tagsPtr)
{
#if 0 // FIXME NCB
    union yaffs_tags_union *tags = (union yaffs_tags_union *)tagsPtr;  // Work in bytes.
    union yaffs_tags_union   temp;

    memset(&temp, 0, sizeof(temp));
    // Ick, I hate magic numbers.
    temp.as_bytes[0] = ((tags->as_bytes[2] & 0x0F) << 4) | ((tags->as_bytes[1] & 0xF0) >> 4);
    temp.as_bytes[1] = ((tags->as_bytes[1] & 0x0F) << 4) | ((tags->as_bytes[0] & 0xF0) >> 4);
    temp.as_bytes[2] = ((tags->as_bytes[0] & 0x0F) << 4) | ((tags->as_bytes[2] & 0x30) >> 2) | ((tags->as_bytes[3] & 0xC0) >> 6);
    temp.as_bytes[3] = ((tags->as_bytes[3] & 0x3F) << 2) | ((tags->as_bytes[2] & 0xC0) >> 6);
    temp.as_bytes[4] = ((tags->as_bytes[6] & 0x03) << 6) | ((tags->as_bytes[5] & 0xFC) >> 2);
    temp.as_bytes[5] = ((tags->as_bytes[5] & 0x03) << 6) | ((tags->as_bytes[4] & 0xFC) >> 2);
    temp.as_bytes[6] = ((tags->as_bytes[4] & 0x03) << 6) | (tags->as_bytes[7] & 0x3F);
    temp.as_bytes[7] = (tags->as_bytes[6] & 0xFC) | ((tags->as_bytes[7] & 0xC0) >> 6);

    // Now copy it back.
    tags->as_bytes[0] = temp.as_bytes[0];
    tags->as_bytes[1] = temp.as_bytes[1];
    tags->as_bytes[2] = temp.as_bytes[2];
    tags->as_bytes[3] = temp.as_bytes[3];
    tags->as_bytes[4] = temp.as_bytes[4];
    tags->as_bytes[5] = temp.as_bytes[5];
    tags->as_bytes[6] = temp.as_bytes[6];
    tags->as_bytes[7] = temp.as_bytes[7];
#endif
}

static void shuffle_oob(char *spareData, struct yaffs_packed_tags2 *pt)
{
    int tag_size = sizeof(pt->t);
    char *rd_ptr;
    char *wr_ptr;

    //assert(sizeof(*pt) <= spareSize);
    assert(tag_size <= 16);
    // NAND LAYOUT: For non-trivial OOB orderings, here would be a good place to shuffle.
    wr_ptr = &spareData[4];
    rd_ptr = (char *)&pt->t;
    while (tag_size >= 4)
    {
        memcpy((void *)wr_ptr, (void *)rd_ptr, 4);
        wr_ptr += 16;
        rd_ptr += 4;
        tag_size -= 4;
    }
    if (tag_size > 0)
    {
        memcpy((void *)wr_ptr, (void *)rd_ptr, tag_size);
    }

}

static void check_usd_space(void)
{
    if (((total_block_num - dummy_dev.param.n_reserved_blocks) * pagesPerBlock) < nPages)
    {
        printf("img size overflow %d\n", total_block_num);
        printf("Disk Total: %d KB\n", (total_block_num - dummy_dev.param.n_reserved_blocks) * dummy_dev.data_bytes_per_chunk * pagesPerBlock / 1024);
        printf("Not packed: %.2f KB\n", (float)remaining_size / 1024);
        exit(1);
    }

}

#ifdef USE_INBAND_TAGS
static int write_chunk(u8 *data, u32 id, u32 chunk_id, u32 n_bytes)
{
    struct yaffs_ext_tags t;
    struct yaffs_packed_tags2_tags_only *pto;
    DWORD wr_size;

    memset(&t, 0, sizeof(t));

    t.chunk_id = chunk_id;
//  t.serial_number = 0;
    t.serial_number = 1;    // **CHECK**
    t.n_bytes = n_bytes;
    t.obj_id = id;

    t.seq_number = YAFFS_LOWEST_SEQUENCE_NUMBER;

// added NCB **CHECK**
    t.chunk_used = 1;

    if (convert_endian)
    {
        little_to_big_endian(&t);
    }

    nPages++;

    check_usd_space();


    pto = (struct yaffs_packed_tags2_tags_only *)(data + dummy_dev.data_bytes_per_chunk);
    yaffs_pack_tags2_tags_only(&dummy_dev, pto, &t);
    //if (write(outFile,data,chunkSize) != chunkSize)
    WriteFile(outFile, data, chunkSize, &wr_size, 0);
    if (wr_size != chunkSize)
        fatal("write");
    return 0;
}
#else
static int write_chunk(u8 *data, u32 id, u32 chunk_id, u32 n_bytes)
{
    struct yaffs_ext_tags t;
    struct yaffs_packed_tags2 pt;
    char spareData[spareSize];
    DWORD wr_size;


    //printf("pos0: %x\r\n", SetFilePointer(outFile, 0, 0, FILE_CURRENT));
    //if (write(outFile,data,chunkSize) != chunkSize)
    WriteFile(outFile, data, chunkSize, &wr_size, 0);
    if (wr_size != chunkSize)
        fatal("write");
    //printf("pos1: %x\r\n", SetFilePointer(outFile, 0, 0, FILE_CURRENT));

    memset(&t, 0, sizeof(t));

    t.chunk_id = chunk_id;
//  t.serial_number = 0;
    t.serial_number = 1;    // **CHECK**
    t.n_bytes = n_bytes;
    t.obj_id = id;

    t.seq_number = YAFFS_LOWEST_SEQUENCE_NUMBER;

// added NCB **CHECK**
    t.chunk_used = 1;

    if (convert_endian)
    {
        little_to_big_endian(&t);
    }

    nPages++;

    check_usd_space();

    memset(&pt, 0, sizeof(pt));
    yaffs_pack_tags2(&dummy_dev, &pt, &t, 1);

    memset(spareData, 0xff, sizeof(spareData));
    shuffle_oob(spareData, &pt);

    //if (write(outFile,spareData,sizeof(spareData)) != sizeof(spareData))
    WriteFile(outFile, spareData, sizeof(spareData), &wr_size, 0);
    if (wr_size != sizeof(spareData))
        fatal("write");
    //printf("pos2: %x\r\n", SetFilePointer(outFile, 0, 0, FILE_CURRENT));
    return 0;
}
#endif /* USE_INBAND_TAGS */


#define SWAP32(x)   ((((x) & 0x000000FF) << 24) | \
                     (((x) & 0x0000FF00) << 8 ) | \
                     (((x) & 0x00FF0000) >> 8 ) | \
                     (((x) & 0xFF000000) >> 24))

#define SWAP16(x)   ((((x) & 0x00FF) << 8) | \
                     (((x) & 0xFF00) >> 8))

// This one is easier, since the types are more standard. No funky shifts here.
static void object_header_little_to_big_endian(struct yaffs_obj_hdr *oh)
{
    oh->type = SWAP32(oh->type); // GCC makes enums 32 bits.
    oh->parent_obj_id = SWAP32(oh->parent_obj_id); // int
    oh->sum_no_longer_used = SWAP16(oh->sum_no_longer_used); // u16 - Not used, but done for completeness.
    // name = skip. Char array. Not swapped.
    oh->yst_mode = SWAP32(oh->yst_mode);

    // Regular POSIX.
    oh->yst_uid = SWAP32(oh->yst_uid);
    oh->yst_gid = SWAP32(oh->yst_gid);
    oh->yst_atime = SWAP32(oh->yst_atime);
    oh->yst_mtime = SWAP32(oh->yst_mtime);
    oh->yst_ctime = SWAP32(oh->yst_ctime);

    oh->file_size_low = SWAP32(oh->file_size_low); // Aiee. An int... signed, at that!
    oh->file_size_high = SWAP32(oh->file_size_high); // Aiee. An int... signed, at that!
    oh->equiv_id = SWAP32(oh->equiv_id);
    // alias  - char array.
    oh->yst_rdev = SWAP32(oh->yst_rdev);

    oh->win_ctime[0] = SWAP32(oh->win_ctime[0]);
    oh->win_ctime[1] = SWAP32(oh->win_ctime[1]);
    oh->win_atime[0] = SWAP32(oh->win_atime[0]);
    oh->win_atime[1] = SWAP32(oh->win_atime[1]);
    oh->win_mtime[0] = SWAP32(oh->win_mtime[0]);
    oh->win_mtime[1] = SWAP32(oh->win_mtime[1]);

    oh->reserved[0] = SWAP32(oh->reserved[0]);
#if 0
    oh->reserved[1] = SWAP32(oh->reserved[1]);
#endif
    oh->inband_shadowed_obj_id = SWAP32(oh->inband_shadowed_obj_id);
    oh->inband_is_shrink = SWAP32(oh->inband_is_shrink);
    oh->shadows_obj = SWAP32(oh->shadows_obj);
    oh->is_shrink = SWAP32(oh->is_shrink);
}


static int write_object_header(int id, enum yaffs_obj_type t, struct stat *s, int parent, const char *name, int equivalentObj, const char *alias)
{
    u8 bytes[chunkSize];


    struct yaffs_obj_hdr *oh = (struct yaffs_obj_hdr *)bytes;

    memset(bytes, 0xff, sizeof(bytes));

    oh->type = t;

    oh->parent_obj_id = parent;

    if (strlen(name) + 1 > sizeof(oh->name))
    {
        errno = ENAMETOOLONG;
        return warn("object name");
    }
    memset(oh->name, 0, sizeof(oh->name));
    strcpy(oh->name, name);


    if (t != YAFFS_OBJECT_TYPE_HARDLINK)
    {
        oh->yst_mode = s->st_mode;
        oh->yst_uid = s->st_uid;
// NCB 12/9/02      oh->yst_gid = s->yst_uid;
        oh->yst_gid = s->st_gid;
        oh->yst_atime = s->st_atime & 0xFFFFFFFF;
        oh->yst_mtime = s->st_mtime & 0xFFFFFFFF;
        oh->yst_ctime = s->st_ctime & 0xFFFFFFFF;
        oh->yst_rdev  = s->st_rdev & 0xFFFFFFFF;
    }

    if (t == YAFFS_OBJECT_TYPE_FILE)
    {
        oh->file_size_low = s->st_size;
        oh->file_size_high = ((unsigned long long)s->st_size >> 32);
    }

    if (t == YAFFS_OBJECT_TYPE_HARDLINK)
    {
        oh->equiv_id = equivalentObj;
    }

    if (t == YAFFS_OBJECT_TYPE_SYMLINK)
    {
        if (strlen(alias) + 1 > sizeof(oh->alias))
        {
            errno = ENAMETOOLONG;
            return warn("object alias");
        }
        memset(oh->alias, 0, sizeof(oh->alias));
        strcpy(oh->alias, alias);
    }

    if (convert_endian)
    {
        object_header_little_to_big_endian(oh);
    }

    return write_chunk(bytes, id, 0, 0xffff);

}

static void pad_image(void)
{
#ifdef USE_INBAND_TAGS
    u8 data[chunkSize];
#else
    u8 data[chunkSize + spareSize];
#endif
    int padPages;
    DWORD wr_size;

    if ((total_block_num * pagesPerBlock) < nPages)
    {
        printf("img size overflow: %d\n", total_block_num);
        exit(1);
    }

    padPages = (total_block_num * pagesPerBlock) - nPages;
    if (padPages)
    {
        memset(data, 0xff, sizeof(data));
        for (; padPages; padPages--)
        {
            //if (write(outFile, data, sizeof(data)) != sizeof(data))
            WriteFile(outFile, data, sizeof(data), &wr_size, 0);
            if (wr_size != sizeof(data))
                fatal("write");
        }
    }
}


static int calc_dir_size(int parent, const char *path)
{
    static WIN32_FIND_DATA find_data;
    HANDLE hdir;
    char *search_files;
    int total_size = 0;

    search_files = malloc(strlen(path) + strlen("/*") + 1);
    assert(search_files);
    search_files[0] = 0;
    strcat(search_files, path);
    strcat(search_files, "/*");

    hdir = FindFirstFile(search_files, &find_data);
    if (hdir == INVALID_HANDLE_VALUE)
    {
        warn("opendir");
    }
    else
    {
        while (1)
        {

            /* Ignore . and .. */
            if (strcmp(find_data.cFileName, ".") &&
                    strcmp(find_data.cFileName, ".."))
            {
                char full_name[500];
                struct stat stats;
                int equivalentObj;

                if (snprintf(full_name, sizeof(full_name), "%s/%s", path, find_data.cFileName) >= (int)sizeof(full_name))
                {
                    error = -1;
                    break;
                }

                if (stat(full_name, &stats) < 0)
                {
                    warn("lstat");
                    break;
                }

                if ( //S_ISLNK(stats.st_mode) ||
                    S_ISREG(stats.st_mode) ||
                    S_ISDIR(stats.st_mode)// ||
                    //S_ISFIFO(stats.st_mode) ||
                    //S_ISBLK(stats.st_mode) ||
                    //S_ISCHR(stats.st_mode) ||
                    //S_ISSOCK(stats.st_mode)
                )
                {

                    /* We're going to create an object for it */
                    if (((equivalentObj = find_obj_in_list(stats.st_dev, stats.st_ino)) > 0) && 0)
                    {
                    }
                    else
                    {
                        if (S_ISREG(stats.st_mode))
                        {
                            HANDLE h;
                            int chunk = 0;

                            h = CreateFile(full_name, GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);
                            if (h != INVALID_HANDLE_VALUE)
                            {
                                total_size += GetFileSize(h, NULL);
                                CloseHandle(h);
                            }
                            else
                            {
                                warn("open");
                            }
                        }
                        else if (S_ISDIR(stats.st_mode))
                        {
                            total_size += calc_dir_size(0, full_name);
                        }
                    }
                }
                else
                {
                    fprintf(stderr, "%s: unhandled type\n", full_name);
                    error |= 2;
                    savedErrno = EINVAL;
                }
            }

            if (!FindNextFile(hdir, &find_data))
            {
                break;
            }
        }
        FindClose(hdir);
    }

    return total_size;

}


static int process_directory(int parent, const char *path)
{
    static WIN32_FIND_DATA find_data;
    HANDLE hdir;
    char *search_files;

    search_files = malloc(strlen(path) + strlen("/*") + 1);
    assert(search_files);
    search_files[0] = 0;
    strcat(search_files, path);
    strcat(search_files, "/*");

    nDirectories++;

    hdir = FindFirstFile(search_files, &find_data);
    if (hdir == INVALID_HANDLE_VALUE)
    {
        warn("opendir");
    }
    else
    {
        while (1)
        {

            /* Ignore . and .. */
            if (strcmp(find_data.cFileName, ".") &&
                    strcmp(find_data.cFileName, ".."))
            {
                char full_name[500];
                struct stat stats;
                int equivalentObj;
                int newObj;

                if (snprintf(full_name, sizeof(full_name), "%s/%s", path, find_data.cFileName) >= (int)sizeof(full_name))
                {
                    error = -1;
                    break;
                }

                if (stat(full_name, &stats) < 0)
                {
                    warn("lstat");
                    break;
                }

                if ( //S_ISLNK(stats.st_mode) ||
                    S_ISREG(stats.st_mode) ||
                    S_ISDIR(stats.st_mode)// ||
                    //S_ISFIFO(stats.st_mode) ||
                    //S_ISBLK(stats.st_mode) ||
                    //S_ISCHR(stats.st_mode) ||
                    //S_ISSOCK(stats.st_mode)
                )
                {

                    newObj = obj_id++;
                    n_obj++;

                    printf("Object %d, %s is a ", newObj, full_name);

                    /* We're going to create an object for it */
                    if (((equivalentObj = find_obj_in_list(stats.st_dev, stats.st_ino)) > 0) && 0)
                    {
                        /* we need to make a hard link */
                        printf("hard link to object %d\n", equivalentObj);
                        write_object_header(newObj, YAFFS_OBJECT_TYPE_HARDLINK, &stats, parent, find_data.cFileName, equivalentObj, NULL);
                    }
                    else
                    {

                        add_obj_to_list(stats.st_dev, stats.st_ino, newObj);

#if 0
                        if (S_ISLNK(stats.st_mode))
                        {

                            char symname[500];

                            memset(symname, 0, sizeof(symname));

                            if (readlink(full_name, symname, sizeof(symname) - 1) < 0)
                            {
                                warn("readlink");
                            }
                            else
                            {
                                printf("symlink to \"%s\"\n", symname);
                                write_object_header(newObj, YAFFS_OBJECT_TYPE_SYMLINK, &stats, parent, find_data.cFileName, -1, symname);
                            }
                        }
                        else
#endif
                            if (S_ISREG(stats.st_mode))
                            {
                                printf("file, start page: %d, ", nPages);
                                if (write_object_header(newObj, YAFFS_OBJECT_TYPE_FILE, &stats, parent, find_data.cFileName, -1, NULL) == 0)
                                {
                                    HANDLE h;
                                    u8 bytes[chunkSize];
                                    int n_bytes;
                                    int chunk = 0;
                                    int file_size;

#ifdef USE_INBAND_TAGS
                                    n_bytes = chunkSize - sizeof(struct yaffs_packed_tags2_tags_only);
#else
                                    n_bytes = chunkSize;
#endif /* USE_INBAND_TAGS */

                                    //h = open(full_name,O_RDONLY);
                                    h = CreateFile(full_name, GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);
                                    //if(h >= 0)
                                    if (h != INVALID_HANDLE_VALUE)
                                    {
                                        file_size = GetFileSize(h, NULL);
                                        memset(bytes, 0xff, sizeof(bytes));

                                        ReadFile(h, bytes, n_bytes, &n_bytes, 0);
                                        while (n_bytes > 0)
                                        {
                                            chunk++;
                                            write_chunk(bytes, newObj, chunk, n_bytes);
                                            memset(bytes, 0xff, sizeof(bytes));
                                            ReadFile(h, bytes, n_bytes, &n_bytes, 0);
                                        }
                                        if (n_bytes < 0)
                                            warn("read");

                                        printf("%d data chunks written\n", chunk);
                                        //close(h);
                                        CloseHandle(h);
                                        if (remaining_size >= file_size)
                                        {
                                            remaining_size -= file_size;
                                        }
                                    }
                                    else
                                    {
                                        warn("open");
                                    }

                                }

                            }
#if 0
                            else if (S_ISSOCK(stats.st_mode))
                            {
                                printf("socket\n");
                                write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, find_data.cFileName, -1, NULL);
                            }

                            else if (S_ISFIFO(stats.st_mode))
                            {
                                printf("fifo\n");
                                write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, find_data.cFileName, -1, NULL);
                            }
                            else if (S_ISCHR(stats.st_mode))
                            {
                                printf("character device\n");
                                write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, find_data.cFileName, -1, NULL);
                            }
                            else if (S_ISBLK(stats.st_mode))
                            {
                                printf("block device\n");
                                write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, find_data.cFileName, -1, NULL);
                            }
#endif
                            else if (S_ISDIR(stats.st_mode))
                            {
                                printf("directory\n");
                                if (write_object_header(newObj, YAFFS_OBJECT_TYPE_DIRECTORY, &stats, parent, find_data.cFileName, -1, NULL) == 0)
                                    process_directory(newObj, full_name);
                            }
                    }
                }
                else
                {
                    fprintf(stderr, "%s: unhandled type\n", full_name);
                    error |= 2;
                    savedErrno = EINVAL;
                }
            }

            if (!FindNextFile(hdir, &find_data))
            {
                break;
            }
        }
        FindClose(hdir);
    }

    return 0;

}


int main(int argc, char *argv[])
{
    struct stat stats;
    int total_size;
    int argv_idx;
    int free_space_size;

    printf("mkyaffs2image: image building tool for YAFFS2 built "__DATE__"\n");

    /* the first argument is program name */
    argc--;
    /* Initialize option parameters */
    argv_idx = 1;
    while (argc >= 1 && *argv[argv_idx] == '-')
    {
        if (!strcmp(argv[argv_idx], "-rsv"))
        {
            if (argc < 2)
            {
                argc = 0;
                break;
            }
            reserved_size = atoi(argv[argv_idx + 1]);
            if (reserved_size <= 0)
            {
                argc = 0;
                break;
            }
            argv_idx += 2;
            argc -= 2;
        }
        else
        {
            argc = 0;
        }
    }

    if (argc < 3)
    {
        printf("usage: mkyaffs2image -rsv rsv_size dir image_file img_size [convert]\n");
        printf("           -rsv       reserved space size in KB\n");
        printf("           dir        the directory tree to be converted\n");
        printf("           image_file the output file to hold the image\n");
        printf("           img_size   image file size in block number\n");
        printf("           'convert'  produce a big-endian image from a little-endian machine\n");
        exit(1);
    }

    total_block_num = atoi(argv[argv_idx + ARGV_IMG_SIZE_POS]);

    if ((argc == 4) && (!strncmp(argv[argv_idx + ARGV_CONVERT_POS], "convert", strlen("convert"))))
    {
        convert_endian = 1;
    }

    if (stat(argv[argv_idx + ARGV_DIR_POS], &stats) < 0)
    {
        printf("Could not stat %s\n", argv[argv_idx]);
        exit(1);
    }

    if (!S_ISDIR(stats.st_mode))
    {
        printf(" %s is not a directory\n", argv[argv_idx + ARGV_DIR_POS]);
        exit(1);
    }

    //outFile = open(argv[2],O_CREAT | O_TRUNC | O_WRONLY, S_IREAD | S_IWRITE);
    outFile = CreateFile(argv[argv_idx + ARGV_IMGFILE_POS], GENERIC_WRITE, 0, 0, CREATE_ALWAYS, 0, 0);


    //if(outFile < 0)
    if (outFile == INVALID_HANDLE_VALUE)
    {
        printf("Could not open output file %s\n", argv[argv_idx + ARGV_IMGFILE_POS]);
        exit(1);
    }

#ifdef USE_INBAND_TAGS
    dummy_dev.data_bytes_per_chunk = chunkSize - sizeof(struct yaffs_packed_tags2_tags_only);
#else
    dummy_dev.data_bytes_per_chunk = chunkSize;
#endif /* USE_INBAND_TAGS */

    total_size = calc_dir_size(0, argv[argv_idx + ARGV_DIR_POS]);
    remaining_size = total_size;
    printf("Processing directory %s into image file %s, %d\n", argv[argv_idx + ARGV_DIR_POS], argv[argv_idx + ARGV_IMGFILE_POS], total_size);
    process_directory(YAFFS_OBJECTID_ROOT, argv[argv_idx + ARGV_DIR_POS]);

    pad_image();

    //close(outFile);
    CloseHandle(outFile);

    if (error)
    {
        errno = savedErrno;
        perror("operation incomplete");
    }
    else
    {
        printf("Operation complete.\n"
               "%d objects in %d directories\n"
               "%d NAND pages\n", n_obj, nDirectories, nPages);
    }

    free_space_size = ((total_block_num - dummy_dev.param.n_reserved_blocks) * pagesPerBlock - nPages) * dummy_dev.data_bytes_per_chunk / 1024;

    printf("Disk Total: %d KB, Free: %d KB\n", (total_block_num - dummy_dev.param.n_reserved_blocks) * dummy_dev.data_bytes_per_chunk * pagesPerBlock / 1024,
           free_space_size);

    if (free_space_size < reserved_size)
    {
        printf("Pack error: free size is less than reserved size: %d KB \n", reserved_size);
        error |= 4;
    }

    exit(error);
}

