/*
 * YAFFS: Yet Another Flash File System. A NAND-flash specific file system.
 *
 * Copyright (C) 2002-2018 Aleph One Ltd.
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 * Nick Bane modifications flagged NCB
 * Endian handling patches by James Ng
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * makeyaffsimage.c
 *
 * Makes a YAFFS file system image that can be used to load up a file system.
 */

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string.h>
#include <unistd.h>
#include "yaffs_ecc.h"
#include "yaffs_guts.h"


#define MAX_OBJECTS 10000

const char * mkyaffsimage_c_version = "$Id: mkyaffsimage.c,v 1.7 2003/07/16 03:00:48 charles Exp $";


typedef struct
{
	dev_t dev;
	ino_t ino;
	int   obj;
} objItem;


static objItem obj_list[MAX_OBJECTS];
static int obj_alloc_id = YAFFS_NOBJECT_BUCKETS + 1;

static int n_obj, nDirectories, nPages;

static int outFile;

static int error;

static int convert_endian = 0;

static int obj_compare(const void *a, const void * b)
{
  objItem *oa, *ob;

  oa = (objItem *)a;
  ob = (objItem *)b;

  if(oa->dev < ob->dev) return -1;
  if(oa->dev > ob->dev) return 1;
  if(oa->ino < ob->ino) return -1;
  if(oa->ino > ob->ino) return 1;

  return 0;
}


static void add_obj_to_list(dev_t dev, ino_t ino, int obj)
{
	if(n_obj < MAX_OBJECTS)
	{
		obj_list[n_obj].dev = dev;
		obj_list[n_obj].ino = ino;
		obj_list[n_obj].obj = obj;
		n_obj++;
		qsort(obj_list,n_obj,sizeof(objItem),obj_compare);

	}
	else
	{
		// oops! not enough space in the object array
		fprintf(stderr,"Not enough space in object array\n");
		exit(2);
	}
}


static int find_obj_in_list(dev_t dev, ino_t ino)
{
	objItem *i = NULL;
	objItem test;

	test.dev = dev;
	test.ino = ino;

	if(n_obj > 0)
	{
		i = bsearch(&test,obj_list,n_obj,sizeof(objItem),obj_compare);
	}

	if(i)
	{
		return i->obj;
	}
	return -1;
}

#if 0
// NCB added 10/9/2002
static u16 yaffs_calc_name_sum(const char *name)
{
	u16 sum = 0;
	u16 i = 1;

	u8 *bname = (u8 *)name;

	while (*bname)
	{
		sum += (*bname) * i;
		i++;
		bname++;
	}
	return sum;
}
#endif

static void yaffs_calc_ecc(const u8 *data, struct yaffs_spare *spare)
{
	yaffs_ecc_calc(data , spare->ecc1);
	yaffs_ecc_calc(&data[256] , spare->ecc2);
}

static void yaffs_calc_tags_ecc(struct yaffs_tags *tags)
{
	// Todo don't do anything yet. Need to calculate ecc
	unsigned char *b = ((union yaffs_tags_union *)tags)->as_bytes;
	unsigned  i,j;
	unsigned  ecc = 0;
	unsigned bit = 0;

    // Clear ECC fields
    if (!convert_endian)
    {
	    tags->ecc = 0;
    }
    else
    {
        // Because we're in "munged tag" mode, we have to clear it manually
        b[6] &= 0xC0;
        b[7] &= 0x03;
    }

	for(i = 0; i < 8; i++)
	{
// NCB modified 20-9-02		for(j = 1; j &0x7f; j<<=1)
		for(j = 1; j &0xff; j<<=1)
		{
			bit++;
			if(b[i] & j)
			{
				ecc ^= bit;
			}
		}
	}

	// Write out ECC
    if (!convert_endian)
    {
        tags->ecc = ecc;
    }
    else
    {
	    // We have to munge the ECC again.
	    b[6] |= ((ecc >> 6) & 0x3F);
        b[7] |= ((ecc & 0x3F) << 2);
    }
}
static void yaffs_load_tags_to_spare(struct yaffs_spare *sparePtr, struct yaffs_tags *tagsPtr)
{
	union yaffs_tags_union *tu = (union yaffs_tags_union *)tagsPtr;

	//yaffs_calc_tags_ecc(tagsPtr);

	sparePtr->tb0 = tu->as_bytes[0];
	sparePtr->tb1 = tu->as_bytes[1];
	sparePtr->tb2 = tu->as_bytes[2];
	sparePtr->tb3 = tu->as_bytes[3];
	sparePtr->tb4 = tu->as_bytes[4];
	sparePtr->tb5 = tu->as_bytes[5];
	sparePtr->tb6 = tu->as_bytes[6];
	sparePtr->tb7 = tu->as_bytes[7];
}

/* This little function converts a little endian tag to a big endian tag.
 * NOTE: The tag is not usable after this other than calculating the CRC
 * with.
 */
static void little_to_big_endian(struct yaffs_tags *tagsPtr)
{
    union yaffs_tags_union * tags = (union yaffs_tags_union* )tagsPtr; // Work in bytes.
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
}

static int write_chunk(u8 *data, u32 obj_id, u32 chunk_id, u32 n_bytes)
{
	struct yaffs_tags t;
	struct yaffs_spare s;

	error = write(outFile,data,512);
	if(error < 0) return error;

	memset(&t,0xff,sizeof (struct yaffs_tags));
	memset(&s,0xff,sizeof (struct yaffs_spare));

	t.chunk_id = chunk_id;
	t.serial_number = 0;
	t.n_bytes_lsb = n_bytes;
	t.obj_id = obj_id;

    if (convert_endian)
    {
        little_to_big_endian(&t);
    }

	yaffs_calc_tags_ecc(&t);
	yaffs_load_tags_to_spare(&s,&t);
	yaffs_calc_ecc(data,&s);

	nPages++;

	return write(outFile,&s,sizeof(struct yaffs_spare));

}

#define SWAP32(x)   ((((x) & 0x000000FF) << 24) | \
                     (((x) & 0x0000FF00) << 8 ) | \
                     (((x) & 0x00FF0000) >> 8 ) | \
                     (((x) & 0xFF000000) >> 24))

#define SWAP16(x)   ((((x) & 0x00FF) << 8) | \
                     (((x) & 0xFF00) >> 8))

// This one is easier, since the types are more standard. No funky shifts here.
static void object_header_little_to_big_endian(struct yaffs_obj_hdr* oh)
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

    oh->inband_shadowed_obj_id = SWAP32(oh->inband_shadowed_obj_id);
    oh->inband_is_shrink = SWAP32(oh->inband_is_shrink);
    oh->shadows_obj = SWAP32(oh->shadows_obj);
    oh->is_shrink = SWAP32(oh->is_shrink);
}

static int write_object_header(int obj_id, enum yaffs_obj_type t, struct stat *s, int parent, const char *name, int equivalentObj, const char * alias)
{
	u8 bytes[512];


	struct yaffs_obj_hdr *oh = (struct yaffs_obj_hdr *)bytes;

	memset(bytes,0xff,512);

	oh->type = t;

	oh->parent_obj_id = parent;

	strncpy(oh->name,name,YAFFS_MAX_NAME_LENGTH);


	if(t != YAFFS_OBJECT_TYPE_HARDLINK)
	{
		oh->yst_mode = s->st_mode;
		oh->yst_uid = s->st_uid;
// NCB 12/9/02		oh->yst_gid = s->yst_uid;
		oh->yst_gid = s->st_gid;
		oh->yst_atime = s->st_atime;
		oh->yst_mtime = s->st_mtime;
		oh->yst_ctime = s->st_ctime;
		oh->yst_rdev  = s->st_rdev;
	}

	if(t == YAFFS_OBJECT_TYPE_FILE)
	{
		oh->file_size_low = s->st_size;
	}

	if(t == YAFFS_OBJECT_TYPE_HARDLINK)
	{
		oh->equiv_id = equivalentObj;
	}

	if(t == YAFFS_OBJECT_TYPE_SYMLINK)
	{
		strncpy(oh->alias,alias,YAFFS_MAX_ALIAS_LENGTH);
	}

    if (convert_endian)
    {
        object_header_little_to_big_endian(oh);
    }

	return write_chunk(bytes,obj_id,0,0xffff);

}


static int process_directory(int parent, const char *path)
{

	DIR *dir;
	struct dirent *entry;

	nDirectories++;

	dir = opendir(path);

	if(dir)
	{
		while((entry = readdir(dir)) != NULL)
		{

			/* Ignore . and .. */
			if(strcmp(entry->d_name,".") &&
			   strcmp(entry->d_name,".."))
 			{
 				char full_name[500];
				struct stat stats;
				int equivalentObj;
				int newObj;

				sprintf(full_name,"%s/%s",path,entry->d_name);

				lstat(full_name,&stats);

				if(S_ISLNK(stats.st_mode) ||
				    S_ISREG(stats.st_mode) ||
				    S_ISDIR(stats.st_mode) ||
				    S_ISFIFO(stats.st_mode) ||
				    S_ISBLK(stats.st_mode) ||
				    S_ISCHR(stats.st_mode) ||
				    S_ISSOCK(stats.st_mode))
				{

					newObj = obj_alloc_id++;
					n_obj++;

					printf("Object %d, %s is a ",newObj,full_name);

					/* We're going to create an object for it */
					if((equivalentObj = find_obj_in_list(stats.st_dev, stats.st_ino)) > 0)
					{
					 	/* we need to make a hard link */
					 	printf("hard link to object %d\n",equivalentObj);
						error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_HARDLINK, &stats, parent, entry->d_name, equivalentObj, NULL);
					}
					else
					{

						add_obj_to_list(stats.st_dev,stats.st_ino,newObj);

						if(S_ISLNK(stats.st_mode))
						{

							char symname[500];
							int ret;

							memset(symname,0, sizeof(symname));

							ret = readlink(full_name,symname,sizeof(symname) -1);

							if (ret < 0)
								perror("readlink of symlink");

							printf("symlink to \"%s\"\n",symname);
							error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_SYMLINK, &stats, parent, entry->d_name, -1, symname);

						}
						else if(S_ISREG(stats.st_mode))
						{
							printf("file, ");
							error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_FILE, &stats, parent, entry->d_name, -1, NULL);

							if(error >= 0)
							{
								int h;
								u8 bytes[512];
								int n_bytes;
								int chunk = 0;

								h = open(full_name,O_RDONLY);
								if(h >= 0)
								{
									memset(bytes,0xff,512);
									while((n_bytes = read(h,bytes,512)) > 0)
									{
										chunk++;
										write_chunk(bytes,newObj,chunk,n_bytes);
										memset(bytes,0xff,512);
									}
									if(n_bytes < 0)
									   error = n_bytes;

									printf("%d data chunks written\n",chunk);
								}
								else
								{
									perror("Error opening file");
								}
								close(h);

							}

						}
						else if(S_ISSOCK(stats.st_mode))
						{
							printf("socket\n");
							error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, entry->d_name, -1, NULL);
						}
						else if(S_ISFIFO(stats.st_mode))
						{
							printf("fifo\n");
							error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, entry->d_name, -1, NULL);
						}
						else if(S_ISCHR(stats.st_mode))
						{
							printf("character device\n");
							error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, entry->d_name, -1, NULL);
						}
						else if(S_ISBLK(stats.st_mode))
						{
							printf("block device\n");
							error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_SPECIAL, &stats, parent, entry->d_name, -1, NULL);
						}
						else if(S_ISDIR(stats.st_mode))
						{
							printf("directory\n");
							error =  write_object_header(newObj, YAFFS_OBJECT_TYPE_DIRECTORY, &stats, parent, entry->d_name, -1, NULL);
// NCB modified 10/9/2001				process_directory(1,full_name);
							process_directory(newObj,full_name);
						}
					}
				}
				else
				{
					printf(" we don't handle this type\n");
				}
			}
		}
	}

	return 0;

}


int main(int argc, char *argv[])
{
	struct stat stats;

	printf("mkyaffsimage: image building tool for YAFFS built "__DATE__"\n");

	if(argc < 3)
	{
		printf("usage: mkyaffsimage dir image_file [convert]\n");
		printf("           dir        the directory tree to be converted\n");
		printf("           image_file the output file to hold the image\n");
        printf("           'convert'  produce a big-endian image from a little-endian machine\n");
		exit(1);
	}

    if ((argc == 4) && (!strncmp(argv[3], "convert", strlen("convert"))))
    {
        convert_endian = 1;
    }

	if(stat(argv[1],&stats) < 0)
	{
		printf("Could not stat %s\n",argv[1]);
		exit(1);
	}

	if(!S_ISDIR(stats.st_mode))
	{
		printf(" %s is not a directory\n",argv[1]);
		exit(1);
	}

	outFile = open(argv[2],O_CREAT | O_TRUNC | O_WRONLY, S_IREAD | S_IWRITE);


	if(outFile < 0)
	{
		printf("Could not open output file %s\n",argv[2]);
		exit(1);
	}

	printf("Processing directory %s into image file %s\n",argv[1],argv[2]);
	error =  write_object_header(1, YAFFS_OBJECT_TYPE_DIRECTORY, &stats, 1,"", -1, NULL);
	if(error)
	error = process_directory(YAFFS_OBJECTID_ROOT,argv[1]);

	close(outFile);

	if(error < 0)
	{
		perror("operation incomplete");
		exit(1);
	}
	else
	{
		printf("Operation complete.\n"
		       "%d objects in %d directories\n"
		       "%d NAND pages\n",n_obj, nDirectories, nPages);
	}

	close(outFile);

	exit(0);
}

