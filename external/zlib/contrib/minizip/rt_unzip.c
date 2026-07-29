
#include "rtthread.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include "unzip.h"





#define FOPEN_FUNC(filename, mode) fopen(filename, mode)
#define FTELLO_FUNC(stream) fseek(stream, 0, SEEK_CUR)
#define FSEEKO_FUNC(stream, offset, origin) fseek(stream, offset, origin)



#define CASESENSITIVITY (0)
#define WRITEBUFFERSIZE (8192)
#define MAXFILENAME (256)

extern int chdir(const char *path);

/*
  mini unzip, demo of unzip package

  usage :
  Usage : miniunz [-exvlo] file.zip [file_to_extract] [-d extractdir]

  list the file in the zipfile, and print the content of FILE_ID.ZIP or README.TXT
    if it exists
*/


/* change_file_date : change the date/time of a file
    filename : the filename of the file where date/time must be modified
    dosdate : the new date at the MSDos format (4 bytes)
    tmu_date : the SAME new date at the tm_unz format */
void change_file_date(
const char *filename,
uLong dosdate,
tm_unz tmu_date)
{
}


/* mymkdir and change_file_date are not 100 % portable
   As I don't know well Unix, I wait feedback for the unix portion */

int mymkdir(const char *dirname)
{
    int ret = 0;
    ret = mkdir(dirname, 0775);
    return ret;
}

int makedir(const char *newdir)
{
    char *buffer ;
    char *p;
    int  len = (int)strlen(newdir);

    if (len <= 0)
        return 0;

    buffer = (char *)malloc(len + 1);
    if (buffer == NULL)
    {
        printf("Error allocating memory\n");
        return UNZ_INTERNALERROR;
    }
    strcpy(buffer, newdir);

    if (buffer[len - 1] == '/')
    {
        buffer[len - 1] = '\0';
    }
    if (mymkdir(buffer) == 0)
    {
        free(buffer);
        return 1;
    }

    p = buffer + 1;
    while (1)
    {
        char hold;

        while (*p && *p != '\\' && *p != '/')
            p++;
        hold = *p;
        *p = 0;
        if ((mymkdir(buffer) == -1) && (errno == ENOENT))
        {
            printf("couldn't create directory %s\n", buffer);
            free(buffer);
            return 0;
        }
        if (hold == 0)
            break;
        *p++ = hold;
    }
    free(buffer);
    return 1;
}

void do_banner()
{
    printf("MiniUnz 1.01b, demo of zLib + Unz package written by Gilles Vollant\n");
    printf("more info at http://www.winimage.com/zLibDll/unzip.html\n\n");
}

void do_help()
{
    printf("Usage : miniunz [-e] [-x] [-v] [-l] [-o] [-p password] file.zip [file_to_extr.] [-d extractdir]\n\n" \
           "  -e  Extract without pathname (junk paths)\n" \
           "  -x  Extract with pathname\n" \
           "  -v  list files\n" \
           "  -l  list files\n" \
           "  -d  directory to extract into\n" \
           "  -o  overwrite files without prompting\n" \
           "  -p  extract crypted file using password\n\n");
}

void Display64BitsSize(ZPOS64_T n, int size_char)
{
    /* to avoid compatibility problem , we do here the conversion */
    char number[21];
    int offset = 19;
    int pos_string = 19;
    number[20] = 0;
    for (;;)
    {
        number[offset] = (char)((n % 10) + '0');
        if (number[offset] != '0')
            pos_string = offset;
        n /= 10;
        if (offset == 0)
            break;
        offset--;
    }
    {
        int size_display_string = 19 - pos_string;
        while (size_char > size_display_string)
        {
            size_char--;
            printf(" ");
        }
    }

    printf("%s", &number[pos_string]);
}

int do_list(unzFile uf)
{
    uLong i;
    unz_global_info64 gi;
    int err;

    err = unzGetGlobalInfo64(uf, &gi);
    if (err != UNZ_OK)
        printf("error %d with zipfile in unzGetGlobalInfo \n", err);
    printf("  Length  Method     Size Ratio   Date    Time   CRC-32     Name\n");
    printf("  ------  ------     ---- -----   ----    ----   ------     ----\n");
    for (i = 0; i < gi.number_entry; i++)
    {
        char filename_inzip[256];
        unz_file_info64 file_info;
        uLong ratio = 0;
        const char *string_method;
        char charCrypt = ' ';
        err = unzGetCurrentFileInfo64(uf, &file_info, filename_inzip, sizeof(filename_inzip), NULL, 0, NULL, 0);
        if (err != UNZ_OK)
        {
            printf("error %d with zipfile in unzGetCurrentFileInfo\n", err);
            break;
        }
        if (file_info.uncompressed_size > 0)
            ratio = (uLong)((file_info.compressed_size * 100) / file_info.uncompressed_size);

        /* display a '*' if the file is crypted */
        if ((file_info.flag & 1) != 0)
            charCrypt = '*';

        if (file_info.compression_method == 0)
            string_method = "Stored";
        else if (file_info.compression_method == Z_DEFLATED)
        {
            uInt iLevel = (uInt)((file_info.flag & 0x6) / 2);
            if (iLevel == 0)
                string_method = "Defl:N";
            else if (iLevel == 1)
                string_method = "Defl:X";
            else if ((iLevel == 2) || (iLevel == 3))
                string_method = "Defl:F"; /* 2:fast , 3 : extra fast*/
        }
        else if (file_info.compression_method == Z_BZIP2ED)
        {
            string_method = "BZip2 ";
        }
        else
            string_method = "Unkn. ";

        Display64BitsSize(file_info.uncompressed_size, 7);
        printf("  %6s%c", string_method, charCrypt);
        Display64BitsSize(file_info.compressed_size, 7);
        printf(" %3lu%%  %2.2lu-%2.2lu-%2.2lu  %2.2lu:%2.2lu  %8.8lx   %s\n",
               ratio,
               (uLong)file_info.tmu_date.tm_mon + 1,
               (uLong)file_info.tmu_date.tm_mday,
               (uLong)file_info.tmu_date.tm_year % 100,
               (uLong)file_info.tmu_date.tm_hour, (uLong)file_info.tmu_date.tm_min,
               (uLong)file_info.crc, filename_inzip);
        if ((i + 1) < gi.number_entry)
        {
            err = unzGoToNextFile(uf);
            if (err != UNZ_OK)
            {
                printf("error %d with zipfile in unzGoToNextFile\n", err);
                break;
            }
        }
    }

    return 0;
}


int do_extract_currentfile(
unzFile uf,
const int *popt_extract_without_path,
int *popt_overwrite,
const char *password)
{
    char filename_inzip[256];
    char *filename_withoutpath;
    char *p;
    int err = UNZ_OK;
    FILE *fout = NULL;
    void *buf;
    uInt size_buf;

    unz_file_info64 file_info;
    uLong ratio = 0;
    err = unzGetCurrentFileInfo64(uf, &file_info, filename_inzip, sizeof(filename_inzip), NULL, 0, NULL, 0);

    if (err != UNZ_OK)
    {
        printf("error %d with zipfile in unzGetCurrentFileInfo\n", err);
        return err;
    }

    size_buf = WRITEBUFFERSIZE;
    buf = (void *)malloc(size_buf);
    if (buf == NULL)
    {
        printf("Error allocating memory\n");
        return UNZ_INTERNALERROR;
    }

    p = filename_withoutpath = filename_inzip;
    while ((*p) != '\0')
    {
        if (((*p) == '/') || ((*p) == '\\'))
            filename_withoutpath = p + 1;
        p++;
    }

    if ((*filename_withoutpath) == '\0')
    {
        if ((*popt_extract_without_path) == 0)
        {
            printf("creating directory: %s\n", filename_inzip);
            mymkdir(filename_inzip);
        }
    }
    else
    {
        const char *write_filename;
        int skip = 0;

        if ((*popt_extract_without_path) == 0)
            write_filename = filename_inzip;
        else
            write_filename = filename_withoutpath;

        err = unzOpenCurrentFilePassword(uf, password);
        if (err != UNZ_OK)
        {
            printf("error %d with zipfile in unzOpenCurrentFilePassword\n", err);
        }

        if (((*popt_overwrite) == 0) && (err == UNZ_OK))
        {
            char rep = 0;
            FILE *ftestexist;
            ftestexist = FOPEN_FUNC(write_filename, "rb");
            if (ftestexist != NULL)
            {
                fclose(ftestexist);
                do
                {
                    char answer[128];
                    int ret;

                    printf("The file %s exists. Overwrite ? [y]es, [n]o, [A]ll: ", write_filename);
                    ret = scanf("%1s", answer);
                    if (ret != 1)
                    {
                        exit(EXIT_FAILURE);
                    }
                    rep = answer[0] ;
                    if ((rep >= 'a') && (rep <= 'z'))
                        rep -= 0x20;
                }
                while ((rep != 'Y') && (rep != 'N') && (rep != 'A'));
            }

            if (rep == 'N')
                skip = 1;

            if (rep == 'A')
                *popt_overwrite = 1;
        }

        if ((skip == 0) && (err == UNZ_OK))
        {
            fout = FOPEN_FUNC(write_filename, "wb");
            /* some zipfile don't contain directory alone before file */
            if ((fout == NULL) && ((*popt_extract_without_path) == 0) &&
                    (filename_withoutpath != (char *)filename_inzip))
            {
                char c = *(filename_withoutpath - 1);
                *(filename_withoutpath - 1) = '\0';
                makedir(write_filename);
                *(filename_withoutpath - 1) = c;
                fout = FOPEN_FUNC(write_filename, "wb");
            }

            if (fout == NULL)
            {
                printf("error opening %s\n", write_filename);
            }
        }

        if (fout != NULL)
        {
            printf(" extracting: %s\n", write_filename);

            do
            {
                err = unzReadCurrentFile(uf, buf, size_buf);
                if (err < 0)
                {
                    printf("error %d with zipfile in unzReadCurrentFile\n", err);
                    break;
                }
                if (err > 0)
                    if (fwrite(buf, err, 1, fout) != 1)
                    {
                        printf("error in writing extracted file\n");
                        err = UNZ_ERRNO;
                        break;
                    }
            }
            while (err > 0);
            if (fout)
                fclose(fout);

            if (err == 0)
                change_file_date(write_filename, file_info.dosDate,
                                 file_info.tmu_date);
        }

        if (err == UNZ_OK)
        {
            err = unzCloseCurrentFile(uf);
            if (err != UNZ_OK)
            {
                printf("error %d with zipfile in unzCloseCurrentFile\n", err);
            }
        }
        else
            unzCloseCurrentFile(uf); /* don't lose the error */
    }

    free(buf);
    return err;
}


int do_extract(
unzFile uf,
int opt_extract_without_path,
int opt_overwrite,
const char *password)
{
    uLong i;
    unz_global_info64 gi;
    int err;
    FILE *fout = NULL;

    err = unzGetGlobalInfo64(uf, &gi);
    if (err != UNZ_OK)
        printf("error %d with zipfile in unzGetGlobalInfo \n", err);

    for (i = 0; i < gi.number_entry; i++)
    {
        if (do_extract_currentfile(uf, &opt_extract_without_path,
                                   &opt_overwrite,
                                   password) != UNZ_OK)
            break;

        if ((i + 1) < gi.number_entry)
        {
            err = unzGoToNextFile(uf);
            if (err != UNZ_OK)
            {
                printf("error %d with zipfile in unzGoToNextFile\n", err);
                break;
            }
        }
    }

    return 0;
}

int do_extract_onefile(
unzFile uf,
const char *filename,
int opt_extract_without_path,
int opt_overwrite,
const char *password)
{
    int err = UNZ_OK;
    if (unzLocateFile(uf, filename, CASESENSITIVITY) != UNZ_OK)
    {
        printf("file %s not found in the zipfile\n", filename);
        return 2;
    }

    if (do_extract_currentfile(uf, &opt_extract_without_path,
                               &opt_overwrite,
                               password) == UNZ_OK)
        return 0;
    else
        return 1;
}

int rt_unzip(const char *zipfilename, const char *dirname)
{
	const char *filename_to_extract = NULL;
	const char *password = NULL;
	int ret_value = 0;
	int opt_do_extract_withoutpath = 0;
	int opt_overwrite = 0;
	unzFile uf = NULL;

	do_banner();
	if (zipfilename != NULL)
	{
		uf = unzOpen64(zipfilename);
	}

	if (uf == NULL)
	{
		printf("Cannot open %s\n", zipfilename);
		return 1;
	}
	printf("%s opened\n", zipfilename);


	if (0==chdir(dirname))
	{
		if (filename_to_extract == NULL)
			ret_value = do_extract(uf, opt_do_extract_withoutpath, opt_overwrite, password);
		else
			ret_value = do_extract_onefile(uf, filename_to_extract, opt_do_extract_withoutpath, opt_overwrite, password);
	}
	else
	{
		printf("Error changing into %s, aborting\n", dirname);
		ret_value = 2;
	}

	unzClose(uf);

	return ret_value;
}



rt_err_t rt_unzip_main(int argc, char **argv)
{
    const char *zipfilename = NULL;
    const char *filename_to_extract = NULL;
    const char *password = NULL;
    char filename_try[MAXFILENAME + 16] = "";
    int i;
    int ret_value = 0;
    int opt_do_list = 0;
    int opt_do_extract = 1;
    int opt_do_extract_withoutpath = 0;
    int opt_overwrite = 0;
    int opt_extractdir = 0;
    const char *dirname = NULL;
    unzFile uf = NULL;

    do_banner();
    if (argc == 1)
    {
        do_help();
        return 0;
    }
    else
    {
        for (i = 1; i < argc; i++)
        {
            if ((*argv[i]) == '-')
            {
                const char *p = argv[i] + 1;

                while ((*p) != '\0')
                {
                    char c = *(p++);;
                    if ((c == 'l') || (c == 'L'))
                        opt_do_list = 1;
                    if ((c == 'v') || (c == 'V'))
                        opt_do_list = 1;
                    if ((c == 'x') || (c == 'X'))
                        opt_do_extract = 1;
                    if ((c == 'e') || (c == 'E'))
                        opt_do_extract = opt_do_extract_withoutpath = 1;
                    if ((c == 'o') || (c == 'O'))
                        opt_overwrite = 1;
                    if ((c == 'd') || (c == 'D'))
                    {
                        opt_extractdir = 1;
                        dirname = argv[i + 1];
                    }

                    if (((c == 'p') || (c == 'P')) && (i + 1 < argc))
                    {
                        password = argv[i + 1];
                        i++;
                    }
                }
            }
            else
            {
                if (zipfilename == NULL)
                    zipfilename = argv[i];
                else if ((filename_to_extract == NULL) && (!opt_extractdir))
                    filename_to_extract = argv[i] ;
            }
        }
    }

    if (zipfilename != NULL)
    {
        strncpy(filename_try, zipfilename, MAXFILENAME - 1);
        /* strncpy doesnt append the trailing NULL, of the string is too long. */
        filename_try[ MAXFILENAME ] = '\0';

        uf = unzOpen64(zipfilename);
        if (uf == NULL)
        {
            strcat(filename_try, ".zip");

            uf = unzOpen64(filename_try);
        }
    }

    if (uf == NULL)
    {
        printf("Cannot open %s or %s.zip\n", zipfilename, zipfilename);
        return 1;
    }
    printf("%s opened\n", filename_try);

    if (opt_do_list == 1)
        ret_value = do_list(uf);
    else if (opt_do_extract == 1)
    {
        if (opt_extractdir && chdir(dirname))
        {
            printf("Error changing into %s, aborting\n", dirname);
            exit(-1);
        }

        if (filename_to_extract == NULL)
            ret_value = do_extract(uf, opt_do_extract_withoutpath, opt_overwrite, password);
        else
            ret_value = do_extract_onefile(uf, filename_to_extract, opt_do_extract_withoutpath, opt_overwrite, password);
    }

    unzClose(uf);

    return ret_value;
}


MSH_CMD_EXPORT(rt_unzip_main, rt_unzip_main);


