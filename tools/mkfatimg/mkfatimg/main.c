/*--------------------------------------------------------/
/  FAT image creator R0.02               (C)ChaN, 2017
/--------------------------------------------------------*/

#include <windows.h>
#include <stdio.h>
#include "ff.h"
#include "diskio.h"


#if FF_MIN_SS != FF_MAX_SS
#error Sector size must be fixed at any value
#endif

#define MIN_FAT16	4086U	/* Minimum number of clusters for FAT16 */
#define	MIN_FAT32	65526U	/* Minimum number of clusters for FAT32 */

#define BPB_NumFATs			16		/* Number of FAT copies (1) */
#define BPB_RootEntCnt		17		/* Number of root directory entries for FAT12/16 (2) */
#define BPB_TotSec16		19		/* Volume size [sector] (2) */
#define BPB_FATSz16			22		/* FAT size [sector] (2) */
#define BPB_TotSec32		32		/* Volume size [sector] (4) */
#define BPB_FATSz32			36		/* FAT size [sector] (4) */

/* External functions (ff.c) */
extern DWORD get_fat (FFOBJID* obj, DWORD);		/* Read an FAT item */
extern DWORD ld_dword (const BYTE* ptr);		/* Load a 4-byte little-endian word */
extern WORD ld_word (const BYTE* ptr);			/* Load a 2-byte little-endian word */
extern void st_word (BYTE* ptr, WORD val);		/* Store a 2-byte word in little-endian */
extern void st_dword (BYTE* ptr, DWORD val);	/* Store a 4-byte word in little-endian */


BYTE *RamDisk;		/* Poiter to the RAM disk */
DWORD RamDiskSize;	/* Size of RAM disk in unit of sector */

int FF_MIN_SS;
int reserved_size;

static FATFS FatFs;
static FIL DstFile;
static HANDLE SrcFile;
static WIN32_FIND_DATAW Fd;
static char SrcPathA[512], DstPath[512];
static BYTE Buff[8192];
static UINT Dirs, Files;
static int remaining_size;


int maketree (void)
{
	HANDLE hdir;
	int slen, dlen, rv = 0;
	DWORD br;
	UINT bw;
    DWORD file_size;
	int unicodeLen;
	int len;


	slen = strlen(SrcPathA);
	dlen = strlen(DstPath);
	sprintf(&SrcPathA[slen], "/*");

	unicodeLen = MultiByteToWideChar(CP_ACP, 0, SrcPathA, -1, NULL, 0);
	wchar_t *SrcPath = (wchar_t*)malloc((unicodeLen + 1) * sizeof(wchar_t));
	memset(SrcPath, 0, (unicodeLen + 1) * sizeof(wchar_t));
	MultiByteToWideChar(CP_ACP, 0, SrcPathA, -1, (LPWSTR)SrcPath, unicodeLen);

	hdir = FindFirstFileW(SrcPath, &Fd);		/* Open directory */
	if (hdir == INVALID_HANDLE_VALUE) {
		printf("Failed to open the source directory.\n");
	} else {
		for (;;) {

			len = WideCharToMultiByte(CP_UTF8, 0, Fd.cFileName, -1, NULL, 0, NULL, NULL);
			char *szUtf8 = (char*)malloc(len + 1);
		
			memset(szUtf8, 0, len + 1);
			WideCharToMultiByte(CP_UTF8, 0, Fd.cFileName, -1, szUtf8, len, NULL, NULL);
			memcpy(SrcPathA + slen, "/", 1);
			memcpy(SrcPathA + slen+1, szUtf8, len);
			memcpy(DstPath + dlen, "/", 1);
			memcpy(DstPath + dlen+1, szUtf8, len);

			//sprintf(&SrcPathA[slen], "/%s", Fd.cFileName);
			//sprintf(&DstPath[dlen], "/%s", Fd.cFileName);
			unicodeLen = MultiByteToWideChar(CP_UTF8, 0, SrcPathA, -1, NULL, 0);
			wchar_t *SrcFilePath = (wchar_t*)malloc((unicodeLen + 1) * sizeof(wchar_t));
			memset(SrcFilePath, 0, (unicodeLen + 1) * sizeof(wchar_t));
			MultiByteToWideChar(CP_UTF8, 0, SrcPathA, -1, (LPWSTR)SrcFilePath, unicodeLen);

			if (Fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {	/* The item is a directory */
				if (wcscmp(Fd.cFileName, L".") && wcscmp(Fd.cFileName, L"..")) 
				{
					if (f_mkdir(DstPath)) {	/* Create destination directory */
						printf("Failed to create directory.\n"); break;
					}
					if (!maketree()) break;	/* Enter the directory */
					Dirs++;
				}
			} else {	/* The item is a file */
				//wprintf(L"%s\n", SrcPath);

				if ((SrcFile = CreateFileW(SrcFilePath, GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0)) == INVALID_HANDLE_VALUE) 
				{	/* Open source file */
					printf("Failed to open source file.\n"); break;
				}

				FRESULT r = f_open(&DstFile, DstPath, FA_CREATE_ALWAYS | FA_WRITE);
				if (r) {	/* Create destination file */
					printf("Failed to create destination file: %d.\n", r); break;
				}
				do {	/* Copy source file to destination file */
					ReadFile(SrcFile, Buff, sizeof Buff, &br, 0);
					if (br == 0) break;
					f_write(&DstFile, Buff, (UINT)br, &bw);
				} while (br == bw);
                file_size = GetFileSize(SrcFile, NULL);
				CloseHandle(SrcFile);
				f_close(&DstFile);
				if (br && br != bw) {
					printf("Failed to write file:%s.\n", DstPath); break;
				}
                if (remaining_size >= file_size)
                {
                    //printf("remaining:%d,%d\n", remaining_size, file_size);
                    remaining_size -= file_size;
                }
				Files++;
			}
			free(SrcFilePath);
			free(szUtf8);
			if (!FindNextFileW(hdir, &Fd)) 
			{
				rv = 1; break;
			}
		}
		FindClose(hdir);
	}
	free(SrcPath);
	SrcPathA[slen] = 0;
	DstPath[dlen] = 0;
	return rv;
}


int calc_dir_size(void)
{
	HANDLE hdir;
	int slen = 0;
	int total_size = 0;
	int unicodeLen;
	int len;

	slen = strlen(SrcPathA);
	sprintf(&SrcPathA[slen], "/*");

	unicodeLen = MultiByteToWideChar(CP_ACP, 0, SrcPathA, -1, NULL, 0);
	wchar_t *SrcPath = (wchar_t*)malloc((unicodeLen + 1) * sizeof(wchar_t));
	memset(SrcPath, 0, (unicodeLen + 1) * sizeof(wchar_t));
	MultiByteToWideChar(CP_ACP, 0, SrcPathA, -1, (LPWSTR)SrcPath, unicodeLen);

	hdir = FindFirstFileW(SrcPath, &Fd);		/* Open directory */
	if (hdir == INVALID_HANDLE_VALUE) {
		printf("Failed to open the source directory.\n");
	}
	else {
		for (;;) {
			//sprintf(&SrcPathA[slen], "/%s", Fd.cFileName);
			len = WideCharToMultiByte(CP_UTF8, 0, Fd.cFileName, -1, NULL, 0, NULL, NULL);
			char *szUtf8 = (char*)malloc(len + 1);
			memset(szUtf8, 0, len + 1);
			WideCharToMultiByte(CP_UTF8, 0, Fd.cFileName, -1, szUtf8, len, NULL, NULL);
			memcpy(SrcPathA + slen, "/", 1);
			memcpy(SrcPathA + slen + 1, szUtf8, len);

			unicodeLen = MultiByteToWideChar(CP_UTF8, 0, SrcPathA, -1, NULL, 0);
			wchar_t *SrcFilePath = (wchar_t*)malloc((unicodeLen + 1) * sizeof(wchar_t));
			memset(SrcFilePath, 0, (unicodeLen + 1) * sizeof(wchar_t));
			MultiByteToWideChar(CP_UTF8, 0, SrcPathA, -1, (LPWSTR)SrcFilePath, unicodeLen);

			if (Fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {	/* The item is a directory */
				if (wcscmp(Fd.cFileName, L".") && wcscmp(Fd.cFileName, L"..")) 
				{
					total_size += calc_dir_size();
					Dirs++;
				}
			}
			else {	/* The item is a file */
				//printf("%s\n", SrcPathA);
				if ((SrcFile = CreateFileW(SrcFilePath, GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0)) == INVALID_HANDLE_VALUE) 
				{	/* Open source file */
					printf("Failed to open source file.\n"); break;
				}

				total_size += GetFileSize(SrcFile, NULL);

				CloseHandle(SrcFile);
				f_close(&DstFile);
				Files++;
			}
			free(SrcFilePath);
			free(szUtf8);
			if (!FindNextFileW(hdir, &Fd)) 
			{
				break;
			}
		}
		FindClose(hdir);
	}
	free(SrcPath);
	SrcPathA[slen] = 0;
	return total_size;
}

int main (int argc, char* argv[])
{
	UINT csz;
	HANDLE wh;
	DWORD wb, szvol;
	DIR dir;
	int ai = 1, truncation = 0;
	const char *outfile;
	int total_size;
	DWORD free_clust, free_sect, total_sect;
	FATFS *fs;

	/* the first argument is program name */
	argc--;
	/* Initialize option parameters */
	while (argc >= 1 && *argv[ai] == '-') {
		if (!strcmp(argv[ai], "-t")) {
			truncation = 1;
			ai++;
			argc--;
        }
		else if (!strcmp(argv[ai], "-rsv")) {
			if (argc < 2) {
				argc = 0;
				break;
			}
			reserved_size = atoi(argv[ai + 1]) * 1024;
			if (reserved_size <= 0) {
				argc = 0;
				break;
			}
			ai += 2;
			argc -= 2;
		}
        else {
			argc = 0;
		}
	}

	if (argc < 3) {
		printf("usage: mkfatimg [-t] [-rsv rsv_sapce_size] <source node> <output image> <image size> [<cluster size>]\n"
				"    -t: Truncate unused area for read only volume.\n"
			    "    -rsv: reserved space size in KiB\n"
				"    <source node>: Source node as root of output image\n"
				"    <output image>: FAT volume image file\n"
				"    <image size>: Size of output image in unit of KiB\n"
				"    <cluster size>: Size of cluster in unit of byte (default:512)\n"
			);
		return 1;
	}
	strcpy(SrcPathA, argv[ai++]);
	outfile = argv[ai++];
	total_size = calc_dir_size();
	RamDiskSize = atoi(argv[ai++]);
	
	csz = (argc >= 4) ? atoi(argv[ai++]) : 512;
	FF_MIN_SS = csz;

	if (0 == RamDiskSize)
	{ 
		RamDiskSize = (total_size + csz - 1) / csz + (Dirs + Files) + 10; // add 1 sector for each entry
	}
	

	printf("disksize:%d  filesize:%d\n", RamDiskSize * FF_MIN_SS, total_size);
	printf("File Size Total: %.2f KB\n", (float)total_size / 1024);

	Dirs = 0;
	Files = 0;
	FRESULT r;

	/* Create an FAT volume (Supports only FAT/FAT32) */
	if (r = f_mkfs("", FM_FAT | FM_FAT32 | FM_SFD, 0, Buff, sizeof Buff)) {
		printf("Failed to create FAT volume. Adjust volume size or cluster size:%d.\n", r);
		return 2;
	}

	remaining_size = total_size;

	/* Copy source directory tree into the FAT volume */
	f_mount(&FatFs, "", 0);
	if (!maketree())
	{
		printf("Not packed: %.2f KB\n", (float)remaining_size / 1024);
		if (FR_OK == f_getfree("0:", &free_clust, &fs))
		{
			printf("Disk Total: %d KB, Free: %d KB\n", (fs->n_fatent - 2) * fs->csize * FF_MIN_SS / 1024,
				free_clust * fs->csize * FF_MIN_SS / 1024);
		}
		return 3;
	}

	if (FR_OK == f_getfree("0:", &free_clust, &fs))
	{
		printf("Disk Total: %d KB, Free: %d KB\n", (fs->n_fatent - 2) * fs->csize * FF_MIN_SS / 1024, 
			   free_clust * fs->csize * FF_MIN_SS / 1024);
		if ((free_clust * fs->csize * FF_MIN_SS) < reserved_size)
		{
			printf("Pack error: free size is less than reserved size: %d KB \n", reserved_size / 1024);
			return 4;
		}
	}
	if (!Files) { printf("No file in the source directory."); return 3; }
	szvol = ld_word(RamDisk + BPB_TotSec16);
	if (!szvol) szvol = ld_dword(RamDisk + BPB_TotSec32);

	if (truncation) {
		DWORD ent, nent;
		DWORD szf, szfp, edf, edfp;
		DWORD szd, szdp, edd, eddp;

		/* Truncate unused root directory entries */
		if (FatFs.fs_type != FS_FAT32) {
			printf("\nTruncating unused root directory area...");
			for (nent = ent = 0; ent < FatFs.n_rootdir; ent++) {	/* Get number of entries on the root dir */
				if (RamDisk[FatFs.dirbase * FF_MIN_SS + ent * 32]) nent = ent + 1;
			}
			szd = (nent + (FF_MIN_SS / 32 - 1)) / (FF_MIN_SS / 32);
			szdp = FatFs.n_rootdir / (FF_MIN_SS / 32);
			if (szd < szdp) {
				edd = FatFs.dirbase + szd;
				eddp = FatFs.database;
				MoveMemory(RamDisk + (edd * FF_MIN_SS), RamDisk + (eddp * FF_MIN_SS), (szvol - eddp) * FF_MIN_SS);
				szvol -= szdp - szd;
				FatFs.database -= szdp - szd;
				st_word(RamDisk + BPB_RootEntCnt, (WORD)(szd * (FF_MIN_SS / 32)));
			}
		}

		/* Truncate unused data area and FAT */
		printf("\nTruncating unused data area...");
		f_opendir(&dir, "");
		for (nent = ent = 2; ent < FatFs.n_fatent; ent++) {	/* Get number of used clusters */
			if (get_fat(&dir.obj, ent)) nent = ent + 1;
		}
		switch (FatFs.fs_type) {
		case FS_FAT12:
			szf = (nent * 3 / 2 + (nent & 1) + (FF_MIN_SS - 1)) / FF_MIN_SS;
			break;
		case FS_FAT16:
			szf = (nent * 2 + (FF_MIN_SS - 1)) / FF_MIN_SS;
			if (nent - 2 < MIN_FAT16) nent = 0;	/* Wrong cluster count for FAT16 */
			break;
		default:
			szf = (nent * 4 + (FF_MIN_SS - 1)) / FF_MIN_SS;
			if (nent - 2 < MIN_FAT32) nent = 0;	/* Wrong cluster count for FAT32 */
			break;
		}
		if (!nent) {
			printf("\nAnother FAT sub-type is requierd for truncation. Adjust volume size or cluster size.\n");
			return 3;
		}
		szfp = ld_word(RamDisk + BPB_FATSz16) * RamDisk[BPB_NumFATs];
		if (!szfp) szfp = ld_dword(RamDisk + BPB_FATSz32) * RamDisk[BPB_NumFATs];
		edf = FatFs.fatbase + szf;
		edfp = (FatFs.fs_type == FS_FAT32) ? FatFs.database : FatFs.dirbase;
		MoveMemory(RamDisk + (edf * FF_MIN_SS), RamDisk + (edfp * FF_MIN_SS), (szvol - edfp) * FF_MIN_SS);
		szvol -= (szfp - szf) + FatFs.csize * (FatFs.n_fatent - nent);
		if (FatFs.fs_type == FS_FAT32) {
			st_dword(RamDisk + BPB_FATSz32, szf);
		} else {
			st_word(RamDisk + BPB_FATSz16, (WORD)szf);
		}
		RamDisk[BPB_NumFATs] = 1;
		if (szvol < 0x10000) {
			st_word(RamDisk + BPB_TotSec16, (WORD)szvol);
			st_dword(RamDisk + BPB_TotSec32, 0);
		} else {
			st_word(RamDisk + BPB_TotSec16, 0);
			st_dword(RamDisk + BPB_TotSec32, szvol);
		}
	}

	/* Output the FAT volume to the file */
	printf("\nWriting output file...");
	wh = CreateFile(outfile, GENERIC_WRITE, 0, 0, CREATE_ALWAYS, 0, 0);
	if (wh == INVALID_HANDLE_VALUE) {
		printf("Failed to create output file.\n");
		return 4;
	}
	szvol *= FF_MIN_SS;
	WriteFile(wh, RamDisk, szvol, &wb, 0);
	CloseHandle(wh);
	if (szvol != wb) {
		DeleteFile(outfile);
		printf("Failed to write output file.\n");
		return 4;
	}

	printf("\n%u files and %u directories in the %uKiB of FAT volume.\n", Files, Dirs, szvol / 1024);

	return 0;
}