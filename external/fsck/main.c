/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (C) 1995 Wolfgang Solfrank
 * Copyright (c) 1995 Martin Husemann
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdarg.h>

#include "fsutil.h"
#include "ext.h"

int alwaysno;		/* assume "no" for all questions */
int alwaysyes;		/* assume "yes" for all questions */
int preen;		/* set when preening */
int rdonly;		/* device is opened read only (supersedes above) */
int skipclean;		/* skip clean file systems if preening */
int allow_mmap;		/* Allow the use of mmap(), if possible */
int issilent;

static void usage(void);

static void
usage(void)
{

	fprintf(stderr, "%s\n%s\n",
	    "usage: fsck_msdosfs -p [-f] filesystem ...",
	    "       fsck_msdosfs [-ny] filesystem ...");
	exit(1);
}

int     opterr = 1,             /* if error message should be printed */
optind = 1,             /* index into parent argv vector */
optopt,                 /* character checked for validity */
optreset;               /* reset getopt */
char* optarg;                /* argument associated with option */

#define BADCH   (int)'?'
#define BADARG  (int)':'
#define EMSG    ""

/*
* getopt --
*      Parse argc/argv argument vector.
*/
int
getopt(int nargc, char* const nargv[], const char* ostr)
{
	static char* place = EMSG;              /* option letter processing */
	const char* oli;                        /* option letter list index */

	if (optreset || !*place) {              /* update scanning pointer */
		optreset = 0;
		if (optind >= nargc || *(place = nargv[optind]) != '-') {
			place = EMSG;
			return (-1);
		}
		if (place[1] && *++place == '-') {      /* found "--" */
			++optind;
			place = EMSG;
			return (-1);
		}
	}                                       /* option letter okay? */
	if ((optopt = (int)*place++) == (int)':' ||
		!(oli = strchr(ostr, optopt))) {
		/*
		* if the user didn't specify '-' as an option,
		* assume it means -1.
		*/
		if (optopt == (int)'-')
			return (-1);
		if (!*place)
			++optind;
		if (opterr && *ostr != ':')
			(void)printf("illegal option -- %c\n", optopt);
		return (BADCH);
	}
	if (*++oli != ':') {                    /* don't need argument */
		optarg = NULL;
		if (!*place)
			++optind;
	}
	else {                                  /* need an argument */
		if (*place)                     /* no white space */
			optarg = place;
		else if (nargc <= ++optind) {   /* no arg */
			place = EMSG;
			if (*ostr == ':')
				return (BADARG);
			if (opterr)
				(void)printf("option requires an argument -- %c\n", optopt);
			return (BADCH);
		}
		else                            /* white space */
			optarg = nargv[optind];
		place = EMSG;
		++optind;
	}
	return (optopt);                        /* dump back option letter */
}

void init_opt_parser(void)
{
    opterr = 1;
    optind = 1;
    optopt = 0;
    optreset = 0;
    optarg = NULL;
}

#ifdef _WIN32
    int
    main(int argc, char **argv)
#else
    #include "rtthread.h"
    int cmd_fsck(int argc, char **argv);
    FINSH_FUNCTION_EXPORT_ALIAS(cmd_fsck, __cmd_fsck, FAT scan tool);
    int cmd_fsck(int argc, char **argv)
#endif
{
    int ret = 0, erg;
    int ch;

    skipclean = 1;
    allow_mmap = 1;
    for (int i = 0; i < argc; i++) {
        printf("argv:%s\n", argv[i]);
    }

    init_opt_parser();
    issilent = 0;
    preen = 0;
    while ((ch = getopt(argc, argv, "CfFnpyMs")) != -1) {
		switch (ch) {
		case 'C': /* for fsck_ffs compatibility */
			break;
		case 'f':
			skipclean = 0;
			break;
		case 'F':
			/*
			 * We can never run in the background.  We must exit
			 * silently with a nonzero exit code so that fsck(8)
			 * can probe our support for -F.  The exit code
			 * doesn't really matter, but we use an unusual one
			 * in case someone tries -F directly.  The -F flag
			 * is intentionally left out of the usage message.
			 */
			exit(5);
		case 'n':
			alwaysno = 1;
			alwaysyes = 0;
			break;
		case 'y':
			alwaysyes = 1;
			alwaysno = 0;
			break;

		case 'p':
			preen = 1;
			break;

		case 'M':
			allow_mmap = 0;
			break;
        case 's':
            issilent = 1;
            break;
        default:
            usage();
            break;
        }
	}
	argc -= optind;
	argv += optind;

	if (!argc)
		usage();

	while (--argc >= 0) {
		setcdevname(*argv, preen);
		erg = checkfilesys(*argv++);
		if (erg > ret)
			ret = erg;
	}

	return ret;
}


/*VARARGS*/
int
ask(int def, const char *fmt, ...)
{
	va_list ap;

    static char prompt[256];
    int c;

	if (alwaysyes || alwaysno || rdonly)
		def = (alwaysyes && !rdonly && !alwaysno);

    if (issilent)
        return def;

	if (preen) {
		if (def)
			printf("FIXED\n");
		return def;
	}

	va_start(ap, fmt);
	vsnprintf(prompt, sizeof(prompt), fmt, ap);
	va_end(ap);
	if (alwaysyes || alwaysno || rdonly) {
		printf("%s? %s\n", prompt, def ? "yes" : "no");
		return def;
	}
	do {
		printf("%s? [yn] ", prompt);
#ifdef _WIN32
		fflush(stdout);
#endif /* _WIN32 */
		c = getchar();
		while (c != '\n' && getchar() != '\n')
			if (feof(stdin))
				return 0;
	} while (c != 'y' && c != 'Y' && c != 'n' && c != 'N');
	return c == 'y' || c == 'Y';
}
