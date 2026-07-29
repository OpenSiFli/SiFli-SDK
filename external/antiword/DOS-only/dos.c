/*
 * dos.c
 * Copyright (C) 2004 A.J. van Os; Released under GNU GPL
 *
 * Description:
 * DOS only functions
 */

#if defined(__DJGPP__)
#define _NAIVE_DOS_REGS 1
#endif /* __DJGPP__ */
#include <dos.h>
#include <string.h>
#include "antiword.h"


/*
 * iGetVersion - get the version of DOS
 *
 * Return the DOS version * 100 or -1 incase of error
 */
static int
iGetVersion(void)
{
	union REGS	uRegs;

	memset(&uRegs, 0, sizeof(uRegs));
	uRegs.h.ah = 0x30;
	uRegs.h.al = 0x00;
	_doserrno = 0;
	intdos(&uRegs, &uRegs);
	if (uRegs.x.cflag != 0) {
		DBG_DEC(uRegs.x.cflag);
		DBG_DEC(_doserrno);
		return -1;
	}
	DBG_DEC(uRegs.h.al);
	DBG_DEC(uRegs.h.ah);
	return uRegs.h.al * 100 + uRegs.h.ah;
} /* end of iGetVersion */

/*
 * iGetCodepage - get the DOS codepage
 *
 * Returns the number of the active codepage (cp437 is DOS ASCII)
 */
int
iGetCodepage(void)
{
	union REGS	uRegs;

	/* DOS function 0x66 first appeared in DOS 3.3 */
	if (iGetVersion() < 330) {
		return 437;
	}
	memset(&uRegs, 0, sizeof(uRegs));
	uRegs.h.ah = 0x66;
	uRegs.h.al = 0x01;
	_doserrno = 0;
	intdos(&uRegs, &uRegs);
	if (uRegs.x.cflag != 0) {
		DBG_DEC(uRegs.x.cflag);
		DBG_DEC(_doserrno);
		return 437;
	}
	DBG_DEC(uRegs.x.bx);
	DBG_DEC(uRegs.x.dx);
	return uRegs.x.bx;
} /* end of iGetCodepage */
