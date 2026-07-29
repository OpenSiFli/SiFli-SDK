/*
 * antiword_pc_compat.c
 *
 * Compatibility helpers for building Antiword in the PC simulator with MSVC.
 */

#include "antiword.h"

#if defined(__dos)
/*
 * The simulator defines __dos to let upstream Antiword use its bundled getopt
 * and stricmp paths instead of POSIX unistd/strcasecmp.  That also enables a
 * DOS code path in misc.c which asks dos.c for the active codepage.  The real
 * DOS implementation uses <dos.h> interrupts and cannot be built by MSVC, so
 * provide a simulator-safe replacement.
 */
int
iGetCodepage(void)
{
	/* Chinese Windows simulator default.  Antiword normalizes this to cp936. */
	return 936;
}
#endif /* __dos */
