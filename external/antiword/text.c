/*
 * text.c
 * Copyright (C) 1999-2004 A.J. van Os; Released under GNU GPL
 *
 * Description:
 * Functions to deal with the Text format
 *
 */

#include <string.h>
#include "antiword.h"

static antiword_text_cb_t pTextOutputCallback = NULL;
static void *pTextOutputCallbackUser = NULL;

void
vSetTextOutputCallback(antiword_text_cb_t pCallback, void *pUser)
{
	pTextOutputCallback = pCallback;
	pTextOutputCallbackUser = pUser;
}

static void
vWriteTextOutput(FILE *pFile, const char *szString, size_t tStringLength)
{
	if (szString == NULL || tStringLength == 0) {
		return;
	}
	if (pTextOutputCallback != NULL) {
		(void)pTextOutputCallback(szString, tStringLength, pTextOutputCallbackUser);
		return;
	}
	if (pFile != NULL) {
		(void)fprintf(pFile, "%.*s", (int)tStringLength, szString);
	}
}

static void
vPutTextOutput(FILE *pFile, int iChar)
{
	char cChar = (char)iChar;

	if (pTextOutputCallback != NULL) {
		(void)pTextOutputCallback(&cChar, 1, pTextOutputCallbackUser);
		return;
	}
	if (pFile != NULL) {
		(void)putc(iChar, pFile);
	}
}

/* The character set */
static encoding_type	eEncoding = encoding_neutral;
/* Current vertical position information */
static long		lYtopCurr = 0;
/* Local representation of the non-breaking space */
static UCHAR		ucNbsp = 0;


/*
 * vPrologueTXT - set options and perform the Text initialization
 */
void
vPrologueTXT(diagram_type *pDiag, const options_type *pOptions)
{
	fail(pDiag == NULL);
	fail(pOptions == NULL);

	eEncoding = pOptions->eEncoding;
	pDiag->lXleft = 0;
	pDiag->lYtop = 0;
	lYtopCurr = 0;
} /* end of vPrologueTXT */

/*
 * vEpilogueTXT - clean up after everything is done
 */
void
vEpilogueTXT(FILE *pOutFile)
{
	fail(pOutFile == NULL);

	vWriteTextOutput(pOutFile, "\n", 1);
} /* end of vEpilogueTXT */

/*
 * vPrintTXT - print a Text string
 */
static void
vPrintTXT(FILE *pFile, const char *szString, size_t tStringLength)
{
	const UCHAR	*ucBytes;
	size_t		tCount;

	fail(szString == NULL);

	if (szString == NULL || szString[0] == '\0' || tStringLength == 0) {
		return;
	}

	if (eEncoding == encoding_utf_8) {
		vWriteTextOutput(pFile, szString, tStringLength);
		return;
	}

	if (ucNbsp == 0) {
		ucNbsp = ucGetNbspCharacter();
		DBG_HEX_C(ucNbsp != 0xa0, ucNbsp);
	}

	ucBytes = (UCHAR *)szString;
	for (tCount = 0; tCount < tStringLength ; tCount++) {
		if (ucBytes[tCount] == ucNbsp) {
			vPutTextOutput(pFile, ' ');
		} else {
			vPutTextOutput(pFile, szString[tCount]);
		}
	}
} /* end of vPrintTXT */

/*
 * vMoveTo - move to the given X,Y coordinates
 *
 * Move the current position of the given diagram to its X,Y coordinates,
 * start on a new page if needed
 */
static void
vMoveTo(diagram_type *pDiag)
{
	int	iCount, iNbr;

	fail(pDiag == NULL);
	fail(pDiag->pOutFile == NULL);

	if (pDiag->lYtop != lYtopCurr) {
		iNbr = iDrawUnits2Char(pDiag->lXleft);
		for (iCount = 0; iCount < iNbr; iCount++) {
			vPutTextOutput(pDiag->pOutFile, FILLER_CHAR);
		}
		lYtopCurr = pDiag->lYtop;
	}
} /* end of vMoveTo */

/*
 * vMove2NextLineTXT - move to the next line
 */
void
vMove2NextLineTXT(diagram_type *pDiag)
{
	fail(pDiag == NULL);
	fail(pDiag->pOutFile == NULL);

	pDiag->lYtop++;
	vWriteTextOutput(pDiag->pOutFile, "\n", 1);
} /* end of vMove2NextLineTXT */

/*
 * vSubstringTXT - print a sub string
 */
void
vSubstringTXT(diagram_type *pDiag,
	const char *szString, size_t tStringLength, long lStringWidth)
{
	fail(pDiag == NULL || szString == NULL);
	fail(pDiag->pOutFile == NULL);
	fail(pDiag->lXleft < 0);
	fail(tStringLength != strlen(szString));

	if (szString[0] == '\0' || tStringLength == 0) {
		return;
	}

	vMoveTo(pDiag);
	vPrintTXT(pDiag->pOutFile, szString, tStringLength);
	pDiag->lXleft += lStringWidth;
} /* end of vSubstringTXT */

/*
 * Create an start of paragraph by moving the y-top mark
 */
void
vStartOfParagraphTXT(diagram_type *pDiag, long lBeforeIndentation)
{
	fail(pDiag == NULL);
	fail(lBeforeIndentation < 0);

	if (lBeforeIndentation >= lTwips2MilliPoints(HEADING_GAP)) {
		/* A large gap is replaced by an empty line */
		vMove2NextLineTXT(pDiag);
	}
} /* end of vStartOfParagraphTXT */

/*
 * Create an end of paragraph by moving the y-top mark
 */
void
vEndOfParagraphTXT(diagram_type *pDiag, long lAfterIndentation)
{
	fail(pDiag == NULL);
	fail(pDiag->pOutFile == NULL);
	fail(lAfterIndentation < 0);

	if (pDiag->lXleft > 0) {
		/* To the start of the line */
		vMove2NextLineTXT(pDiag);
	}

	if (lAfterIndentation >= lTwips2MilliPoints(HEADING_GAP)) {
		/* A large gap is replaced by an empty line */
		vMove2NextLineTXT(pDiag);
	}
} /* end of vEndOfParagraphTXT */

/*
 * Create an end of page
 */
void
vEndOfPageTXT(diagram_type *pDiag, long lAfterIndentation)
{
	vEndOfParagraphTXT(pDiag, lAfterIndentation);
} /* end of vEndOfPageTXT */
