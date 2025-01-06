/*!
 *  Insert the doxygen style comments of file.
 */
macro GDoxyFileHeaderComment()
{
    var hwnd
    var hbuf
    /*prepare*/
    hwnd = GetCurrentWnd()
    hbuf = GetCurrentBuf()
    if(hbuf == hNil || hwnd == hNil)
    {
        Msg("Can't open file")
        stop
    }


    /*Get need information*/
    var fFullName
    var fName
    fFullName = GetBufName(hbuf)
    fName = GGetFileName(fFullName)


    var szTime
    var Year
    var Month
    var Day
    szTime = GetSysTime(1)
    Year   = szTime.Year
    Month  = szTime.Month
    Day    = szTime.Day


    var user
    var siInfo
    siInfo = GetProgramEnvironmentInfo()
    user   = siInfo.UserName


    /*Insert comments*/
    ln = 0 //this will cause the file comments will always stay on the top of file
    InsBufLine(hbuf, ln++, "/**")
    InsBufLine(hbuf, ln++, "  ******************************************************************************")
    InsBufLine(hbuf, ln++, "  * @@file   @fName@")
    InsBufLine(hbuf, ln++, "  * @@author Sifli software development team")
    InsBufLine(hbuf, ln++, "  ******************************************************************************")
    InsBufLine(hbuf, ln++, "*/")
    InsBufLine(hbuf, ln++, "/**")
    InsBufLine(hbuf, ln++, " * @@attention")
    InsBufLine(hbuf, ln++, " * Copyright (c) 2019 - 2022,  Sifli Technology")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * All rights reserved.")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * Redistribution and use in source and binary forms, with or without modification,")
    InsBufLine(hbuf, ln++, " * are permitted provided that the following conditions are met:")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * 1. Redistributions of source code must retain the above copyright notice, this")
    InsBufLine(hbuf, ln++, " *    list of conditions and the following disclaimer.")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit")
    InsBufLine(hbuf, ln++, " *    in a product or a software update for such product, must reproduce the above")
    InsBufLine(hbuf, ln++, " *    copyright notice, this list of conditions and the following disclaimer in the")
    InsBufLine(hbuf, ln++, " *    documentation and/or other materials provided with the distribution.")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse")
    InsBufLine(hbuf, ln++, " *    or promote products derived from this software without specific prior written permission.")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * 4. This software, with or without modification, must only be used with a")
    InsBufLine(hbuf, ln++, " *    Sifli integrated circuit.")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * 5. Any software provided in binary form under this license must not be reverse")
    InsBufLine(hbuf, ln++, " *    engineered, decompiled, modified and/or disassembled.")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY \"AS IS\" AND ANY EXPRESS")
    InsBufLine(hbuf, ln++, " * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES")
    InsBufLine(hbuf, ln++, " * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE")
    InsBufLine(hbuf, ln++, " * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE")
    InsBufLine(hbuf, ln++, " * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR")
    InsBufLine(hbuf, ln++, " * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE")
    InsBufLine(hbuf, ln++, " * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)")
    InsBufLine(hbuf, ln++, " * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT")
    InsBufLine(hbuf, ln++, " * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT")
    InsBufLine(hbuf, ln++, " * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.")
    InsBufLine(hbuf, ln++, " *")
    InsBufLine(hbuf, ln++, " */")

    /*Locate to the file begin*/
    ScrollWndToLine(hwnd, 0)
}


/*!
 *  Insert doxygen style comments of c++ classes.
 */
macro GDoxyClassHeaderComment()
{
    var hwnd
    var hbuf
    var ln
    var symbolrecord
    var name
    var strHeader


    /*prepare*/
    hwnd = GetCurrentWnd()
    hbuf = GetCurrentBuf()
    if(hwnd == hNil || hwnd == hNil)
    {
        Msg("Can't open file")
        stop
    }


    ln = GetBufLnCur(hbuf)
    symbolrecord = GetSymbolLocationFromLn(hbuf, ln)
    if(symbolrecord == Nil)
    {
        Msg("Can't get current symbol record info.")
        stop
    }


    /*check current symbol type*/
    if(symbolrecord.Type != "Class")
    {
        Msg("Current symbol is not a class.")
        stop
    }


    /*Get need info*/
    name = symbolrecord.Symbol


    /*Insert comments*/
    ln = symbolrecord.lnFirst
    strHeader = GGetHeaderSpaceByLn(hbuf, ln)
    InsBufLine(hbuf, ln++, strHeader#"/*!")
    InsBufLine(hbuf, ln++, strHeader#" * @@class @name@")
    InsBufLine(hbuf, ln++, strHeader#" * @@brief")
    InsBufLine(hbuf, ln++, strHeader#" *")
    InsBufLine(hbuf, ln++, strHeader#" * \\n")
    InsBufLine(hbuf, ln++, strHeader#" * @@detail")
    InsBufLine(hbuf, ln++, strHeader#" *")
    InsBufLine(hbuf, ln++, strHeader#" * \\n")
    InsBufLine(hbuf, ln++, strHeader#" */")


    /*Relocate window*/
    ScrollWndToLine(hwnd, symbolrecord.lnFirst)
}


/*!
 *  Insert the doxygen style comments of function.
 */
macro GDoxyFunctionComment()
{
    var hWnd
    var hBuf
    var ln
    var symbolrecord
    var strHeader


    var locateLn  //locate info after insert comments
    var locateCur //locate info after insert comments


    /*prepare*/
    hWnd = GetCurrentWnd()
    hBuf = GetCurrentBuf()
    if(hBuf == hNil || hWnd == hNil)
    {
        Msg("Can't open the file")
        stop
    }


    ln = GetBufLnCur(hBuf)
    symbolrecord = GetSymbolLocationFromLn(hBuf, ln)
    if(symbolrecord == Nil)
    {
        Msg("Can't get current symbol record info.")
        stop
    }


    /*check current symbol type*/
    var type
    type = symbolrecord.Type
    if(type != "Function" && type != "Function Prototype" &&
       type != "Method"   && type != "Method Prototype")
    {
        Msg("Current symbol is not a function.")
        stop
    }


    /*Get need information*/
    ln = symbolrecord.lnFirst
    locateLn = ln + 1 //locate info after insert comments
    strHeader = GGetHeaderSpaceByLn(hBuf, ln)//align with the current function


    //analysis function
    var name
    var type


    var nChild
    var listChild
    var listParam
    var nParam
	var Paramsym
	var idxParam

	//sort listChild symbols as it's occurrence, and save in listParam
    listChild = SymbolChildren(symbolrecord)
    nChild    = SymListCount(listChild)
    listParam = SymListNew()
    if(nChild != invalid)
    {
        var idxChildList
        var childsym
        idxChildList = 0
        while(idxChildList < nChild)
        {
			idxParam = 0
			
            childsym = SymListItem(listChild, idxChildList)

			nParam = SymListCount(listParam)
			while(idxParam < nParam)
			{
	            Paramsym = SymListItem(listParam, idxParam)
				if(Paramsym.ichName > childsym.ichName)
				    break
				else
					idxParam++
			}

			if(idxParam == nParam)
				SymListInsert (listParam, invalid, childsym)
			else
				SymListInsert (listParam, idxParam, childsym)

            idxChildList++
        }
    }
    SymListFree(listChild)


    /*Insert comments*/
    InsBufLine(hBuf, ln++, strHeader#"/**")
    InsBufLine(hBuf, ln++, strHeader#" * @@brief ")//Function description
    locateCur = strlen(strHeader#" * @@brief ")//locate info after insert comments
    //InsBufLine(hBuf, ln++, strHeader#" * \\n")
    //InsBufLine(hBuf, ln++, strHeader#" *")


	nParam = SymListCount(listParam)

    idxParam = 0
    while(idxParam < nParam)
    {
		Paramsym = SymListItem(listParam, idxParam)
        type = Paramsym.type
        name = GGetSymExactName(Paramsym.Symbol)

		//function param
        if(type == "Parameter")
        {
            InsBufLine(hBuf, ln++, strHeader#" * @@param @name@ - ")//[in/out]
        }
        ++idxParam
    }

    idxParam = 0
    while(idxParam < nParam)
    {
		Paramsym = SymListItem(listParam, idxParam)
        type = Paramsym.type
        name = GGetSymExactName(Paramsym.Symbol)

        if(type == "Type Reference" && name != "void")
        {
            InsBufLine(hBuf, ln++, strHeader#" * @@return")
            //InsBufLine(hBuf, ln++, strHeader#" * @@retval value description")//different style
        }
        ++idxParam
    }

    SymListFree(listParam)

    //InsBufLine(hBuf, ln++, strHeader#" * \\n")
    //InsBufLine(hBuf, ln++, strHeader#" * @@see")
    InsBufLine(hBuf, ln++, strHeader#" */")


    //locate the cursor
    SetBufIns(hBuf, locateLn, locateCur)
}



/*!
 *  Insert example codes after the cursor line with doxygen style in the block
 *comments.
 */
macro GDoxyInsExampleCodes()
{
    var hbuf
    hbuf = GetCurrentBuf()
    if(hbuf == hNil)
    {
        Msg("Current file handler is invalid.")
        stop
    }


    var lnCursor
    lnCursor = GetBufLnCur(hbuf)


    var strHead
    strHead = GGetHeaderSpaceByLn(hbuf, lnCursor)


    lnCursor++  ///<after the line of the cursor to insert the codes
    InsBufLine(hbuf, lnCursor++, strHead#"*")
    InsBufLine(hbuf, lnCursor++, strHead#"* @@code")


    ///reserve four line to insert example codes
    var locateLn
    var locateCol
    locateLn  = lnCursor
    locateCol = strlen(strHead#"*")
    InsBufLine(hbuf, lnCursor++, strHead#"*")
    InsBufLine(hbuf, lnCursor++, strHead#"*")
    InsBufLine(hbuf, lnCursor++, strHead#"*")
    InsBufLine(hbuf, lnCursor++, strHead#"*")


    InsBufLine(hbuf, lnCursor++, strHead#"* @@endcode")


    //locate cursor
    SetBufIns(hbuf, locateLn, locateCol)
}


/*!
 *Insert a doxygen common comments before the line of cursor.
 */
macro GDoxyInsBlockComment()
{
    var hbuf
    hbuf = GetCurrentBuf()
    if(hbuf == hnil)
    {
        msg("Current file handler is invalid.")
        stop
    }


    var lnCursor
    lnCursor = GetBufLnCur(hbuf)


    var strHeader
    var lnIter
    lnIter = lnCursor
    strHeader = GGetHeaderSpaceByLn(hbuf, lnCursor)


    InsBufLine(hbuf, lnIter++, strHeader#"/**")
    InsBufLine(hbuf, lnIter++, strHeader#" *")
    InsBufLine(hbuf, lnIter++, strHeader#" *")
    InsBufLine(hbuf, lnIter++, strHeader#" */")


    SetBufIns(hbuf, lnCursor + 1, strlen(strHeader#" *"))
}


/*!
 *  Insert the doxygen style comments of enumeration.
 * @note
 *     Make sure the cursor in the enumeration region before call this macro.
 */
macro GDoxyEnumComment()
{
    var hbuf
    hbuf = GetCurrentBuf()
    if(hbuf == hNil)
    {
        Msg("Current file handler is invalid.")
        stop
    }


    var lnCursor
    lnCursor = GetBufLnCur(hbuf)


    var eSymbol
    eSymbol = GGetEnumSymbol(hbuf, lnCursor)
    if(eSymbol == Nil)
    {
        Msg("The symbol at the cursor looks like not a enumeration.")
        stop
    }


    var strHeader
    var strTemp
    strHeader = GGetHeaderSpaceByLn(hbuf, eSymbol.lnFirst)


    //reset the symbol's first and last line text
    strTemp = GetBufLine(hbuf, eSymbol.lnFirst)
    idxSearch = GStrStr(strTemp, "{")
    strTemp = strmid(strTemp, 0, idxSearch + 1)
    PutBufLine(hbuf, eSymbol.lnFirst, strTemp)
    strTemp = GetBufLine(hbuf, eSymbol.lnLast)
    idxSearch = GStrStr(strTemp, "}")
    strTemp = strmid(strTemp, idxSearch, strlen(strTemp))
    PutBufLine(hbuf, eSymbol.lnLast, strTemp)


    //delete the lines text before
    var lnIter
    lnIter = eSymbol.lnLast - 1
    while(lnIter > eSymbol.lnFirst)
    {
        DelBufLine(hbuf, lnIter)
        lnIter--
    }


    //rewrite members and insert comments
    var nMembers
    nMembers = 0
    lnIter   = eSymbol.lnFirst + 1
    while(nMembers < eSymbol.count)
    {
        strTemp = GGetStringBySeparateCh(eSymbol.members, eSymbol.chSeparator, nMembers)
        if(!(GStrBeginWith(strTemp, "#if") || GStrBeginWith(strTemp, "#end") || GStrBeginWith("#else")))
        {
            strTemp = GStrAppendTailSpace(strTemp, eSymbol.maxMemberLen)
            strTemp = strHeader#"    "#strTemp#" ///!<"
        }
        InsBufLine(hbuf, lnIter, strTemp)


        nMembers++
        lnIter++
    }


    //locate cursor
    SetBufIns(hbuf, eSymbol.lnFirst + 1, strlen(strTemp))
}


macro GDoxyDefgroup()
{
	var gp_name
	var lnIter
    var locateLn  //locate info after insert comments
    var locateCur //locate info after insert comments

	gp_name = Ask("Enter group name:")
	if (gp_name != "")
	{
		gp_desc = Ask("Enter group description:")
		// Wrap around the current selection
		hwnd = GetCurrentWnd()
		hbuf = GetCurrentBuf()
		
		lnFirst = GetWndSelLnFirst(hwnd)
		lnLast = GetWndSelLnLast(hwnd)

	  	lnIter = lnFirst
		InsBufLine(hbuf, lnIter++, "/** @@defgroup @gp_name@ @gp_desc@")
		InsBufLine(hbuf, lnIter++, "  * @@brief ")
		locateLn = lnIter - 1
		locateCur = strlen("  * @@brief ")
		InsBufLine(hbuf, lnIter++, "  * @@{")
		InsBufLine(hbuf, lnIter++, "  */")

	  	lnIter = lnLast + lnIter - lnFirst + 1
		InsBufLine(hbuf, lnIter++, "/**")
		InsBufLine(hbuf, lnIter++, "  * @@}")
		InsBufLine(hbuf, lnIter++, "  */")

	    SetBufIns(hbuf, locateLn, locateCur)
	}
}

macro GDoxyAdd2group()
{
	var gp_name
	var lnIter
    var locateLn  //locate info after insert comments
    var locateCur //locate info after insert comments

	gp_name = Ask("Enter group name:")
	if (gp_name != "")
	{
		// Wrap around the current selection
		hwnd = GetCurrentWnd()
		lnFirst = GetWndSelLnFirst(hwnd)
		lnLast = GetWndSelLnLast(hwnd)
		 
		hbuf = GetCurrentBuf()
	  	lnIter = lnFirst
		InsBufLine(hbuf, lnIter++, "/** @@addtogroup @gp_name@")
		InsBufLine(hbuf, lnIter++, "  * @@{")
		InsBufLine(hbuf, lnIter++, "  */")

	  	lnIter = lnLast + lnIter - lnFirst + 1
		InsBufLine(hbuf, lnIter++, "/**")
		InsBufLine(hbuf, lnIter++, "  * @@}")
		InsBufLine(hbuf, lnIter++, "  */")
	}
}
/*
————————————————
版权声明：本文为CSDN博主「hard-water」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/dayong419/java/article/details/7932467
*/
