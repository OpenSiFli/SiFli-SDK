
///////////////////////////////////////////////////////////////////////////////
///  Tool macros under.These macros are offering service for the other macros.
///Note:It's not need to assign keys or menus for the macros under, even if you
///can do this operation.
///////////////////////////////////////////////////////////////////////////////


/*!
 *  Get the enumeration info,if the symbol at the cursor is not a enumeration,
 *it will return a Nil symbol recorder.
 *
 *construct an enumeration symbol recorder,it contains the fields:
 *  type         enum type name
 *  members      string of members which is separated by specail character, and the spaces will be deleted
 *  chSeparator  the separate character of members
 *  count        the number of the members
 *  lnFirst      symbol first line
 *  lnLast       symbol last line
 *  maxMemberLen the longest member's character number
 */
macro GGetEnumSymbol(hbuf, lnCursor)
{
    var eSymbol


var region
    region = GGetEnumRegion(hbuf, lnCursor)
    if(region == Nil)
    {//can't get enumeration symbol info
        return Nil
    }


    eSymbol.type        = nil
    eSymbol.members     = nil
    eSymbol.chSeparator = ";"
    eSymbol.count       = 0
    eSymbol.maxMemberLen= 0
    eSymbol.lnFirst     = region.first
    eSymbol.lnLast      = region.last


    //analysis codes
    var lenMember//the length of the enumeration's one member
    var lnIter   //line iterator from the enumeration's first line to the last
    var lnString //used to get the string text of one line
    var tempStr  //used to contains the member string
    var lenLnStr //the character num of the string of line
    var isComment//indicate whether need to skip the characters


    lnIter    = eSymbol.lnFirst
    isComment = false
    lenMember = 0
    while(lnIter <= eSymbol.lnLast)
    {
        tempStr  = nil
        lnString = Nil


        //get line text
    lnString = GetBufLine(hbuf, lnIter)
    lenLnStr = strlen(lnString)
    if(lenLnStr == 0)
    {
       lnIter++
       continue
    }


        //deal with the indicator
    lenLnStr = strlen(lnString)
    lIndicate= GStrStr(lnString, "{")
    if(lIndicate != invalid)
    {
       lIndicate++//skip "{" character
       lnString = strmid(lnString, lIndicate, lenLnStr)
    }
    rIndicate= GStrStr(lnString, "}")
    lenLnStr = strlen(lnString)
    if(rIndicate != invalid)
    {
            lnString = strmid(lnString, 0, rIndicate)
    }


        //prepare the pure string
    lnString = GStrTrimJustify(lnString)//GStrRemoveAllSpace(lnString)
    lenLnStr = strlen(lnString)
    if(lenLnStr == 0)
    {
       lnIter++
       continue
    }


        //analysising
    ich = 0
    while(ich < lenLnStr)
    {
            if(!isComment && lnString[ich] == "/" && lnString[ich+1] == "/")
            {//don't need to analysis the rest charaters in this line
                break
            }


            if(lnString[ich] == "/" && lnString[ich+1] == "*")
            {
                isComment = true
            }


            if(!isComment)
            {
                if(lnString[ich] == "," && tempStr != nil)
                {//it is possible that one line has several members
                    tempStr = GStrTrimJustify(tempStr)
                    tempStr = cat(tempStr, lnString[ich])
                    lenMember = strlen(tempStr)
                    if(lenMember != 0)
                    {
                        eSymbol.members = eSymbol.members#tempStr#";"
                        eSymbol.count   = eSymbol.count + 1
                        if(lenMember > eSymbol.maxMemberLen)
                        {
                            eSymbol.maxMemberLen = lenMember
                        }
                    }
                    tempStr = nil
                }
                else
                {
                    tempStr = cat(tempStr, lnString[ich])
                }
            }


            if(lnString[ich] == "*" && lnString[ich+1] == "/")
            {
                ich++
                isComment = false
            }
       ich++
    }


        tempStr = GStrTrimJustify(tempStr)
        lenMember = strlen(tempStr)
        if(lenMember != 0)
        {
            eSymbol.members = eSymbol.members#tempStr#";"
            eSymbol.count = eSymbol.count + 1
            if(lenMember > eSymbol.maxMemberLen)
            {
                eSymbol.maxMemberLen = lenMember
            }
        }


    lnIter++
    }


    return eSymbol
}


/*!
 *Get the enumeration region which contains the first line and last line.
 *The first line is the "{" appears line.
 *The last line is the "}" appears line.
 */
macro GGetEnumRegion(hbuf, lnCursor)
{
    //variables
    var region
    var tempStr
    var lnIter
    var iAppear
    var lnMax//file line count


    region  = Nil
    tempStr = Nil
    iAppear = invalid


    //first line
    lnIter  = lnCursor
    while(lnIter >= 0)
    {
        tempStr = GetBufLine(hbuf, lnIter)
        iAppear = GStrStr(tempStr, "{")
        if(iAppear != invalid)
        {
            region.first = lnIter
            break
        }
        lnIter--
    }


    if(lnIter < 0)
    {
        return Nil
    }


    //last line
    lnIter  = lnCursor
    iAppear = invalid
    lnMax   = GetBufLineCount(hbuf)
    while(lnIter < lnMax)
    {
        tempStr = GetBufLine(hbuf, lnIter)
        iAppear = GStrStr(tempStr, "}")
        if(iAppear != invalid)
        {
            region.last = lnIter
            break
        }
        lnIter++
    }


    if(lnIter == lnMax)
    {
        return Nil
    }


    return region
}


/*!
 *   Get a string from strParent which is separate by character.
 * @note
 *    The separate character that at the begining will be ignored.
 *
 *  Example 1:
 *    ch = ";"
 *    strParent = "first;second;third;"
 *    If index equals 0, it will return "first"
 *    If index equals 1, it will return "second"
 *    If index equals 2, it will return "third"
 *    If index equals 3, it will return ""
 *    ...
 *
 *  Example 2:
 *    ch = ";"
 *    strParent = ";first;second;third;"
 *    If index equals 0, it will return "first"
 *    If index equals 1, it will return "second"
 *    If index equals 2, it will return "third"
 *    ...
 */
macro GGetStringBySeparateCh(strParent, ch, index)
{
    /*variables*/
    var len
    var iChBeg
    var iChEnd
    var iLoop
    var countIdx
    var checkEndIdx


    /*codes*/
    len = strlen(strParent)
    if(0 == len)
    {
        return Nil
    }


    iChBeg = 0 //result begin index
    while(iChBeg < len)
    {//ignore all the characters ch that at the begin o strParent
        if(strParent[iChBeg] == ch)
        {
            iChBeg++
        }
        else
        {
            break
        }
    }
    iChEnd = iChBeg //result end index


    iLoop = iChBeg
    countIdx = 0
    checkEndIdx = False
    while(iLoop < len)
    {
        if(countIdx == index)
        {
            checkEndIdx = True
        }
        if(strParent[iLoop] == ch)
        {
            if(checkEndIdx)
            {//get end index
                iChEnd = iLoop
                break
            }
            else
            {//get begin index
                countIdx++
                iChBeg = iLoop + 1
            }
        }
        iLoop++
    }


    if(iChBeg == iChEnd)
    {
        if(index == 0)
        {
            return strParent
        }
        else
        {
            return Nil
        }
    }


    if(index != countIdx)
    {
        return Nil
    }


    return strmid(strParent, iChBeg, iChEnd)
}

/******************************************************************************
 * String macros definition
 *****************************************************************************/


/*!
 *   Insert spaces at the begining of string until the string's length reaches
 * lenMax.It will return string itself if the string's length is less then
 * lenMax.
 */
macro GStrInsertHeadSpace(string, lenMax)
{
    var len
    var nSpace
    var tempStr


    len    = strlen(string)
    nSpace = lenMax - len
    tempStr= nil
    while(nSpace > 0)
    {
        tempStr = cat(tempStr, " ")
        nSpace--
    }


    return cat(tempStr, string)
}


/*!
 *   Append spaces at the end of string until string's length reaches lenMax.It
 * will returns the string itself if the string's length is less then lenMax.
 */
macro GStrAppendTailSpace(string, lenMax)
{
    nStr = strlen(string)
    while(nStr < lenMax)
    {
        string = cat(string, " ")
        nStr++
    }


    return string
}


/*!
 *Test whether the string contains the character ch.
 */
macro GStrContainsCh(string, ch)
{
    var index
    index = GStrStr(string, ch)


    if(index == invalid)
    {
        return False
    }


    return True
}


/*!
 *  Returns the last place of appearance that the ch appears in string.
 *  If the ch is not appears in string, it will returns invalid(-1).
 */
macro GStrRFind(string, ch)
{
    var len
    var idx


    len = strlen(string)


    idx = len - 1
    while(idx >= 0)
    {
        if(string[idx] == ch)
        {
            break
        }
        idx--
    }


    return idx
}


/*!
 *   Test whether all the characters in the string are spaces.
 */
macro GStrAllSpace(string)
{
    len = strlen(string)
    iter = 0
    while(iter < len)
    {
        if(!GIsCharSpace(string[iter]))
        {
            return false
        }
        iter++
    }


    return true
}


/*!
 *Test if string is empty.
 * @param  string need to be tested string
 * @retval True   the string is empty
 * @retval False  the string is not empty
 */
macro GStrEmpty(string)
{
    var len


    len = strlen(string)
    if(len == 0)
    {
        return True
    }
    return False
}


/*!
 *Remove all the space and tab characters in the string.
 */
macro GStrRemoveAllSpace(string)
{
    var len
    var idx
    var str


    len = strlen(string)
    idx = 0
    str = Nil


    while(idx < len)
    {
        if(!GIsCharSpace(string[idx]))
        {
            str = cat(str, string[idx])
        }
        idx++
    }


    return str
}


/*!
 *Remove the space and tab characters at the begin and end of string.
 */
macro GStrTrimJustify(string)
{
    var len
    var index  //first not space character index
    var rIndex //last not space character index


    len = strlen(string)


    index = 0
    while(index < len)
    {
        if(GIsCharSpace(string[index]))
        {
            index++
        }
        else
        {
            break
        }
    }


    if(index == len)
    {
        return nil
    }


    rIndex = len - 1
    while(rIndex >= 0)
    {
        if(GIsCharSpace(string[rIndex]))
        {
            rIndex--
        }
        else
        {
            break
        }
    }


    rIndex++ //strmid function will not get the character at rIndex palce
    return strmid(string, index, rIndex)
}


/*!
 *   Remove the characters between the index iFirst and index iEnd from string.
 * The character at iFirst and iEnd will be deleted also.
 */
macro GStrRemoveStrByIndex(string, iFirst, iEnd)
{
    var len
    len = strlen(string)


    if(iFirst < 0 || iEnd < 0 || iEnd > len || iFirst > len || iEnd < iFirst)
    {
        return string
    }


    var temp
    temp = nil
    temp = strmid(string, 0, iFirst)
    temp = cat(temp, strmid(string, iEnd + 1, len))


    return temp
}


/*!
 *GStrStr
 *  Test whether the strTest is existing in strSrc.And returns
 *the place where the strTest first appears.Otherwise returns
 *-1.
 */
macro GStrStr(strSrc, strTest)
{
    var nSrc
    var nTest


    nSrc  = strlen(strSrc)
    nTest = strlen(strTest)


    if(nTest == 0)
    {
        return 0
    }
    else if(nSrc == 0)
    {
        return invalid
    }


    var iSrc
    var iTest
    iSrc  = 0
    iTest = 0
    while(iSrc < nSrc)
    {
        iTest = 0
        while(iTest < nTest)
        {
            if(strSrc[iSrc+iTest] == strTest[iTest])
            {
                iTest++
            }
            else
            {
                break
            }
        }
        if(iTest == nTest)
        {
            return iSrc
        }
        iSrc++
    }


    return invalid
}


/*!
 *GIsCharSpace
 *  Test whether the char ch is a space or a tab character.
 */
macro GIsCharSpace(ch)
{
    if(ch == Nil)
    {
        return False
    }


    if(AsciiFromChar(ch) == 9 || AsciiFromChar(ch) == 32)
    {
        return True
    }
    return False
}


/*!
 *Test whether the string is begin wiht string with.
 *note:
 *  it will ignore the first spaces
 */
macro GStrBeginWith(string, with)
{
string = GStrTrimJustify(string)
index  = GStrStr(string, with)


if(index == 0)
{
return true
}


return false
}






/******************************************************************************
 * File and buffer macros definition
 *****************************************************************************/


/*!
 *GGetHeaderSpaceByLn
 *  Get the spaces that at the begin of line in the hbuf file.
 *note
 *  Make sure the hbuf is a valid handler, and the line number is a
 *valid line for hbuf.
 */
macro GGetHeaderSpaceByLn(hbuf, line)
{
    var headSpace
    headSpace = ""


    var strContent
    var len
    strContent = GetBufLine(hbuf, line)
    len = strlen(strContent)


    var idx
    idx = 0
    while(idx < len)
    {
        if(GIsCharSpace(strContent[idx]))
        {
            headSpace = cat(headSpace, strContent[idx])
        }
        else
        {
            break
        }
        idx++
    }
    return headSpace
}


/*
 *GGetFileName
 *  Get file name from file full path.
 */
macro GGetFileName(fFullName)
{
    var nLength
    var fName


    nLength = strlen(fFullName)
    fName   = ""


    var i
    var ch
    i = nLength - 1
    while(i >= 0)
    {
        ch = fFullName[i]
        if("@ch@" == "\\")
        {
            i++ //don't take the '\' charater
            break
        }
        i--
    }
    fName = strmid(fFullName, i, nLength)
    return fName
}






/******************************************************************************
 * Symbol macros definition
 *****************************************************************************/


/*!
 *  If we get the symbol name by symbolrecord.Symbol, we will get the string
 *like "class.func, func.void", et.This function will get the last one name that
 *seperate by ".".
 *  @param[in] symbol is the Symbol Record's Symbol field
 *note
 *  see Symbol Record of SI macro language
 */
macro GGetSymExactName(symbol)
{
    var len
    var idxAppear
    var strRet


    len = strlen(symbol)
    idxAppear = GStrRFind(symbol, ".")


    if(idxAppear == invalid)
    {//can't get the expectant name, return the default one
        return symbol
    }


    idxAppear++
    strRet = strmid(symbol, idxAppear, len)
    return strRet
}

/*
————————————————
版权声明：本文为CSDN博主「hard-water」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/dayong419/java/article/details/7932467
*/

