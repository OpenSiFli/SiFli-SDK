/******************************************************************************
 * self-defining array macros definition
 *
 *Note the under ideas:
 *  1.Since the source insight macro language is only support string variable,
 *so the array is implemented as string.
 *  2.The array string looks like string "item1;item2;item3".And the separator
 *character is ";" which can be specified by yourself.
 *  3.The array index is begin with 0.And the separator can't be a string, it
 *only can contains one character.
 *  4.The item can be a empty string, such as set array like this "item1;;item3",
 *and the second item is a empty string.
 *  5.When using this array type, please reset the array value when the array
 *changed no matter whatever the operation is.Because the source insight don't
 *support the output parameter.
 *  6.If you assign the array string by yourself, please make sure the end
 *character is separator.
 *  7.The character separator is not a part of item string.And it must be not
 *empty.Don't set it as the character that source insight not support in string.
 *****************************************************************************/




/*!
 *   Append an item at the back.If the separator is empty, it will do nothing.
 *   Returns the new array string.
 *
 * @code
 *   item = "itemN"
 *   separator = "?"
 *   array = GArrayAppendItem(array, item, separator)
 * @endcode
 */
macro GArrayAppendItem(array, item, separator)
{
    if(GStrEmpty(separator))
    {//separator is needed even though the arry or item is empty
        return array
    }


    return array#item#separator
}


/*!
 *   Insert an item at index place, and after this operation, the item at the
 * index will be the inserted item.
 *   If the index is larger then the array's count,the item will not be appended.
 *   If the index is a negative number or the separator is empty, this macro
 * will do nothing.
 *   Returns the new array string.
 */
macro GArrayInsertItem(array, index, item, separator)
{
    if(index < 0 || GStrEmpty(separator))
    {
        return array
    }


    var cItem
    var len
    var iter
    var iInsert
    iter    = 0
    len     = strlen(array)
    cItem   = 0
    iInsert = 0
    while(iter < len)
    {
        if(cItem == index)
        {
            iInsert = iter
            break
        }
        if(array[iter] == separator)
        {
            cItem++
        }
        iter++
    }


    if(iter == len && cItem <= index)
    {
        return GArrayAppendItem(array, item, separator)
    }


    return cat(strmid(array, 0, iInsert)#item#separator, strmid(array, iInsert, len))
}


/*!
 *   Returns the count of array.That is the array's items num.It will returns
 * an error if the separator is empty.
 */
macro GArrayGetCount(array, separator)
{
    if(GStrEmpty(separator))
    {
        return invalid
    }


    var count
    var len


    count = 0
    len = strlen(array)
    if(len == 0)
    {
        return count
    }


    var ich
    ich = 0
    while(ich < len)
    {
        if(array[ich] == separator)
        {
            count++
        }
        ich++
    }


    return count
}


/*!
 *   Remove the items that is referened by item string from array and returns
 * the new array string.
 *   It will do nothing if the item is not in the array.If item string appears
 * not one time in array it will revome all the item string from array.
 */
macro GArrayRemoveItemByItem(array, item, separator)
{
    if(GStrEmpty(separator))
    {
        return array
    }


    var iter
    var itemIter //one item string
    var len
    var itemIdx//current item index
    var nArray //new array string


    iter    = 0
    itemIdx = -1
    len     = strlen(array)
    nArray  = nil
    iBegin  = 0 //current item start index
    while(iter < len)
    {
        if(array[iter] == separator)
        {
            iEnd = iter //set current item end index
            itemIdx++


            itemIter = strmid(array, iBegin, iEnd)


            if(itemIter != item)
            {
                nArray = cat(nArray, itemIter#separator)
            }
            iBegin = iter + 1
        }
        iter++
    }


    return nArray
}


/*!
 *   Remove an item that is at the index of array.It will do nothing if the
 * index is larger then the array's count.
 *   Returns the new array string.
 */
macro GArrayRemoveItemByIndex(array, index, separator)
{
    if(index < 0 || GStrEmpty(separator))
    {
        return array
    }


    var iter      //character iterator of array
    var itemStart //the index item string's start character index
    var itemEnd   //the index item string's end character index
    var cItem     //indicates the current item index that is dealing with
    var len       //character num of array


    itemStart = 0
    itemEnd   = invalid
    iter      = 0
    cItem     = -1
    len       = strlen(array)
    while(iter < len)
    {
        if(array[iter] == separator)
        {
            cItem++
            if(cItem == index)
            {
                itemEnd = iter + 1
            }
            else
            {
                itemStart = iter + 1
            }
        }
        if(itemEnd != invalid)
        {
            break
        }
        iter++
    }


    if(itemEnd == invalid)
    {
        return array
    }


    return cat(strmid(array, 0, itemStart), strmid(array, itemEnd, len))
}


/*!
 *    Get the index item from array.It will returns an empty string if the index
 * is larger then the array's count or the item at index is empty.
 *    Returns the item string at index.
 */
macro GArrayGetItemByIndex(array, index, separator)
{
    if(index < 0 || GStrEmpty(separator))
    {
        return nil
    }


    var item
    var iter
    var cItem
    var len


    item   = nil
    iter   = 0
    len    = strlen(array)
    cItem  = -1 //It starts at 0
    while(iter < len)
    {
        if(array[iter] == separator)
        {
            cItem++
            if(cItem == index)
            {
                break
            }
            item = nil
        }
        else
        {
            item = cat(item, array[iter])
        }
        iter++
    }


    return item
}


/*!
 *    Get the index of the item.It will returns -1 if the item is not in the
 * array.It will only get the index of the item that first appears in array if
 * the item is not a only one.
 *    If the separator is empty it will return -1.
 */
macro GArrayGetItemIndex(array, item, separator)
{
    if(GStrEmpty(separator))
    {
        return invalid
    }


    var iter
    var len
    var itemIdx//current item index


    iter    = 0
    itemIdx = -1
    len     = strlen(array)
    iBegin  = 0 //current item start index
    while(iter < len)
    {
        if(array[iter] == separator)
        {
            iEnd = iter //set current item end index
            itemIdx++


            if(strmid(array, iBegin, iEnd) == item)
            {
                return itemIdx
            }
            else
            {
                iBegin = iter + 1
            }
        }
        iter++
    }


    return invalid
}


/*!
 *   Check whether the item is in the array.
 *   If the separator is empty, it will always returns false.
 */
macro GArrayIsItemExist(array, item, separator)
{
    if(GStrEmpty(separator))
    {
        return false
    }
    else
    {
        if(GArrayGetItemIndex(array, item, separator) != invalid)
        {
            return true
        }
    }


    return false
}

/*
————————————————
版权声明：本文为CSDN博主「hard-water」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/dayong419/java/article/details/7932467
*/
