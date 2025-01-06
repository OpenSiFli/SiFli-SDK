#ifndef hpp_ROString_hpp
#define hpp_ROString_hpp

// String view is only available since C++17 which might not be supported by the embedded compiler
// So, provide a simple read only string class that than be reused in your code
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#pragma warning(disable:4307)
#endif
/** Compute a string hash at compile time */
unsigned constexpr constHash(char const * input)
{
    return *input ? static_cast<unsigned int>(*input) + 257 * constHash( input + 1 ) : 5381;
}
/** Compute a string hash at compile time */
size_t constexpr operator "" _hash( const char* str, size_t len )
{
    return constHash( str );
}

const char usualTrimSequence[] = " \t\v\f\r\n";

/** Well, the name says it all, this is a very simple read only string.
    The main advantage of this class is that it doesn't allocate any
    memory at all, and works on fixed size buffer correctly.
    So you can/should use it on embedded system wherever applicable or for any parsing work.
    Please notice that it supports mutating its heads (start of string and length of string)
    but not the string content itself.
    There is no (const char*) operator since it might not be zero terminated (obviously). */
struct ROString
{
public:
    typedef unsigned int        uint32;
    typedef unsigned long int   uint64;
    typedef unsigned char       uint8;

    template <typename T>
    static inline T min(T a, T b) { return a < b ? a: b; }

    template <typename T>
    static inline T max(T a, T b) { return a > b ? a: b; }

private:
    /** The data pointer */
    const char * data;
    /** The current string length */
    const size_t length;

    friend class RWString;

    // Interface
public:
    /** Get a pointer on the data */
    inline const char * getData() const     { return data; }
    /** Get the string length */
    inline size_t       getLength() const   { return length; }
    /** Limit the string length to the given value
        @param newLength the new length
        @return true on success */
    inline bool limitTo(const size_t newLength)   { if (newLength > length) return false; (void)Mutate(data, newLength); return true; }
    /** Get the substring from this string */
    ROString midString(int left, int len) const { return ROString((size_t)left < length ? &data[left] : "", max(0, min(len, (int)length - left))); }
    /** Split at the given position.
        For example, the following code gives:
        @code
            String text = "abcdefdef";
            String ret = text.splitAt(3); // ret = "abc", text = "defdef"
            ret = text.splitAt(3, 1);     // ret = "de", text = "def"
            ret = text.splitAt(9);        // ret = "def", text = ""
        @endcode
        @param pos    The position to split this string.
                      If the position is larger than the string's length, the complete string is returned,
                      and this string is modified to be empty.
                      If the position is negative, an empty string is returned, and this string is left
                      unmodified.
        @param stripFromRet   This is the amount of characters to strip from the right of the returned string.
                              This is equivalent to .limitTo(getLength() - stripFromRet)
        @return The part from the left to the given position. */
    ROString splitAt(int pos, int stripFromRet = 0) 
    {
        if (stripFromRet > pos) stripFromRet = pos;
        if (pos < 0) return ROString();
        ROString ret(data, min(pos - stripFromRet, (int)length));
        if ((size_t)pos > length) (void)Mutate(data, 0);
        else (void)Mutate(data + pos, length - (size_t)pos);
        return ret;
    }

    /** Trim the string from the given char (and direction) */
    ROString trimRight(const char ch) const { size_t len = length; while(len > 1 && data && data[len - 1] == ch) len--; return ROString(data, (int)len); }
    /** Trim the string from the given char (and direction) */
    ROString trimLeft(const char ch) const { size_t len = length; while(len > 1 && data && data[length - len] == ch) len--; return ROString(data + (length - len), (int)len); }
    /** Trim the beginning of string from any char in the given array */
    ROString trimmedLeft(const char* chars, size_t nlen = 0) const
    {
        size_t len = length;
        if (!nlen && chars) nlen = strlen(chars);
        while(len > 1 && data && memchr(chars, data[length - len], nlen) != NULL) len--;
        return ROString(data + (length - len), (int)len);
    }
    /** Trim the beginning of string from any char in the given array */
    ROString trimmedLeft() const { return trimmedLeft(usualTrimSequence, sizeof(usualTrimSequence)); }
    /** Trim the end of string from any char in the given array */
    ROString trimmedRight(const char* chars, size_t nlen = 0) const
    {
        size_t len = length;
        if (!nlen && chars) nlen = strlen(chars);
        while(len > 1 && data && memchr(chars, data[len - 1], nlen) != NULL) len--;
        return ROString(data, (int)len);
    }
    /** Trim the end of string from any char in the given array */
    ROString trimmedRight() const { return trimmedRight(usualTrimSequence, sizeof(usualTrimSequence)); }
    /** Trim the beginning of string from any char in the given array.
        This is using fluent interface and modifies the internal object. */
    ROString & leftTrim(const char* chars, size_t nlen = 0)
    {
        size_t len = length;
        if (!nlen && chars) nlen = strlen(chars);
        while(len > 1 && data && memchr(chars, data[length - len], nlen) != NULL) len--;
        return Mutate(data + (length - len), len);
    }
    /** Trim the beginning of string from any char in the given array.
        This is using fluent interface and modifies the internal object. */
    template <size_t nlen> ROString & leftTrim(const char (&chars)[nlen])
    {
        size_t len = length;
        while(len > 1 && data && memchr(chars, data[length - len], nlen-1) != NULL) len--;
        return Mutate(data + (length - len), len);
    }
    /** Trim the beginning of string from any char in the given array
        This is using fluent interface and modifies the internal object. */
    ROString & leftTrim() { return leftTrim(usualTrimSequence, sizeof(usualTrimSequence)); }
    /** Trim the end of string from any char in the given array
        This is using fluent interface and modifies the internal object. */
    ROString & rightTrim(const char* chars, size_t nlen = 0)
    {
        size_t len = length;
        if (!nlen && chars) nlen = strlen(chars);
        while(len > 1 && data && memchr(chars, data[len - 1], nlen) != NULL) len--;
        return Mutate(data, len);
    }
       /** Trim the end of string from any char in the given array
       This is using fluent interface and modifies the internal object. */
    template <size_t nlen> ROString & rightTrim(const char (&chars)[nlen])
    {
        size_t len = length;
        while(len > 1 && data && memchr(chars, data[len - 1], nlen - 1) != NULL) len--;
        return Mutate(data, len);
    }
    /** Trim the end of string from any char in the given array
       This is using fluent interface and modifies the internal object. */
    ROString & rightTrim() { return rightTrim(usualTrimSequence, sizeof(usualTrimSequence)); }
    /** Trim the string from any char in the given array */
    ROString Trimmed(const char* chars, size_t nlen = 0) const
    {
       size_t llen = length, rlen = length;
       if (!nlen && chars) nlen = strlen(chars);
       while(nlen && llen > 1 && data && memchr(chars, data[length - llen], nlen) != NULL) llen--;
       while(nlen && rlen > 1 && data && memchr(chars, data[rlen - 1], nlen) != NULL) rlen--;
       return ROString(data + (length - llen), (int)(rlen - (length  - llen)));
    }
    /** Trim the string from any char in the given array */
    ROString Trimmed() const { return Trimmed(usualTrimSequence, sizeof(usualTrimSequence)); }
    /** Trim the string from any char in the given array */
    ROString Trimmed(const ROString & t) const
    {
       size_t llen = length, rlen = length;
       while(t.length && llen > 1 && data && memchr(t.data, data[length - llen], t.length) != NULL) llen--;
       while(t.length && rlen > 1 && data && memchr(t.data, data[rlen - 1], t.length) != NULL) rlen--;
       return ROString(data + (length - llen), (int)(rlen - (length  - llen)));
    }
    /** Trim the string from any char in the given array
       This is using fluent interface and modifies the internal object. */
    ROString & Trim(const char* chars, size_t nlen = 0)
    {
       size_t llen = length, rlen = length;
       if (!nlen && chars) nlen = strlen(chars);
       while(nlen && llen > 1 && data && memchr(chars, data[length - llen], nlen) != NULL) llen--;
       while(nlen && rlen > 1 && data && memchr(chars, data[rlen - 1], nlen) != NULL) rlen--;
       return Mutate(data + (length - llen), rlen - (length  - llen));
    }
    /** Trim the string from any char in the given array
       This is using fluent interface and modifies the internal object. */
    ROString & Trim() { return Trim(usualTrimSequence, sizeof(usualTrimSequence)); }
    /** Trim the string from any char in the given array
       This is using fluent interface and modifies the internal object. */
    ROString & Trim(const ROString & t)
    {
       size_t llen = length, rlen = length;
       while(t.length && llen > 1 && data && memchr(t.data, data[length - llen], t.length) != NULL) llen--;
       while(t.length && rlen > 1 && data && memchr(t.data, data[rlen - 1], t.length) != NULL) rlen--;
       return Mutate(data + (length - llen), rlen - (length  - llen));
    }

    /** Find the specific needle in the string.
        This is a very simple O(n*m) search.
        @return the position of the needle, or getLength() if not found. */
    size_t Find(const ROString & needle, size_t pos = 0) const
    {
        for (size_t j = 0; pos + j < length;) {
            if (needle.data[j] == data[pos + j]) {
                j++;
                if (j == needle.length) return pos;
                continue;
            }
            pos++;
            j = 0;
        }
        return length;
    }

    /** Find any of the given set of chars
        @return the position of the needle, or getLength() if not found. */
    size_t findAnyChar(const char * chars, size_t pos = 0, size_t nlen = 0) const
    {
        if (!nlen && chars) nlen = strlen(chars);
        while(pos < length && data && memchr(chars, data[pos], nlen) == NULL) pos++;
        return pos;
    }
    /** Find first char that's not in the given set of chars
        @return the position of the needle, or getLength() if not found. */
    size_t invFindAnyChar(const char * chars, size_t pos = 0, size_t nlen = 0) const
    {
        if (!nlen && chars) nlen = strlen(chars);
        while(pos < length && data && memchr(chars, data[pos], nlen) != NULL) pos++;
        return pos;
    }
    /** Find the specific needle in the string, starting from the end of the string.
        This is a very simple O(n*m) search.
        @return the position of the needle, or getLength() if not found.
        @warning For historical reasons, Strings::FastString::reverseFind returns -1 if not found */
    size_t reverseFind(const ROString & needle, size_t pos = (size_t)-1) const
    {
        if (needle.length > length) return length;
        size_t i = min(pos, (length - needle.length)); // If there is no space to find out the needle at the end, simply snap back
        for (size_t j = 0;;)
            if (needle.data[j] == data[i + j]) {
                j ++;
                if (j >= needle.length) return i;
            } else {
                if (i-- == 0) break;
                j = 0;
            }
        return length;
    }
    /** Count the number of times the given substring appears in the string */
    size_t Count(const ROString & needle) const
    {
        size_t pos = 0; size_t count = 0;
        while ((pos = Find(needle, pos)) != length) { count++; pos++; }
        return count;
    }


    /** Split a string when the needle is found first, returning the part before the needle, and
        updating the string to start on or after the needle.
        If the needle is not found, it returns an empty string if includeFind is false, or the whole string if true.
        For example this code returns:
        @code
            String text = "abcdefdef";
            String ret = text.splitFrom("d"); // ret = "abc", text = "efdef"
            ret = text.splitFrom("f", true);  // ret = "e", text = "fdef"
        @endcode
        @param find         The string to look for
        @param includeFind  If true the string is updated to start on the find text. */
    ROString splitFrom(const ROString & find, const bool includeFind = false)
    {
        const size_t pos = Find(find);
        if (pos == length) {
            if (includeFind) {
                ROString ret(*this);
                (void)Mutate(data + length, 0);
                return ret;
            }
            return ROString("", 0);
        }
        const size_t size = pos + find.length;
        ROString ret(data, (int)(includeFind ? size : pos));
        (void)Mutate(data + size, length - size);
        return ret;
    }

    /** Get the substring from the given needle up to the given needle.
        For example, this code returns:
        @code
            String text = "abcdefdef";
            String ret = text.fromTo("d", "f"); // ret = "e"
            ret = text.fromTo("d", "f", true);  // ret = "def"
            ret = text.fromTo("d", "g"); // ret = ""
            ret = text.fromTo("d", "g", true); // ret = "defdef"
            ret = text.fromTo("g", "f", [true or false]); // ret = ""
        @endcode

        @param from         The first needle to look for
        @param to           The second needle to look for
        @param includeFind  If true, the text searched for is included in the result
        @return If "from" needle is not found, it returns an empty string, else if "to" needle is not found,
                it returns an empty string upon includeFind being false, or the string starting from "from" if true. */
    ROString fromTo(const ROString & from, const ROString & to, const bool includeFind = false) const
    {
        const size_t fromPos = Find(from);
        const size_t toPos = Find(to, fromPos + from.length);
        return ROString(fromPos >= length ? "" : &data[includeFind ? fromPos : fromPos + from.length],
        (int)(toPos < length ? (includeFind ? toPos + to.length - fromPos : toPos - fromPos - from.length)
        // If the "to" needle was not found, either we return the whole string (includeFind) or an empty string
                            : (includeFind ? length - fromPos : 0)));
    }    

    /** Get the string up to the first occurrence of the given string
        If not found, it returns the whole string unless includeFind is true (empty string in that case).
        For example, this code returns:
        @code
            String ret = String("abcdefdef").upToFirst("d"); // ret == "abc"
            String ret = String("abcdefdef").upToFirst("g"); // ret == "abcdefdef"
        @endcode
        @param find         The text to look for
        @param includeFind  If set, the needle is included in the result */
    ROString upToFirst(const ROString & find, const bool includeFind = false) const { const size_t pos = Find(find); return ROString(pos == length && includeFind ? "" : data, (int)(includeFind ? (pos == length ? 0 : pos + find.length) : pos)); }
    /** Get the string up to the last occurrence of the given string
        If not found, it returns the whole string unless includeFind is true (empty string in that case).
        For example, this code returns:
        @code
            String ret = String("abcdefdef").upToLast("d"); // ret == "abcdef"
            String ret = String("abcdefdef").upToLast("g"); // ret == "abcdefdef"
        @endcode
        @param find         The text to look for
        @param includeFind  If set, the needle is included in the result */
    ROString upToLast(const ROString & find, const bool includeFind = false) const { const size_t pos = reverseFind(find); return ROString(pos == length && includeFind ? "" : data, (int)(includeFind ? (pos == length ? 0 : pos + find.length) : pos)); }

    /** Get the string from the last occurrence of the given string.
        If not found, it returns an empty string if includeFind is false, or the whole string if true
        For example, this code returns:
    @code
        String ret = String("abcdefdef").fromLast("d"); // ret == "ef"
        String ret = String("abcdefdef").fromLast("d", true); // ret == "def"
        String ret = String("abcdefdef").fromLast("g"); // ret == ""
    @endcode
    @param find         The text to look for
    @param includeFind  If set, the needle is included in the result */
    ROString fromLast(const ROString & find, const bool includeFind = false) const 
    {
        const size_t pos = reverseFind(find);
        return ROString(pos == length ? (includeFind ? data : "") : &data[includeFind ? pos : pos + find.length],
        (int)(pos == length ? (includeFind ? length : 0) : (includeFind ? length - pos : length - pos - find.length)));
    }

    /** Get the string from the first occurrence of the given string
        If not found, it returns an empty string if includeFind is false, or the whole string if true
        For example, this code returns:
        @code
            String ret = String("abcdefdef").fromFirst("d"); // ret == "efdef"
            String ret = String("abcdefdef").fromFirst("d", true); // ret == "defdef"
            String ret = String("abcdefdef").fromFirst("g"); // ret == ""
        @endcode
        @param find         The text to look for
        @param includeFind  If set, the needle is included in the result */
    ROString fromFirst(const ROString & find, const bool includeFind = false) const
    {
        const size_t pos = Find(find);
        return ROString(pos == length ? (includeFind ? data : "") : &data[includeFind ? pos : pos + find.length],
        (int)(pos == length ? (includeFind ? length : 0)
                            : (includeFind ? length - pos
                                        : length - pos - find.length)));
    }
     /** Get the substring from the given needle if found, or the whole string if not.
        For example, this code returns:
        @code
            String text = "abcdefdef";
            String ret = text.dropUpTo("d"); // ret = "efdef"
            ret = text.dropUpTo("d", true); // ret = "defdef"
            ret = text.dropUpTo("g", [true or false]); // ret = "abcdefdef"
        @endcode
        @param find         The string to look for
        @param includeFind  If true the string is updated to start on the find text. */
    ROString dropUpTo(const ROString & find, const bool includeFind = false) const
    {
        const size_t pos = Find(find);
        return ROString(pos == length ? data : &data[includeFind ? pos : pos + find.length], 
        (int)(pos == length ? length : (includeFind ? length - pos
                                                    : length - pos - find.length)));
    }
    /** Get the substring up to the given needle if found, or the whole string if not, and split from here.
        For example, this code returns:
        @code
            String text = "abcdefdef";
            String ret = text.splitUpTo("d"); // ret = "abc", text = "efdef"
            ret = text.splitUpTo("g", [true or false]); // ret = "efdef", text = ""
            text = "abcdefdef";
            ret = text.splitUpTo("d", true); // ret = "abcd", text = "efdef"
        @endcode
        @param find         The string to look for
        @param includeFind  If true the string is updated to start on the find text. */
    ROString splitUpTo(const ROString & find, const bool includeFind = false)
    {
        const size_t pos = Find(find);
        if (pos == length) {
            ROString ret(*this);
            (void)Mutate(data + length, 0);
            return ret;
        }
        const size_t size = pos + find.length;
        ROString ret(data, (int)(includeFind ? size : pos));
        (void)Mutate(data+size, length - size);
        return ret;
    }

    /** Swap with another string */
    void swapWith(ROString & other) 
    { 
        const char * tmp = data; data = other.data; other.data = tmp; 
        const size_t t = length; const_cast<size_t&>(length) = other.length; const_cast<size_t&>(other.length) = t; 
    }

    /** The basic conversion operators */
    operator int() const { return (int)strtol(data, 0, 0); }

#if 0
    /** The basic conversion operators */
    operator size_t() const { return (size_t)strtoul(data, 0, 0); }
    /** The basic conversion operators */
    operator uint32() const { return (uint32)strtoul(data, 0, 0); }
#endif
    /** The basic conversion operators */
    operator double() const { return strtod(data, 0); }

    /** Get the integer out of this string.
        This method support any usual encoding of the integer, and detect the integer format automatically.
        This method is optimized for speed, and does no memory allocation on heap
        Supported formats examples: "0x1234, 0700, -1234, 0b00010101"
        @param base     If provided, only the given base is supported (default to 0 for auto-detection).
        @param consumed If provided, will be filled with the number of consumed characters.
        @return The largest possible integer that's parseable. */
    int parseInt(const int base = 0, int * consumed = 0) const { char * end = 0; int out = (int)strtol((const char*)data, &end, base); if (consumed) *consumed = (int)(end - (char*)data); return out; }

    /** Get the double stored in this string
        @param consumed If provided, will be filled with the number of consumed characters.
        @return The largest possible double number that's parseable. */
    double parseDouble(int * consumed = 0) const { char * end = 0; double out = strtod((const char*)data, &end); if (consumed) *consumed = (int)(end - (char*)data); return out; }

    /** So you can check the string directly for emptiness */
    inline bool operator !() const { return length == 0; }
    /** So you can check the string directly for emptiness */
    inline operator bool() const { return length > 0; }
    /** Operator [] to access a single char */
    char operator[] (size_t index) const { return index < length ? data[index] : 0; }

    /** Copy the value in the given fixed size array */
    template <size_t N>
    bool copyInto(char (&_data)[N]) const { size_t c = (length < N-1) ? length : N-1; if (data) memcpy(_data, data, c); _data[c] = 0; return c == length; }
    /** Copy the value in the given fixed size array */
    template <size_t N>
    bool copyInto(uint8 (&_data)[N]) const { size_t c = (length < N-1) ? length : N-1; if (data) memcpy(_data, data, c); _data[c] = 0; return c == length; }

    /** Compute the hash of the string using the h = Recurse(x + h * 257) + 5381 formula */
    uint32 hash() const { uint32 ret = 5381; for (size_t i = length; i != 0; i--) ret = data[i-1] + ret * 257; return ret; }

    // Construction and operators
public:
    /** Default constructor */
    ROString(const char* _data = 0, const int _length = -1) : data(_data), length(_length == -1 ? (_data ? strlen(_data) : 0) : (size_t)_length) { }
    /** Constant string build */
    template <size_t N>
    ROString(const char (&_data)[N]) : data(_data), length(N-1) { }
    /** Copy constructor */
    ROString(const ROString & copy) : data(copy.data), length(copy.length) {}
    /** Equal operator */
    inline ROString & operator = (const ROString & copy) { if (&copy != this) return Mutate(copy.data, copy.length); return *this; }
    /** Compare operator */
    inline bool operator == (const ROString & copy) const { return length == copy.length && memcmp(data, copy.data, length) == 0; }
    /** Compare operator */
    inline bool operator == (const char* copy) const { return length == strlen(copy) && memcmp(data, copy, length) == 0; }
    /** Compare operator */
    template <size_t N> inline bool operator == (const char (&copy)[N]) const { return length == N-1 && memcmp(data, copy, length) == 0; }
    /** Inverted compare operator */
    inline bool operator != (const ROString & copy) const { return !operator ==(copy); }
    /** Inverted compare operator */
    inline bool operator != (const char* copy) const { return !operator ==(copy); }
    /** Inverted compare operator */
    template <size_t N> inline bool operator != (const char (&copy)[N]) const { return !operator ==(copy); }

private:
    /** Mutate this string head positions (not the content) */
    inline ROString & Mutate(const char* d, const size_t len) { const_cast<const char * & >(data) = d; const_cast<size_t&>(length) = len; return *this; }
};



#endif
