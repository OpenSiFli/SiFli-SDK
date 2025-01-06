#include <string.h>


static bool isEnd(char c)
{
    return c == ',' || c == ']' || c == '}' || c == ' ' || c == '\t' || c == '\n';
}

#ifdef UnescapeJSON
template <typename IndexType>
ROString TokenT<IndexType>::unescape(char * buf)
{
    if (type != String && type != Key) return ROString();

    // Already decoded ?
    if (buf[start-1] == '\0') return ROString(&buf[start]); // The actual string size is unknown here, so need to scan for 0
    if (buf[start-1] == '!') return ROString(); // Error

    // Decode the string... it is valid utf8... so no worries
    char * dst = &buf[start];
    for (const char * src = dst, *last = &buf[end]; src != last; src++)
    {
        if (*src & 0x80)
        {   // Extract utf8 as-is
            while (*src & 0x80)	*(dst++) = *(src++);
            continue;
        }

        // This can not happen unescaped
        if (*src == '"') { buf[start - 1] = '!'; return ROString(); }

        if (*src == '\\')
        {
            // Unicode char are not converted, but since they are not converted either when being served, it should be safe.
            switch (*(src+1))
            {
                case 'b': *(dst++) = '\b'; break;
                case 'f': *(dst++) = '\f'; break;
                case 'n': *(dst++) = '\n'; break;
                case 'r': *(dst++) = '\r'; break;
                case 't': *(dst++) = '\t'; break;

                // Decode next as if it was unescaped
                default: *(dst++) = *(src + 1); break;
            }

            src++;
            continue;
        }

    	*(dst++) = *src;
    }
    *dst = '\0';

    buf[start - 1] = '\0';
    return ROString(&buf[start], (size_t)(dst - &buf[start]));
}
#endif

// Match the given input string
template <typename IndexType>
IndexType JSONT<IndexType>::match(const char * in, const IndexType len, Token & token, const char * m)
{
    for (IndexType i = pos; i < len; i++)
    {
        char c = in[i];
        IndexType k = i - pos;

        if (m[k] == '\0')
        {
            if (isEnd(c))
            {
                token.end = i;
                pos = i - 1;
                return 0;
            }
            return Invalid;
        }
        if (m[k] != c) return Invalid;
    }
    return Starving;
}

// Parse the given primitive 
template <typename IndexType>
IndexType JSONT<IndexType>::parsePrimitive(const char * in, const IndexType len, Token & token)
{
    IndexType c = match(in, len, token, "null");
    if (c == 0) return token.changeType(Token::Null);
    else if (c == Starving) return Starving;

    c = match(in, len, token, "true");
    if (c == 0) return token.changeType(Token::True);
    else if (c == Starving) return Starving;

    c = match(in, len, token, "false");
    if (c == 0) return token.changeType(Token::False);
    else if (c == Starving) return Starving;

    if (in[pos] == '-') pos++;
    for (; pos < len; pos++)
    {
        char u = in[pos];
        if (u == '\0') return Invalid;

        if (isEnd(u))
        {
            token.end = pos--;
            return 0; // token.changeType(Token::Number);
        }
        // This is not correct since it allows 123.32.4.eeeee++--234
        if ((u < '0' || u > '9') && u != 'e' && u != 'E' && u != '.' && u != '-' && u != '+') return Invalid;
    }

    return Starving;
}

template <typename IndexType>
static IndexType isValidUTF8(const char * in, const IndexType len)
{
    char c = in[0];
    int k = 0;
         if ((c & 0xE0) == 0xC0)  k = 1;
    else if ((c & 0xF0) == 0xE0)  k = 2;
    else if ((c & 0xF8) == 0xF0)  k = 3;
    else return JSONT<IndexType>::Invalid;

    if (k >= len) return JSONT<IndexType>::Starving;
    for (IndexType i = 1; i < k; i++)
        if ((in[i] & 0xC0) != 0x80)
            return JSONT<IndexType>::Invalid;

    return k;
}

template <typename IndexType>
IndexType JSONT<IndexType>::parseString(const char * in, const IndexType len, Token & token)
{
    for (; pos < len; pos++)
    {
        char c = in[pos];

        if (c == '\0')  return Invalid;
        if (c & 0x80)
        {
            IndexType r = isValidUTF8(&in[pos], len-pos);
            if (r <= 0) return r;
            pos += r;
            continue;
        }

        /* Quote: end of string */
        if (c == '\"')
        {
            token.end = pos;
            return 0;
        }

        /* Backslash: Quoted symbol expected */
        if (c == '\\')
        {
            if (pos >= len - 1)     return Starving;

            c = in[pos + 1];
            if (c == 127 || c < 32) return Invalid;
            switch (c)
            {
            case 'u':
                //get 4 chars!
                //pos is the bslash
                //pos+1 is the u
                if (pos + 6 >= len) return Starving;
                for (IndexType i = 2; i < 6 && pos + i < len; i++)
                {
                    c = in[pos + i];
                    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) continue;
                    return Invalid;
                }
                pos += 5;
                break;
            case '"': case 'b': case 'f': case 'n': case 'r': case 't': case '/': case '\\':
                pos++;
                break;
            default:
                return Invalid;
            }
        }
    }

    return Starving;
}

template <typename IndexType>
IndexType JSONT<IndexType>::parse(const char * in, const IndexType len, Token * tokens, const IndexType tokenCount)
{
    IndexType tk;
    char c = 0;

    for (; pos < len; pos++)
    {
        c = in[pos];

        switch(c)
        {
        case '\0': return rememberLastError(Invalid);

        case '{':
        case '[':
            if (state != ExpectValue)       return rememberLastError(Invalid);

            tk = allocToken(tokenCount);
            if (tk == InvalidPos)           return rememberLastError(NotEnoughTokens);
            tokens[tk].init(c == '{' ? Token::Object : Token::Array, super, pos, 0, ++lastId);

            if (super != InvalidPos)    tokens[super].elementCount++;

            super = next - 1;
            state = c == '{' ? ExpectKey : ExpectValue;
            break;

        case '}':
        case ']':
            if (super == InvalidPos)        return rememberLastError(Invalid);
            if (tokens[super].type != (c == '}' ? Token::Object : Token::Array))
                return rememberLastError(Invalid);

            super = tokens[super].parent;

            if (super == InvalidPos)        return rememberLastError(next);
            state = ExpectComma;
            break;

        case '"':
            if (state != ExpectValue && state != ExpectKey)  return rememberLastError(Invalid);

            tk = allocToken(tokenCount);
            if (tk == InvalidPos)           return rememberLastError(NotEnoughTokens);
            tokens[tk].init(state == ExpectKey ? Token::Key : Token::String, super, ++pos, 0);

            tk = parseString(in, len, tokens[tk]);
            if (tk != 0)                    return rememberLastError(tk);

            if (state == ExpectKey) state = ExpectColon;
            else if (state == ExpectValue)
            {
                state = ExpectComma;
                if (super != InvalidPos) tokens[super].elementCount++;
            }
            break;
        case ',':
            if (state != ExpectComma)       return rememberLastError(Invalid);
            if (super == InvalidPos)        return rememberLastError(Invalid);

            state = tokens[super].type == Token::Object ? ExpectKey : ExpectValue;
            break;
        case ':':
            if (state != ExpectColon)       return rememberLastError(Invalid);
            state = ExpectValue;
            break;
        case '\t':
        case '\n':
        case ' ':
            break;
        default:
            if (state != ExpectValue)       return rememberLastError(Invalid);
            tk = allocToken(tokenCount);
            if (tk == InvalidPos)           return rememberLastError(NotEnoughTokens);
            tokens[tk].init(Token::Number, super, pos, 0); // Set number by default and fix up if not a number
            tk = parsePrimitive(in, len, tokens[tk]);
            if (tk != 0)                    return rememberLastError(tk);

            state = ExpectComma;
            if (super != InvalidPos) tokens[super].elementCount++;
            break;
        }
    }

    return rememberLastError(Starving);
}

template <typename IndexType>
IndexType JSONT<IndexType>::getCurrentContainerCount(const char * in, const IndexType len, const Token & token)
{
    bool isObj = token.state == EnteringObject, isArr = token.state == EnteringArray;
    if (!isObj && !isArr) return 0;
    IndexType count = 0;

    IndexType bracketCount = isArr, blockCount = isObj;
    // The algorithm here is quite simple 
    // Depending on the current token state (array or object), we count the number of bracket or block
    // until we exit the current object or array. We skip everything while in the string mode.
    bool inTxt = false, escaped = false;
    for (IndexType i = pos; i < len; i++)
    {
        char c = in[i];
        if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
            continue;
        } else if (inTxt) {
            if (!escaped && c == '"') { 
                inTxt = false;
            }
            else if (!escaped && c == '\\') {
                escaped = true;
            }
            else if (escaped) {
                escaped = false;
            }
        } else {
            if (isObj && blockCount == 1 && !count) {
                count++; // It's a value or a key, anyway, let's count it
            } else if (isArr && bracketCount == 1 && !count) {
                count++; // It's a value or a key, anyway, let's count it
            }

            if (c == '"') {
                inTxt = true; 
            } else if (c == '{') {
                blockCount++;
            } else if (c == '}') {
                blockCount--;
                if (blockCount == 0 && isObj) return count;
            } else if (c == '[') {
                bracketCount++;
            } else if (c == ']') {
                bracketCount--;
                if (bracketCount == 0 && isArr) return count;
            } else if (c == ',') {
                if (isObj && blockCount == 1 && bracketCount == 0) count++;
                else if (isArr && bracketCount == 1 && blockCount == 0) count++;
            } 
        }
    }
    return count;
}


template <typename IndexType>
IndexType JSONT<IndexType>::parseOne(const char * in, const IndexType len, Token & token, IndexType & lastSuper)
{
    IndexType tk;
    char c = 0;
    bool skip = false;

    if (pos == 0) { next = InvalidPos; lastSuper = InvalidPos; }

    if (pos >= len)        return rememberLastError(Starving);
    for (; pos < len; pos++) 
    {
        c = in[pos];

        switch(c)
        {
        case '\0': return rememberLastError(Invalid);

        case '{':
        case '[':
            if (state != ExpectValue)       return rememberLastError(Invalid);

            token.init(c == '{' ? Token::Object : Token::Array, super, pos, 0, ++lastId);

            lastSuper = super; // Play musical chair to deal with delay in saving
            super = pos;
            state = c == '{' ? ExpectKey : ExpectValue;
            pos++;
            token.state = c == '{' ? EnteringObject : EnteringArray;
            return rememberLastError(SaveSuper);

        case '}':
        case ']':
            if (super == InvalidPos)        return rememberLastError(Invalid);
            if ((in[super] == '{' && c == ']')  || (in[super] == '[' && c == '}'))
                return rememberLastError(Invalid);

            token.start = super;
            token.type = c == '}' ? Token::Object : Token::Array;
            token.state = c == '}' ? LeavingObject : LeavingArray;
            token.end = pos;

            super = lastSuper;

            if (lastSuper == InvalidPos)        { state = Done; pos++; return rememberLastError(Finished); }
            state = ExpectComma;
            pos++;
            return rememberLastError(RestoreSuper);

        case '"':
            if (state != ExpectValue && state != ExpectKey)  return rememberLastError(Invalid);

            token.init(state == ExpectKey ? Token::Key : Token::String, super, ++pos, 0);

            tk = parseString(in, len, token);
            if (tk != 0)                    return rememberLastError(tk);

            if (state == ExpectKey)         state = ExpectColon;
            else if (state == ExpectValue)  state = ExpectComma;

            token.state = state == ExpectColon ? HadKey : HadValue;
            break;
        case ',':
            if (state != ExpectComma)       return rememberLastError(Invalid);
            if (super == InvalidPos)        return rememberLastError(Invalid);

            state = in[super] == '{' ? ExpectKey : ExpectValue;
            skip = true;
            break;
        case ':':
            if (state != ExpectColon)       return rememberLastError(Invalid);
            state = ExpectValue;
            skip = true;
            break;
            // Ignore whitespace
        case '\t':
        case '\n':
        case ' ':
            skip = true;
            break;
        default:
            if (state != ExpectValue)       return rememberLastError(Invalid);
            
            token.init(Token::Number, super, pos, 0); // Set number by default and fix up if not a number
            tk = parsePrimitive(in, len, token);
            if (tk != 0)                    return rememberLastError(tk);

            state = ExpectComma;
            token.state = HadValue;
            break;
        }
        if (skip) { skip = false; continue; }
        
        pos++;
        return rememberLastError(OneTokenFound);
    }
    return rememberLastError(Starving);
}


#ifndef SkipJSONPartialParsing
template <typename IndexType>
IndexType JSONT<IndexType>::partialParse(char * in, IndexType & len, Token * tokens, const IndexType tokenCount, IndexType & lastTokenPos)
{
    // No need to do anything if everything is already parsed
    if (!partialState) return 0;
    // If a previous parsing error happened, let's return it
    if (partialState > Starving && partialState < 0) return partialState;

    if (partialState == Starving)
    {   // The previous call to parse was starving, it's time to return everything plausible
        // Check the last valid token
        IndexType ret = next - 1;
        // Need to rewind so a key value pair is always starting by a key
        if (tokens[ret].type != Token::Key && ret > 0 && tokens[ret - 1].type == Token::Key) ret--;
        // Done for extracting tokens for now, will need to be called later on
        partialState = NeedFixing;
        return ret == 0 ? 1 : ret;
    }
    if (partialState == NeedFixing)
    {
        IndexType n = next, tmp;
        IndexType ret = next - 1, keep = next - 1;
        if (tokens[keep].type != Token::Key && keep > 0 && tokens[keep - 1].type == Token::Key) keep--;
        IndexType offset = (tokens[keep].type == Token::String || tokens[keep].type == Token::Key);
        IndexType inStart = tokens[keep].start - offset;
        tmp = ret;
        while (tokens[tmp].parent != InvalidPos)
        {
            tmp = tokens[tmp].parent;
            // Save the parent token after the last used position
            if (n + 1 >= tokenCount) return NotEnoughTokens;
            // Only save it if it's still being used
            if (tmp <= super) tokens[n++] = tokens[tmp];
        }
        // Now rewrite the tokens backward
        lastTokenPos = 0; IndexType p = InvalidPos;
        for (IndexType i = n - 1; i >= next; i--)
        {
            tokens[lastTokenPos] = tokens[i];
            tokens[lastTokenPos].start = InvalidPos;
            tokens[lastTokenPos].parent = p;
            p = lastTokenPos++;
        }
        // Fix the text buffer too
        memmove(in, &in[inStart], len - inStart);
        len -= inStart;
        pos -= inStart;

        if (super >= tokens[ret].parent)
        {   // This is only possible if we did not stop after finishing an object (after one or multiple "}")
            super = p;
            // Then copy the last key (and or value is applicable)
            p++;
            for (IndexType i = keep; i < next; i++)
            {
                tokens[p].type = tokens[i].type;
                tokens[p].start = tokens[i].start - inStart;
                if (tokens[p].type >= Token::Key) tokens[p].end = tokens[i].end - inStart;
                tokens[p].parent = lastTokenPos - 1;
                p++;
            }
            // Let's say the next token was complete, and fix later if not
            next = p;
            // If the last token is a container, make sure it's the super points to it
            if (tokens[p - 1].type == Token::Array || tokens[p - 1].type == Token::Object)
                super = p-1;
            else if (tokens[p - 1].end <= 0)
            {   // The last token was incomplete, so we'll have to reparse it
                pos = tokens[p - 1].start - offset;
                next--;
            }
        } else
        {   // Stopping after finishing an object or array
            super = p;
            next = p+1;
        }
        return partialState = NeedRefill;
    }
    if (partialState == NeedRefill)
        return parse(in, len, tokens, tokenCount);
    
    return 0;
}
#endif
