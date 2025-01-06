#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../JSON.hpp"

typedef short int IndexType;
typedef JSONT<IndexType> JSON;


void useTokens(char * buffer, JSON::Token * start, JSON::Token * end)
{
    static IndexType lastParent = JSON::InvalidPos;
    static int level = 0;
    const char * type[] = {"Undefined", "Object", "Array", "Key", "String", "Null", "True", "False", "Number"};
    int i = 0;
    while (start != end)
    {
        JSON::Token & t = *start;
        if (t.parent > lastParent) level++;
        if (t.parent < lastParent) level--; // This is wrong, but the easiest to do
        lastParent = t.parent;
        
        fprintf(stdout, "%*s%d. Token[%s], parent: %d, start at %d", level, "", i, type[t.type], t.parent, t.start);
        if (t.type == JSON::Token::Object || t.type == JSON::Token::Array)
            fprintf(stdout, " (id:%u) with %d elements:\n", (unsigned)t.id, t.elementCount);
        else
            fprintf(stdout, " with value: %.*s\n", t.end - t.start, &buffer[t.start]);
        ++start;
        ++i;
    }
}

int handleError(JSON & a, IndexType err)
{
    if (err == JSON::NotEnoughTokens) fprintf(stderr, "Not enough tokens\n");
    else if (err == JSON::Invalid)    fprintf(stderr, "Invalid stream at pos: %d\n", a.pos);
    else if (err == JSON::Starving)   fprintf(stderr, "Starving at pos: %d\n", a.pos);
    else if (err == JSON::NeedRefill) fprintf(stderr, "Need refill at pos: %d\n", a.pos);
    return -1;
}
int runTest(char * buffer, size_t len, IndexType realLen, bool verbose)
{
    JSON a;
    JSON::Token tokens[2000];
    
    if (verbose) fprintf(stdout, "Splitting input buffer at: %hd/%lu\n", realLen, len);
    IndexType r = a.parse(buffer, realLen, tokens, 2000);
    if (r > 0) { if (verbose) useTokens(buffer, tokens, &tokens[r]); }
    else if (r < 0 && r > JSON::Starving) return handleError(a, r);
    else while (r)
    {
        IndexType l = 0, prevLen = realLen;
        r = a.partialParse(buffer, realLen, tokens, 2000, l);
        if (r == JSON::NeedRefill)
        {
            memmove(&buffer[realLen], &buffer[prevLen], len - realLen);
            realLen = (len - prevLen) + realLen;
            continue;
        }
        if (r < 0 && r > JSON::Starving) return handleError(a, r);
        if (!r) return 0; // Done
        
        if (r > 0 && verbose) useTokens(buffer, &tokens[l], &tokens[r]);
    }
    return 0;

}


int main(int argc, char ** argv)
{
    char * buffer = new char[0x10000];
    FILE * f = argc > 1 ? fopen(argv[1], "rb") : stdin;
    size_t len = fread(buffer, 1, 0x10000, f);
    
    
    volatile int u = 0;
    IndexType realLen = 0;
    if (argc > 2) realLen = atoi(argv[2]);
    else
    {
        srand(time(NULL) ^ 0x3457FDEa);
        if (clock() & 1) u = rand();
        size_t rLen = rand() * len;
        size_t rrLen = rLen / RAND_MAX;
        realLen = (IndexType)(rLen / RAND_MAX);
    }
    if (!realLen)
    {
        char * buffer2 = new char[0x10000];
        memcpy(buffer2, buffer, len);
        bool worked = true;
        // Incremental test
        fprintf(stdout, "Running incremental test\n");
        for (size_t i = 1; i < len; i++)
        {
            memcpy(buffer, buffer2, len);
            if (runTest(buffer, len, i, false))
            {
                fprintf(stderr, "Failed test for split at %lu\n", i);
                worked = false;
            } else fprintf(stdout, "Test ok for size %lu\n", i);
        }
        return worked ? 0 : -1;
    }
    else return runTest(buffer, len, realLen, true);
}
