#include <stdio.h>
#include <stdlib.h>

#include "../JSON.hpp"

typedef short int IndexType;
typedef JSONT<IndexType> JSON;


int main(int argc, char ** argv)
{
    char * buffer = new char[0x10000];
    FILE * f = argc == 2 ? fopen(argv[1], "rb") : stdin; 
    size_t len = fread(buffer, 1, 0x10000, f);
    
    JSON a;
    JSON::Token tokens[2000];
    printf("JSON Token size in bytes: %u\n", sizeof(tokens[0]));
    IndexType r = a.parse(buffer, len, tokens, 2000);
    if (r < 0)
    {
        if (r == JSON::NotEnoughTokens) fprintf(stderr, "Not enough tokens\n");
        else if (r == JSON::Invalid)	fprintf(stderr, "Invalid stream at pos: %d\n", a.pos);
        else if (r == JSON::Starving)   fprintf(stderr, "Starving input at pos: %d\n", a.pos);
        return -1;
    }
    for (IndexType i = 0, level = 0; i < r; i++)
    {
        const char * type[] = {"Undefined", "Object", "Array", "Key", "String", "Null", "True", "False", "Number"};
        JSON::Token & t = tokens[i];
        fprintf(stdout, "%d. Token[%s], parent: %d, start at %d", i, type[t.type], t.parent, t.start);
        if (t.type == JSON::Token::Object || t.type == JSON::Token::Array)
        {
            fprintf(stdout, " (id:%u) with %d elements:\n", (unsigned)t.id, t.elementCount);
            level++;
        }
        else 
        {
            fprintf(stdout, " with value: %.*s\n", t.end - t.start, &buffer[t.start]);
        }
    }
    return 0;
}
