#include <stdio.h>
#include <string.h>

static const struct FXFT_AdobeNameMap {
    const char* name;
    unsigned short unicode;
} g_fxft_adobe_name_map[] = {
    {"space", 0x0020},
    {"exclam", 0x0021},
    {"quotedbl", 0x0022},
    {"numbersign", 0x0023},
    {"dollar", 0x0024},
    {"percent", 0x0025},
    {"ampersand", 0x0026},
    {"quotesingle", 0x0027},
    {"parenleft", 0x0028},
    {"parenright", 0x0029},
    {"asterisk", 0x002A},
    {"plus", 0x002B},
    {"comma", 0x002C},
    {"hyphen", 0x002D},
    {"period", 0x002E},
    {"slash", 0x002F},
    {"zero", 0x0030},
    {"one", 0x0031},
    {"two", 0x0032},
    {"three", 0x0033},
    {"four", 0x0034},
    {"five", 0x0035},
    {"six", 0x0036},
    {"seven", 0x0037},
    {"eight", 0x0038},
    {"nine", 0x0039},
    {"colon", 0x003A},
    {"semicolon", 0x003B},
    {"less", 0x003C},
    {"equal", 0x003D},
    {"greater", 0x003E},
    {"question", 0x003F},
    {"at", 0x0040},
    {"A", 0x0041},
    {"B", 0x0042},
    {"C", 0x0043},
    {"D", 0x0044},
    {"E", 0x0045},
    {"F", 0x0046},
    {"G", 0x0047},
    {"H", 0x0048},
    {"I", 0x0049},
    {"J", 0x004A},
    {"K", 0x004B},
    {"L", 0x004C},
    {"M", 0x004D},
    {"N", 0x004E},
    {"O", 0x004F},
    {"P", 0x0050},
    {"Q", 0x0051},
    {"R", 0x0052},
    {"S", 0x0053},
    {"T", 0x0054},
    {"U", 0x0055},
    {"V", 0x0056},
    {"W", 0x0057},
    {"X", 0x0058},
    {"Y", 0x0059},
    {"Z", 0x005A},
    {"bracketleft", 0x005B},
    {"backslash", 0x005C},
    {"bracketright", 0x005D},
    {"asciicircum", 0x005E},
    {"underscore", 0x005F},
    {"grave", 0x0060},
    {"a", 0x0061},
    {"b", 0x0062},
    {"c", 0x0063},
    {"d", 0x0064},
    {"e", 0x0065},
    {"f", 0x0066},
    {"g", 0x0067},
    {"h", 0x0068},
    {"i", 0x0069},
    {"j", 0x006A},
    {"k", 0x006B},
    {"l", 0x006C},
    {"m", 0x006D},
    {"n", 0x006E},
    {"o", 0x006F},
    {"p", 0x0070},
    {"q", 0x0071},
    {"r", 0x0072},
    {"s", 0x0073},
    {"t", 0x0074},
    {"u", 0x0075},
    {"v", 0x0076},
    {"w", 0x0077},
    {"x", 0x0078},
    {"y", 0x0079},
    {"z", 0x007A},
    {"braceleft", 0x007B},
    {"bar", 0x007C},
    {"braceright", 0x007D},
    {"asciitilde", 0x007E},
    {"nbspace", 0x00A0},
    {"exclamdown", 0x00A1},
    {"cent", 0x00A2},
    {"sterling", 0x00A3},
    {"currency", 0x00A4},
    {"yen", 0x00A5},
    {"brokenbar", 0x00A6},
    {"section", 0x00A7},
    {"dieresis", 0x00A8},
    {"copyright", 0x00A9},
    {"ordfeminine", 0x00AA},
    {"guillemotleft", 0x00AB},
    {"logicalnot", 0x00AC},
    {"registered", 0x00AE},
    {"macron", 0x00AF},
    {"degree", 0x00B0},
    {"plusminus", 0x00B1},
    {"twosuperior", 0x00B2},
    {"threesuperior", 0x00B3},
    {"acute", 0x00B4},
    {"mu", 0x00B5},
    {"paragraph", 0x00B6},
    {"periodcentered", 0x00B7},
    {"cedilla", 0x00B8},
    {"onesuperior", 0x00B9},
    {"ordmasculine", 0x00BA},
    {"guillemotright", 0x00BB},
    {"onequarter", 0x00BC},
    {"onehalf", 0x00BD},
    {"threequarters", 0x00BE},
    {"questiondown", 0x00BF},
    {"Agrave", 0x00C0},
    {"Aacute", 0x00C1},
    {"Acircumflex", 0x00C2},
    {"Atilde", 0x00C3},
    {"Adieresis", 0x00C4},
    {"Aring", 0x00C5},
    {"AE", 0x00C6},
    {"Ccedilla", 0x00C7},
    {"Egrave", 0x00C8},
    {"Eacute", 0x00C9},
    {"Ecircumflex", 0x00CA},
    {"Edieresis", 0x00CB},
    {"Igrave", 0x00CC},
    {"Iacute", 0x00CD},
    {"Icircumflex", 0x00CE},
    {"Idieresis", 0x00CF},
    {"Eth", 0x00D0},
    {"Ntilde", 0x00D1},
    {"Ograve", 0x00D2},
    {"Oacute", 0x00D3},
    {"Ocircumflex", 0x00D4},
    {"Otilde", 0x00D5},
    {"Odieresis", 0x00D6},
    {"multiply", 0x00D7},
    {"Oslash", 0x00D8},
    {"Ugrave", 0x00D9},
    {"Uacute", 0x00DA},
    {"Ucircumflex", 0x00DB},
    {"Udieresis", 0x00DC},
    {"Yacute", 0x00DD},
    {"Thorn", 0x00DE},
    {"germandbls", 0x00DF},
    {"agrave", 0x00E0},
    {"aacute", 0x00E1},
    {"acircumflex", 0x00E2},
    {"atilde", 0x00E3},
    {"adieresis", 0x00E4},
    {"aring", 0x00E5},
    {"ae", 0x00E6},
    {"ccedilla", 0x00E7},
    {"egrave", 0x00E8},
    {"eacute", 0x00E9},
    {"ecircumflex", 0x00EA},
    {"edieresis", 0x00EB},
    {"igrave", 0x00EC},
    {"iacute", 0x00ED},
    {"icircumflex", 0x00EE},
    {"idieresis", 0x00EF},
    {"eth", 0x00F0},
    {"ntilde", 0x00F1},
    {"ograve", 0x00F2},
    {"oacute", 0x00F3},
    {"ocircumflex", 0x00F4},
    {"otilde", 0x00F5},
    {"odieresis", 0x00F6},
    {"divide", 0x00F7},
    {"oslash", 0x00F8},
    {"ugrave", 0x00F9},
    {"uacute", 0x00FA},
    {"ucircumflex", 0x00FB},
    {"udieresis", 0x00FC},
    {"yacute", 0x00FD},
    {"thorn", 0x00FE},
    {"ydieresis", 0x00FF},
    {"bullet", 0x2022},
    {"ellipsis", 0x2026},
    {"emdash", 0x2014},
    {"endash", 0x2013},
    {"quoteleft", 0x2018},
    {"quoteright", 0x2019},
    {"quotedblleft", 0x201C},
    {"quotedblright", 0x201D},
    {"dagger", 0x2020},
    {"daggerdbl", 0x2021},
    {"perthousand", 0x2030},
    {"Euro", 0x20AC},
    {"trademark", 0x2122},
    {"minus", 0x2212},
    {"fi", 0xFB01},
    {"fl", 0xFB02},
};

static int FXFT_FindAdobeName(const char* name)
{
    size_t i;
    if (!name || !name[0])
        return 0;

    for (i = 0; i < sizeof(g_fxft_adobe_name_map) / sizeof(g_fxft_adobe_name_map[0]); ++i)
    {
        if (strcmp(g_fxft_adobe_name_map[i].name, name) == 0)
            return g_fxft_adobe_name_map[i].unicode;
    }

    if (name[0] == 'u' && strlen(name) == 5)
    {
        unsigned int value = 0;
        if (sscanf(name + 1, "%4x", &value) == 1)
            return (int)value;
    }

    if (name[0] == 'u' && name[1] == 'n' && name[2] == 'i' && strlen(name) == 7)
    {
        unsigned int value = 0;
        if (sscanf(name + 3, "%4x", &value) == 1)
            return (int)value;
    }

    return 0;
}

extern "C" int FXFT_unicode_from_adobe_name(const char* name)
{
    return FXFT_FindAdobeName(name);
}

extern "C" void FXFT_adobe_name_from_unicode(char* name, unsigned int unicode)
{
    size_t i;
    if (!name)
        return;

    for (i = 0; i < sizeof(g_fxft_adobe_name_map) / sizeof(g_fxft_adobe_name_map[0]); ++i)
    {
        if (g_fxft_adobe_name_map[i].unicode == unicode)
        {
            strcpy(name, g_fxft_adobe_name_map[i].name);
            return;
        }
    }

    if (unicode <= 0xFFFF)
        sprintf(name, "uni%04X", unicode);
    else
        name[0] = '\0';
}
