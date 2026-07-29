// Copyright 2026
// Embedded/generic platform glue for the local PDFium integration.

#ifndef _FX_LINUX_EMBEDDED_
#define _FX_LINUX_EMBEDDED_ 5
#endif

#ifndef _FX_OS_
#define _FX_OS_ _FX_LINUX_EMBEDDED_
#endif

#ifndef _FXM_PLATFORM_
#define _FXM_PLATFORM_ 0
#endif

#include "../../../include/fxge/fx_ge.h"
#include "../../../../../../middleware/lvgl/lvsf/lvsf_font.h"

#include <rtthread.h>
#include <string.h>

#define RT_PDF_PLATFORM_LOG(fmt, ...)

#if !defined(_FPDFAPI_MINI_) && \
    _FXM_PLATFORM_ != _FXM_PLATFORM_WINDOWS_ && \
    _FXM_PLATFORM_ != _FXM_PLATFORM_LINUX_ && \
    _FXM_PLATFORM_ != _FXM_PLATFORM_APPLE_ && \
    _FXM_PLATFORM_ != _FXM_PLATFORM_ANDROID_

namespace {

#define RT_PDF_FONT_LOG(fmt, ...)

static FX_DWORD RtPdfGetBE32(const FX_BYTE* data)
{
    return ((FX_DWORD)data[0] << 24) |
           ((FX_DWORD)data[1] << 16) |
           ((FX_DWORD)data[2] << 8) |
           (FX_DWORD)data[3];
}

static FX_BOOL RtPdfFontNameEquals(FX_LPCSTR lhs, FX_LPCSTR rhs)
{
    if (!lhs || !rhs) {
        return FALSE;
    }
    while (*lhs && *rhs) {
        char lc = *lhs;
        char rc = *rhs;
        if (lc >= 'A' && lc <= 'Z') {
            lc = (char)(lc - 'A' + 'a');
        }
        if (rc >= 'A' && rc <= 'Z') {
            rc = (char)(rc - 'A' + 'a');
        }
        if (lc != rc) {
            return FALSE;
        }
        ++lhs;
        ++rhs;
    }
    return *lhs == 0 && *rhs == 0;
}

static FX_BOOL RtPdfFontNameContains(FX_LPCSTR text, FX_LPCSTR token)
{
    if (!text || !token || !*token) {
        return FALSE;
    }
    size_t text_len = strlen(text);
    size_t token_len = strlen(token);
    if (text_len < token_len) {
        return FALSE;
    }
    for (size_t i = 0; i + token_len <= text_len; ++i) {
        if (RtPdfFontNameEquals(text + i, token)) {
            return TRUE;
        }
        size_t j = 0;
        for (; j < token_len; ++j) {
            char lc = text[i + j];
            char rc = token[j];
            if (lc >= 'A' && lc <= 'Z') {
                lc = (char)(lc - 'A' + 'a');
            }
            if (rc >= 'A' && rc <= 'Z') {
                rc = (char)(rc - 'A' + 'a');
            }
            if (lc != rc) {
                break;
            }
        }
        if (j == token_len) {
            return TRUE;
        }
    }
    return FALSE;
}

static FX_BOOL RtPdfGetFontBlob(lv_font_t* font,
                                const char** blob,
                                FX_DWORD* size,
                                FX_BOOL* from_file)
{
    if (!font || !blob || !size || !from_file || !font->font_lib_data) {
        RT_PDF_FONT_LOG("GetFontBlob invalid font=%p blob=%p size=%p from_file=%p lib_data=%p",
                        font, blob, size, from_file,
                        font ? font->font_lib_data : NULL);
        return FALSE;
    }
    *blob = font->font_lib_data;
    *size = (FX_DWORD)font->font_lib_size;
    *from_file = font->font_lib_size == 0 ? TRUE : FALSE;
    RT_PDF_FONT_LOG("GetFontBlob font=%p name=%s lib_data=%p lib_size=%lu from_file=%d",
                    font,
                    font->font_name ? font->font_name : "(null)",
                    font->font_lib_data,
                    (unsigned long)*size,
                    *from_file);
    return TRUE;
}

static FX_DWORD RtPdfReadFileSize(FX_LPCSTR file_path)
{
    FXSYS_FILE* file = FXSYS_fopen(file_path, "rb");
    if (!file) {
        return 0;
    }
    FXSYS_fseek(file, 0, FXSYS_SEEK_END);
    FX_DWORD size = (FX_DWORD)FXSYS_ftell(file);
    FXSYS_fclose(file);
    return size;
}

static FX_DWORD RtPdfReadFontFile(FX_LPCSTR file_path, FX_LPBYTE buffer, FX_DWORD size)
{
    FX_DWORD file_size = RtPdfReadFileSize(file_path);
    if (!file_size) {
        return 0;
    }
    if (!buffer) {
        return file_size;
    }
    if (size < file_size) {
        return file_size;
    }
    FXSYS_FILE* file = FXSYS_fopen(file_path, "rb");
    if (!file) {
        return 0;
    }
    FXSYS_fseek(file, 0, FXSYS_SEEK_SET);
    FXSYS_fread(buffer, file_size, 1, file);
    FXSYS_fclose(file);
    return file_size;
}

static FX_BOOL RtPdfLooksLikeTtc(const FX_LPCBYTE data, FX_DWORD size)
{
    return data && size >= 4 && RtPdfGetBE32(data) == 0x74746366;
}

class CFX_RtthreadFontInfo : public IFX_SystemFontInfo {
public:
    virtual void Release()
    {
        delete this;
    }

    virtual FX_BOOL EnumFontList(CFX_FontMapper* pMapper)
    {
        if (!pMapper) {
            RT_PDF_FONT_LOG("EnumFontList mapper null");
            return FALSE;
        }
        lv_font_t* font = lvsf_get_font_from_size(FONT_NORMAL);
        if (!font || !font->font_name) {
            RT_PDF_FONT_LOG("EnumFontList default font missing font=%p name=%s",
                            font,
                            (font && font->font_name) ? font->font_name : "(null)");
            return FALSE;
        }
        RT_PDF_FONT_LOG("EnumFontList add default font=%p name=%s lib_data=%p lib_size=%lu",
                        font,
                        font->font_name,
                        font->font_lib_data,
                        (unsigned long)font->font_lib_size);
        pMapper->AddInstalledFont(font->font_name, FXFONT_ANSI_CHARSET);
        pMapper->AddInstalledFont(font->font_name, FXFONT_GB2312_CHARSET);
        pMapper->AddInstalledFont(font->font_name, FXFONT_CHINESEBIG5_CHARSET);
        pMapper->AddInstalledFont(font->font_name, FXFONT_SHIFTJIS_CHARSET);
        pMapper->AddInstalledFont(font->font_name, FXFONT_HANGEUL_CHARSET);
        return TRUE;
    }

    virtual void* MapFont(int weight,
                          FX_BOOL bItalic,
                          int charset,
                          int pitch_family,
                          FX_LPCSTR face,
                          FX_BOOL& bExact)
    {
        (void)weight;
        (void)bItalic;
        (void)charset;
        (void)pitch_family;
        bExact = FALSE;

        RT_PDF_FONT_LOG("MapFont request face=%s weight=%d italic=%d charset=%d pitch_family=%d",
                        face ? face : "(null)", weight, bItalic, charset, pitch_family);

        lv_font_t* font = FindFont(face, TRUE);
        if (font) {
            bExact = TRUE;
            RT_PDF_FONT_LOG("MapFont exact hit face=%s -> font=%p name=%s",
                            face ? face : "(null)",
                            font,
                            font->font_name ? font->font_name : "(null)");
            return font;
        }

        if (face && *face) {
            font = FindFont(face, FALSE);
            if (font) {
                RT_PDF_FONT_LOG("MapFont fuzzy hit face=%s -> font=%p name=%s",
                                face,
                                font,
                                font->font_name ? font->font_name : "(null)");
                return font;
            }
        }

        font = lvsf_get_font_from_size(FONT_NORMAL);
        RT_PDF_FONT_LOG("MapFont fallback default face=%s -> font=%p name=%s",
                        face ? face : "(null)",
                        font,
                        (font && font->font_name) ? font->font_name : "(null)");
        return font;
    }

    virtual void* GetFont(FX_LPCSTR face)
    {
        return FindFont(face, TRUE);
    }

    virtual FX_DWORD GetFontData(void* hFont, FX_DWORD table, FX_LPBYTE buffer, FX_DWORD size)
    {
        lv_font_t* font = (lv_font_t*)hFont;
        const char* blob = NULL;
        FX_DWORD blob_size = 0;
        FX_BOOL from_file = FALSE;
        RT_PDF_FONT_LOG("GetFontData request hFont=%p name=%s table=0x%08lx buffer=%p size=%lu",
                        hFont,
                        (font && font->font_name) ? font->font_name : "(null)",
                        (unsigned long)table,
                        buffer,
                        (unsigned long)size);
        if (!RtPdfGetFontBlob(font, &blob, &blob_size, &from_file)) {
            RT_PDF_FONT_LOG("GetFontData no blob hFont=%p", hFont);
            return 0;
        }

        if (from_file) {
            RT_PDF_FONT_LOG("GetFontData using file path=%s", blob ? blob : "(null)");
            if (table == 0) {
                FX_DWORD ret = RtPdfReadFontFile(blob, buffer, size);
                RT_PDF_FONT_LOG("GetFontData file full ret=%lu", (unsigned long)ret);
                return ret;
            }
            if (table == 0x74746366) {
                FX_BYTE tag[4] = {0};
                FXSYS_FILE* file = FXSYS_fopen(blob, "rb");
                if (!file) {
                    return 0;
                }
                FXSYS_fread(tag, 4, 1, file);
                FXSYS_fclose(file);
                if (RtPdfGetBE32(tag) == 0x74746366) {
                    FX_DWORD ret = RtPdfReadFontFile(blob, buffer, size);
                    RT_PDF_FONT_LOG("GetFontData file ttc ret=%lu", (unsigned long)ret);
                    return ret;
                }
            }
            RT_PDF_FONT_LOG("GetFontData file table miss table=0x%08lx", (unsigned long)table);
            return 0;
        }

        if (table == 0) {
            if (!buffer) {
                RT_PDF_FONT_LOG("GetFontData mem full query ret=%lu", (unsigned long)blob_size);
                return blob_size;
            }
            if (size < blob_size) {
                RT_PDF_FONT_LOG("GetFontData mem full short buffer need=%lu got=%lu",
                                (unsigned long)blob_size, (unsigned long)size);
                return blob_size;
            }
            FXSYS_memcpy32(buffer, blob, blob_size);
            RT_PDF_FONT_LOG("GetFontData mem full copy ret=%lu", (unsigned long)blob_size);
            return blob_size;
        }
        if (table == 0x74746366 && RtPdfLooksLikeTtc((FX_LPCBYTE)blob, blob_size)) {
            if (!buffer) {
                RT_PDF_FONT_LOG("GetFontData mem ttc query ret=%lu", (unsigned long)blob_size);
                return blob_size;
            }
            if (size < blob_size) {
                RT_PDF_FONT_LOG("GetFontData mem ttc short buffer need=%lu got=%lu",
                                (unsigned long)blob_size, (unsigned long)size);
                return blob_size;
            }
            FXSYS_memcpy32(buffer, blob, blob_size);
            RT_PDF_FONT_LOG("GetFontData mem ttc copy ret=%lu", (unsigned long)blob_size);
            return blob_size;
        }
        RT_PDF_FONT_LOG("GetFontData mem table miss table=0x%08lx blob_size=%lu",
                        (unsigned long)table, (unsigned long)blob_size);
        return 0;
    }

    virtual FX_BOOL GetFaceName(void* hFont, CFX_ByteString& name)
    {
        lv_font_t* font = (lv_font_t*)hFont;
        if (!font || !font->font_name) {
            RT_PDF_FONT_LOG("GetFaceName failed hFont=%p", hFont);
            return FALSE;
        }
        name = font->font_name;
        RT_PDF_FONT_LOG("GetFaceName hFont=%p -> %s", hFont, font->font_name);
        return TRUE;
    }

    virtual FX_BOOL GetFontCharset(void* hFont, int& charset)
    {
        RT_PDF_FONT_LOG("GetFontCharset hFont=%p -> %d", hFont, FXFONT_GB2312_CHARSET);
        charset = FXFONT_GB2312_CHARSET;
        return TRUE;
    }

    virtual void DeleteFont(void* hFont)
    {
        (void)hFont;
    }

private:
    static lv_font_t* FindFont(FX_LPCSTR face, FX_BOOL exact_only)
    {
        lv_font_t* font = lvsf_get_font_from_size(FONT_NORMAL);
        if (!font) {
            RT_PDF_FONT_LOG("FindFont no default font face=%s exact=%d",
                            face ? face : "(null)", exact_only);
            return NULL;
        }
        if (!face || !*face) {
            RT_PDF_FONT_LOG("FindFont empty face exact=%d -> %p/%s",
                            exact_only, font,
                            font->font_name ? font->font_name : "(null)");
            return exact_only ? NULL : font;
        }
        if (!font->font_name) {
            RT_PDF_FONT_LOG("FindFont default font name null face=%s exact=%d",
                            face, exact_only);
            return exact_only ? NULL : font;
        }
        if (RtPdfFontNameEquals(font->font_name, face)) {
            RT_PDF_FONT_LOG("FindFont exact match face=%s -> %p/%s",
                            face, font, font->font_name);
            return font;
        }
        if (!exact_only &&
            (RtPdfFontNameContains(font->font_name, face) || RtPdfFontNameContains(face, font->font_name))) {
            RT_PDF_FONT_LOG("FindFont fuzzy match face=%s -> %p/%s",
                            face, font, font->font_name);
            return font;
        }
        RT_PDF_FONT_LOG("FindFont miss face=%s exact=%d fallback=%p/%s",
                        face, exact_only, font, font->font_name);
        return exact_only ? NULL : font;
    }
};

} // namespace

IFX_SystemFontInfo* IFX_SystemFontInfo::CreateDefault()
{
    IFX_SystemFontInfo* info = FX_NEW CFX_RtthreadFontInfo;
    RT_PDF_PLATFORM_LOG("CreateDefault -> %p", info);
    return info;
}

void CFX_GEModule::InitPlatform()
{
    RT_PDF_PLATFORM_LOG("InitPlatform enter this=%p fontmgr=%p", this, m_pFontMgr);
    m_pPlatformData = NULL;
    if (m_pFontMgr) {
        IFX_SystemFontInfo* info = IFX_SystemFontInfo::CreateDefault();
        RT_PDF_PLATFORM_LOG("InitPlatform SetSystemFontInfo info=%p", info);
        m_pFontMgr->SetSystemFontInfo(info);
    } else {
        RT_PDF_PLATFORM_LOG("InitPlatform skip because fontmgr is null");
    }
}

void CFX_GEModule::DestroyPlatform()
{
    RT_PDF_PLATFORM_LOG("DestroyPlatform this=%p", this);
    m_pPlatformData = NULL;
}

#endif
