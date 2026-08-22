/*----------------------------------------------*/
/* TJpgDec System Configurations R0.03          */
/*----------------------------------------------*/

#ifndef LVSF_SJPG_OUTPUT_YUV420
    #define LVSF_SJPG_OUTPUT_YUV420 0
#endif

#if LVSF_USING_SJPG
    #define JD_SZBUF        1280
    /* Specifies size of stream input buffer */
    #if LVSF_SJPG_OUTPUT_YUV420 && !defined(_MSC_VER)
        #define JD_FORMAT       3
    #else
        #define JD_FORMAT       0
    #endif
    /* Specifies output pixel format.
    /  0: RGB888 (24-bit/pix)
    /  1: RGB565 (16-bit/pix)
    /  2: Grayscale (8-bit/pix)
    /  3: YUV420 PLANAR
    */

    #define JD_USE_SCALE    0
    /* Switches output descaling feature.
    /  0: Disable
    /  1: Enable
    */
#else
    #define JD_SZBUF        512
    /* Specifies size of stream input buffer */

    #define JD_FORMAT       0
    /* Specifies output pixel format.
    /  0: RGB888 (24-bit/pix)
    /  1: RGB565 (16-bit/pix)
    /  2: Grayscale (8-bit/pix)
    */

    #define JD_USE_SCALE    1
    /* Switches output descaling feature.
    /  0: Disable
    /  1: Enable
    */
#endif

#define JD_TBLCLIP      1
/* Use table conversion for saturation arithmetic. A bit faster, but increases 1 KB of code size.
/  0: Disable
/  1: Enable
*/

#define JD_FASTDECODE   0
/* Optimization level
/  0: Basic optimization. Suitable for 8/16-bit MCUs.
/  1: + 32-bit barrel shifter. Suitable for 32-bit MCUs.
/  2: + Table conversion for huffman decoding (wants 6 << HUFF_BIT bytes of RAM)
*/
