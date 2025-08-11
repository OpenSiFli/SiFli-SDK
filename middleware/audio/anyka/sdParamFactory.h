
#ifndef __SD_PARAM_FACTORY_H__
#define __SD_PARAM_FACTORY_H__

#include "anyka_types.h"

#ifdef __cplusplus
extern "C" {
#endif


/*************** APIs ****************/

typedef T_VOID *T_pSD_PARAM_FACTORY;

typedef struct
{
    // range of section content, not including section name
    const char *pStart;
    const char *pEnd;

    // section name, not including embracing []
    const char *pName;   // note: it is not a null-terminated string
    T_S32       nName;   // size of section name
} T_SD_INI_SECTION_INFO;


/**
 * @brief    设置 ParamFactory 库的 debug zones，选择打印信息的类型
 * @param    [in] bit-or of SD_ZONE_ID_XXX.
 *                Default is SD_DEFAULT_DEBUG_ZONES
 * @return   T_VOID
 */
T_VOID SD_ParamFactory_SetDebugZones(T_U32 debugZones);

// strCmdLine: input string. The API will not use it after return.
//   If line break or '\0' exists, all following characters are omitted.
// size:
//    -1: strCmdLine is a null-terminated string
//   >=0: bytes of the content in strCmdLine, may or may not including the terninal '\0'.
T_pSD_PARAM_FACTORY SD_ParamFactory_Create_ByCmdLine(const char *strCmdLine, T_S32 size);

// only used in main() to parse cmdline args
T_pSD_PARAM_FACTORY SD_ParamFactory_Create_ByCmdLineArgs(T_S32 argc, char *argv[]);

// strCfg: content of the cfg file. It may not be null-terminated.
//   The API will not use it after return. The string content is not modified.
// fileSize: size in byte of the strCfg buffer.
// strSectionName: name string of the section, not including embracing []. Only the designated section is parsed.
//   Can be null if there is no section name in the cfg file, so all params in it are parsed.
T_pSD_PARAM_FACTORY SD_ParamFactory_Create_ByCfgFile(const char *strCfg, T_S32 fileSize, char *strSectionName);

// merge two factories.
// return AK_TRUE or AK_FALSE
// 后续调用 (get / destory 等）只用对 main_factory 操作
// 如果 main_factory 和 secondary_factory 中都有某个参数，以 main_factory 优先
T_S32 SD_ParamFactory_Merge(T_pSD_PARAM_FACTORY main_factory, T_pSD_PARAM_FACTORY secondary_factory);

T_VOID SD_ParamFactory_Destroy(T_pSD_PARAM_FACTORY param_factory);

// return: value string or AK_NULL if not found
//         the string shall be used immediately, for its content may be changed by other SD_ParamFactory_ calls
// each successful call will increase the param's ref count
char *SD_ParamFactory_GetValue(T_pSD_PARAM_FACTORY param_factory, const char *param_name);
// return: const value string or AK_NULL if not found
//         the string is asynchronous, which can be used freely until SD_ParamFactory_Destroy
// each successful call will increase the param's ref count
T_pCSTR SD_ParamFactory_GetValueAsynchronous(T_pSD_PARAM_FACTORY param_factory, const char *param_name);

// masked param can not be GetValue.
// return AK_TRUE or AK_FALSE
// each successful call will increase the param's ref count
T_S32 SD_ParamFactory_MaskParam(T_pSD_PARAM_FACTORY param_factory, const char *param_name);
// each successful call will increase the param's ref count
T_S32 SD_ParamFactory_UnmaskParam(T_pSD_PARAM_FACTORY param_factory, const char *param_name);

// reset enumeration state. all following calls to SD_ParamFactory_EnumXxx will enumerate from the first param.
T_VOID SD_ParamFactory_Enum_Reset(T_pSD_PARAM_FACTORY param_factory);

// Enumerate those params one by one, whose ref count is less than max_count
// return a param_name string or AK_NULL for the end
char *SD_ParamFactory_EnumRefCountLessThan(T_pSD_PARAM_FACTORY param_factory, T_S32 max_count);

/*************** methods with no handle ****************/

// These methods should be called in a single thread manner. That is, they are not multi-thead safe.

// return
//   AK_TRUE: using subsequent callings of SD_ParamFactory_CfgFile_EnumSection to enumerate section_info one by one.
//   AK_FALSE: there is no section in the cfgFile
T_S32 SD_ParamFactory_CfgFile_EnumSectionStart(const char *strCfg, T_S32 fileSize);

// brief: consecutive calls to this method enumerate section info from the CfgFile.
// return
//   AK_NULL: to the end of the CfgFile.
//   others: a ptr to section_info struct, describing section name etc.
//           User should use the content immediately, and should not change anything in it.
const T_SD_INI_SECTION_INFO *SD_ParamFactory_CfgFile_EnumSection(T_VOID);


/*************** helper tools to handle literal value ****************/

// 参数配置项的形式是 name_string=value_string.

struct S_NAME_MAP
{
    T_pCSTR  name;
    T_S32    val;
};

const char *print_map_names(const struct S_NAME_MAP *map, const char *delimit);
// return:
//   int value corresponding to name found in map
//   same as str_to_int, if name is not found in map
T_S32 name_to_int(const struct S_NAME_MAP *map, const char *name);
// return:
//   name string corresponding to value found in map
//   AK_NULL, if value is not found in map
const char *int_to_name(const struct S_NAME_MAP *map, T_S32 value);

// 用 str_to_xxx(value_string)) 将字符串转成合适的数据类型. 如果转换不成功，返回 0.
T_S32 str_to_int(const char *str); // str can be decimal number or prefixed with 0x, 0X or 0
double str_to_float(const char *str);
T_S32 str_to_pcm(const char *str); // str can be pcm value, x.xxdB or x.xxFS. return Q15 pcm value.

#ifndef PARAM_FACTORY_NAME
#define PARAM_FACTORY_NAME factory
#endif

// 用 VALUE_OF(name_string) 抓取 value_string. 如果参数集中没有找到 name_string，value_string 是 NULL.
// 用 var=str_to_xxx(VALUE_OF(name_string)) 转成合适的数据类型. 如果参数集中没有找到 name_string，var 是 0.
// 如果 var 已有初始值，只在参数集中存在 name_string 时才覆盖，就用 OVERRIDE_IF_EXIST(var, str_to_xxx, name_string).
#define VALUE_OF(name)  SD_ParamFactory_GetValue(PARAM_FACTORY_NAME, name)
#define PARAM_EXIST(name)  ((strValue=VALUE_OF(name)) ? 1 : 0)
#define OVERRIDE_IF_EXIST(var, convert_func, name)  (PARAM_EXIST(name)? (var=convert_func(strValue)) : 0)


/*************** a demo to illustrate how to parse values ****************
#include <string.h>

static const struct S_NAME_MAP eqmode_name_map[] =
{
    {"normal", _SD_EQ_MODE_NORMAL},
    {"user",   _SD_EQ_USER_DEFINE},
    {NULL,     0},
};
static const struct S_NAME_MAP eqbandtype_name_map[] =
{
    {"hpf",    FILTER_TYPE_HPF},
    {"HPF",    FILTER_TYPE_HPF},
    {"lpf",    FILTER_TYPE_LPF},
    {"LPF",    FILTER_TYPE_LPF},
    {"hsf",    FILTER_TYPE_HSF},
    {"HSF",    FILTER_TYPE_HSF},
    {"lsf",    FILTER_TYPE_LSF},
    {"LSF",    FILTER_TYPE_LSF},
    {"pf1",    FILTER_TYPE_PF1},
    {"PF1",    FILTER_TYPE_PF1},
    {NULL,     0},
};
static const struct S_NAME_MAP eqbypass_name_map[] =
{
    {"bypass",   1},
    {"enable",   0},
    {"disable",  1},
    {NULL,       0},
};

static void parse_eq_param(T_pSD_PARAM_FACTORY factory, struct sd_param_eq *eq_param)
{
    char *strValue;
    const char *delimit = " ,\t";
    int i;

    if (PARAM_EXIST("eqmode"))
    {
        eq_param->eqmode  = name_to_int(eqmode_name_map, strValue);
        eqLoad = strValue? 1 : 0;
    }
    OVERRIDE_IF_EXIST(eqLoad, str_to_int, "eqLoad");
    if (PARAM_EXIST("preGain"))
        eq_param->preGain = AK16Q10(str_to_float(strValue));

    if (PARAM_EXIST("bands"))
    {
        char *band_params = strValue;
        char *next_tok;

        band_params = strtok_r(band_params, delimit, &next_tok);
        eq_param->bands = str_to_int(band_params);
        AK_ASSERT(eq_param->bands <= _SD_EQ_MAX_BANDS);
        for (i=0; i<eq_param->bands; i++)
        {
            T_S32  type;
            T_S32  freq_Hz;
            double gain_dB;
            double q;
            T_S32  bypass;

            type    = name_to_int(eqbandtype_name_map, strtok_r(NULL, delimit, &next_tok));
            freq_Hz = str_to_int(strtok_r(NULL, delimit, &next_tok));
            gain_dB = str_to_float(strtok_r(NULL, delimit, &next_tok));
            q       = str_to_float(strtok_r(NULL, delimit, &next_tok));
            bypass  = name_to_int(eqbypass_name_map, strtok_r(NULL, delimit, &next_tok));

            _SET_EQ_BAND_ALL(eq_param, i, freq_Hz, gain_dB, q, type, bypass);
        }
    }
}


static void print_eq_param(void)
{
    printf("  eqLoad={1|0}                      load eq? default is whether eqmode is set\n");
    printf("  eqmode={ %s }\n", print_map_names(eqmode_name_map, " | "));
    printf("  preGain={x.xxdB}\n");
    printf("  bands=\"<#band>, <band0_param>, <band1_param>, ...\"\n");
    printf("      each <bandx_param> is a set of <type> <freq_Hz> <gain_dB> <Q> <bypass>\n");
    printf("      type is one of { %s }\n", print_map_names(eqbandtype_name_map, ", "));
    printf("      bypass is one of { %s }\n", print_map_names(eqbypass_name_map, ", "));
}

*/

#ifdef __cplusplus
}
#endif

#endif // __SD_PARAM_FACTORY_H__
