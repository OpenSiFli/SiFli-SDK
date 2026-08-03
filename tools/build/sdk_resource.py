# SPDX-FileCopyrightText: 2019-2026 SiFli
# SPDX-License-Identifier: Apache-2.0

import codecs
import glob
import json
import os
import subprocess
import shutil
import sys
# import png
import logging
import ptab
import re
from collections import OrderedDict

output_folder = "output"
img_folder = "images"
string_folder = "strings"
font_folder = "fonts"
font_config_src_file = "fonts/config.json"
font_config_output_file = "fonts_config"
lang_pack_name = "lang_pack"
# Keep string keys in a list, not a set.
# The generated lang_pack.h field order must stay deterministic and must match
# the packing order used by external language-pack generation.
resource_db = {'images': set(),
               'strings': []}

# gen_src_file_folder = "../applications"

# Generate image resource file
def GenerateImgRes(res_root):
    img_src_dir = os.path.join(res_root)
    img_output_dir = os.path.join(output_folder, img_folder)
    imgs = (glob.glob(img_src_dir + '/*.png')
            + glob.glob(img_src_dir + '/*.jpg')
            + glob.glob(img_src_dir + '/*.jpeg'))
    if not os.path.exists(img_output_dir):
        os.makedirs(img_output_dir)
    else:
        file_list = glob.glob(os.path.join(img_output_dir, "*.*"))
        file_list += glob.glob(os.path.join(img_output_dir, "SConscript"))
        [os.remove(f) for f in file_list]

    tools_dir = os.path.join(os.getenv('SIFLI_SDK'), "tools")
    PHP_PATH = os.path.join(tools_dir, "img_converter/php")
    SCRIPT_PATH = os.path.join(tools_dir, "img_converter/lv_utils-master/img_conv_core.php")
    OUTPUT_PATH = img_output_dir
    for f in imgs:
        filename = os.path.basename(f)
        res_name = "img_" + os.path.splitext(filename)[0]
        png1 = png.Reader(f)
        if png1.asDirect()[3]['alpha'] == False:
            print("%s is true_color" % (res_name))
            param = "cf=true_color&name={}&img={}&outdir={}".format(res_name, f, OUTPUT_PATH)
        else:
            print("%s is true_color_alpha" % (res_name))
            param = "cf=true_color_alpha&name={}&img={}&outdir={}".format(res_name, f, OUTPUT_PATH)
        subprocess.call([PHP_PATH, SCRIPT_PATH, param])
        resource_db['images'].add(res_name)

    if ns:
        return

    # Generate SConscript and build .so file
    template = \
        '''
from building import *

src = Glob('{}')
cwd = GetCurrentDir()

CPPPATH = [cwd]
group = DefineGroup('', src, depend = [''], CPPPATH=CPPPATH)

Return('group')
'''
    data = template.format("*.c")
    script_file_fullpath = os.path.join(img_output_dir, "SConscript")
    try:
        f = codecs.open(script_file_fullpath, "w", "utf-8")
        f.write(data)
    finally:
        f.close()

    so_file_fullpath = os.path.join(output_folder, "images.so")
    BuildLibrary(so_file_fullpath, script_file_fullpath)


# s: translation dictionary
def GenerateLangCFile(lang, s, output_dir):
    template = \
        u'''
#include "lv_ext_resource_manager.h"

const lang_translation_t {}_lang_translation = 
{{
{}
}};

const lv_i18n_lang_t {}_lang = 
{{
    .locale = "{}",
    .translation = &{}_lang_translation,
}};
'''
    data = u''
    # Do not collect resource_db['strings'] here.
    # Key schema is resolved centrally from the full language set so builtin
    # headers and external .bin packs always share the same field order.
    for k, v in s.items():
        # k = "str_" + k
        # data = data + u'char *{} = "{}";\n'.format(k,v)
        # key and singular
        translation = u'"{}", "{}"'.format(k, v)
        # plural
        translation = translation + u', {0}'
        data = data + u"    .{} = {{{}}},\n".format(k, translation)
    data = template.format(lang, data, lang, lang, lang)

    str_cfile_filename = lang + ".c"
    str_cfile_fullpath = os.path.join(output_dir, str_cfile_filename)
    try:
        f = codecs.open(str_cfile_fullpath, "w", "utf-8")
        f.write(data)
    finally:
        f.close()


def _load_lang_jsons(str_src_dir):
    lang_files = sorted(glob.glob(os.path.join(str_src_dir, '*.json')))
    lang_items = []

    for f_name in lang_files:
        try:
            f = open(f_name, encoding='utf-8')
            # Preserve key order from the source json so the generated header
            # layout follows the declared language schema exactly.
            s = json.load(f, object_pairs_hook=OrderedDict)
        finally:
            f.close()

        str_file_basename = os.path.basename(f_name)
        str_file_basename = os.path.splitext(str_file_basename)[0]
        lang_items.append((str_file_basename, s))

    return lang_items


def _resolve_string_key_order(lang_items):
    if not lang_items:
        return []

    source_dict = None

    # Use the default language as the canonical schema source when possible.
    # This keeps generated struct fields stable across builds and boards.
    for lang, data in lang_items:
        if lang == default_language:
            source_dict = data
            break

    if source_dict is None:
        _, source_dict = lang_items[0]

    key_order = list(source_dict.keys())
    key_set = set(key_order)

    # All language json files must describe the same key set. If not, fail
    # early instead of generating a mismatched lang_pack.h silently.
    for lang, data in lang_items:
        current_key_set = set(data.keys())
        if current_key_set != key_set:
            missing = sorted(key_set - current_key_set)
            extra = sorted(current_key_set - key_set)
            err = ["language json keys mismatch: {}".format(lang)]
            if missing:
                err.append("missing: {}".format(", ".join(missing)))
            if extra:
                err.append("extra: {}".format(", ".join(extra)))
            raise RuntimeError("; ".join(err))

    resource_db['strings'] = key_order
    return key_order


def GenerateLangPackCFile(lang_pack, output_dir):
    InitIndentation()
    s = MakeLine('#include "lv_ext_resource_manager.h"')
    #  s += MakeLine('#include "resource.h"')
    s += MakeLine('')
    s += MakeLine('')

    for lang in lang_pack:
        s += MakeLine('extern const lv_i18n_lang_t {}_lang;'.format(lang))

    s += MakeLine('')
    s += MakeLine('const lv_i18n_lang_t * const lv_i18n_lang_pack[] = ')
    s += MakeLine('{')
    IncIndentation()
    if default_language in lang_pack:
        s += MakeLine('&{}_lang,'.format(default_language))
    for lang in lang_pack:
        if lang != default_language:
            s += MakeLine('&{}_lang,'.format(lang))
    s += MakeLine('NULL //end mark')
    DecIndentation()
    s += MakeLine('};')

    str_cfile_filename = "{}.c".format(lang_pack_name)
    str_cfile_fullpath = os.path.join(output_dir, str_cfile_filename)
    try:
        f = codecs.open(str_cfile_fullpath, "w", "utf-8")
        f.write(s)
    finally:
        f.close()


def GenerateLangPackHFile(lang_pack, output_dir):
    str_cfile_filename = "{}.h".format(lang_pack_name)
    FILENAME_MACRO = "__{}__".format(lang_pack_name)
    PREFIX = "m_res_"

    InitIndentation()
    # file header
    s = MakeLine('#ifndef {}'.format(FILENAME_MACRO))
    s += MakeLine('#define {}'.format(FILENAME_MACRO))

    s += MakeLine('')
    s += MakeLine('')

    # file body

    s += MakeLine("typedef struct")
    s += MakeLine("{")
    IncIndentation()
    for i in resource_db['strings']:
        s += MakeLine('lv_i18n_phrase_t {};'.format(i))
    DecIndentation()
    s += MakeLine("} lang_translation_t;")
    s += MakeLine("")
    s += MakeLine("extern const lv_i18n_lang_t * const lv_i18n_lang_pack[];")

    # file footer
    s += MakeLine('')
    s += MakeLine('#endif /* {} */'.format(FILENAME_MACRO))

    try:
        f = open(os.path.join(output_dir, str_cfile_filename), "w")
        f.write(s)
    finally:
        f.close()

    # shutil.copy(os.path.join(output_root, filename), gen_src_file_folder)


# Generate string resource file
def GenerateStrRes(str_src_dir, str_output_dir):
    lang_items = _load_lang_jsons(str_src_dir)
    # Reset the shared key cache for each generation pass so incremental builds
    # do not reuse stale schema data from previous invocations.
    resource_db['strings'] = []
    if not os.path.exists(str_output_dir):
        os.makedirs(str_output_dir)
    else:
        file_list = glob.glob(os.path.join(str_output_dir, "*.*"))
        file_list += glob.glob(os.path.join(str_output_dir, "SConscript"))
        [os.remove(f) for f in file_list]

    key_order = _resolve_string_key_order(lang_items)
    if not key_order:
        return

    # Keep language pack names ordered deterministically for reproducible
    # generated sources. Do not switch this back to a set.
    lang_pack = []
    for str_file_basename, s in lang_items:
        GenerateLangCFile(str_file_basename, s, str_output_dir)
        lang_pack.append(str_file_basename)

    GenerateLangPackCFile(lang_pack, str_output_dir)
    GenerateLangPackHFile(lang_pack, str_output_dir)

    if ns:
        return

    # Generate SConscript and build .so file
    template = \
        '''
from building import *

src = Glob('./*.c')
cwd = GetCurrentDir()

CPPPATH = [cwd]
group = DefineGroup('language', src, depend = [''], CPPPATH=CPPPATH)

Return('group')
'''
    data = template.format(lang_pack_name)
    script_file_fullpath = os.path.join(str_output_dir, "SConscript")
    try:
        f = codecs.open(script_file_fullpath, "w", "utf-8")
        f.write(data)
    finally:
        f.close()

    from ua import BuildLibrary
    str_so_file_fullpath = os.path.join(str_output_dir, "{}.so".format(lang_pack_name))
    BuildLibrary("{}.so".format(lang_pack_name), script_file_fullpath)


# Generate string resource file
def GenerateStrRes2(str_output_dir, lang_pack_name):
    # Generate SConscript and build .so file
    template = \
        '''
from building import *

src = Glob('./*.c')
cwd = GetCurrentDir()

CPPPATH = [cwd]
group = DefineGroup('language', src, depend = [''], CPPPATH=CPPPATH)

Return('group')
'''
    data = template.format(lang_pack_name)
    script_file_fullpath = os.path.join(str_output_dir, "SConscript")
    try:
        f = codecs.open(script_file_fullpath, "w", "utf-8")
        f.write(data)
    finally:
        f.close()

    if not os.path.isdir("output"):
        os.mkdir("output")

    from ua import BuildLibrary
    str_so_file_fullpath = os.path.join(str_output_dir, "{}.so".format(lang_pack_name))
    BuildLibrary("{}.so".format(lang_pack_name), script_file_fullpath)

    SIFLI_SDK = os.getenv('SIFLI_SDK')
    pgm_output = "{}.so".format(lang_pack_name)
    dsc_filename = os.path.join("output", "{}.dsc".format(lang_pack_name))
    dsc_generator = os.path.join(SIFLI_SDK, "tools/dsc_generator/dsc_generator.exe")
    subprocess.call([dsc_generator, lang_pack_name, lang_pack_name, pgm_output, "", "", dsc_filename])


# cfg: fonts config dictionary
def GenerateFontsConfigCFile(cfg, output_dir):
    InitIndentation()
    s = MakeLine('#include "lv_ext_resource_manager.h"')
    s += MakeLine('#include "resource.h"')
    s += MakeLine('')
    s += MakeLine('const lv_ex_font_config_t lv_ex_font_config[] = ')
    s += MakeLine('{')
    IncIndentation()
    for k in sorted(cfg):
        s += MakeLine('{{{}, &{}}},'.format(int(k), cfg[k]))
    s += MakeLine('{-1, NULL}')
    DecIndentation()
    s += MakeLine('};')

    cfile_filename = "{}.c".format(font_config_output_file)
    cfile_fullpath = os.path.join(output_dir, cfile_filename)
    try:
        f = codecs.open(cfile_fullpath, "w", "utf-8")
        f.write(s)
    finally:
        f.close()

    # Generate string resource file


def GenerateFontsRes(res_root):
    src_dir = os.path.join(res_root, font_config_src_file)
    fonts_output_dir = os.path.join(output_folder, font_folder)
    if not os.path.exists(fonts_output_dir):
        os.makedirs(fonts_output_dir)
    else:
        file_list = glob.glob(os.path.join(fonts_output_dir, "*.*"))
        file_list += glob.glob(os.path.join(fonts_output_dir, "SConscript"))
        [os.remove(f) for f in file_list]

    try:
        f = open(src_dir)
        s = json.load(f)
    finally:
        f.close()

    file_basename = os.path.basename(src_dir)
    file_basename = os.path.splitext(file_basename)[0]

    GenerateFontsConfigCFile(s, fonts_output_dir)

    if ns:
        return

    # Generate SConscript and build .so file
    template = \
        '''
from building import *

src = Glob('{}.c')
cwd = GetCurrentDir()

CPPPATH = [cwd]
group = DefineGroup('', src, depend = [''], CPPPATH=CPPPATH)

Return('group')
'''
    data = template.format(font_config_output_file)
    script_file_fullpath = os.path.join(str_output_dir, "SConscript")
    try:
        f = codecs.open(script_file_fullpath, "w", "utf-8")
        f.write(data)
    finally:
        f.close()

    str_so_file_fullpath = os.path.join(output_folder, "{}.so".format(font_config_output_file))
    BuildLibrary(str_so_file_fullpath, script_file_fullpath)


def CleanGeneratedFile():
    output_root = output_folder
    print(output_folder)
    [os.remove(f) for f in glob.glob(output_root + "/*.*")]


def InitIndentation():
    global level
    level = 0


def IncIndentation():
    global level
    level += 1


def DecIndentation():
    global level
    level -= 1
    if (level < 0):
        level = 0


# return space for indentation
def GetIndentationSpace():
    return ' ' * level * 4


# make one line according to indentation level
def MakeLine(s):
    return GetIndentationSpace() + s + '\n'


# Generate resource.h
def GenerateResourceHFile(res_root):
    output_root = output_folder
    filename = "resource.h"
    FILENAME_MACRO = "RESOURCE_H_"
    PREFIX = "m_res_"

    InitIndentation()
    # file header
    s = MakeLine('#ifndef {}'.format(FILENAME_MACRO))
    s += MakeLine('#define {}'.format(FILENAME_MACRO))

    s += MakeLine('')

    s += MakeLine('lv_res_t resource_init(void);')

    s += MakeLine('')

    # file body
    for i in resource_db['images']:
        s += MakeLine('#define {}{} "{}"'.format(PREFIX, i, i))

    s += MakeLine("typedef struct")
    s += MakeLine("{")
    IncIndentation()
    for i in resource_db['strings']:
        s += MakeLine('lv_i18n_phrase_t {};'.format(i))
    DecIndentation()
    s += MakeLine("} lang_translation_t;")
    s += MakeLine("")
    s += MakeLine("extern const lv_i18n_lang_t * const lv_i18n_lang_pack[];")
    s += MakeLine("extern const lv_ex_font_config_t lv_ex_font_config[];")

    # file footer
    s += MakeLine('')
    s += MakeLine('#endif /* {} */'.format(FILENAME_MACRO))

    try:
        f = open(os.path.join(output_root, filename), "w")
        f.write(s)
    finally:
        f.close()

    # shutil.copy(os.path.join(output_root, filename), gen_src_file_folder)


def GenerateResourceCFileForStandalone(res_root, output_root, filename):
    InitIndentation()
    # file header
    s = MakeLine('#include <rtthread.h>')
    s += MakeLine('#include "lvgl.h"')
    s += MakeLine('#include "lv_ext_resource_manager.h"')

    s += MakeLine('')
    s += MakeLine('')
    s += MakeLine('static lv_ext_res_mng_dl_impl_data_t res_mng_data;')
    s += MakeLine('')

    # file body
    s += MakeLine('lv_res_t resource_init(void)')
    s += MakeLine('{')

    # function body
    IncIndentation()
    s += MakeLine('res_mng_data.img_module_name = "/images.so";')
    s += MakeLine('res_mng_data.str_module_name = "/{}.so";'.format(lang_pack_name))
    s += MakeLine('if (LV_RES_OK != lv_ext_res_mng_init(&res_mng_data))')
    s += MakeLine('{')
    IncIndentation()
    s += MakeLine('rt_kprintf("resource loading fail\\n");')
    s += MakeLine('rt_kprintf("please download resource file first\\n");')
    s += MakeLine('return LV_RES_INV;')
    DecIndentation()
    s += MakeLine('}')

    s += MakeLine('')

    s += MakeLine('return LV_RES_OK;')

    DecIndentation()
    s += MakeLine('}')

    try:
        f = open(os.path.join(output_root, filename), "w")
        f.write(s)
    finally:
        f.close()


def GenerateResourceCFileForNonStandalone(res_root, output_root, filename):
    InitIndentation()
    # file header
    s = MakeLine('#include <rtthread.h>')
    s += MakeLine('#include "lvgl.h"')
    s += MakeLine('#include "lv_ext_resource_manager.h"')

    s += MakeLine('')
    s += MakeLine('/* Include resource file */')
    s += MakeLine('/* image resource */')
    for i in resource_db['images']:
        s += MakeLine('#define LV_ATTRIBUTE_IMG_{} SECTION({})'.format(i.upper(), ns_sec_name))
        s += MakeLine(
            '#include "../resources/{output}/{images}/{file}.c"'.format(output=output_folder, images=img_folder,
                                                                        file=i))
    s += MakeLine('/* string resource */')
    s += MakeLine(
        '#include "../resources/{output}/{strings}/{file}.c"'.format(output=output_folder, strings=string_folder,
                                                                     file=lang_pack_name))
    s += MakeLine('#include "../resources/{output}/{fonts}/{file}.c"'.format(output=output_folder, fonts=font_folder,
                                                                             file=font_config_output_file))

    s += MakeLine('')
    # file body
    s += MakeLine('lv_res_t resource_init(void)')
    s += MakeLine('{')

    # function body
    IncIndentation()
    s += MakeLine('return lv_ext_res_mng_init(NULL);')
    DecIndentation()
    s += MakeLine('}')

    try:
        f = open(os.path.join(output_root, filename), "w")
        f.write(s)
    finally:
        f.close()


# Generate resource.c
def GenerateResourceCFile(res_root):
    output_root = output_folder
    filename = "resource.c"

    if ns:
        GenerateResourceCFileForNonStandalone(res_root, output_root, filename)
    else:
        GenerateResourceCFileForStandalone(res_root, output_root, filename)

    # shutil.copy(os.path.join(output_root, filename), gen_src_file_folder)


def BuildResourcePackage2(res_root, resolution, ns2, ns_sec_name2, default_language2):
    global ns
    global ns_sec_name
    global default_language

    ns = ns2
    ns_sec_name = ns_sec_name2
    default_language = default_language2
    CleanGeneratedFile()
    GenerateImgRes(res_root + '/' + resolution)
    GenerateStrRes(res_root)
    GenerateFontsRes(res_root)
    GenerateResourceHFile(res_root)
    GenerateResourceCFile(res_root)


def BuildStringPackage(str_src_dir, str_output_dir, ns2, default_language2):
    global ns
    global ns_sec_name
    global default_language

    ns = ns2
    default_language = default_language2

    # str_output_dir = os.path.join(res_root, output_folder)

    GenerateStrRes(str_src_dir, str_output_dir)


#    GenerateResourceHFile(res_root)
#    GenerateResourceCFile(res_root)


def BuildStringPackage2(str_output_dir, lang_pack_name):
    GenerateStrRes2(str_output_dir, lang_pack_name)


def _GetVarName(s):
    var_name = None
    s = s.strip()
    if s[0] == '$':
        var_name = s[1:]

    return var_name


# variables are dict which specify the value of all variables used by the template file
def GenPartitionJsonFile(src, output_dir, output_name, variables):
    import building
    from collections import OrderedDict

    f = open(src)
    try:
        mems = json.load(f, object_pairs_hook=OrderedDict)
    finally:
        f.close()

    for mem in mems:
        for region in mem['regions']:
            var_name = _GetVarName(region['offset'])
            if var_name:
                assert var_name in variables, "offset variable {} not found".format(var_name)
                region['offset'] = '0x{:08X}'.format(variables[var_name])
            var_name = _GetVarName(region['max_size'])
            if var_name:
                assert var_name in variables, "max_size variable {} not found".format(var_name)
                region['max_size'] = '0x{:08X}'.format(variables[var_name])

    f = open(os.path.join(output_dir, output_name + '.json'), "w")
    json.dump(mems, f, indent=4)
    f.close()

def Convert2CBUSAddr(addr, offset, core=None):
    return ptab.convert_to_cbus_addr(addr, offset, core)


def Convert2SBUSAddr(addr, offset, core=None):
    return ptab.convert_to_sbus_addr(addr, offset, core)


def _legacy_ftab_xip_addr(start_addr, offset, region, same_region_exec):
    if same_region_exec:
        return start_addr

    xip_addr, _ = Convert2SBUSAddr(start_addr, offset, region.get('core'))
    return xip_addr


def PtabAddAddDefaultRegion(mems):
    ptab.add_default_regions(mems)


def _PtabGetHalMemTypeMacro(mem_type):
    mem_type = str(mem_type or '').strip().lower()
    if mem_type == 'nor':
        return 'HAL_MEM_TYPE_NOR_FLASH'
    elif mem_type == 'nand':
        return 'HAL_MEM_TYPE_NAND_FLASH'
    elif mem_type in ('sd', 'sdmmc', 'emmc'):
        return 'HAL_MEM_TYPE_SDMMC_STORAGE'
    elif mem_type == 'psram':
        return 'HAL_MEM_TYPE_PSRAM'
    else:
        return None


def _PtabDefineMemType(name, mem_type, clear=False):
    mem_type_macro = _PtabGetHalMemTypeMacro(mem_type)
    s = ''
    if clear or mem_type_macro is not None:
        s += MakeLine('#undef  {}'.format(name))
    if mem_type_macro is not None:
        s += MakeLine('#define {:<50} {}'.format(name, mem_type_macro))
    return s


def PtabGetMemType(mem, base):
    mem = str(mem or '').strip().lower()
    if "flash" in mem and (base >= 0x10000000) and (base <= 0x1FFFFFFF):
        return "nor"
    elif "flash" in mem and (base >= 0x60000000) and (base <= 0x6FFFFFFF):
        return "nand"
    elif "emmc" in mem or "sdmmc" in mem or mem.startswith("sd"):
        return "sd"
    elif "psram" in mem:
        return "psram"
    else:
        return ""


def PtabGetMemType(mem, base):
    if "flash" in mem and (base >= 0x10000000) and (base <= 0x1FFFFFFF):
        return "HAL_MEM_TYPE_NOR_FLASH"
    elif "flash" in mem and (base >= 0x60000000) and (base <= 0x6FFFFFFF):
        return "HAL_MEM_TYPE_NAND_FLASH"
    elif "emmc" in mem:
        return "HAL_MEM_TYPE_SDMMC_STORAGE"
    elif "psram" in mem:
        return "HAL_MEM_TYPE_PSRAM"
    else:
        return "HAL_MEM_TYPE_UNKNOWN"

def GenPartitionTableHeaderContentV2(env, mems):
    s =  ''
    PtabAddAddDefaultRegion(mems)
    for mem in mems:
        mem_base = int(mem['base'], 0)
        mem_type = PtabGetMemType(mem['mem'], mem_base)
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* {} */'.format(mem['mem']))
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            max_size = int(region['max_size'], 0)
            start_addr = mem_base + offset

            if 'tags' in region:
                for tag in region['tags']:
                    if tag.strip() == '':
                        continue
                    start_addr_name = '{}_START_ADDR'.format(tag)
                    size_name = '{}_SIZE'.format(tag)
                    offset_name = '{}_OFFSET'.format(tag)
                    mem_type_name = '{}_MEM_TYPE'.format(tag)
                    s += MakeLine('#undef  {}'.format(start_addr_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, start_addr))
                    s += MakeLine('#undef  {}'.format(size_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                    s += MakeLine('#undef  {}'.format(offset_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, offset))
                    s += _PtabDefineMemType(mem_type_name, mem_type)

            if 'custom' in region:
                for custom in region['custom']:
                    s += MakeLine('#undef  {}'.format(custom))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(custom, region['custom'][custom]))

            # Add cbus related macro for code region
            if 'name' in region:
                temp = region['name'].split(':')
                if len(temp) > 1:
                    base, ext =  temp
                else:
                    base = temp[0]
                    ext = None

                if ('type' in region) and ('app_exec' in region['type']):
                    if (not ext) or ('1' == ext):
                        name_prefix = 'APP_{}_CODE'.format(base.upper())  
                    else:
                        name_prefix = 'APP_{}_CODE{}'.format(base.upper(), ext)  

                    start_addr_name = f'{name_prefix}_START_ADDR'
                    size_name = f'{name_prefix}_SIZE'
                    offset_name = f'{name_prefix}_OFFSET'

                    core =  region.get('core')
                    if core and str(core).strip().upper() == 'ACPU':
                        # ACPU uses the shared SBUS view. Its local CBUS view
                        # (e.g. 0-based hpsys_ram on SF32LB58) is unusable by
                        # HCPU when ACPU passes the exec address over.
                        exec_addr = start_addr
                        exec_offset = offset
                    else:
                        exec_addr, exec_offset = Convert2CBUSAddr(start_addr, offset, core)
                    s += MakeLine('#undef  {}'.format(start_addr_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, exec_addr))
                    s += MakeLine('#undef  {}'.format(size_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                    s += MakeLine('#undef  {}'.format(offset_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, exec_offset))

                    if (base == env['name']):
                        if (not ext) or ('1' == ext):
                            s += MakeLine('#define {:<50} ({})'.format("CODE_START_ADDR", start_addr_name))
                            s += MakeLine('#define {:<50} ({})'.format("CODE_SIZE", size_name))
                        else:
                            s += MakeLine('#define {:<50} ({})'.format(f"CODE{ext}_START_ADDR", start_addr_name))
                            s += MakeLine('#define {:<50} ({})'.format(f"CODE{ext}_SIZE", size_name))

                    # Add `ACPU_CODE_REGION1` for compatibility as link file uses this macro
                    if core and (core.lower() == 'acpu') :
                        if ext:
                            num = ext
                        else:
                            num =  1
                        name_prefix = f'ACPU_CODE_REGION{num}'
                        start_addr_name = f'{name_prefix}_START_ADDR'
                        size_name = f'{name_prefix}_SIZE'
                        offset_name = f'{name_prefix}_OFFSET'                        
                        s += MakeLine('#undef  {}'.format(start_addr_name))
                        s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, exec_addr))
                        s += MakeLine('#undef  {}'.format(size_name))
                        s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                        s += MakeLine('#undef  {}'.format(offset_name))
                        s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, exec_offset))
    
    return s               


def GenPartitionTableHeaderContentV1(env, mems):
    s = ''
    for mem in mems:
        mem_base = int(mem['base'], 0)
        mem_type = PtabGetMemType(mem['mem'], mem_base)
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* {} */'.format(mem['mem']))
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            max_size = int(region['max_size'], 0)
            start_addr = mem_base + offset
            for tag in region['tags']:
                start_addr_name = '{}_START_ADDR'.format(tag)
                size_name = '{}_SIZE'.format(tag)
                offset_name = '{}_OFFSET'.format(tag)
                mem_type_name = '{}_MEM_TYPE'.format(tag)
                s += MakeLine('#undef  {}'.format(start_addr_name))
                s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, start_addr))
                s += MakeLine('#undef  {}'.format(size_name))
                s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                s += MakeLine('#undef  {}'.format(offset_name))
                s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, offset))
                s += _PtabDefineMemType(mem_type_name, mem_type)
            if 'custom' in region:
                for custom in region['custom']:
                    s += MakeLine('#undef  {}'.format(custom))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(custom, region['custom'][custom]))
            if 'exec' in region:
                if (region['exec'] == env['name']):
                    if  (len(region['tags']) > 0):
                        s += MakeLine('#define {:<50} ({})'.format("CODE_START_ADDR", start_addr_name))
                        s += MakeLine('#define {:<50} ({})'.format("CODE_SIZE", size_name))
                    else:
                        s += MakeLine('#define {:<50} (0x{:08X})'.format("CODE_START_ADDR", start_addr))
                        s += MakeLine('#define {:<50} (0x{:08X})'.format("CODE_SIZE", max_size))
    return s


def GenPartitionTableHeaderContentV3(env, ptab_obj):
    """Generate ptab.h content for ptab v3 (exec + aliases).

    This generator keeps v1/v2 compatibility by generating:
    - Name macros: <NAME>_START_ADDR/_SIZE/_OFFSET
    - Name macros: <NAME>_MEM_TYPE where memory type is known
    - Alias macros: <ALIAS>_START_ADDR/_SIZE/_OFFSET
    - Alias macros: <ALIAS>_MEM_TYPE where memory type is known
    - FLASH_BOOT_LOADER_* storage alias emitted for the bootloader partition
    - FS_REGION_* for filesystem-like partitions
    - FAL_PART_TABLE for auto-exported FAL partitions
    - FLASH_BOOT_LOADER_EXEC_* from bootloader.exec (execution address)
    - HCPU_FLASH_CODE_EXEC_* from HCPU factory app.exec (execution address)
    - ACPU_CODE_REGION<N>_EXEC[_SBUS]_* from ACPU factory app.exec
    - CODE_START_ADDR/CODE_SIZE for the current env image (best-effort)

    Storage vs execution address: the per-partition pass always emits
    <NAME>_START_ADDR/_SIZE/_OFFSET from the partition's storage region,
    and additionally FLASH_BOOT_LOADER_* for the bootloader partition (the
    legacy name for its flash storage region). The exec-address blocks always
    emit their macros with a trailing _EXEC suffix (<NAME>_EXEC_START_ADDR/
    _SIZE/_OFFSET) so they never collide with the storage macros. For
    backward compatibility, the legacy ACPU_CODE_REGION<N>[_SBUS]_* names
    are also emitted as references to the _EXEC variants (ACPU link scripts
    use those names directly).
    """

    s = ''
    chip_config = ptab_obj.get_chip_config()
    partitions = ptab_obj.partitions

    def _mpi_name_from_region(region):
        if not region:
            return None
        if region.startswith('mpi'):
            return region
        if region.startswith('psram'):
            if region == 'psram':
                return 'mpi1'
            suffix = region.replace('psram', '')
            if suffix.isdigit():
                return 'mpi{}'.format(suffix)
        return None

    def _get_region_mem_type(region):
        if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
            return 'ram'
        if region == 'psram' or region.startswith('psram'):
            return 'psram'
        if region.startswith('sdmmc'):
            return 'sd'
        mpi_name = _mpi_name_from_region(region)
        if mpi_name:
            info = chip_config.get('memory_info', {}).get(mpi_name, {})
            mtype = (info.get('type') or '').lower()
            return mtype or 'nor'
        return ''

    def _select_start_addr(region, sbus_addr, cbus_addr, subtype):
        # For RAM-like partitions, always use base address
        if subtype == 'ram':
            return sbus_addr
        mem_type = _get_region_mem_type(region)
        if mem_type == 'nor':
            return cbus_addr
        return sbus_addr

    def _select_exec_addr(region, sbus_addr, cbus_addr, core=None):
        # Execution address selection:
        # - RAM/NAND: base
        # - NOR/PSRAM: XIP
        #
        # NOTE: ACPU deliberately uses the same SBUS-based rule instead of its
        # local CBUS view (e.g. SF32LB58 hpsys_ram is 0-based for ACPU while
        # the HCPU/DFU/SBUS view is 0x20200000). Picking CBUS for ACPU made
        # the exec address (ACPU_CODE_REGION<N>_EXEC_START_ADDR) unusable for
        # HCPU when ACPU passes that address over. The common rule below keeps
        # the macro valid in every core's view.
        mem_type = _get_region_mem_type(region)
        return sbus_addr if mem_type in ('ram', 'nand') else cbus_addr

    def _is_auto_fal_partition(partition):
        if not isinstance(partition, dict):
            return False
        ptype = str(partition.get('type') or '').strip()
        subtype = str(partition.get('subtype') or '').strip()
        if ptype == 'data':
            return subtype in ('flashdb_kv', 'filesystem')
        if ptype == 'app':
            return subtype != 'ex'
        return False

    def _fal_device_macro_for_region(region):
        region = str(region or '').strip().lower()
        match = re.match(r'^mpi([1-5])$', region)
        if match:
            return 'NOR_FLASH{}_DEV_NAME'.format(match.group(1))
        if region == 'sdmmc1':
            return 'SDMMC1_DEV_NAME'
        if region == 'sdmmc2':
            return 'SDMMC2_DEV_NAME'
        return None

    def _define_u32(name, value):
        out = ''
        out += MakeLine('#undef  {}'.format(name))
        out += MakeLine('#define {:<50} (0x{:08X})'.format(name, value & 0xFFFFFFFF))
        return out

    def _define_ref(name, ref):
        out = ''
        out += MakeLine('#undef  {}'.format(name))
        out += MakeLine('#define {:<50} ({})'.format(name, ref))
        return out

    def _define_mem_type(base_name, mem_type, clear=False):
        return _PtabDefineMemType('{}_MEM_TYPE'.format(base_name), mem_type, clear=clear)

    def _get_flash_boot_loader_size_default():
        # FLASH_BOOT_LOADER_* macros are used by bootloader link scripts as the
        # execution region in RAM. For historical compatibility, its SIZE is
        # derived from chip mem_map.h default (not necessarily the flash
        # storage partition size).
        try:
            sifli_sdk = os.getenv('SIFLI_SDK')
            if not sifli_sdk:
                # Fallback for offline tools (e.g. regression script)
                sifli_sdk = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            if not sifli_sdk:
                return None
            chip_dir = (ptab_obj.chip_series or '').lower()
            if chip_dir and not chip_dir.endswith('x'):
                chip_dir = chip_dir + 'x'
            if not chip_dir:
                return None
            import gen_link_lds
            mem_ints = gen_link_lds._load_mem_map_ints(sifli_sdk, chip_dir)
            v = mem_ints.get('FLASH_BOOT_LOADER_SIZE')
            if v is None:
                return None
            v = int(v)
            return v if v > 0 else None
        except Exception:
            return None

    def _infer_acpu_code_region_num(partition):
        candidates = []
        name = partition.get('name')
        if name:
            candidates.append(name)
        candidates.extend(partition.get('aliases', []) or [])

        for candidate in candidates:
            token = str(candidate).strip().upper()
            match = re.match(r'^ACPU_CODE_LOAD_REGION(\d+)$', token)
            if not match:
                match = re.match(r'^ACPU_CODE_REGION(\d+)(?:_SBUS)?$', token)
            if match:
                return match.group(1)
        return '1'

    # Quick lookup maps
    by_name = {}
    factory_by_core = {}
    dfu_by_core = {}
    bootloader_partition = None

    for p in partitions:
        pname = p.get('name', '')
        if pname:
            by_name[pname.lower()] = p
        ptype = p.get('type', '')
        subtype = p.get('subtype', '')
        core = (p.get('core') or 'HCPU').upper()
        if ptype == 'bootloader':
            bootloader_partition = p
        if ptype == 'app' and subtype == 'factory' and core not in factory_by_core:
            factory_by_core[core] = p
        if ptype == 'app' and subtype == 'dfu' and core not in dfu_by_core:
            dfu_by_core[core] = p

    # Collect auto-exported FAL partitions for FAL_PART_TABLE generation
    fal_partitions = []

    # Group partitions by region for readability
    region_groups = {}
    for partition in partitions:
        region = partition.get('region', '')
        region_groups.setdefault(region, []).append(partition)

    for region, parts in region_groups.items():
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* {} */'.format(region))

        for partition in parts:
            name = partition.get('name', '')
            if not name:
                continue

            offset = ptab.parse_size(partition.get('offset', 0))
            size = ptab.parse_size(partition.get('size', 0))
            ptype = partition.get('type', '')
            subtype = partition.get('subtype', '')
            core = partition.get('core')

            sbus_addr, cbus_addr = ptab.resolve_region_address(region, offset, chip_config, core=core)
            start_addr = _select_start_addr(region, sbus_addr, cbus_addr, subtype)
            region_mem_type = _get_region_mem_type(region)

            name_upper = name.upper()
            s += _define_u32('{}_START_ADDR'.format(name_upper), start_addr)
            s += _define_u32('{}_SIZE'.format(name_upper), size)
            s += _define_u32('{}_OFFSET'.format(name_upper), offset)
            s += _define_mem_type(name_upper, region_mem_type)

            # Alias macros
            alias_set = []
            for a in (partition.get('aliases', []) or []):
                a = str(a).strip()
                if not a:
                    continue
                a = a.upper()
                if a not in alias_set:
                    alias_set.append(a)
            for a in alias_set:
                s += _define_u32('{}_START_ADDR'.format(a), start_addr)
                s += _define_u32('{}_SIZE'.format(a), size)
                s += _define_u32('{}_OFFSET'.format(a), offset)
                s += _define_mem_type(a, region_mem_type)

            # FLASH_BOOT_LOADER_* is the legacy name for the bootloader's
            # flash STORAGE region, so it is emitted here together with the
            # other storage macros (not as an exec macro).
            if ptype == 'bootloader':
                s += _define_u32('FLASH_BOOT_LOADER_START_ADDR', start_addr)
                s += _define_u32('FLASH_BOOT_LOADER_SIZE', size)
                s += _define_u32('FLASH_BOOT_LOADER_OFFSET', offset)
                s += _define_mem_type('FLASH_BOOT_LOADER', region_mem_type)

            # FS_REGION_* macros (compat)
            fs_subtypes = ('littlefs', 'fat', 'fatfs', 'flashdb', 'filesystem')
            if ptype == 'data' and subtype in fs_subtypes:
                s += _define_u32('FS_REGION_START_ADDR', start_addr)
                s += _define_u32('FS_REGION_SIZE', size)
                s += _define_u32('FS_REGION_OFFSET', offset)
                s += _define_mem_type('FS_REGION', region_mem_type)

            # FlashDB KV macros
            if ptype == 'data' and subtype == 'flashdb_kv':
                # NOTE: `name` is the FlashDB KV DB name and also the FAL
                # partition name string (e.g. `dfu`, `ble`).
                kv_base = 'KVDB_{}_REGION'.format(name_upper)
                s += _define_u32('{}_START_ADDR'.format(kv_base), start_addr)
                s += _define_u32('{}_OFFSET'.format(kv_base), offset)
                s += _define_u32('{}_SIZE'.format(kv_base), size)
                s += _define_mem_type(kv_base, region_mem_type)

            # Auto-export selected partitions into FAL_PART_TABLE.
            if _is_auto_fal_partition(partition):
                fal_partitions.append({
                    'name': name,
                    'region': region,
                    'offset': offset,
                    'size': size,
                })

            # attrs custom macros
            attrs = partition.get('attrs', {})
            if isinstance(attrs, dict):
                for key, value in attrs.items():
                    if isinstance(value, int):
                        s += _define_u32(str(key), value)

    # FLASH_BOOT_LOADER_EXEC_* macros (execution address, from exec). All
    # exec macros carry the _EXEC suffix so they never collide with the
    # storage macros emitted by the per-partition pass.
    if bootloader_partition and isinstance(bootloader_partition.get('exec'), dict):
        exec_def = bootloader_partition['exec']
        exec_region = str(exec_def.get('region', '')).strip()
        exec_offset = ptab.parse_size(exec_def.get('offset', 0))
        bl_size = _get_flash_boot_loader_size_default()
        if bl_size is None:
            bl_size = ptab.parse_size(bootloader_partition.get('size', 0))
        else:
            # Keep it >= flash storage size (best-effort)
            storage_size = ptab.parse_size(bootloader_partition.get('size', 0))
            if storage_size > bl_size:
                bl_size = storage_size
        bl_core = bootloader_partition.get('core')
        exec_sbus, exec_cbus = ptab.resolve_region_address(exec_region, exec_offset, chip_config, core=bl_core)
        bl_exec_addr = _select_exec_addr(exec_region, exec_sbus, exec_cbus, bl_core)

        bl_base = 'FLASH_BOOT_LOADER_EXEC'
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* bootloader exec addr */')
        s += _define_u32('{}_START_ADDR'.format(bl_base), bl_exec_addr)
        s += _define_u32('{}_SIZE'.format(bl_base), bl_size)
        s += _define_u32('{}_OFFSET'.format(bl_base), exec_offset)
        s += _define_mem_type(bl_base, _get_region_mem_type(exec_region), clear=True)

    # HCPU_FLASH_CODE_EXEC_* macros (execution address, from exec). All exec
    # macros carry the _EXEC suffix so they never collide with the storage
    # macros emitted by the per-partition pass (the partition is usually named
    # `hcpu_flash_code`).
    hcpu_factory_partition = factory_by_core.get('HCPU')
    if hcpu_factory_partition and isinstance(hcpu_factory_partition.get('exec'), dict):
        exec_def = hcpu_factory_partition['exec']
        exec_region = str(exec_def.get('region', '')).strip()
        exec_offset = ptab.parse_size(exec_def.get('offset', 0))
        exec_sbus, exec_cbus = ptab.resolve_region_address(exec_region, exec_offset, chip_config, core='HCPU')
        hcpu_exec_addr = _select_exec_addr(exec_region, exec_sbus, exec_cbus)
        hcpu_size = ptab.parse_size(hcpu_factory_partition.get('size', 0))

        hcpu_base = 'HCPU_FLASH_CODE_EXEC'
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* HCPU factory app exec addr */')
        s += _define_u32('{}_START_ADDR'.format(hcpu_base), hcpu_exec_addr)
        s += _define_u32('{}_SIZE'.format(hcpu_base), hcpu_size)
        s += _define_u32('{}_OFFSET'.format(hcpu_base), exec_offset)
        s += _define_mem_type(hcpu_base, _get_region_mem_type(exec_region), clear=True)

    # ACPU_CODE_REGION<N>_EXEC[_SBUS]_* macros (execution address, from exec).
    # v3 keeps the ACPU execution RAM in the app's exec descriptor instead of
    # a separate legacy RAM partition.
    emitted_acpu_code_regions = set()
    for partition in partitions:
        if partition.get('type') != 'app' or partition.get('subtype') != 'factory':
            continue
        if str(partition.get('core') or '').strip().upper() != 'ACPU':
            continue
        exec_def = partition.get('exec')
        if not isinstance(exec_def, dict):
            continue

        region_num = _infer_acpu_code_region_num(partition)
        if region_num in emitted_acpu_code_regions:
            continue
        emitted_acpu_code_regions.add(region_num)

        exec_region = str(exec_def.get('region', '')).strip()
        exec_offset = ptab.parse_size(exec_def.get('offset', 0))
        exec_sbus, exec_cbus = ptab.resolve_region_address(exec_region, exec_offset, chip_config, core='ACPU')
        acpu_exec_addr = _select_exec_addr(exec_region, exec_sbus, exec_cbus, 'ACPU')
        acpu_size = ptab.parse_size(partition.get('size', 0))

        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* ACPU factory app exec addr */')
        # ACPU_CODE_REGION<N>_EXEC_SBUS_* uses the SBUS view of the exec
        # region; ACPU_CODE_REGION<N>_EXEC_* uses the selected execution
        # address (cbus for NOR/PSRAM, sbus for RAM/NAND). The two can differ.
        acpu_bases = (
            ('ACPU_CODE_REGION{}_EXEC_SBUS'.format(region_num), int(exec_sbus)),
            ('ACPU_CODE_REGION{}_EXEC'.format(region_num), int(acpu_exec_addr)),
        )
        for base_name, addr in acpu_bases:
            s += _define_u32('{}_START_ADDR'.format(base_name), addr)
            s += _define_u32('{}_SIZE'.format(base_name), acpu_size)
            s += _define_u32('{}_OFFSET'.format(base_name), exec_offset)

        # Legacy aliases: ACPU link scripts reference
        # ACPU_CODE_REGION<N>[_SBUS]_START_ADDR/_SIZE/_OFFSET directly.
        for legacy_base in (
            'ACPU_CODE_REGION{}_SBUS'.format(region_num),
            'ACPU_CODE_REGION{}'.format(region_num),
        ):
            if legacy_base.endswith('_SBUS'):
                exec_base = legacy_base[:-len('_SBUS')] + '_EXEC_SBUS'
            else:
                exec_base = legacy_base + '_EXEC'
            s += _define_ref('{}_START_ADDR'.format(legacy_base), '{}_START_ADDR'.format(exec_base))
            s += _define_ref('{}_SIZE'.format(legacy_base), '{}_SIZE'.format(exec_base))
            s += _define_ref('{}_OFFSET'.format(legacy_base), '{}_OFFSET'.format(exec_base))

    # CODE_START_ADDR/CODE_SIZE for current image (best-effort)
    env_name = (env.get('name') or '').strip().lower()
    try:
        import rtconfig
        build_core = getattr(rtconfig, 'CORE', None)
    except Exception:
        build_core = None
    build_core = (build_core or 'HCPU').upper()

    code_partition = None
    if env_name == 'bootloader':
        code_partition = bootloader_partition
    elif env_name == 'dfu':
        code_partition = dfu_by_core.get(build_core)
    elif env_name == 'main':
        code_partition = factory_by_core.get(build_core)
    elif env_name:
        code_partition = by_name.get(env_name)

    if code_partition:
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* code start */')

        if code_partition.get('type') == 'bootloader' and isinstance(code_partition.get('exec'), dict):
            s += _define_ref('CODE_START_ADDR', 'FLASH_BOOT_LOADER_EXEC_START_ADDR')
            s += _define_ref('CODE_SIZE', 'FLASH_BOOT_LOADER_EXEC_SIZE')
        else:
            size = ptab.parse_size(code_partition.get('size', 0))
            exec_def = code_partition.get('exec')
            core = code_partition.get('core')
            if isinstance(exec_def, dict):
                exec_region = str(exec_def.get('region', '')).strip()
                exec_offset = ptab.parse_size(exec_def.get('offset', 0))
                exec_sbus, exec_cbus = ptab.resolve_region_address(exec_region, exec_offset, chip_config, core=core)
                exec_addr = _select_exec_addr(exec_region, exec_sbus, exec_cbus, core)
            else:
                region = code_partition.get('region', '')
                offset = ptab.parse_size(code_partition.get('offset', 0))
                sbus_addr, cbus_addr = ptab.resolve_region_address(region, offset, chip_config, core=core)
                exec_addr = _select_exec_addr(region, sbus_addr, cbus_addr, core)

            s += _define_u32('CODE_START_ADDR', exec_addr)
            s += _define_u32('CODE_SIZE', size)

    # Generate FAL_PART_TABLE if auto-exported partitions exist
    if fal_partitions:
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* FAL Partition Table */')
        s += MakeLine('#ifndef FAL_PART_TABLE')
        s += MakeLine('#define FAL_PART_TABLE \\')
        s += MakeLine('{ \\')
        for idx, fal_part in enumerate(fal_partitions):
            dev_name = _fal_device_macro_for_region(fal_part['region'])
            if not dev_name:
                raise ValueError(
                    "partition '{}' must use region mpi1..mpi5 or sdmmc1/sdmmc2 for FAL_PART_TABLE generation (got region '{}')".format(
                        fal_part.get('name') or '?', fal_part.get('region')
                    )
                )

            part_name = fal_part['name']
            part_offset = fal_part['offset']
            part_size = fal_part['size']

            # Last entry doesn't have trailing comma
            if idx == len(fal_partitions) - 1:
                s += MakeLine('    {{FAL_PART_MAGIC_WORD, "{}", {}, 0x{:08X}, 0x{:08X}, 0}} \\'.format(
                    part_name, dev_name, part_offset, part_size))
            else:
                s += MakeLine('    {{FAL_PART_MAGIC_WORD, "{}", {}, 0x{:08X}, 0x{:08X}, 0}}, \\'.format(
                    part_name, dev_name, part_offset, part_size))
        s += MakeLine('}')
        s += MakeLine('#endif /* FAL_PART_TABLE */')

    return s


def GenPartitionTableHeaderFile(src, output_dir, output_name, env=None):
    import building
    import rtconfig

    logging.debug('output ptab.h:{}'.format(output_dir))
    if env is None:
        env = building.GetCurrentEnv()

    ptab_obj = ptab.load_ptab(src, fatal=True)
    InitIndentation()
    s = MakeLine('#ifndef __{}__H__'.format(output_name.upper()))
    s += MakeLine('#define __{}__H__'.format(output_name.upper()))
    s += MakeLine('')
    s += MakeLine('')

    if ptab_obj.is_v3():
        s += GenPartitionTableHeaderContentV3(env, ptab_obj)
    elif ptab_obj.is_v2():
        mems = ptab_obj.content_mems()
        s += GenPartitionTableHeaderContentV2(env, mems)
    else:
        mems = ptab_obj.content_mems()
        s += GenPartitionTableHeaderContentV1(env, mems)

    s += MakeLine('')
    s += MakeLine('')
    s += MakeLine('')
    s += MakeLine('#endif')

    f = open(os.path.join(output_dir, output_name + '.h'), "w")
    f.write(s)
    f.close()

def CalcBinarySize(binary_file):
    if os.path.isdir(binary_file):
        dir_list = os.listdir(binary_file)
        bin_size = []
        for d in dir_list:
            f = os.path.join(binary_file, d)
            if os.path.isfile(f):
                bin_size.append({"name": d, "size": os.path.getsize(f)})
    else:
        bin_size = [{"name": os.path.basename(binary_file), "size": os.path.getsize(binary_file)}]

    return bin_size

def GetMultiBinaryName(ext):
    return f'ER_IROM{ext}.bin'

def ConstructFtabDictV2(ftab, mems, img_size):
    import building
    flash_table_start = None
    PtabAddAddDefaultRegion(mems)
    for mem in mems:
        mem_base = int(mem['base'], 0)
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            max_size = int(region['max_size'], 0)
            start_addr = mem_base + offset
            if 'tags' in region and 'FLASH_TABLE' in region['tags']:
                flash_table_start = start_addr
            if 'name' in region:
                temp = region['name'].split(':')
                if len(temp) > 1:
                    base, ext =  temp
                else:
                    base = temp[0]
                    ext = None

                if 'ftab' == base:
                    # no need to add ftab item
                    continue
                
                if 'type' not in region or (("app_img" not in region['type']) and ('app_exec' not in region['type']) and ("app_img2" not in region['type'])):
                    continue

                if ext and ext != '1':
                    # no need to construct ftab for non-first part
                    continue

                region_type_list = region['type']
                item_name = base
                if item_name not in ftab:
                    ftab[item_name] = {}

                ftab_item = ftab[item_name]
                assert ('max_size' not in ftab_item) or (
                        max_size == ftab_item['max_size']), "{} max_size must be same, old: 0x{:08X}, new: 0x{:08X}".format(item_name, ftab_item['max_size'], max_size)
                ftab_item['max_size'] = max_size
                if 'app_exec' in region_type_list and ((not ext) or ('1' == ext)):
                    # only first binary need to be described in ftab
                    assert 'xip' not in ftab_item, 'xip address already configured in {}'.format(item_name)
                    same_region_exec = ('app_img' in region_type_list) or ('app_img2' in region_type_list)
                    ftab_item['xip'] = _legacy_ftab_xip_addr(start_addr, offset, region, same_region_exec)
                if 'app_img' in region_type_list and ((not ext) or ('1' == ext)):
                    assert 'base' not in ftab_item, 'base address already configured in {}'.format(item_name)
                    ftab_item['base'] = start_addr
                if 'app_img2' in region_type_list:
                    assert 'base2' not in ftab_item, 'base2 address already configured in {}'.format(item_name)
                    ftab_item['base2'] = start_addr
                
                # add img info if img is specified
                proj_name = base
                if proj_name in img_size:
                    if not ext:
                        assert len(img_size[
                                    proj_name]) == 1, "img '{}' is expected to be a file but it's a directory " \
                                                        "now".format(proj_name)
                        size = img_size[proj_name][0]['size']
                    else:
                        bin_name = GetMultiBinaryName(ext)
                        size = 0
                        for img in img_size[proj_name]:
                            if img['name'] == bin_name:
                                size = img['size']
                                break
                    assert size > 0, 'Size of img {} cannot be determined'.format(region['img'])
                    ftab_item['img'] = {'name': region['name'], 'size': size}

    # check if all images defined in img_size are in ftab
    for k, v in img_size.items():
        if "main" != k and "bootloader" != k and "dfu" != k:
            continue

        if k not in ftab:
            raise Exception("Image {} not defined in ptab for ftab construction".format(k))
        

    #  Add default bootloader configuration if not specified in ptab
    if not building.GetDepend("SOC_SF32BL55X") and "bootloader" not in ftab:
        ftab['bootloader'] = {}
        ftab_item = ftab["bootloader"]
        ftab_item['max_size'] = max_size
        if 'app_exec' in region_type_list and ((not ext) or ('1' == ext)):
            # only first binary need to be described in ftab
            assert 'xip' not in ftab_item, 'xip address already configured in {}'.format(item_name)
            same_region_exec = ('app_img' in region_type_list) or ('app_img2' in region_type_list)
            ftab_item['xip'] = _legacy_ftab_xip_addr(start_addr, offset, region, same_region_exec)
        if 'app_img' in region_type_list and ((not ext) or ('1' == ext)):
            assert 'base' not in ftab_item, 'base address already configured in {}'.format(item_name)
            ftab_item['base'] = start_addr
        if 'app_img2' in region_type_list:
            assert 'base2' not in ftab_item, 'base2 address already configured in {}'.format(item_name)
            ftab_item['base2'] = start_addr
        
        # add img info if img is specified
        proj_name = base
        if proj_name in img_size:
            if not ext:
                assert len(img_size[
                            proj_name]) == 1, "img '{}' is expected to be a file but it's a directory " \
                                                "now".format(proj_name)
                size = img_size[proj_name][0]['size']
            else:
                bin_name = GetMultiBinaryName(ext)
                size = 0
                for img in img_size[proj_name]:
                    if img['name'] == bin_name:
                        size = img['size']
                        break
            assert size > 0, 'Size of img {} cannot be determined'.format(region['img'])
            ftab_item['img'] = {'name': region['name'], 'size': size}


    return flash_table_start

def ConstructFtabDictV1(ftab, mems, img_size):
    flash_table_start = None
    for mem in mems:
        mem_base = int(mem['base'], 0)
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            max_size = int(region['max_size'], 0)
            start_addr = mem_base + offset
            if 'tags' in region and 'FLASH_TABLE' in region['tags']:
                flash_table_start = start_addr
            if 'ftab' in region:
                item_name = region['ftab']['name']
                if item_name not in ftab:
                    ftab[item_name] = {}

                ftab_item = ftab[item_name]
                assert ('max_size' not in ftab) or (
                        max_size == ftab_item['max_size']), "{} max_size must be same, old: 0x{:08X}, new: 0x{:08X}".format(item_name, ftab['max_size'], max_size)
                ftab_item['max_size'] = max_size
                if 'xip' in region['ftab']['address']:
                    assert 'xip' not in ftab_item, 'xip address already configured in {}'.format(item_name)
                    same_region_exec = 'base' in region['ftab']['address']
                    ftab_item['xip'] = _legacy_ftab_xip_addr(start_addr, offset, region, same_region_exec)
                if 'base' in region['ftab']['address']:
                    assert 'base' not in ftab_item, 'base address already configured in {}'.format(item_name)
                    ftab_item['base'] = start_addr
                if 'base2' in region['ftab']['address']:
                    assert 'base2' not in ftab_item, 'base2 address already configured in {}'.format(item_name)
                    ftab_item['base2'] = start_addr
                # add img info if img is specified
                if 'img' in region:
                    img_name = region['img'].split(':')
                    proj_name = img_name[0]
                    if proj_name in img_size:
                        if len(img_name) == 1:
                            assert len(img_size[
                                           proj_name]) == 1, "img '{}' is expected to be a file but it's a directory " \
                                                             "now".format(proj_name)
                            size = img_size[proj_name][0]['size']
                        else:
                            bin_name = img_name[1]
                            size = 0
                            for img in img_size[proj_name]:
                                if img['name'] == bin_name:
                                    size = img['size']
                                    break
                        assert size > 0, 'Size of img {} cannot be determined'.format(region['img'])
                        ftab_item['img'] = {'name': region['img'], 'size': size}
                    else:
                        print('WARNING: img "{}" not found'.format(proj_name))



    return flash_table_start


def GenFtabCFile(src, output_name, imgs_info):
    import building
    ptab_obj = ptab.load_ptab(src, fatal=True)

    # Get binary image size
    img_size = {}
    for img in imgs_info:
        img_name = img['name']
        img_bin = str(img['binary'][0])
        img_size[img_name] = CalcBinarySize(img_bin)

    # construct ftab dictionary
    ftab = {}
    flash_table_start = None

    mems = ptab_obj.content_mems()
    if ptab_obj.is_v2():
        flash_table_start = ConstructFtabDictV2(ftab, mems, img_size)
    else:
        flash_table_start = ConstructFtabDictV1(ftab, mems, img_size)

    bootloader_needed = not building.GetDepend('SOC_SF32LB55X')
    if bootloader_needed:
        assert 'bootloader' in ftab, "bootloader not configured"
    elif 'bootloader' in ftab:
        bootloader_needed = True
    # assert 'main' in ftab, "main not configured"
    if 'dfu' in img_size:
        dfu_present = True
    else:
        dfu_present = False

    InitIndentation()
    s = ''
    s += MakeLine('#include <rtconfig.h>')
    s += MakeLine('#include <board.h>')
    s += MakeLine('#include <string.h>')
    s += MakeLine('#include <dfu.h>')
    s += MakeLine('')
    s += MakeLine('')

    if flash_table_start is not None:
        s += MakeLine('#undef FLASH_TABLE_START_ADDR')
        s += MakeLine('#define FLASH_TABLE_START_ADDR (0x{:08X})'.format(flash_table_start))
        s += MakeLine('')

    s += MakeLine('RT_USED const struct sec_configuration sec_config =')
    s += MakeLine('{')
    IncIndentation()
    s += MakeLine('.magic = SEC_CONFIG_MAGIC,')
    s += MakeLine(
        '.ftab[0] = {.base = FLASH_TABLE_START_ADDR,      .size = FLASH_TABLE_SIZE,      .xip_base = 0, .flags = 0},')
    s += MakeLine(
        '.ftab[1] = {.base = FLASH_CAL_TABLE_START_ADDR,  .size = FLASH_CAL_TABLE_SIZE,  .xip_base = 0, .flags = 0},')
    
    if dfu_present:
        s += MakeLine('.ftab[2] = {{.base = 0x{:08X}, .size = 0x{:08X},  .xip_base = 0x{:08X}, .flags = 0}},'.format(
            ftab['dfu']['base'], ftab['dfu']['max_size'], ftab['dfu']['xip']))

    if bootloader_needed:
        s += MakeLine('.ftab[3] = {{.base = 0x{:08X}, .size = 0x{:08X},  .xip_base = 0x{:08X}, .flags = 0}},'.format(
            ftab['bootloader']['base'], ftab['bootloader']['max_size'], ftab['bootloader']['xip']))
    if 'main' in ftab:
        s += MakeLine('.ftab[4] = {{.base = 0x{:08X}, .size = 0x{:08X},  .xip_base = 0x{:08X}, .flags = 0}},'.format(
            ftab['main']['base'], ftab['main']['max_size'], ftab['main']['xip']))
    s += MakeLine(
        '.ftab[5] = {.base = FLASH_BOOT_PATCH_START_ADDR, .size = FLASH_BOOT_PATCH_SIZE, .xip_base = '
        'BOOTLOADER_PATCH_CODE_ADDR, .flags = 0},')
    if bootloader_needed:
        s += MakeLine('.ftab[7] = {{.base = 0x{:08X}, .size = 0x{:08X},  .xip_base = 0x{:08X}, .flags = 0}},'.format(
            ftab['bootloader']['base'], ftab['bootloader']['max_size'], ftab['bootloader']['xip']))
    if 'main' in ftab:
        if 'base2' in ftab['main']:
            s += MakeLine('.ftab[8] = {{.base = 0x{:08X}, .size = 0x{:08X},  .xip_base = 0x{:08X}, .flags = 0}},'.format(
                ftab['main']['base2'], ftab['main']['max_size'], ftab['main']['xip']))
        else:
            s += MakeLine('.ftab[8] = {{.base = 0x{:08X}, .size = 0x{:08X},  .xip_base = 0x{:08X}, .flags = 0}},'.format(
                ftab['main']['base'], ftab['main']['max_size'], ftab['main']['xip']))
    s += MakeLine(
        '.ftab[9] = {.base = BOOTLOADER_PATCH_CODE_ADDR,  .size = FLASH_BOOT_PATCH_SIZE, .xip_base = '
        'BOOTLOADER_PATCH_CODE_ADDR, .flags = 0},')

    if building.GetConfigValue("FLASH_CONFIG_PAGE_SIZE") != '' and building.GetConfigValue("FLASH_CONFIG_BLOCK_SIZE") != '':
        s += MakeLine(
            '.ftab[10] = {.base = FLASH_CONFIG_PAGE_SIZE,  .size = FLASH_CONFIG_BLOCK_SIZE, .xip_base = 0, .flags = 0},')

    if 'main' in img_size:
        size = img_size['main'][0]['size']
    else:
        size = 0x200000
    if 'bootloader' in img_size:
        bl_size = img_size['bootloader'][0]['size']
    else:
        bl_size = 0x10000
    s += MakeLine(
        '.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU)] = {{.length = 0x{:08X}, .blksize = 512, .flags = DFU_FLAG_AUTO}},'.format(
            size))
    if dfu_present:
        s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)] = {{.length = 0x{:08X}, .blksize = 512, .flags = DFU_FLAG_AUTO}},'.format(
            img_size['dfu'][0]['size']))
    else:
        s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)] = {.length = 0xFFFFFFFF},')
    if bootloader_needed:
        s += MakeLine(
            '.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BL)] = {{.length = 0x{:08X}, .blksize = 512, .flags = DFU_FLAG_AUTO}},'.format(
                bl_size))
    else:
        s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BL)] = {.length = 0xFFFFFFFF},')
    s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BOOT)] = {.length = 0xFFFFFFFF},')
    s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU2)] = {.length = 0xFFFFFFFF},')
    s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BCPU2)] = {.length = 0xFFFFFFFF},')
    s +=  MakeLine(
        '.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2)] = {{.length = 0x{:08X}, .blksize = 512, .flags = DFU_FLAG_AUTO}},'.format(
            size))
    s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BOOT2)] = {.length = 0xFFFFFFFF},')

    # If more images are specified in partition table, use EXT region to describe how it's saved
    # ex_region_names = ['DFU_FLASH_HCPU_EXT1', 'DFU_FLASH_HCPU_EXT2', 'DFU_FLASH_LCPU_EXT1', 'DFU_FLASH_LCPU_EXT2']
    ex_region_names = ['DFU_FLASH_HCPU_EXT2', 'DFU_FLASH_LCPU_EXT1', 'DFU_FLASH_LCPU_EXT2']
    ex_region_idx = 0
    for k, v in ftab.items():
        if (k != 'main') and (k != 'bootloader') and (k != 'dfu') :
            ftab_item = v
            if 'img' in ftab_item:
                assert 'base' in ftab_item, 'base of {} is not defined'.format(k)
                assert 'xip' in ftab_item, 'xip of {} is not defined'.format(k)
                assert 'max_size' in ftab_item, 'max_size of {} is not defined'.format(k)
                s += MakeLine(
                    '.ftab[{}] = {{.base = 0x{:08X}, .size = 0x{:08X},  .xip_base = 0x{:08X}, .flags = 0}},'.format(
                        ex_region_names[ex_region_idx], ftab_item['base'], ftab_item['max_size'], ftab_item['xip']))
                assert 'img' in ftab_item, "img binary name for {} must be specified if img needs to be copied from " \
                                           "base to xip address".format(k)
                img_info = ftab_item['img']
                size = img_info['size']
                s += MakeLine(
                    '.imgs[DFU_FLASH_IMG_IDX({})] = {{.length = 0x{:08X}, .blksize = 512, .flags = DFU_FLAG_AUTO}},'.format(
                        ex_region_names[ex_region_idx], size))
                ex_region_idx += 1
            else:
                print('WARNING: img info is not defined for ftab item "{}", ignored'.format(k))

    for region_name in ex_region_names[ex_region_idx:]:
        s += MakeLine('.imgs[DFU_FLASH_IMG_IDX({})] = {{.length = 0xFFFFFFFF}},'.format(region_name))

    s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_RESERVED)] = {.length = 0xFFFFFFFF},')
    s += MakeLine('.imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_SINGLE)] = {.length = 0xFFFFFFFF},')
    if 'main' in ftab:
        s += MakeLine(
            '.running_imgs[CORE_HCPU] = (struct image_header_enc *)&(((struct sec_configuration '
            '*)FLASH_TABLE_START_ADDR)->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU)]),')
    else:
        s += MakeLine('.running_imgs[CORE_HCPU] = (struct image_header_enc *)0xFFFFFFFF,')

    if dfu_present:
        s += MakeLine(
            '.running_imgs[CORE_LCPU] = (struct image_header_enc *)&(((struct sec_configuration '
            '*)FLASH_TABLE_START_ADDR)->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)]),')
    else:
        s += MakeLine('.running_imgs[CORE_LCPU] = (struct image_header_enc *)0xFFFFFFFF,')

    if bootloader_needed:
        s += MakeLine(
            '.running_imgs[CORE_BL] = (struct image_header_enc *)&(((struct sec_configuration '
            '*)FLASH_TABLE_START_ADDR)->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_BL)]),')
    else:
        s += MakeLine('.running_imgs[CORE_BL] = (struct image_header_enc *)0xFFFFFFFF,')    
    s += MakeLine('.running_imgs[CORE_BOOT] = (struct image_header_enc *)0xFFFFFFFF,')

    DecIndentation()
    s += MakeLine('};')
    s += MakeLine('')

    f = open(os.path.join(output_name), "w")
    f.write(s)
    f.close()


def ConstructImgDownloadInfoV2(img_download_info, mems):
    PtabAddAddDefaultRegion(mems)
    for mem in mems:
        mem_base = int(mem['base'], 0)
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            start_addr = mem_base + offset
            if 'name' in region:
                temp = region['name'].split(':')
                if len(temp) > 1:
                    base, ext =  temp
                else:
                    base = temp[0]
                    ext = None

                if 'type' in region and 'app_exec' in region['type'] and (1 == len(region['type'])):
                    # no need to download to region with app_exec type only
                    continue

                if 'type' in region and 'app_img2' in region['type'] and (1 == len(region['type'])):
                    # no need to download to region with app_img2 type only, as it's only used by ftab
                    continue

                proj_name = base
                if not ext:
                    assert proj_name not in img_download_info, "{} download address already configured".format(proj_name)
                    img_download_info[proj_name] = start_addr
                else:
                    if proj_name not in img_download_info:
                        img_download_info[proj_name] = {}
                    assert ext not in img_download_info[
                        proj_name], "{} download address already configured".format(region['img'])
                    img_download_info[proj_name][GetMultiBinaryName(ext)] = start_addr


def ConstructImgDownloadInfoV1(img_download_info, mems):    
    for mem in mems:
        mem_base = int(mem['base'], 0)
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            start_addr = mem_base + offset
            if 'img' in region:
                img_name = region['img']
                img_name = img_name.split(':')
                proj_name = img_name[0]
                if len(img_name) == 1:
                    assert proj_name not in img_download_info, "{} download address already configured".format(proj_name)
                    img_download_info[proj_name] = start_addr
                else:
                    if proj_name not in img_download_info:
                        img_download_info[proj_name] = {}
                    assert img_name[1] not in img_download_info[
                        proj_name], "{} download address already configured".format(region['img'])
                    img_download_info[proj_name][img_name[1]] = start_addr


def ConstructImgDownloadInfoV3(img_download_info, ptab_obj):
    chip_config = ptab_obj.get_chip_config()
    fs_subtypes = ('filesystem', 'littlefs', 'fat', 'fatfs', 'flashdb')

    def add_download_alias(name, addr):
        if not name:
            return
        if name in img_download_info:
            assert img_download_info[name] == addr, "{} download address already configured".format(name)
            return
        img_download_info[name] = addr

    for partition in ptab_obj.partitions:
        name = partition.get('name', '')
        ptype = partition.get('type', '')
        subtype = partition.get('subtype', '')
        region = partition.get('region', '')
        offset = ptab.parse_size(partition.get('offset', 0))
        core = partition.get('core')

        download_addr = ptab.get_download_addr_v3(region, offset, chip_config, core=core)

        if ptype == 'ftab':
            img_download_info['ftab'] = download_addr
            continue

        if ptype == 'data' and str(subtype or '').strip().lower() in fs_subtypes:
            add_download_alias(name, download_addr)
            add_download_alias('fs_root', download_addr)
            continue

        if ptype not in ('app', 'bootloader'):
            continue
        if ptype == 'app' and str(subtype or '').strip().lower() == 'ex':
            continue

        img_download_info[name] = download_addr

        # Add 'main' alias for HCPU factory app (env['name'] is usually 'main')
        part_core = (core or 'HCPU').upper()
        if ptype == 'app' and subtype == 'factory' and part_core == 'HCPU':
            img_download_info['main'] = download_addr
        # Add 'dfu' alias for HCPU dfu app (env['name'] is usually 'dfu')
        if ptype == 'app' and subtype == 'dfu' and part_core == 'HCPU':
            img_download_info['dfu'] = download_addr


def BuildJLinkLoadScript(main_env):
    import building

    # proj name as key, if proj has multiple binary, the value is a dictionary too. And binary name is used as key
    # example1:  {"main": 0x18000000}
    # example1:  {"main": {"ROM1.bin": 0x18000000, "ROM2.bin": 0x18200000}}
    img_download_info = {}
    is_ptab_v3 = False
    int_res_download_items = {}

    if 'PARTITION_TABLE' in main_env:
        ptab_obj = ptab.load_ptab(main_env['PARTITION_TABLE'], fatal=True)
        mems = ptab_obj.content_mems()

        if ptab_obj.is_v3():
            is_ptab_v3 = True
            ConstructImgDownloadInfoV3(img_download_info, ptab_obj)
            # Pre-compute int_res download addresses (per core)
            chip_config = ptab_obj.get_chip_config()
            for core in ('HCPU', 'LCPU', 'ACPU'):
                items = []
                for part in ptab.iter_int_res_partitions_v3(ptab_obj, core=core):
                    name = part.get('name', '')
                    if not name:
                        continue
                    region = part.get('region', '')
                    offset = ptab.parse_size(part.get('offset', 0))
                    addr = ptab.get_download_addr_v3(region, offset, chip_config, core=part.get('core'))
                    items.append({'name': name, 'addr': addr})
                int_res_download_items[core] = items
        elif ptab_obj.is_v2():
            ConstructImgDownloadInfoV2(img_download_info, mems)
        else:
            ConstructImgDownloadInfoV1(img_download_info, mems)

    work_dir = main_env['build_dir']
    InitIndentation()
    s = ''
    s += MakeLine('si SWD')
    s += MakeLine('speed 10000')
    s += MakeLine('r')

    # prepare for `ImgBurnList.ini`
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    ImgDownUart_PATH = os.path.join(SIFLI_SDK, "tools/uart_download/ImgDownUart.exe")
    s_file = MakeLine('[FILEINFO]')
    s_num = 0

    download_file = []
    env_list = building.GetEnvList()
    for env in env_list:
        # if building.IsEmbeddedProjEnv(env):
        #  continue
        if env['name'] in img_download_info:
            # load address is defined in map, load binary using address
            info = img_download_info[env['name']]
            bin_file = str(env['program_binary'][0])
            if type(info) is dict:
                assert os.path.isdir(bin_file), "{} should have multiple binaries as map defines".format(env['name'])

                dir_list = os.listdir(bin_file)
                for d in dir_list:
                    assert d in info, "{}.{} download address is not configured".format(env['name'], d)
                for d in dir_list:
                    bin_path = os.path.join(bin_file, d)
                    s += MakeLine('loadbin {} 0x{:08X}'.format(os.path.relpath(bin_path, work_dir), info[d]))
                    download_file.append({
                        'name': os.path.relpath(bin_path, work_dir),
                        'addr': info[d]
                    })
                    s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(bin_path, work_dir)))
                    s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,info[d]))
                    s_num += 1
            else:
                if is_ptab_v3 and building.IsEmbeddedProjEnv(env) and env.get('name') == 'lcpu':
                    base_name = building.GetPtabV3ArtifactBaseName(env)
                    bin_file = building.ResolvePtabV3ArtifactContainerFromRef(
                        bin_file,
                        base_name,
                        '.bin',
                    )
                    if os.path.isdir(bin_file):
                        bin_file = building.GetPtabV3ArtifactPath(bin_file, base_name, '.bin', 'ER_IROM2')
                    else:
                        bin_file = building.GetPtabV3ArtifactPath(
                            os.path.dirname(bin_file),
                            base_name,
                            '.bin',
                            'ER_IROM2',
                        )

                    if not os.path.isfile(bin_file) or os.path.getsize(bin_file) <= 0:
                        logging.error(
                            "ptab v3 embedded LCPU flash artifact ER_IROM2.bin not found in %s",
                            bin_file,
                        )
                        raise SystemExit(1)

                    s += MakeLine('loadbin {} 0x{:08X}'.format(os.path.relpath(bin_file, work_dir), info))
                    download_file.append({
                            'name': os.path.relpath(bin_file, work_dir),
                            'addr': info
                        })
                    s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(bin_file, work_dir)))
                    s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,info))
                    s_num += 1
                else:
                    if is_ptab_v3:
                        bin_file = building.ResolvePtabV3CodeArtifactFromRef(
                            bin_file,
                            building.GetPtabV3ArtifactBaseName(env),
                            '.bin',
                        )
                    elif os.path.isdir(bin_file):
                        preferred = os.path.join(bin_file, 'ER_IROM1.bin')
                        assert os.path.isfile(preferred), "{} should contain ER_IROM1.bin as code image".format(env['name'])
                        bin_file = preferred
                    assert os.path.isfile(bin_file), "{} should be a file as map defines".format(env['name'])
                    s += MakeLine('loadbin {} 0x{:08X}'.format(os.path.relpath(bin_file, work_dir), info))
                    download_file.append({
                            'name': os.path.relpath(bin_file, work_dir),
                            'addr': info
                        })
                    s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(bin_file, work_dir)))
                    s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,info))
                    s_num += 1

                # ptab v3: load app/ex resource bins from the same `output/` directory
                if is_ptab_v3:
                    core = 'HCPU'
                    if env.get('name') == 'lcpu':
                        core = 'LCPU'
                    elif env.get('name') == 'acpu':
                        core = 'ACPU'

                    res_dir = bin_file if os.path.isdir(bin_file) else os.path.dirname(bin_file)
                    base_name = building.GetPtabV3ArtifactBaseName(env)
                    for item in int_res_download_items.get(core, []):
                        res_path = building.GetPtabV3ArtifactPath(
                            res_dir,
                            base_name,
                            '.bin',
                            item['name'],
                        )
                        if res_path == bin_file or not os.path.isfile(res_path):
                            continue
                        if os.path.getsize(res_path) <= 0:
                            continue
                        s += MakeLine('loadbin {} 0x{:08X}'.format(os.path.relpath(res_path, work_dir), item['addr']))
                        download_file.append({
                            'name': os.path.relpath(res_path, work_dir),
                            'addr': item['addr']
                        })
                        s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(res_path, work_dir)))
                        s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,item['addr']))
                        s_num += 1
        else:
            hex_file = str(env['program_hex'][0])
            # load address is not defined, load hex
            if is_ptab_v3:
                hex_file = building.ResolvePtabV3ArtifactContainerFromRef(
                    hex_file,
                    building.GetPtabV3ArtifactBaseName(env),
                    '.hex',
                )
            if os.path.isdir(hex_file):
                dir_list = os.listdir(hex_file)
                for d in dir_list:
                    if not d.lower().endswith('.hex'):
                        continue
                    if building.IsEmbeddedProjEnv(env) and 'ER_IROM1' in d:
                        # LCPU RAM code is embedded in parent, others need to be downloaded.
                        continue
                    hex_path = os.path.join(hex_file, d)
                    s += MakeLine('loadfile {}'.format(os.path.relpath(hex_path, work_dir)))
                    download_file.append({
                        'name': os.path.relpath(hex_path, work_dir),
                        'addr': 0xFFFFFFFF
                    })
                    s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(hex_path, work_dir)))
                    s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,0XFFFFFFFF))
                    s_num += 1
            elif not building.IsEmbeddedProjEnv(env):
                s += MakeLine('loadfile {}'.format(os.path.relpath(hex_file, work_dir)))
                download_file.append({
                        'name': os.path.relpath(hex_file, work_dir),
                        'addr': 0xFFFFFFFF
                    })
                s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(hex_file, work_dir)))
                s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,0XFFFFFFFF))
                s_num += 1

    # Process ftab.bin separately (not the project, but the directly generated binary file)
    if is_ptab_v3 and 'ftab' in img_download_info:
        ftab_bin_path = os.path.join(work_dir, 'ftab.bin')
        if os.path.exists(ftab_bin_path):
            ftab_addr = img_download_info['ftab']
            s += MakeLine('loadbin {} 0x{:08X}'.format('ftab.bin', ftab_addr))
            download_file.append({
                'name': 'ftab.bin',
                'addr': ftab_addr
            })
            s_file += MakeLine('FILE{}={}'.format(s_num, 'ftab.bin'))
            s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num, ftab_addr))
            s_num += 1


    custom_img_list = building.GetCustomImgList()
    
    for env in custom_img_list:
        if env['name'] in img_download_info:
            # load address is defined in map, load binary using address
            info = img_download_info[env['name']]
            if 0 == len(env['program_binary']):
                continue
            bin_file = str(env['program_binary'][0])
            if type(info) is dict:
                assert os.path.isdir(bin_file), "{} should have multiple binaries as map defines".format(env['name'])

                dir_list = os.listdir(bin_file)
                for d in dir_list:
                    assert d in info, "{}.{} download address is not configured".format(env['name'], d)
                for d in dir_list:
                    bin_path = os.path.join(bin_file, d)
                    s += MakeLine('loadbin {} 0x{:08X}'.format(os.path.relpath(bin_path, work_dir), info[d]))

                    s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(bin_path, work_dir)))
                    s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,info[d]))
                    download_file.append({
                        'name': os.path.relpath(bin_path, work_dir),
                        'addr': info[d]
                    })
                    s_num += 1
            else:
                assert os.path.isfile(bin_file), "{} should be a file as map defines".format(bin_file)
                s += MakeLine('loadbin {} 0x{:08X}'.format(os.path.relpath(bin_file, work_dir), info))
                download_file.append({
                        'name': os.path.relpath(bin_file, work_dir),
                        'addr': info
                    })
                s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(bin_file, work_dir)))
                s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,info))
                s_num += 1
        else:
            try:
                if 0 == len(env['program_hex']):
                    continue
            except:
                logging.debug('No program_hex file specified')
                continue;
            hex_file = str(env['program_hex'][0])
            # load address is not defined, load hex
            if os.path.isdir(hex_file):
                dir_list = os.listdir(hex_file)
                for d in dir_list:
                    if not d.lower().endswith('.hex'):
                        continue
                    if building.IsEmbeddedProjEnv(env) and 'ER_IROM1' in d:
                        # LCPU RAM code is embedded in parent, others need to be downloaded.
                        continue
                    hex_path = os.path.join(hex_file, d)
                    s += MakeLine('loadfile {}'.format(os.path.relpath(hex_path, work_dir)))
                    download_file.append({
                        'name': os.path.relpath(hex_path, work_dir),
                        'addr': 0XFFFFFFFF
                    })
                    s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(hex_path, work_dir)))
                    s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,0XFFFFFFFF))
                    s_num += 1
            elif not building.IsEmbeddedProjEnv(env):
                s += MakeLine('loadfile {}'.format(os.path.relpath(hex_file, work_dir)))
                download_file.append({
                        'name': os.path.relpath(hex_file, work_dir),
                        'addr': 0XFFFFFFFF
                    })
                s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(hex_file, work_dir)))
                s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,0XFFFFFFFF))
                s_num += 1
    s += MakeLine('exit')
    f = open(os.path.join(main_env['build_dir'], 'download.jlink'), 'w')
    f.write(s)
    f.close()

    s_file += MakeLine('NUM={}'.format(s_num))
    sf = open(os.path.join(main_env['build_dir'], 'ImgBurnList.ini'), 'w')
    sf.write(s_file)
    sf.close()

    InitIndentation()
    s = MakeLine('set WORK_PATH=%~dp0')
    s += MakeLine('set CURR_PATH=%cd%')
    s += MakeLine('cd %WORK_PATH%')
    temp = 'jlink.exe'
    # DIF:devices interface
    if 'DIF' in main_env:
        temp += ' -ip 127.0.0.1:19025 -device {}'.format(main_env['JLINK_DEVICE'])
    else:
        if 'JLINK_DEVICE' in main_env:
            temp += ' -device {}'.format(main_env['JLINK_DEVICE'])
        else:
            temp += ' -device ?'
    temp += ' -CommandFile download.jlink'
    s += MakeLine(temp)
    s += MakeLine('cd %CURR_PATH%')
    f = open(os.path.join(main_env['build_dir'], 'download.bat'), 'w')
    f.write(s)
    f.close()

    # Generate download.sh for Linux/macOS
    InitIndentation()
    s = MakeLine('#!/bin/bash')
    s += MakeLine('')
    s += MakeLine('WORK_PATH="$(cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd)"')
    s += MakeLine('CURR_PATH="$(pwd)"')
    s += MakeLine('cd "$WORK_PATH"')
    s += MakeLine('')
    temp = 'JLinkExe'
    # DIF:devices interface
    if 'DIF' in main_env:
        temp += ' -ip 127.0.0.1:19025 -device {}'.format(main_env['JLINK_DEVICE'])
    else:
        if 'JLINK_DEVICE' in main_env:
            temp += ' -device {}'.format(main_env['JLINK_DEVICE'])
        else:
            temp += ' -device ?'
    temp += ' -CommandFile download.jlink'
    s += MakeLine(temp)
    s += MakeLine('')
    s += MakeLine('cd "$CURR_PATH"')
    sh_path = os.path.join(main_env['build_dir'], 'download.sh')
    f = open(sh_path, 'w')
    f.write(s)
    f.close()
    # Make the script executable
    os.chmod(sh_path, 0o755)

    decice_memory = main_env['JLINK_DEVICE'].split('_')
    device = decice_memory[0][:-1]
    memory = decice_memory[1] if len(decice_memory) > 1 else 'NOR'

    download_list = ' '.join(
        f"\"{file['name']}\"" if file['name'].lower().endswith(('.elf', '.hex')) 
        else f"\"{file['name']}@0x{file['addr']:08X}\"" 
        for file in download_file
    )

    generate_uart_download_bat(main_env, device, memory, download_list, ImgDownUart_PATH)
    generate_uart_download_sh(main_env, device, memory, download_list)
    generate_sftool_param(main_env, device, memory, download_file)


def generate_uart_download_bat(main_env, device, memory, download_list, ImgDownUart_PATH):
    uart_comment = '''@echo off
title=uart download
set WORK_PATH=%~dp0
set CURR_PATH=%cd%
cd %WORK_PATH%
:start
echo,
echo      Uart Download
echo,
set /p input=please input the serial port num:
goto download
:download
echo com%input%
'''
    if os.getenv("LEGACY_ENV"):
        uart_comment += MakeLine('''{}  --func 0 --port com%input% --baund 1000000 --loadram 1 --postact 1 --compare --verify --device {} --file ImgBurnList.ini --log ImgBurn.log
if %errorlevel%==0 (
    echo Download Successful
)else (
    echo Download Failed
    echo logfile:%WORK_PATH%ImgBurn.log
)
cd %CURR_PATH%
'''.format(ImgDownUart_PATH, main_env['JLINK_DEVICE']))
    else:
        uart_comment += MakeLine(f"sftool -p COM%input% -c {device} -m {memory.lower()} write_flash {download_list}\n")
    uart_comment += MakeLine('if "%ENV_ROOT%"=="" pause\n')
    
    uart_f = open(os.path.join(main_env['build_dir'], 'uart_download.bat'), 'w')
    uart_f.write(uart_comment)
    uart_f.close()

def generate_uart_download_sh(main_env, device, memory,download_list):
    uart_comment = '''#!/bin/bash

WORK_PATH="$(cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd)"
cd "$WORK_PATH"

echo ""
echo "      Uart Download"
echo ""

input=""
if [ $# -gt 0 ]; then
  case "$1" in
    -p|--port)
      shift
      input="${1:-}"
      ;;
    *)
      input="$1"
      ;;
  esac
fi

if [ -n "$input" ]; then
  # Port provided as argument
  echo "Using port from args: $input"
else
  # If no port is specified, enumerate available ports
  ports=()
  unameOut="$(uname)"
  if [ "$unameOut" = "Darwin" ]; then
    for p in /dev/cu.*; do
      if [ -e "$p" ] && echo "$p" | grep -qi "usb"; then
        ports+=("$p")
      fi
    done
  elif [ "$unameOut" = "Linux" ]; then
    for p in /dev/ttyUSB* /dev/ttyACM*; do
      [ -e "$p" ] && ports+=("$p")
    done
  else
    echo "Unsupported OS: $unameOut"
    exit 1
  fi

  if [ "${#ports[@]}" -gt 0 ]; then
    echo "Available serial ports:"
    for i in "${!ports[@]}"; do
      printf "  [%d] %s\n" "$i" "${ports[$i]}"
    done
  else
    echo "No ports auto-detected."
  fi

  read -r -p "Enter index OR type a port (e.g. /dev/ttyUSB0): " sel
  if [[ "$sel" =~ ^[0-9]+$ ]] && [ "$sel" -ge 0 ] && [ "$sel" -lt "${#ports[@]}" ]; then
    # Index provided
    input="${ports[$sel]}"
  else
    # Assume direct port input, the old way
    input="$sel"
  fi
fi

echo "$input"

'''
    
    if os.getenv("LEGACY_ENV"):
        uart_comment += MakeLine('echo "Legacy mode is not supported on Linux/macOS"')
    else:
        uart_comment += MakeLine(f'sftool -p "$input" -c {device} -m {memory.lower()} write_flash {download_list}\n')
    
    uart_sh_path = os.path.join(main_env['build_dir'], 'uart_download.sh')
    uart_f = open(uart_sh_path, 'w')
    uart_f.write(uart_comment)
    uart_f.close()
    
    os.chmod(uart_sh_path, 0o755)

def generate_sftool_param(main_env, device, memory, download_list):
    import json
    
    # Convert download_list to sftool format
    files = []
    for item in download_list:      
        addr_str = f"0x{item['addr']:08X}"
        
        if item['name'].lower().endswith(('.elf', '.hex')):
            files.append({"path": item['name']})
        else:
            files.append({"path": item['name'], "address": addr_str})

    # Create sftool configuration
    config = {
        "chip": device,
        "memory": memory,
    }
    
    config["write_flash"] = {
        "verify": True,
        "files": files
    }
    
    # Write JSON file
    json_path = os.path.join(main_env['build_dir'], 'sftool_param.json')
    try:
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        print(f"Generated sftool param: {json_path}")
    except Exception as e:
        print(f"Error writing sftool param: {e}")

    return json_path
