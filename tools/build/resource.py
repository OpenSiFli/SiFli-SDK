import codecs
import glob
import json
import os
import subprocess
import shutil
import sys
# import png
import logging

output_folder = "output"
img_folder = "images"
string_folder = "strings"
font_folder = "fonts"
font_config_src_file = "fonts/config.json"
font_config_output_file = "fonts_config"
lang_pack_name = "lang_pack"
resource_db = {'images': set(),
               'strings': set()}

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
    for k, v in s.items():
        # k = "str_" + k
        # data = data + u'char *{} = "{}";\n'.format(k,v)
        # key and singular
        translation = u'"{}", "{}"'.format(k, v)
        # plural
        translation = translation + u', {0}'
        data = data + u"    .{} = {{{}}},\n".format(k, translation)
        resource_db['strings'].add(k)

    data = template.format(lang, data, lang, lang, lang)

    str_cfile_filename = lang + ".c"
    str_cfile_fullpath = os.path.join(output_dir, str_cfile_filename)
    try:
        f = codecs.open(str_cfile_fullpath, "w", "utf-8")
        f.write(data)
    finally:
        f.close()


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
    string_res_file = glob.glob(str_src_dir + '/*.json')
    if not os.path.exists(str_output_dir):
        os.makedirs(str_output_dir)
    else:
        file_list = glob.glob(os.path.join(str_output_dir, "*.*"))
        file_list += glob.glob(os.path.join(str_output_dir, "SConscript"))
        [os.remove(f) for f in file_list]

    lang_pack = set()
    for f_name in string_res_file:
        try:
            f = open(f_name, encoding='utf-8')
            s = json.load(f)
        finally:
            f.close()

        str_file_basename = os.path.basename(f_name)
        str_file_basename = os.path.splitext(str_file_basename)[0]

        GenerateLangCFile(str_file_basename, s, str_output_dir)
        lang_pack.add(str_file_basename)

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

def Convert2CBUSAddr55x(addr, offset, core=None):
    return addr, offset


def Convert2CBUSAddr56x(addr, offset, core=None):
    cbus_addr  = addr
    cbus_offset = offset

    if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
        cbus_addr -= 0x50000000

    return cbus_addr, cbus_offset


def Convert2CBUSAddr58x(addr, offset, core=None):
    cbus_addr  = addr
    cbus_offset = offset

    if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
        cbus_addr -= 0x50000000
    elif (addr >= 0x20000000) and (addr <= 0x2FFFFFFF) and core and core.lower() == "acpu":
        cbus_addr -= 0x20200000
        assert cbus_addr >=  0, "0x{:8X} is not a valid address for ACPU"
        cbus_offset -= 0x00200000

    return cbus_addr, cbus_offset

def Convert2CBUSAddr52x(addr, offset, core=None):
    cbus_addr  = addr
    cbus_offset = offset

    if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
        cbus_addr -= 0x50000000

    return cbus_addr, cbus_offset


def Convert2CBUSAddr(addr, offset, core=None):
    import building

    if building.GetDepend("SOC_SF32LB55X"):
        return Convert2CBUSAddr55x(addr, offset, core)
    elif building.GetDepend("SOC_SF32LB56X"):
        return Convert2CBUSAddr56x(addr, offset, core)
    elif building.GetDepend("SOC_SF32LB58X"):
        return Convert2CBUSAddr58x(addr, offset, core)
    elif building.GetDepend("SOC_SF32LB52X"):
        return Convert2CBUSAddr52x(addr, offset, core)
    else:
        raise Exception("unknown chip")
   


def PtabAddAddDefaultRegion55x(mems):
    ftab_found = False
    flash1_mem = None
    for mem in mems:
        if 'flash1' == mem['mem']:
            flash1_mem = mem
        if "regions" not in mem:
            continue    
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True

        if ftab_found:
            break

    if not ftab_found:
        if not flash1_mem:
            flash1_mem = {
                "mem": "flash1", 
                "base": "0x10000000", 
                "regions": []
            }
            mems.insert(0, flash1_mem)

        if 'regions' not in flash1_mem:
            flash1_mem['regions'] =  []


        ftab_region = {
            "offset": "0x00000000", 
            "max_size": "0x00005000", 
            "tags": [
                "FLASH_TABLE"
            ], 
            "name": "ftab",
            "type": ["app_img", "app_exec"]
        }                    
        flash1_mem['regions'].insert(0, ftab_region)


def PtabAddAddDefaultRegion56x(mems):
    ftab_found = False
    bootloader_found = False
    flash5_mem = None
    for mem in mems:
        if 'flash5' == mem['mem']:
            flash5_mem = mem
        if "regions" not in mem:
            continue    
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_found = True

        if ftab_found and bootloader_found:
            break

    if (not ftab_found) or (not bootloader_found):
        if not flash5_mem:
            flash5_mem = {
                "mem": "flash5", 
                "base": "0x1C000000", 
                "regions": []
            }
            mems.insert(0, flash5_mem)

        if 'regions' not in flash5_mem:
            flash5_mem['regions'] =  []

        if not ftab_found:
            ftab_region = {
                "offset": "0x00000000", 
                "max_size": "0x00004000", 
                "tags": [
                    "FLASH_TABLE"
                ], 
                "name": "ftab",
                "type": ["app_img", "app_exec"]
            }                    
            flash5_mem['regions'].insert(0, ftab_region)
   
        if not bootloader_found:
            bootloader_region =  {
                "offset": "0x00020000", 
                "max_size": "0x0000C000", 
                "tags": [
                    "FLASH_BOOT_LOADER"
                ], 
                "name": "bootloader",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, bootloader_region)


def PtabAddAddDefaultRegion58x(mems):
    ftab_found = False
    bootloader_found = False
    flash5_mem = None
    for mem in mems:
        if 'flash5' == mem['mem']:
            flash5_mem = mem
        if "regions" not in mem:
            continue    
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_found = True

        if ftab_found and bootloader_found:
            break

    if (not ftab_found) or (not bootloader_found):
        if not flash5_mem:
            flash5_mem = {
                "mem": "flash5", 
                "base": "0x1C000000", 
                "regions": []
            }
            mems.insert(0, flash5_mem)

        if 'regions' not in flash5_mem:
            flash5_mem['regions'] =  []

        if not ftab_found:
            ftab_region = {
                "offset": "0x00000000", 
                "max_size": "0x00005000", 
                "tags": [
                    "FLASH_TABLE"
                ], 
                "name": "ftab",
                "type": ["app_img", "app_exec"]
            }                    
            flash5_mem['regions'].insert(0, ftab_region)
   
        if not bootloader_found:
            bootloader_region =  {
                "offset": "0x00020000", 
                "max_size": "0x00020000", 
                "tags": [
                    "FLASH_BOOT_LOADER"
                ], 
                "name": "bootloader",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, bootloader_region)


def PtabAddAddDefaultRegion52x(mems):
    import building
    ftab_found = False
    bootloader_exec_found = False
    bootloader_img_found = False
    bootloader_data_found = False
    boot_mem = None
    hpsys_ram_mem = None
    
    for mem in mems:
        # guess boot_dev_type and boot_mem by memory name and address
        if "flash1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = "nor"
        elif "flash2" == mem['mem']:
            boot_mem = mem
            if "0x12000000" == mem['base']:
                boot_dev_type = "nor"
            else:
                boot_dev_type = "nand"
        elif "sd1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = 'sd'

        if "hpsys_ram" == mem['mem']:
            hpsys_ram_mem = mem
            continue

        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_img_found = True

    for region in hpsys_ram_mem:
        if "name" in region and 'bootloader' == region['name'] and 'type' in region and "app_exec" in region['type']:
            bootloader_exec_found = True
            
        if "name" in region and 'bootloader' == region['name']:
            bootloader_data_found = True

    if not bootloader_exec_found:
        bootloader_region = {
            "offset": "0x00020000", 
            "max_size": "0x00010000", 
            "name": "bootloader",
            "type": ["app_exec"],
            "tags": ["FLASH_BOOT_LOADER"]
        }
        hpsys_ram_mem["regions"].insert(0, bootloader_region)

    if not bootloader_data_found:
        bootloader_region = {
            "offset": "0x00040000", 
            "max_size": "0x00010000", 
            "tags": ["BOOTLOADER_RAM_DATA"]
        }
        hpsys_ram_mem['regions'].insert(0, bootloader_region)

    if (not ftab_found) or (not bootloader_img_found):
        if not ftab_found:
            if "sd" == boot_dev_type :
                # MBR uses first 4096 bytes
                ftab_region = {
                    "offset": "0x00001000", 
                    "max_size": "0x00008000", 
                    "tags": ["FLASH_TABLE"], 
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }                    
            else:
                ftab_region = {
                    "offset": "0x00000000", 
                    "max_size": "0x00008000", 
                    "tags": ["FLASH_TABLE"], 
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }

            boot_mem['regions'].insert(0, ftab_region)
   
        if not bootloader_img_found:
            if "sd" == boot_dev_type:
                bootloader_region =  {
                    "offset": "0x00011000", 
                    "max_size": "0x00010000", 
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            elif "nor" == boot_dev_type:
                bootloader_region =  {
                    "offset": "0x00010000", 
                    "max_size": "0x00010000", 
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }    
            elif "nand" == boot_dev_type:
                bootloader_region =  {
                    "offset": "0x00080000", 
                    "max_size": "0x00010000", 
                    "tags": [],                    
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            else:
                raise Exception(f"unknown type {boot_dev_type}")
            boot_mem['regions'].insert(0, bootloader_region)


def PtabAddAddDefaultRegion(mems):
    import building
    
    if building.GetDepend("SOC_SF32LB55X"):
        PtabAddAddDefaultRegion55x(mems)
    elif building.GetDepend("SOC_SF32LB56X"):
        PtabAddAddDefaultRegion56x(mems)
    elif building.GetDepend("SOC_SF32LB58X"):
        PtabAddAddDefaultRegion58x(mems)
    elif building.GetDepend("SOC_SF32LB52X"):
        PtabAddAddDefaultRegion52x(mems)
    else:
        raise Exception("unknown chip")


def GenPartitionTableHeaderContentV2(env, mems):
    s =  ''
    PtabAddAddDefaultRegion(mems)
    for mem in mems:
        mem_base = int(mem['base'], 0)
        s += MakeLine('')
        s += MakeLine('')
        s += MakeLine('/* {} */'.format(mem['mem']))
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            max_size = int(region['max_size'], 0)
            start_addr = mem_base + offset

            if 'tags' in region:
                for tag in region['tags']:
                    start_addr_name = '{}_START_ADDR'.format(tag)
                    size_name = '{}_SIZE'.format(tag)
                    offset_name = '{}_OFFSET'.format(tag)
                    s += MakeLine('#undef  {}'.format(start_addr_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, start_addr))
                    s += MakeLine('#undef  {}'.format(size_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                    s += MakeLine('#undef  {}'.format(offset_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, offset))

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
                    cbus_addr, cbus_offset = Convert2CBUSAddr(start_addr, offset, core)
                    s += MakeLine('#undef  {}'.format(start_addr_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, cbus_addr))
                    s += MakeLine('#undef  {}'.format(size_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                    s += MakeLine('#undef  {}'.format(offset_name))
                    s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, cbus_offset))

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
                        s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, cbus_addr))
                        s += MakeLine('#undef  {}'.format(size_name))
                        s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                        s += MakeLine('#undef  {}'.format(offset_name))
                        s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, cbus_offset))
    
    return s               


def GenPartitionTableHeaderContentV1(env, mems):
    s = ''
    for mem in mems:
        mem_base = int(mem['base'], 0)
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
                s += MakeLine('#undef  {}'.format(start_addr_name))
                s += MakeLine('#define {:<50} (0x{:08X})'.format(start_addr_name, start_addr))
                s += MakeLine('#undef  {}'.format(size_name))
                s += MakeLine('#define {:<50} (0x{:08X})'.format(size_name, max_size))
                s += MakeLine('#undef  {}'.format(offset_name))
                s += MakeLine('#define {:<50} (0x{:08X})'.format(offset_name, offset))
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


def GenPartitionTableHeaderFile(src, output_dir, output_name):
    import building
    import rtconfig

    logging.debug('output ptab.h:{}'.format(output_dir))
    env = building.GetCurrentEnv()

    f = open(src)
    try:
        mems = json.load(f)
    except ValueError as e:
        print("ptab.json syntax error, might be caused by trailing comma of last item")
        print("Error message: {}".format(e))
        print("Please check file {}".format(src))
        exit(1)
    finally:
        f.close()
    InitIndentation()
    s = MakeLine('#ifndef __{}__H__'.format(output_name.upper()))
    s += MakeLine('#define __{}__H__'.format(output_name.upper()))
    s += MakeLine('')
    s += MakeLine('')

    header = mems[0] if len(mems) > 0 else {}
    if 'version' in header:
        if "2" == header['version']:
            s += GenPartitionTableHeaderContentV2(env, mems[1:])
        else:
            s += GenPartitionTableHeaderContentV1(env, mems[1:])
    else:
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
                        max_size == ftab_item['max_size']), "{} max_size must be same, old: 0x{:08X}, new: 0x{:08X}".format(item_name, ftab['max_size'], max_size)
                ftab_item['max_size'] = max_size
                if 'app_exec' in region_type_list and ((not ext) or ('1' == ext)):
                    # only first binary need to be described in ftab
                    assert 'xip' not in ftab_item, 'xip address already configured in {}'.format(item_name)
                    ftab_item['xip'] = start_addr
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
            ftab_item['xip'] = start_addr
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
                    ftab_item['xip'] = start_addr
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
    f = open(src)
    try:
        mems = json.load(f)
    finally:
        f.close()

    # Get binary image size
    img_size = {}
    for img in imgs_info:
        img_name = img['name']
        img_bin = str(img['binary'][0])
        img_size[img_name] = CalcBinarySize(img_bin)

    # construct ftab dictionary
    ftab = {}
    flash_table_start = None

    header = mems[0]
    if 'version' in header:
        if "2" == header['version']:
            flash_table_start = ConstructFtabDictV2(ftab, mems[1:], img_size)
        else:
            flash_table_start = ConstructFtabDictV1(ftab, mems[1:], img_size)
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


def BuildJLinkLoadScript(main_env):
    import building

    # proj name as key, if proj has multiple binary, the value is a dictionary too. And binary name is used as key
    # example1:  {"main": 0x18000000}
    # example1:  {"main": {"ROM1.bin": 0x18000000, "ROM2.bin": 0x18200000}}
    img_download_info = {}
    if 'PARTITION_TABLE' in main_env:
        f = open(main_env['PARTITION_TABLE'])
        try:
            mems = json.load(f)
        finally:
            f.close()

        header = mems[0]
        if 'version' in header:
            if "2" == header['version']:
                ConstructImgDownloadInfoV2(img_download_info, mems[1:])
            else:
                ConstructImgDownloadInfoV1(img_download_info, mems[1:])
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
                assert os.path.isfile(bin_file), "{} should be a file as map defines".format(env['name'])
                s += MakeLine('loadbin {} 0x{:08X}'.format(os.path.relpath(bin_file, work_dir), info))
                download_file.append({
                        'name': os.path.relpath(bin_file, work_dir),
                        'addr': info
                    })
                s_file += MakeLine('FILE{}={}'.format(s_num,os.path.relpath(bin_file, work_dir)))
                s_file += MakeLine('ADDR{}=0x{:08X}'.format(s_num,info))
                s_num += 1
        else:
            hex_file = str(env['program_hex'][0])
            # load address is not defined, load hex
            if os.path.isdir(hex_file):
                dir_list = os.listdir(hex_file)
                for d in dir_list:
                    if building.IsEmbeddedProjEnv(env) and 'ER_IROM1' in d:
                        # ER_IROM1 is embedded in parent, others need to be downloaded
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
                    if building.IsEmbeddedProjEnv(env) and 'ER_IROM1' in d:
                        # ER_IROM1 is embedded in parent, others need to be downloaded
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

    decice_memory = main_env['JLINK_DEVICE'].split('_')
    device = decice_memory[0][:-1]
    memory = decice_memory[1] if len(decice_memory) > 1 else 'NOR'

    download_list = ' '.join(
        f"\"{file['name']}\"" if file['name'].lower().endswith(('.elf', '.hex')) 
        else f"\"{file['name']}@0x{file['addr']:08X}\"" 
        for file in download_file
    )

    generate_uart_download_bat(main_env, device, download_list, ImgDownUart_PATH)
    generate_uart_download_sh(main_env, device, download_list, ImgDownUart_PATH)
    generate_sftool_param(main_env, device, memory, download_file)


def generate_uart_download_bat(main_env, device, download_list, ImgDownUart_PATH):
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
        uart_comment += MakeLine(f"sftool -p COM%input% -c {device} write_flash {download_list}\n")
    uart_comment += MakeLine('if "%ENV_ROOT%"=="" pause\n')
    
    uart_f = open(os.path.join(main_env['build_dir'], 'uart_download.bat'), 'w')
    uart_f.write(uart_comment)
    uart_f.close()

def generate_uart_download_sh(main_env, device, download_list, ImgDownUart_PATH):
    uart_comment = '''#!/bin/bash

WORK_PATH=$(dirname "$0")
CURR_PATH=$(pwd)
cd "$WORK_PATH"

echo ""
echo "      Uart Download"
echo ""
read -p "please input the serial port (e.g. /dev/ttyUSB0): " input
echo "$input"

'''
    
    if os.getenv("LEGACY_ENV"):
        uart_comment += MakeLine('echo "Legacy mode is not supported on Linux/macOS"')
    else:
        uart_comment += MakeLine(f'sftool -p "$input" -c {device} write_flash {download_list}\n')
    
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