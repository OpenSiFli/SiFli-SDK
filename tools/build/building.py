# SPDX-FileCopyrightText: 2006-2015 RT-Thread Development Team
# SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: GPL-2.0-or-later

import importlib
import hashlib
import json
import os
import re
import shutil
import string
import subprocess
import sys
import logging
from pathlib import Path
import atexit
import tempfile
from pathlib import Path
import threading

import utils
from mkdist import do_copy_file
from SCons.Script import *
from utils import _make_path_relative

BuildOptions = {}
Projects = []
Rtt_Root = ''
Env = None
ChildProjList = []
EnvList = []
ParentProjStack = [{'name': 'main'}]
CustomImgList = []
BOARD_SEARCH_PATH = os.path.abspath(os.environ.get('SIFLI_SDK_BOARD_SEARCH_PATH', '')) if 'SIFLI_SDK_BOARD_SEARCH_PATH' in os.environ else None
_sdk_size_registered = False
_main_build_dir = None
_json_export_registered = False

def is_verbose():
    if (logging.root.level<=logging.DEBUG):
        return True
    else:
        return False

# maximum extended image number
MAX_EX_IMG_NUM = 5
PTAB_V3_PROGRAM_BINARY_STAMP = '.program_binary.stamp'
PTAB_V3_PROGRAM_HEX_STAMP = '.program_hex.stamp'

# SCons PreProcessor patch
def start_handling_includes(self, t=None):
    """
    Causes the PreProcessor object to start processing #import,
    #include and #include_next lines.

    This method will be called when a #if, #ifdef, #ifndef or #elif
    evaluates True, or when we reach the #else in a #if, #ifdef,
    #ifndef or #elif block where a condition already evaluated
    False.

    """
    d = self.dispatch_table
    p = self.stack[-1] if self.stack else self.default_table

    for k in ('import', 'include', 'include_next', 'define'):
        d[k] = p[k]

def stop_handling_includes(self, t=None):
    """
    Causes the PreProcessor object to stop processing #import,
    #include and #include_next lines.

    This method will be called when a #if, #ifdef, #ifndef or #elif
    evaluates False, or when we reach the #else in a #if, #ifdef,
    #ifndef or #elif block where a condition already evaluated True.
    """
    d = self.dispatch_table
    d['import'] = self.do_nothing
    d['include'] =  self.do_nothing
    d['include_next'] =  self.do_nothing
    d['define'] =  self.do_nothing

PatchedPreProcessor = SCons.cpp.PreProcessor
PatchedPreProcessor.start_handling_includes = start_handling_includes
PatchedPreProcessor.stop_handling_includes = stop_handling_includes

class Win32Spawn:
    def spawn(self, sh, escape, cmd, args, env):
        # deal with the cmd build-in commands which cannot be used in
        # subprocess.Popen
        if cmd == 'del':
            for f in args[1:]:
                try:
                    os.remove(f)
                except Exception as e:
                    logging.error ('Error removing file: ' + e)
                    return -1
            return 0

        import subprocess

        newargs = ' '.join(args[1:])
        cmdline = cmd + " " + newargs

        # Make sure the env is constructed by strings
        _e = dict([(k, str(v)) for k, v in env.items()])

        # Windows(tm) CreateProcess does not use the env passed to it to find
        # the executables. So we have to modify our own PATH to make Popen
        # work.
        old_path = os.environ['PATH']
        os.environ['PATH'] = _e['PATH']

        try:
            proc = subprocess.Popen(cmdline, env=_e, shell=False)
        except Exception as e:
            logging.error ('Error in calling command:' + cmdline.split(' ')[0])
            logging.error ('Exception: ' + os.strerror(e.errno))
            if (os.strerror(e.errno) == "No such file or directory"):
                logging.error ("\nPlease check Toolchains PATH setting.\n")

            return e.errno
        finally:
            os.environ['PATH'] = old_path

        return proc.wait()

# generate cconfig.h file
def GenCconfigFile(env, BuildOptions):
    import rtconfig

    if GetBoardName():
        path = rtconfig.OUTPUT_DIR
    else:
        path = env['BSP_ROOT']

    fullpath = os.path.join(path, 'cconfig.h')
    if rtconfig.PLATFORM == 'gcc':
        contents = ''
        if not os.path.isfile(fullpath):
            import gcc
            gcc.GenerateGCCConfig(rtconfig, path)

        # try again
        if os.path.isfile(fullpath):
            f = open(fullpath, 'r')
            if f:
                contents = f.read()
                f.close()

                prep = PatchedPreProcessor()
                prep.process_contents(contents)
                options = prep.cpp_namespace

                BuildOptions.update(options)

                # add HAVE_CCONFIG_H definition
                env.AppendUnique(CPPDEFINES = ['HAVE_CCONFIG_H'])

def ImgFileBuilder(target, source, env):
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    EZIP_PATH = os.path.join(SIFLI_SDK, f"tools/png2ezip/ezip{env['tool_suffix']}")
    filename = os.path.basename("{}".format(target[0]))
    tgt_directory = os.path.dirname("{}".format(target[0]))
    logging.info('ImgFileBuilder= '+env['FLAGS'])
    # if ".gif" in str(source[0]):
    if 0: # Merge cases (.gif and .png)
        subprocess.run(EZIP_PATH+' -gif '+str(source[0])+ ' ' + env['FLAGS'], shell=True, check=True)
        logging.info("gif")
        target_filename = os.path.basename("{}".format(target[0]))
        source_path = os.path.dirname("{}".format(source[0]))
        logging.info(target_filename)
        logging.info(source_path)
        shutil.move(os.path.join(source_path, target_filename), '{}'.format(target[0]))
    else:
        subprocess.run(EZIP_PATH + ' -convert ' + str(source[0]) + ' ' + env['FLAGS'] + ' -outdir {}'.format(tgt_directory), shell=True, check=True)
        # 从.c转为.c的情况下会让scons陷入到依赖循环导致出错。
        # 因此将转换后的文件后缀修改为`.tmp.c`。
        # 现在将转换后的真实的.c文件转为.tmp.c后缀以让scons后续编译能正常进行
        shutil.move('{}'.format(target[0]).replace(".tmp.c", ".c"), '{}'.format(target[0])) 

            

def BinFileBuilder(target, source, env):
    """A builder specifically designed for generating binary.ezip files"""
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    EZIP_PATH = os.path.join(SIFLI_SDK, f"tools/png2ezip/ezip{env['tool_suffix']}")
    filename = os.path.basename("{}".format(target[0]))
    tgt_directory = os.path.dirname("{}".format(target[0]))

    logging.info('BinFileBuilder= '+env['FLAGS'])
    cmd = EZIP_PATH + ' -convert ' + str(source[0]) + ' ' + env['FLAGS'] + ' -outdir {}'.format(tgt_directory)

    subprocess.run(cmd, shell=True, check=True)

def FontConvertBuild(target, source, env):
    """
    Build action to convert font files to C array using the internal converter
    """
    try:
        source_file = str(source[0])
        target_file = str(target[0])
        
        # Get font name from environment or use default
        font_name = env['FLAGS']

        # Check if source font file exists
        if not os.path.exists(source_file):
            logging.error(f"Source font file not found: {source_file}")
            return -1
            
        # Convert font to C array directly in this function
        with open(source_file, 'rb') as f:
            font_data = f.read()
        
        font_size = len(font_data)
        
        # Generate C content
        c_content = f"__attribute__((section(\".font_data\"))) const unsigned char {font_name}[{font_size}] = {{\n"
        
        # Convert font data to C array format, 12 bytes per line
        bytes_per_line = 12
        for i in range(0, font_size, bytes_per_line):
            line_data = font_data[i:i + bytes_per_line]
            hex_values = ', '.join(f'0x{b:02X}' for b in line_data)
            
            if i + bytes_per_line < font_size:
                c_content += f"    {hex_values},\n"
            else:
                c_content += f"    {hex_values}\n"
        
        c_content += "};\n\n\n"
        c_content += f"const int {font_name}_size = sizeof({font_name});\n"


        # Write C file directly to target location
        with open(target_file, 'w', encoding='utf-8') as f:
            f.write(c_content)
    

        logging.info(f"Font conversion completed!")
        logging.info(f"Input file: {source_file}")
        logging.info(f"Output file: {target_file}")
        logging.info(f"Array name: {font_name}")
        logging.info(f"File size: {font_size} bytes")
        
        return 0
        
    except Exception as e:
        logging.error(f"Error during font conversion: {e}")
        return -1


def ModifyFontConvertTargets(target, source, env):
    """
    Modify the target file name of the FontConvert builder
    """
    target = []
    
    # If no font name is specified, use the default name.
    font_name = env.get('FLAGS', 'font_data')

    target_path = font_name + '.c'

    target.append(target_path)
    return target, source

def FontFileBuild(target, source, env):
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    FONT2C_PATH = os.path.join(SIFLI_SDK, f"tools/font2c/font2c{env['tool_suffix']}")
    filename = os.path.basename("{}".format(target[0]))
    subprocess.run([FONT2C_PATH, str(source[0])], check=True)
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
    content = content.replace('#include "app_module.h"', '#include "mem_section.h"')
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(content)
    shutil.move(filename, '{}'.format(target[0]))

def ModifyFontTargets(target, source, env):
    target_path = str(target[0])
    target = []
    if "tiny" in target_path:
        target_path = target_path.replace("lvsf_font", "lvsf_tiny_font")
    target.append(target_path)
            
    return target, source

def ConvertFont(env, source, flags):
    """
    A helper method to convert font files in a more convenient way
    Usage: 
        objs1 = env.ConvertFont('path/to/font.ttf1', 'my_font_name1')
        objs2 = env.ConvertFont('path/to/font.ttf2', 'my_font_name2')
    """
    target = []
    if isinstance(source, str):
        source = [source]
        
    for s in source:
        t = env.FontConvert(s,FLAGS = flags)
        target.extend(t)
    
    return target

def ImgResource(env, source, flags):
    target = []
    if GetDepend('SOC_SF32LB57X'):
        if '-winsize' not in flags:
            flags += ' -winsize 2048 '
            
    for s in source:
        if '-binfile' in flags:
            # If the -binfile parameter is included, use the binary builder.
            t = env.BinFile(s, FLAGS=flags)
        else:
            # Otherwise, use the default C file builder
            t = env.ImgFile(s, FLAGS=flags)
        target.extend(t)
    
    return target

def LangBuild(target, source, env):
    import sdk_resource
    
    src_path = os.path.dirname(str(source[0]))
    dst_path = os.path.dirname(str(target[0]))
    sdk_resource.ns = True
    sdk_resource.default_language = env['DEFAULT_LANG']
    sdk_resource.GenerateStrRes(src_path, dst_path)
    
def ModifyLangTargets(target, source, env):
    target = []
    for s in source:
        t = os.path.basename(str(s))
        t = os.path.splitext(t)[0]
        t = t + '.c'
        target.append(t)
            
    target.append('lang_pack.c')
    target.append('lang_pack.h')

    # Do not remove this dependency.
    # Lang targets are generated by sdk_resource.py, so changes in the shared
    # generator must invalidate cached outputs such as lang_pack.c/.h.
    sdk_resource_py = os.path.join(os.path.dirname(__file__), 'sdk_resource.py')
    Depends(target, sdk_resource_py)
    
    return target, source

def GetLangBuildObject(objs):
    objs_bak = objs[:]
    SrcRemove(objs_bak, 'lang_pack.h')
    return objs_bak


# /*--------- Builtin/External Language-Pack Extension Begin ---------*/
def LoadLangExcelPackModule():
    script_path = os.path.join(os.path.dirname(__file__), 'lang_excel_pack.py')
    spec = importlib.util.spec_from_file_location('sifli_lang_excel_pack', script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError('failed to load lang excel pack module: {}'.format(script_path))

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def NormalizeLangExcelSelection(selected):
    module = LoadLangExcelPackModule()
    return module.normalize_selected_langs(selected)


def NormalizeLangExcelOptionValue(value):
    if value is None:
        return None

    text = str(value).strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in ("'", '"'):
        text = text[1:-1].strip()

    return text or None


def ResolveLangExcelTargets(xlsx_path, sheet_name, selected_langs, label):
    module = LoadLangExcelPackModule()
    _, languages, _, _, _ = module.load_xlsx_table(Path(xlsx_path), sheet_name)
    lang_stems = [stem for _, stem, _ in languages]
    return module.validate_selected_langs(
        module.normalize_selected_langs(selected_langs),
        lang_stems,
        label,
    )


def LangExcelBuild(target, source, env):
    if len(source) != 1:
        raise RuntimeError(
            'LangExcel builder expects exactly one workbook per builtin source root; '
            'one workbook already carries multiple language columns and one shared key schema, got {}'.format(len(source))
        )

    xlsx_path = os.path.abspath(str(source[0]))
    output_dir = os.path.dirname(str(target[0]))
    script_path = os.path.join(os.path.dirname(__file__), 'lang_excel_pack.py')

    os.makedirs(output_dir, exist_ok=True)

    cmd = [
        sys.executable,
        script_path,
        '--xlsx',
        xlsx_path,
        '--output-dir',
        output_dir,
        '--no-bin',
    ]

    default_lang = NormalizeLangExcelOptionValue(env.get('DEFAULT_LANG'))
    if default_lang:
        cmd.extend(['--default-lang', str(default_lang)])

    sheet_name = NormalizeLangExcelOptionValue(env.get('LANG_EXCEL_SHEET'))
    if sheet_name:
        cmd.extend(['--sheet', str(sheet_name)])

    c_langs = NormalizeLangExcelSelection(env.get('LANG_EXCEL_C_LANGS'))
    if c_langs:
        cmd.extend(['--c-langs', ','.join(c_langs)])

    cmd.append('--verbose')
    subprocess.check_call(cmd)


def ModifyLangExcelTargets(target, source, env):
    if len(source) != 1:
        raise RuntimeError(
            'LangExcel builder expects exactly one workbook per builtin source root; '
            'one workbook already carries multiple language columns and one shared key schema, got {}'.format(len(source))
        )

    xlsx_path = os.path.abspath(str(source[0]))
    sheet_name = NormalizeLangExcelOptionValue(env.get('LANG_EXCEL_SHEET'))
    c_langs = ResolveLangExcelTargets(
        xlsx_path,
        sheet_name,
        env.get('LANG_EXCEL_C_LANGS'),
        'LANG_EXCEL_C_LANGS',
    )

    target = []
    for lang in c_langs:
        target.append('{}.c'.format(lang))

    target.append('lang_pack.c')
    target.append('lang_pack.h')

    # Do not remove these dependencies.
    # LangExcel targets are generated by lang_excel_pack.py directly from the
    # workbook, so builder logic and source workbook changes must invalidate
    # cached outputs together.
    lang_excel_pack_py = os.path.join(os.path.dirname(__file__), 'lang_excel_pack.py')
    Depends(target, lang_excel_pack_py)
    Depends(target, xlsx_path)

    return target, source

def ExternalLangPackBuild(target, source, env):
    installer_dir = os.path.abspath(str(env['EXT_LANG_INSTALLER_DIR']))
    lang_header = os.path.abspath(str(env['EXT_LANG_HEADER']))
    script_path = os.path.join(os.path.dirname(__file__), 'lang_bin_pack.py')
    pack_version = NormalizeLangExcelOptionValue(env.get('EXT_LANG_PACK_VERSION'))

    os.makedirs(installer_dir, exist_ok=True)
    if not os.path.isfile(lang_header):
        raise RuntimeError("missing generated builtin language header: {}".format(lang_header))

    # Remove stale installer artifacts so deleted external packs and legacy
    # manifest files do not linger in the disk image after incremental builds.
    for entry in os.listdir(installer_dir):
        path = os.path.join(installer_dir, entry)
        if not os.path.isfile(path):
            continue
        if entry.endswith('.bin') or entry.endswith('.lst'):
            os.remove(path)

    with tempfile.TemporaryDirectory(prefix='sifli_external_lang_pack_') as staging_dir:
        for json_path in source:
            src = os.path.abspath(str(json_path))
            shutil.copyfile(src, os.path.join(staging_dir, os.path.basename(src)))

        cmd = [
            sys.executable,
            script_path,
            '--strings-dir',
            staging_dir,
            '--output-dir',
            installer_dir,
            '--header',
            lang_header,
            '--verbose',
        ]

        if pack_version:
            cmd.extend(['--version', str(pack_version)])

        subprocess.check_call(cmd)

    for out in target:
        if not os.path.isfile(str(out)):
            raise RuntimeError("missing generated external language-pack file: {}".format(out))


def ExternalLangPackExcelBuild(target, source, env):
    installer_dir = os.path.abspath(str(env['EXT_LANG_INSTALLER_DIR']))
    if len(source) != 1:
        raise RuntimeError(
            'ExternalLangPackExcel builder expects exactly one workbook per external source root; '
            'one workbook already carries multiple language columns and one shared key schema, got {}'.format(len(source))
        )

    xlsx_path = os.path.abspath(str(source[0]))
    script_path = os.path.join(os.path.dirname(__file__), 'lang_excel_pack.py')
    pack_version = NormalizeLangExcelOptionValue(env.get('EXT_LANG_PACK_VERSION'))

    os.makedirs(installer_dir, exist_ok=True)

    # Remove stale installer artifacts so deleted external packs do not linger
    # in the disk image after incremental workbook changes.
    for entry in os.listdir(installer_dir):
        path = os.path.join(installer_dir, entry)
        if not os.path.isfile(path):
            continue
        if entry.endswith('.bin') or entry.endswith('.lst'):
            os.remove(path)

    cmd = [
        sys.executable,
        script_path,
        '--xlsx',
        xlsx_path,
        '--output-dir',
        installer_dir,
        '--no-c',
    ]

    sheet_name = NormalizeLangExcelOptionValue(env.get('LANG_EXCEL_SHEET'))
    if sheet_name:
        cmd.extend(['--sheet', str(sheet_name)])

    bin_langs = NormalizeLangExcelSelection(env.get('LANG_EXCEL_BIN_LANGS'))
    if bin_langs:
        cmd.extend(['--bin-langs', ','.join(bin_langs)])

    if pack_version:
        cmd.extend(['--version', str(pack_version)])

    cmd.append('--verbose')
    subprocess.check_call(cmd)

    for out in target:
        if not os.path.isfile(str(out)):
            raise RuntimeError("missing generated external language-pack file: {}".format(out))

def ModifyExternalLangPackTargets(target, source, env):
    installer_dir = env.get('EXT_LANG_INSTALLER_DIR')
    lang_header = env.get('EXT_LANG_HEADER')
    pack_version = NormalizeLangExcelOptionValue(env.get('EXT_LANG_PACK_VERSION'))
    if not installer_dir:
        raise RuntimeError('EXT_LANG_INSTALLER_DIR is required for ExternalLangPack builder')
    if not lang_header:
        raise RuntimeError('EXT_LANG_HEADER is required for ExternalLangPack builder')

    installer_dir = os.path.abspath(str(installer_dir))
    target = []

    for s in source:
        stem = os.path.splitext(os.path.basename(str(s)))[0]
        target.append(os.path.join(installer_dir, '{}.bin'.format(stem)))

    # Do not remove these dependencies.
    # External pack targets are generated by lang_bin_pack.py and validated
    # against the shared sdk_resource.py-generated builtin schema header.
    sdk_resource_py = os.path.join(os.path.dirname(__file__), 'sdk_resource.py')
    lang_bin_pack_py = os.path.join(os.path.dirname(__file__), 'lang_bin_pack.py')
    Depends(target, sdk_resource_py)
    Depends(target, lang_bin_pack_py)
    Depends(target, os.path.abspath(str(lang_header)))
    Depends(target, Value(pack_version or ''))

    return target, source


def ModifyExternalLangPackExcelTargets(target, source, env):
    installer_dir = env.get('EXT_LANG_INSTALLER_DIR')
    pack_version = NormalizeLangExcelOptionValue(env.get('EXT_LANG_PACK_VERSION'))
    if not installer_dir:
        raise RuntimeError('EXT_LANG_INSTALLER_DIR is required for ExternalLangPackExcel builder')
    if len(source) != 1:
        raise RuntimeError(
            'ExternalLangPackExcel builder expects exactly one workbook per external source root; '
            'one workbook already carries multiple language columns and one shared key schema, got {}'.format(len(source))
        )

    xlsx_path = os.path.abspath(str(source[0]))
    installer_dir = os.path.abspath(str(installer_dir))
    sheet_name = NormalizeLangExcelOptionValue(env.get('LANG_EXCEL_SHEET'))
    bin_langs = ResolveLangExcelTargets(
        xlsx_path,
        sheet_name,
        env.get('LANG_EXCEL_BIN_LANGS'),
        'LANG_EXCEL_BIN_LANGS',
    )

    target = []
    for lang in bin_langs:
        target.append(os.path.join(installer_dir, '{}.bin'.format(lang)))

    # Do not remove these dependencies.
    # ExternalLangPackExcel targets are emitted directly from the workbook, so
    # both the generator and the workbook must be tracked for incremental builds.
    lang_excel_pack_py = os.path.join(os.path.dirname(__file__), 'lang_excel_pack.py')
    Depends(target, lang_excel_pack_py)
    Depends(target, xlsx_path)
    Depends(target, Value(pack_version or ''))

    return target, source
# /*---------- Builtin/External Language-Pack Extension End ----------*/

def rom_sym_filter(srcfile, dstfile, filter):
    import rtconfig
    fp = open(srcfile, 'r')
    fp2 = open(dstfile, 'w')
    for line in fp:
        if (re.match(r"^#",line)):
            fp2.writelines(line)
            continue
        line = line.rstrip('\n')
        if rtconfig.PLATFORM == 'armcc':
            info = re.split(' ', line)
            renamed = 0
            for rule in filter:
                if (re.match(rule,info[2])):
                    info[2] = info[2]+"_rom"
                    fp2.writelines(info[0]+" "+info[1]+" "+info[2]+'\n')
                    renamed = 1
                    break
        elif rtconfig.PLATFORM == 'gcc':
            info = re.split('=', line)
            renamed = 0
            for rule in filter:
                if (re.match(rule,info[0])):
                    info[0] = info[0].rstrip() + "_rom"
                    fp2.writelines(info[0] + " =" + info[1]+'\n')
                    renamed = 1
                    break
        else:
            assert False, "Unknown PLATFORM: {}".format(rtconfig.PLATFORM)
            
        if (renamed == 0):
            fp2.writelines(line+'\n')
    fp.close()
    fp2.close()

def RomLibBuild(target, source, env):
    target_name = str(target[0])
    target_path = os.path.dirname(target_name)
    if ('ROM_SYM_FILTER' in env) and (env['ROM_SYM_FILTER'] is not None):
        rom_sym_filter(str(source[0]), target_name, env['ROM_SYM_FILTER'])
    else:
        shutil.copy(str(source[0]), target_name)    

def SetRomSymFilter(filter):
    Env['ROM_SYM_FILTER'] = filter

def _remove_file_or_dir(path: str) -> None:
    if os.path.isdir(path):
        shutil.rmtree(path)
    elif os.path.exists(path):
        os.remove(path)

def _ihex_has_data(path: str) -> bool:
    """Return True if an Intel HEX file contains any data record."""
    try:
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = (line or '').strip()
                if not line.startswith(':') or len(line) < 11:
                    continue
                try:
                    length = int(line[1:3], 16)
                    rectype = int(line[7:9], 16)
                except Exception:
                    continue
                if rectype == 0 and length > 0:
                    return True
        return False
    except Exception:
        return False


def _infer_build_core(env, fallback: str = 'HCPU') -> str:
    """Infer build core type (HCPU/LCPU/ACPU) from a SCons env."""
    try:
        name = str(env.get('name') or '').strip().lower()
    except Exception:
        name = ''
    if name == 'lcpu':
        return 'LCPU'
    if name == 'acpu':
        return 'ACPU'

    try:
        link_src = str(env.get('LINK_SCRIPT_SRC') or '').replace('\\', '/').lower()
    except Exception:
        link_src = ''
    if '/lcpu/' in link_src or link_src.endswith('_lcpu'):
        return 'LCPU'
    if '/acpu/' in link_src or link_src.endswith('_acpu'):
        return 'ACPU'
    if '/hcpu/' in link_src or link_src.endswith('_hcpu'):
        return 'HCPU'

    try:
        full_name = str(env.get('full_name') or '').strip().lower()
    except Exception:
        full_name = ''
    if full_name.endswith('.lcpu'):
        return 'LCPU'
    if full_name.endswith('.acpu'):
        return 'ACPU'

    try:
        fb = str(fallback or 'HCPU').strip().upper()
        return fb or 'HCPU'
    except Exception:
        return 'HCPU'


def _get_ptab_v3_split_section(partition: dict, ptab_module) -> str:
    """Map a ptab v3 app/ex partition to the ELF section used for objcopy."""
    name = (partition.get('name') or '').strip()
    return '.{}'.format(name) if name else ''


def _collect_ptab_v3_split_partitions(ptab_obj, core: str, ptab_module):
    """Collect app/ex resource partitions and their objcopy split section for a core."""
    out = []

    for partition in ptab_module.iter_int_res_partitions_v3(ptab_obj, core=core):
        name = (partition.get('name') or '').strip()
        if not name:
            continue

        section = _get_ptab_v3_split_section(partition, ptab_module)
        if not section:
            continue

        out.append({
            'name': name,
            'section': section,
        })

    return out


def _uses_ptab_v3_standard_artifacts(rtconfig, ptab_obj) -> bool:
    return bool(
        ptab_obj
        and ptab_obj.is_v3()
        and getattr(rtconfig, 'PLATFORM', None) in ('gcc', 'armcc')
    )


def IsPtabV3ArtifactStampPath(path: str) -> bool:
    """Return True when path is a synthetic ptab v3 artifact stamp."""
    name = os.path.basename(str(path))
    return name in (PTAB_V3_PROGRAM_BINARY_STAMP, PTAB_V3_PROGRAM_HEX_STAMP)


def GetPtabV3ArtifactBaseName(env, program_file: str = '') -> str:
    """Return the base artifact name for a ptab v3 project."""
    try:
        proj_name = str(env.get('name') or '').strip()
    except Exception:
        proj_name = ''
    if proj_name:
        return proj_name
    if program_file:
        return os.path.splitext(os.path.basename(str(program_file)))[0] or 'main'
    return 'main'


def GetPtabV3ArtifactOutputDir(program_file: str) -> str:
    """Return the output directory for final ptab v3 bin/hex artifacts."""
    return os.path.join(os.path.dirname(str(program_file)), 'output')


def GetPtabV3ArtifactPath(output_dir: str, base_name: str, ext: str, suffix: str = None) -> str:
    """Build a final ptab v3 artifact path."""
    stem = base_name if not suffix else '{}.{}'.format(base_name, suffix)
    return os.path.join(output_dir, stem + ext)


def GetPtabV3CodeArtifactCandidates(output_dir: str, base_name: str, ext: str):
    """Return candidate code artifact paths, preferring multi-output naming."""
    return [
        GetPtabV3ArtifactPath(output_dir, base_name, ext, 'app'),
        GetPtabV3ArtifactPath(output_dir, base_name, ext),
    ]


def _ptab_v3_artifact_has_payload(path: str, ext: str) -> bool:
    """Return True when a candidate artifact exists and contains data."""
    if not os.path.isfile(path):
        return False
    if os.path.getsize(path) <= 0:
        return False
    if ext.lower() == '.hex':
        return _ihex_has_data(path)
    return True


def GetLegacyMultiImageArtifactPaths(output_dir: str, ext: str):
    """Return legacy ER_IROMN artifacts from a flat output directory."""
    if not os.path.isdir(output_dir):
        return []

    matched = []
    pattern = re.compile(r'^(?:.+\.)?ER_IROM(\d+){}$'.format(re.escape(ext)), re.IGNORECASE)
    for name in os.listdir(output_dir):
        m = pattern.match(name)
        if not m:
            continue
        idx = int(m.group(1), 10)
        path = os.path.join(output_dir, name)
        if _ptab_v3_artifact_has_payload(path, ext):
            matched.append((idx, path))

    matched.sort(key=lambda item: item[0])
    return [path for _, path in matched]


def _read_ptab_v3_stamp_artifact(path: str, ext: str) -> str:
    """Return the artifact recorded in a ptab v3 stamp, if it is usable."""
    if not os.path.isfile(path):
        return ''

    try:
        with open(path, 'r', encoding='utf-8') as f:
            recorded = next((line.strip() for line in f if line.strip()), '')
    except (OSError, UnicodeError):
        return ''

    if not recorded or not recorded.lower().endswith(ext.lower()):
        return ''
    if _ptab_v3_artifact_has_payload(recorded, ext):
        return os.path.normpath(recorded)

    if not os.path.isabs(recorded):
        local_recorded = os.path.join(os.path.dirname(path), os.path.basename(recorded))
        if _ptab_v3_artifact_has_payload(local_recorded, ext):
            return os.path.normpath(local_recorded)
    return ''


def ResolvePtabV3CodeArtifact(output_dir: str, base_name: str, ext: str) -> str:
    """Resolve the primary code artifact from a ptab v3 or legacy multi-image output dir."""
    candidates = GetPtabV3CodeArtifactCandidates(output_dir, base_name, ext)
    for candidate in candidates:
        if _ptab_v3_artifact_has_payload(candidate, ext):
            return candidate

    legacy = GetLegacyMultiImageArtifactPaths(output_dir, ext)
    if legacy:
        prefixed = GetPtabV3ArtifactPath(output_dir, base_name, ext, 'ER_IROM1')
        legacy_name = os.path.join(output_dir, 'ER_IROM1{}'.format(ext))
        for preferred in (prefixed, legacy_name):
            if preferred in legacy:
                return preferred
        return legacy[0]

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[-1]


def ResolvePtabV3ArtifactContainer(output_dir: str, base_name: str, ext: str) -> str:
    """Resolve a file or directory that callers may need to iterate/load."""
    candidates = GetPtabV3CodeArtifactCandidates(output_dir, base_name, ext)
    for candidate in candidates:
        if _ptab_v3_artifact_has_payload(candidate, ext):
            return candidate
    if GetLegacyMultiImageArtifactPaths(output_dir, ext):
        return output_dir

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[-1]


def _resolve_ptab_v3_artifact_ref(path: str, base_name: str, ext: str, as_container: bool = False) -> str:
    path = str(path)
    if IsPtabV3ArtifactStampPath(path):
        output_dir = os.path.dirname(path)
        if as_container and GetLegacyMultiImageArtifactPaths(output_dir, ext):
            return output_dir
        stamped = _read_ptab_v3_stamp_artifact(path, ext)
        if stamped:
            return stamped
        path = output_dir

    if os.path.isdir(path):
        if as_container:
            return ResolvePtabV3ArtifactContainer(path, base_name, ext)
        return ResolvePtabV3CodeArtifact(path, base_name, ext)
    return path


def ResolvePtabV3CodeArtifactFromRef(path: str, base_name: str, ext: str) -> str:
    """Resolve a primary code artifact file from a stamp or output directory reference."""
    return _resolve_ptab_v3_artifact_ref(path, base_name, ext, as_container=False)


def ResolvePtabV3ArtifactContainerFromRef(path: str, base_name: str, ext: str) -> str:
    """Resolve a file or iterable output directory from a stamp/output reference."""
    return _resolve_ptab_v3_artifact_ref(path, base_name, ext, as_container=True)


def _cleanup_ptab_v3_output_dir(out_dir: str, ext: str, stamp_name: str) -> None:
    """Clean stale ptab v3 output artifacts for one file type."""
    if os.path.exists(out_dir) and not os.path.isdir(out_dir):
        _remove_file_or_dir(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    for name in os.listdir(out_dir):
        if name == stamp_name or name == 'int_res' or name.lower().endswith(ext.lower()):
            _remove_file_or_dir(os.path.join(out_dir, name))


def _cleanup_ptab_v3_legacy_artifacts(program_file: str, ext: str) -> None:
    """Remove stale legacy ptab v3 artifacts from old locations."""
    target_dir = os.path.dirname(str(program_file))
    legacy_whole = os.path.join(
        target_dir,
        os.path.splitext(os.path.basename(str(program_file)))[0] + ext,
    )
    if os.path.exists(legacy_whole):
        _remove_file_or_dir(legacy_whole)

    int_res_dir = os.path.join(target_dir, 'int_res')
    if os.path.isdir(int_res_dir):
        for name in os.listdir(int_res_dir):
            if name.lower().endswith(ext.lower()):
                _remove_file_or_dir(os.path.join(int_res_dir, name))


def _validate_ptab_v3_artifact_names(base_name: str, split_entries, out_dir: str) -> None:
    """Validate split artifact names against code suffix and case-insensitive collisions."""
    seen = {'app': 'code image'}
    for entry in split_entries:
        name = str(entry.get('name') or '').strip()
        if not name:
            continue
        key = name.lower()
        if key in seen:
            logging.error(
                "ptab v3 artifact suffix '%s' conflicts with %s in '%s'",
                name,
                seen[key],
                out_dir,
            )
            raise SystemExit(1)
        seen[key] = "partition '{}'".format(name)


def _ptab_v3_fromelf_key(path: str, ext: str) -> str:
    name = os.path.basename(str(path)).strip()
    if name.lower().endswith(ext.lower()):
        name = name[:-len(ext)]
    return name.lower()


def _ptab_v3_fromelf_region_aliases(region_name: str):
    upper = str(region_name or '').strip().upper()
    if not upper:
        return set()
    base = upper[3:] if upper.startswith(('ER_', 'LR_')) else upper
    aliases = {upper.lower(), base.lower(), 'er_{}'.format(base).lower(), 'lr_{}'.format(base).lower()}
    if base in ('IROM1', 'ROM1'):
        aliases.update(('irom1', 'rom1', 'er_irom1', 'er_rom1', 'lr_irom1', 'lr_rom1'))
    elif base in ('IROM2', 'ROM2'):
        aliases.update(('irom2', 'rom2', 'er_irom2', 'er_rom2', 'lr_irom2', 'lr_rom2'))
    return aliases


def _ptab_v3_fromelf_region_name(entry, ext: str) -> str:
    name = os.path.basename(str(entry.get('path') or '')).strip()
    if name.lower().endswith(ext.lower()):
        name = name[:-len(ext)]
    return name.strip()


def _ptab_v3_lcpu_fromelf_region_suffix(entry, ext: str) -> str:
    key = str(entry.get('key') or '').strip()
    if key in _ptab_v3_fromelf_region_aliases('ER_IROM1'):
        return 'ER_IROM1'
    if key in _ptab_v3_fromelf_region_aliases('ER_IROM2'):
        return 'ER_IROM2'
    return ''


def _ptab_v3_fromelf_entries(export_path: str, ext: str):
    if os.path.isfile(export_path):
        return [{'path': export_path, 'key': _ptab_v3_fromelf_key(export_path, ext)}]
    if not os.path.isdir(export_path):
        return []

    entries = []
    for name in sorted(os.listdir(export_path)):
        path = os.path.join(export_path, name)
        if os.path.isfile(path):
            entries.append({'path': path, 'key': _ptab_v3_fromelf_key(path, ext)})
    return entries


def _find_ptab_v3_fromelf_entry(entries, aliases, used_paths, ext: str):
    for entry in entries:
        path = entry['path']
        if path in used_paths:
            continue
        if entry['key'] in aliases and _ptab_v3_artifact_has_payload(path, ext):
            return entry
    return None


def _select_ptab_v3_fromelf_code_entry(entries, used_paths, ext: str):
    candidates = [
        entry for entry in entries
        if entry['path'] not in used_paths and _ptab_v3_artifact_has_payload(entry['path'], ext)
    ]
    if len(candidates) == 1:
        return candidates[0]
    return _find_ptab_v3_fromelf_entry(candidates, _ptab_v3_fromelf_region_aliases('IROM1'), set(), ext)


def _build_ptab_v3_keil_artifacts(program_file: str, out_dir: str, base_name: str,
                                  ext: str, split_entries, stamp_path: str) -> str:
    """Export Keil ptab v3 artifacts and normalize fromelf region names."""
    export_path = os.path.join(out_dir, '.fromelf_{}{}'.format(base_name, ext))
    _remove_file_or_dir(export_path)
    fromelf_mode = '--bin' if ext.lower() == '.bin' else '--i32'
    subprocess.run(['fromelf', fromelf_mode, program_file, '--output', export_path], check=True)
    entries = _ptab_v3_fromelf_entries(export_path, ext)
    used_paths = set()
    used_resources = []

    for entry in split_entries:
        name = entry['name']
        aliases = _ptab_v3_fromelf_region_aliases(name)
        src = _find_ptab_v3_fromelf_entry(entries, aliases, used_paths, ext)
        if not src:
            continue
        out_file = GetPtabV3ArtifactPath(out_dir, base_name, ext, name)
        used_paths.add(src['path'])
        _remove_file_or_dir(out_file)
        shutil.move(src['path'], out_file)
        used_resources.append(name)

    code_entry = _select_ptab_v3_fromelf_code_entry(entries, used_paths, ext)
    if not code_entry:
        _remove_file_or_dir(export_path)
        return ''

    code_path = GetPtabV3ArtifactPath(
        out_dir,
        base_name,
        ext,
        'app' if used_resources else None,
    )
    _remove_file_or_dir(code_path)
    shutil.move(code_entry['path'], code_path)
    _write_ptab_v3_artifact_stamp(stamp_path, code_path)
    _remove_file_or_dir(export_path)
    return code_path


def _build_ptab_v3_keil_embedded_lcpu_artifacts(program_file: str, out_dir: str, base_name: str,
                                                ext: str, stamp_path: str) -> str:
    """Export embedded LCPU Keil ptab v3 artifacts while preserving fromelf regions."""
    export_path = os.path.join(out_dir, '.fromelf_{}{}'.format(base_name, ext))
    _remove_file_or_dir(export_path)
    fromelf_mode = '--bin' if ext.lower() == '.bin' else '--i32'
    subprocess.run(['fromelf', fromelf_mode, program_file, '--output', export_path], check=True)

    entries = [
        entry for entry in _ptab_v3_fromelf_entries(export_path, ext)
        if _ptab_v3_artifact_has_payload(entry['path'], ext)
    ]
    if not entries:
        _remove_file_or_dir(export_path)
        return ''

    code_entry = _find_ptab_v3_fromelf_entry(
        entries,
        _ptab_v3_fromelf_region_aliases('ER_IROM1'),
        set(),
        ext,
    )
    if not code_entry:
        _remove_file_or_dir(export_path)
        return ''
    flash_entry = _find_ptab_v3_fromelf_entry(
        entries,
        _ptab_v3_fromelf_region_aliases('ER_IROM2'),
        {code_entry['path']},
        ext,
    )
    if not flash_entry:
        _remove_file_or_dir(export_path)
        return ''

    code_path = ''
    flash_path = ''
    unknown_regions = []
    for entry in entries:
        region_name = _ptab_v3_lcpu_fromelf_region_suffix(entry, ext)
        if not region_name:
            unknown_regions.append(_ptab_v3_fromelf_region_name(entry, ext) or entry['path'])
            continue
        out_file = GetPtabV3ArtifactPath(out_dir, base_name, ext, region_name)
        _remove_file_or_dir(out_file)
        shutil.move(entry['path'], out_file)
        if entry['path'] == code_entry['path']:
            code_path = out_file
        elif entry['path'] == flash_entry['path']:
            flash_path = out_file

    if unknown_regions:
        logging.warning(
            "Ignore unexpected embedded LCPU fromelf region(s): %s",
            ', '.join(unknown_regions),
        )

    _remove_file_or_dir(export_path)
    if code_path and flash_path:
        _write_ptab_v3_artifact_stamp(stamp_path, code_path)
        return code_path
    return ''


def ModifyProgramBinaryTargets(target, source, env):
    import ptab as ptab_module
    import rtconfig

    ptab_obj = env.GetPtab() if hasattr(env, "GetPtab") else None
    if ptab_obj is None and 'PARTITION_TABLE' in env:
        ptab_obj = ptab_module.load_ptab(env['PARTITION_TABLE'], fatal=False)

    if _uses_ptab_v3_standard_artifacts(rtconfig, ptab_obj):
        elf_path = str(source[0]) if source else ''
        out_dir = GetPtabV3ArtifactOutputDir(elf_path)
        target = [os.path.join(out_dir, PTAB_V3_PROGRAM_BINARY_STAMP)]
    return target, source


def ModifyProgramHexTargets(target, source, env):
    import ptab as ptab_module
    import rtconfig

    ptab_obj = env.GetPtab() if hasattr(env, "GetPtab") else None
    if ptab_obj is None and 'PARTITION_TABLE' in env:
        ptab_obj = ptab_module.load_ptab(env['PARTITION_TABLE'], fatal=False)

    if _uses_ptab_v3_standard_artifacts(rtconfig, ptab_obj):
        elf_path = str(source[0]) if source else ''
        out_dir = GetPtabV3ArtifactOutputDir(elf_path)
        target = [os.path.join(out_dir, PTAB_V3_PROGRAM_HEX_STAMP)]
    return target, source


def _write_ptab_v3_artifact_stamp(stamp_path: str, artifact_path: str) -> None:
    lines = [artifact_path]
    if artifact_path and os.path.isfile(artifact_path):
        h = hashlib.sha256()
        with open(artifact_path, 'rb') as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b''):
                h.update(chunk)
        lines.append('size={}'.format(os.path.getsize(artifact_path)))
        lines.append('sha256={}'.format(h.hexdigest()))
    content = '\n'.join(lines) + '\n'

    if os.path.exists(stamp_path):
        with open(stamp_path, 'r', encoding='utf-8') as f:
            if f.read() == content:
                return

    stamp_dir = os.path.dirname(stamp_path)
    if stamp_dir:
        os.makedirs(stamp_dir, exist_ok=True)
    with open(stamp_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(content)


class LcpuImgStrategy:
    def build(self, program_file: str, out_dir: str, base_name: str, ext: str, rtconfig) -> str:
        raise NotImplementedError

    def path(self, out_dir: str, base_name: str, ext: str, suffix: str = None) -> str:
        return GetPtabV3ArtifactPath(out_dir, base_name, ext, suffix)


class GccLcpuImg(LcpuImgStrategy):
    def build(self, program_file: str, out_dir: str, base_name: str, ext: str, rtconfig) -> str:
        objcopy_args = ['-Obinary'] if ext.lower() == '.bin' else ['-O', 'ihex']
        ex_imgs = []
        tempfile_path = os.path.join(os.path.dirname(program_file), 'rom_temp' + ext)
        for i in range(2, 2 + MAX_EX_IMG_NUM):
            subprocess.run([rtconfig.OBJCPY, '-Obinary', '-j.rom{}'.format(i), program_file, tempfile_path], check=True)
            size = os.path.getsize(tempfile_path)
            os.remove(tempfile_path)
            if size > 0:
                ex_imgs.append(i)

        if not ex_imgs:
            code_path = self.path(out_dir, base_name, ext)
            _remove_file_or_dir(code_path)
            subprocess.run([rtconfig.OBJCPY] + objcopy_args + [program_file, code_path], check=True)
            return code_path

        exclude_ex_imgs = []
        for i in ex_imgs:
            ex_img_path = self.path(out_dir, base_name, ext, 'ER_IROM{}'.format(i))
            _remove_file_or_dir(ex_img_path)
            subprocess.run([rtconfig.OBJCPY] + objcopy_args + ['-j.rom{}'.format(i), program_file, ex_img_path], check=True)
            exclude_ex_imgs.append('-R.rom{}'.format(i))

        code_path = self.path(out_dir, base_name, ext, 'ER_IROM1')
        _remove_file_or_dir(code_path)
        subprocess.run([rtconfig.OBJCPY] + objcopy_args + exclude_ex_imgs + [program_file, code_path], check=True)
        return code_path


class KeilLcpuImg(LcpuImgStrategy):
    def build(self, program_file: str, out_dir: str, base_name: str, ext: str, rtconfig) -> str:
        export_path = tempfile.mkdtemp(
            prefix='.fromelf_lcpu_{}_'.format(ext.strip('.').lower()),
            dir=out_dir,
        )
        _remove_file_or_dir(export_path)
        fromelf_mode = '--bin' if ext.lower() == '.bin' else '--i32'

        try:
            subprocess.run(['fromelf', fromelf_mode, program_file, '--output', export_path], check=True)
            entries = _ptab_v3_fromelf_entries(export_path, ext)
            primary = self._find_irom(entries, 1, set(), ext)
            if not primary and os.path.isfile(export_path):
                payload_entries = [
                    entry for entry in entries
                    if _ptab_v3_artifact_has_payload(entry['path'], ext)
                ]
                if len(payload_entries) == 1:
                    primary = payload_entries[0]

            if not primary:
                logging.error(
                    "Keil ptab v3 embedded LCPU did not generate ER_IROM1 in %s",
                    out_dir,
                )
                raise SystemExit(1)

            used_paths = {primary['path']}
            ex_entries = []
            for i in range(2, 2 + MAX_EX_IMG_NUM):
                entry = self._find_irom(entries, i, used_paths, ext)
                if entry:
                    used_paths.add(entry['path'])
                    ex_entries.append((i, entry))

            code_suffix = 'ER_IROM1' if ex_entries else None
            code_path = self.path(out_dir, base_name, ext, code_suffix)
            self._move(primary['path'], code_path)

            for i, entry in ex_entries:
                ex_path = self.path(out_dir, base_name, ext, 'ER_IROM{}'.format(i))
                self._move(entry['path'], ex_path)

            return code_path
        finally:
            _remove_file_or_dir(export_path)

    def _find_irom(self, entries, index: int, used_paths, ext: str):
        aliases = _ptab_v3_fromelf_region_aliases('ER_IROM{}'.format(index))
        return _find_ptab_v3_fromelf_entry(entries, aliases, used_paths, ext)

    def _move(self, src: str, dst: str) -> None:
        _remove_file_or_dir(dst)
        shutil.move(src, dst)


def _get_lcpu_img_strategy(platform: str):
    if platform == 'gcc':
        return GccLcpuImg()
    if platform == 'armcc':
        return KeilLcpuImg()
    return None


def _build_ptab_v3_embedded_lcpu_artifact(program_file: str, out_dir: str, base_name: str, ext: str, rtconfig) -> str:
    strategy = _get_lcpu_img_strategy(getattr(rtconfig, 'PLATFORM', ''))
    if not strategy:
        logging.error("Unsupported ptab v3 embedded LCPU platform: %s", getattr(rtconfig, 'PLATFORM', ''))
        raise SystemExit(1)
    return strategy.build(program_file, out_dir, base_name, ext, rtconfig)


def ProgramBinaryBuild(target, source, env):
    import rtconfig
    import ptab as ptab_module

    program_file = str(source[0])
    program_filename = os.path.basename(program_file)
    program_name = os.path.splitext(program_filename)[0]
    target_path = os.path.dirname(program_file)
    whole_bin_path = os.path.join(target_path, program_name + '.bin')
    code_bin_path = str(target[0])
    out_dir = os.path.dirname(code_bin_path)

    ptab_obj = env.GetPtab() if hasattr(env, "GetPtab") else None
    if ptab_obj is None and 'PARTITION_TABLE' in env:
        ptab_obj = ptab_module.load_ptab(env['PARTITION_TABLE'], fatal=False)

    core = _infer_build_core(env, getattr(rtconfig, 'CORE', 'HCPU'))
    # ptab v3: embedded LCPU keeps ER_IROMN sections, but uses the same
    # output naming rule as other multi-artifact projects: <base>.ER_IROMN.ext.
    if rtconfig.PLATFORM in ('gcc', 'armcc') and ptab_obj and ptab_obj.is_v3() and core == 'LCPU' and env.get('IMG_EMBEDDED'):
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_bin_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.bin', PTAB_V3_PROGRAM_BINARY_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.bin')

        if rtconfig.PLATFORM == 'gcc':
            shutil.copy(str(source[0]), str(source[0]) + '.strip.elf')
            subprocess.run([rtconfig.STRIP, str(source[0]) + '.strip.elf'], check=True)
        code_bin_path = _build_ptab_v3_embedded_lcpu_artifact(program_file, out_dir, base_name, '.bin', rtconfig)
        _write_ptab_v3_artifact_stamp(stamp_path, code_bin_path)
        if rtconfig.PLATFORM == 'armcc':
            subprocess.run(['fromelf', '-z', str(source[0])], check=True)
        return

    if rtconfig.PLATFORM == 'armcc' and ptab_obj and ptab_obj.is_v3() and core == 'LCPU' and env.get('IMG_EMBEDDED'):
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_bin_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.bin', PTAB_V3_PROGRAM_BINARY_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.bin')

        code_bin_path = _build_ptab_v3_keil_embedded_lcpu_artifacts(
            program_file,
            out_dir,
            base_name,
            '.bin',
            stamp_path,
        )
        if (not code_bin_path
                or not os.path.isfile(code_bin_path)):
            logging.error(
                "Keil ptab v3 embedded LCPU did not generate expected artifacts "
                "(expected ER_IROM1 and ER_IROM2) in %s",
                out_dir,
            )
            raise SystemExit(1)
        subprocess.run(['fromelf', '-z', str(source[0])], check=True)
        return

    # ptab v3: split app/ex sections and emit final artifacts into `output/`
    if rtconfig.PLATFORM == 'gcc' and ptab_obj and ptab_obj.is_v3():
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_bin_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.bin', PTAB_V3_PROGRAM_BINARY_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.bin')

        split_entries = _collect_ptab_v3_split_partitions(ptab_obj, core, ptab_module)
        _validate_ptab_v3_artifact_names(base_name, split_entries, out_dir)

        used_sections = []
        all_sections = []
        present_sections = []
        for entry in split_entries:
            name = entry['name']
            if not name:
                continue
            sec = entry['section']
            all_sections.append(sec)
            out_file = GetPtabV3ArtifactPath(out_dir, base_name, '.bin', name)
            _remove_file_or_dir(out_file)
            res = subprocess.run([rtconfig.OBJCPY, '-Obinary', '-j{}'.format(sec), program_file, out_file])
            if res.returncode != 0:
                _remove_file_or_dir(out_file)
                continue
            present_sections.append(sec)
            if not _ptab_v3_artifact_has_payload(out_file, '.bin'):
                _remove_file_or_dir(out_file)
                continue
            used_sections.append(sec)

        code_bin_path = GetPtabV3ArtifactPath(
            out_dir,
            base_name,
            '.bin',
            'app' if used_sections else None,
        )
        exclude_args = ['-R{}'.format(s) for s in dict.fromkeys(all_sections)]
        _remove_file_or_dir(code_bin_path)
        res = subprocess.run([rtconfig.OBJCPY, '-Obinary'] + exclude_args + [program_file, code_bin_path])
        if res.returncode != 0 and present_sections and present_sections != all_sections:
            exclude_args = ['-R{}'.format(s) for s in dict.fromkeys(present_sections)]
            subprocess.run([rtconfig.OBJCPY, '-Obinary'] + exclude_args + [program_file, code_bin_path], check=True)
        else:
            res.check_returncode()
        _write_ptab_v3_artifact_stamp(stamp_path, code_bin_path)
        return

    if rtconfig.PLATFORM == 'armcc' and ptab_obj and ptab_obj.is_v3():
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_bin_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.bin', PTAB_V3_PROGRAM_BINARY_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.bin')

        split_entries = _collect_ptab_v3_split_partitions(ptab_obj, core, ptab_module)
        _validate_ptab_v3_artifact_names(base_name, split_entries, out_dir)
        _build_ptab_v3_keil_artifacts(
            program_file,
            out_dir,
            base_name,
            '.bin',
            split_entries,
            stamp_path,
        )
        subprocess.run(['fromelf', '-z', str(source[0])], check=True)
        return

    if os.path.exists(whole_bin_path):
        _remove_file_or_dir(whole_bin_path)
    # TODO: only support keil and gcc
    if rtconfig.PLATFORM == 'armcc':
        subprocess.run(['fromelf', '--bin', str(source[0]), '--output', whole_bin_path], check=True)
        if os.path.isdir(whole_bin_path):
            # delete the folder to clean old files
            shutil.rmtree(whole_bin_path)
            subprocess.run(['fromelf', '--bin', str(source[0]), '--output', whole_bin_path], check=True)        
            dir_list = os.listdir(whole_bin_path)
            for d in dir_list:
                if '.bin' not in d:
                    shutil.move(os.path.join(whole_bin_path, d), os.path.join(whole_bin_path, d + '.bin'))
        # print Object/Image Component Sizes
        subprocess.run(['fromelf', '-z', str(source[0])], check=True)
    elif rtconfig.PLATFORM == 'gcc':
        shutil.copy(str(source[0]),str(source[0])+'.strip.elf')
        subprocess.run([rtconfig.STRIP, str(source[0])+'.strip.elf'], check=True)
        # check whether there're multiple binary
        ex_imgs = []
        tempfile_path = os.path.join(target_path, 'rom_temp.bin')
        for i in range(2, 2 + MAX_EX_IMG_NUM):
            subprocess.run([rtconfig.OBJCPY, '-Obinary', '-j.rom{}'.format(i), str(source[0]), tempfile_path], check=True)
            size = os.path.getsize(tempfile_path)
            os.remove(tempfile_path)
            if size > 0:
                ex_imgs.append(i)

        if len(ex_imgs) == 0:
            subprocess.run([rtconfig.OBJCPY, '-Obinary', str(source[0]), whole_bin_path], check=True)
        else:
            os.mkdir(whole_bin_path)
            exclude_ex_imgs = []
            for i in ex_imgs:
                ex_img_path = os.path.join(whole_bin_path, 'ER_IROM{}.bin'.format(i))
                subprocess.run([rtconfig.OBJCPY, '-Obinary', '-j.rom{}'.format(i), str(source[0]), ex_img_path], check=True)
                exclude_ex_imgs += ['-R.rom{}'.format(i)]

            rom1_path = ex_img_path = os.path.join(whole_bin_path, 'ER_IROM1.bin')
            subprocess.run([rtconfig.OBJCPY, '-Obinary'] + exclude_ex_imgs + [str(source[0]), rom1_path], check=True)


def ProgramHexBuild(target, source, env):
    import ptab as ptab_module

    program_file = str(source[0])
    program_filename = os.path.basename(program_file)
    program_name = os.path.splitext(program_filename)[0]
    target_path = os.path.dirname(program_file)
    whole_hex_path = os.path.join(target_path, program_name + '.hex')
    code_hex_path = str(target[0])
    out_dir = os.path.dirname(code_hex_path)
 
    import rtconfig

    ptab_obj = env.GetPtab() if hasattr(env, "GetPtab") else None
    if ptab_obj is None and 'PARTITION_TABLE' in env:
        ptab_obj = ptab_module.load_ptab(env['PARTITION_TABLE'], fatal=False)

    core = _infer_build_core(env, getattr(rtconfig, 'CORE', 'HCPU'))
    # ptab v3: embedded LCPU keeps ER_IROMN sections, but uses the same
    # output naming rule as other multi-artifact projects: <base>.ER_IROMN.ext.
    if rtconfig.PLATFORM in ('gcc', 'armcc') and ptab_obj and ptab_obj.is_v3() and core == 'LCPU' and env.get('IMG_EMBEDDED'):
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_hex_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.hex', PTAB_V3_PROGRAM_HEX_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.hex')

        code_hex_path = _build_ptab_v3_embedded_lcpu_artifact(program_file, out_dir, base_name, '.hex', rtconfig)
        _write_ptab_v3_artifact_stamp(stamp_path, code_hex_path)
        return

    if rtconfig.PLATFORM == 'armcc' and ptab_obj and ptab_obj.is_v3() and core == 'LCPU' and env.get('IMG_EMBEDDED'):
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_hex_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.hex', PTAB_V3_PROGRAM_HEX_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.hex')

        code_hex_path = _build_ptab_v3_keil_embedded_lcpu_artifacts(
            program_file,
            out_dir,
            base_name,
            '.hex',
            stamp_path,
        )
        if (not code_hex_path
                or not os.path.isfile(code_hex_path)):
            logging.error(
                "Keil ptab v3 embedded LCPU did not generate expected artifacts "
                "(expected ER_IROM1 and ER_IROM2) in %s",
                out_dir,
            )
            raise SystemExit(1)
        return

    # ptab v3: split app/ex sections and emit final artifacts into `output/`
    if rtconfig.PLATFORM == 'gcc' and ptab_obj and ptab_obj.is_v3():
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_hex_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.hex', PTAB_V3_PROGRAM_HEX_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.hex')

        split_entries = _collect_ptab_v3_split_partitions(ptab_obj, core, ptab_module)
        _validate_ptab_v3_artifact_names(base_name, split_entries, out_dir)

        used_sections = []
        all_sections = []
        present_sections = []
        for entry in split_entries:
            name = entry['name']
            if not name:
                continue
            sec = entry['section']
            all_sections.append(sec)
            out_file = GetPtabV3ArtifactPath(out_dir, base_name, '.hex', name)
            _remove_file_or_dir(out_file)
            res = subprocess.run([rtconfig.OBJCPY, '-O', 'ihex', '-j{}'.format(sec), program_file, out_file])
            if res.returncode != 0:
                _remove_file_or_dir(out_file)
                continue
            present_sections.append(sec)
            if not _ptab_v3_artifact_has_payload(out_file, '.hex'):
                _remove_file_or_dir(out_file)
                continue
            used_sections.append(sec)

        code_hex_path = GetPtabV3ArtifactPath(
            out_dir,
            base_name,
            '.hex',
            'app' if used_sections else None,
        )
        exclude_args = ['-R{}'.format(s) for s in dict.fromkeys(all_sections)]
        _remove_file_or_dir(code_hex_path)
        res = subprocess.run([rtconfig.OBJCPY, '-O', 'ihex'] + exclude_args + [program_file, code_hex_path])
        if res.returncode != 0 and present_sections and present_sections != all_sections:
            exclude_args = ['-R{}'.format(s) for s in dict.fromkeys(present_sections)]
            subprocess.run([rtconfig.OBJCPY, '-O', 'ihex'] + exclude_args + [program_file, code_hex_path], check=True)
        else:
            res.check_returncode()
        _write_ptab_v3_artifact_stamp(stamp_path, code_hex_path)
        return

    if rtconfig.PLATFORM == 'armcc' and ptab_obj and ptab_obj.is_v3():
        base_name = GetPtabV3ArtifactBaseName(env, program_file)
        stamp_path = code_hex_path
        out_dir = os.path.dirname(stamp_path)
        _cleanup_ptab_v3_output_dir(out_dir, '.hex', PTAB_V3_PROGRAM_HEX_STAMP)
        _cleanup_ptab_v3_legacy_artifacts(program_file, '.hex')

        split_entries = _collect_ptab_v3_split_partitions(ptab_obj, core, ptab_module)
        _validate_ptab_v3_artifact_names(base_name, split_entries, out_dir)
        _build_ptab_v3_keil_artifacts(
            program_file,
            out_dir,
            base_name,
            '.hex',
            split_entries,
            stamp_path,
        )
        return

    if os.path.exists(whole_hex_path):
        _remove_file_or_dir(whole_hex_path)
    #TODO: only support keil and gcc
    if rtconfig.PLATFORM == 'gcc':
        # check whether there're multiple binary 
        ex_imgs = []
        tempfile_path = os.path.join(target_path, 'rom_temp.hex')
        for i in range(2, 2 + MAX_EX_IMG_NUM):
            subprocess.run([rtconfig.OBJCPY, '-Obinary', '-j.rom{}'.format(i), str(source[0]), tempfile_path], check=True)
            size = os.path.getsize(tempfile_path)
            os.remove(tempfile_path)
            if size > 0:
                ex_imgs.append(i)

        if len(ex_imgs) == 0:
            subprocess.run([rtconfig.OBJCPY, '-O', 'ihex', str(source[0]), whole_hex_path], check=True)
        else:
            os.mkdir(whole_hex_path)
            exclude_ex_imgs = []
            for i in ex_imgs:
                ex_img_path = os.path.join(whole_hex_path, 'ER_IROM{}.hex'.format(i))
                subprocess.run([rtconfig.OBJCPY, '-O', 'ihex', '-j.rom{}'.format(i), str(source[0]), ex_img_path], check=True)
                exclude_ex_imgs += ['-R.rom{}'.format(i)]

            rom1_path = ex_img_path = os.path.join(whole_hex_path, 'ER_IROM1.hex')
            subprocess.run([rtconfig.OBJCPY, '-O', 'ihex'] + exclude_ex_imgs + [str(source[0]), rom1_path], check=True)


    else:    
        subprocess.run(['fromelf', '--i32', str(source[0]), '--output', whole_hex_path], check=True)    
        if os.path.isdir(whole_hex_path):
            # delete the folder to clean old files
            shutil.rmtree(whole_hex_path)
            subprocess.run(['fromelf', '--i32', str(source[0]), '--output', whole_hex_path], check=True)    
            dir_list = os.listdir(whole_hex_path)
            for d in dir_list:
                if '.hex' not in d:
                    shutil.move(os.path.join(whole_hex_path, d), os.path.join(whole_hex_path, d + '.hex'))


def ProgramAsmBuild(target, source, env):
    program_file = str(source[0])
    program_filename = os.path.basename(program_file)
    program_name = os.path.splitext(program_filename)[0]
    target_path = os.path.dirname(program_file)
    asm_path = os.path.join(target_path, program_name + '.asm')

    #TODO: only support keil and gcc
    import rtconfig
    if rtconfig.PLATFORM == 'armcc':
        if GetDepend("SOC_SF32LB58X") or GetDepend("SOC_SF32LB56X"):
            subprocess.run(['fromelf', '--cpu=8-M.Main', '--coproc1=cde', '--text', '-c', str(source[0]), '--output', asm_path], check=True)
        else:
            subprocess.run(['fromelf', '--text', '-c', str(source[0]), '--output', asm_path], check=True)
    elif rtconfig.PLATFORM == 'gcc':
        subprocess.run([rtconfig.OBJDUMP, '-d', str(source[0])], stdout=open(asm_path,"wb+"), check=True)
    
def LdsBuild(target, source, env):
    import rtconfig
    import ptab as ptab_module

    # Detect ptab version
    ptab_obj = env.GetPtab() if hasattr(env, "GetPtab") else None
    if ptab_obj is None and 'PARTITION_TABLE' in env:
        ptab_obj = ptab_module.load_ptab(env['PARTITION_TABLE'], fatal=False)

    target_path = str(target[0])

    if ptab_obj and ptab_obj.is_v3():
        # v3: Jinja2-only rendering (no gcc -E / armclang -E)
        import gen_link_lds

        build_dir = os.path.dirname(os.path.abspath(target_path))
        rtconfig_h_path = os.path.join(build_dir, 'rtconfig.h')
        rtconfig_defines = gen_link_lds._parse_rtconfig_defines(rtconfig_h_path)
        build_core = _infer_build_core(env, getattr(rtconfig, 'CORE', 'HCPU'))
        defines = gen_link_lds.compute_link_defines(
            ptab_obj,
            env.get('name', 'main'),
            build_core,
            rtconfig_defines,
        )
        gen_link_lds.render_link_lds(str(source[0]), target_path, defines)
        return

    if rtconfig.PLATFORM == 'armcc':
        with open(str(source[0]), 'r', encoding='utf-8', errors='ignore') as f:
            script = f.read()

        with open(target_path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(_replace_link_script_env_vars(script, str(env.get('BSP_ROOT', ''))))
        return

    # v1/v2: legacy gcc -E preprocessing
    include_paths = ['-I{}'.format(path.replace('\\', '/')) for path in env['CPPPATH']]
    p = subprocess.Popen([rtconfig.CC, '-E', '-P'] + include_paths + ['-x', 'c', str(source[0])], stdout=subprocess.PIPE)
    (result, error) = p.communicate()
    with open(target_path, "wb") as f:
        f.write(result)

def FsBuild(target, source, env):
    import rtconfig
    
    ptab_obj = env.GetPtab() if hasattr(env, "GetPtab") else None
    if ptab_obj is None:
        import ptab as _ptab
        ptab_obj = _ptab.load_ptab(env['PARTITION_TABLE'], fatal=True)
    mems = ptab_obj.content_mems()
        
    found=0
    for mem in mems:
        if 'version' in mem:
            # skip header
            continue
        mem_base = int(mem['base'], 0)
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            max_size = int(region['max_size'], 0)
            start_addr = mem_base + offset
            if 'tags' in region and "FS_REGION" in region['tags']:
                found=1
                break
        if (found==1):
            break
    if (found==1):    
        target=os.path.join(env['build_dir'],'fs_root.bin') 
        page_size=env['page_size']
        page_number=max_size/page_size
        subprocess.run([env['fs_mkimg'],env['fs_root'],target,str(page_number),str(page_size)], check=True)

def ModifyLdsTargets(target, source, env):
    import ptab as ptab_module

    src0 = str(source[0]) if source else ''
    target_ext = '.sct' if src0.endswith('.sct') or src0.endswith('.sct.jinja2') else '.lds'
    target = [os.path.join(env['build_dir'], 'link_copy' + target_ext)]
    if 'PTAB_HEADER' in env:
        env.Depends(target, env['PTAB_HEADER'])

    ptab_obj = env.GetPtab() if hasattr(env, "GetPtab") else None
    if ptab_obj is None and 'PARTITION_TABLE' in env:
        ptab_obj = ptab_module.load_ptab(env['PARTITION_TABLE'], fatal=False)

    template_path = _get_link_template_path(src0) if src0 else ''
    if ptab_obj and ptab_obj.is_v3():
        if not template_path or not os.path.exists(template_path):
            logging.error(f"ptab v3 requires jinja2 link template: {template_path}")
            raise SystemExit(1)
        source = [File(template_path)]
    elif src0 and not os.path.exists(src0) and template_path and os.path.exists(template_path):
        fallback = _get_chip_link_fallback(src0, env)
        if fallback:
            logging.warning(
                "ptab v1/v2 fallback to chip legacy link script: %s (skip template-only override %s)",
                fallback,
                template_path,
            )
            source = [File(fallback)]
        else:
            logging.error(f"ptab v1/v2 does not support jinja2-only link template: {template_path}")
            raise SystemExit(1)

    return target, source

def EmbeddedImgCFileBuild(target, source, env):
    target_path = os.path.dirname(str(target[0]))
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    GEN_SRC_PATH = os.path.join(SIFLI_SDK, "tools/patch/gen_src.py")
    s = str(source[0])
    if os.path.isdir(s) or IsPtabV3ArtifactStampPath(s):
        s = ResolvePtabV3CodeArtifactFromRef(s, GetPtabV3ArtifactBaseName(env), '.bin')
    if "acpu" in s:
        subprocess.run(['python', GEN_SRC_PATH, 'general', s, target_path, "acpu"], check=True)
        shutil.move(os.path.join(target_path, 'acpu_img.c'), str(target[0]))
    else:
        subprocess.run(['python', GEN_SRC_PATH, 'lcpu', s, target_path], check=True)
        shutil.move(os.path.join(target_path, 'lcpu_img.c'), str(target[0]))


def FtabCFileBuild(target, source, env):
    import sdk_resource
    import ptab as ptab_module

    src_file = str(source[0])
    target_file = str(target[0])

    if 'template' in os.path.splitext(src_file)[1]:
        shutil.copy(src_file, target_file)
    else:
        # v3 uses FtabBin builder to generate ftab.bin directly (no ftab subproject / ftab.c)
        ptab_obj = ptab_module.load_ptab(src_file, fatal=False)
        if ptab_obj and ptab_obj.is_v3():
            logging.error(
                "ptab v3 does not support generating ftab.c; use FtabBin (ftab.bin generated by script) instead."
            )
            raise SystemExit(1)

        # v1/v2: use legacy C generation flow (ftab subproject)
        sdk_resource.GenFtabCFile(src_file, target_file, env['IMGS_INFO'])


def FtabBinBuild(target, source, env):
    """直接生成 ftab.bin（用于 ptab v3）

    当使用 ptab v3 格式时，可以直接生成 ftab.bin 而不需要编译 ftab 子工程。
    """
    import gen_ftab
    import ptab as ptab_module

    src_file = str(source[0])
    target_file = str(target[0])

    # 加载 ptab
    ptab_obj = ptab_module.load_ptab(src_file, fatal=True)

    # 获取 chip config
    if ptab_obj.is_v3():
        chip_config = ptab_obj.get_chip_config()
    else:
        # 尝试从环境中获取芯片信息
        chip = env.get('CHIP', '').lower().replace('sf32lb', 'sf32lb').rstrip('x')
        if chip:
            chip_config = ptab_module.load_chip_config(chip)
        else:
            chip_config = {'mpi': {}, 'ram': {}}
            logging.warning("No chip config available for ftab.bin generation")

    # 获取 image 大小
    imgs_info = env.get('IMGS_INFO', [])
    bootloader_size = 0x10000
    main_size = 0x200000
    image_sizes = {}
    enabled_images = set()

    def _calc_binary_size(path, base_name='main'):
        path = str(path)
        if IsPtabV3ArtifactStampPath(path) or os.path.isdir(path):
            path = ResolvePtabV3CodeArtifactFromRef(path, base_name, '.bin')
        if os.path.isfile(path):
            return os.path.getsize(path)
        if os.path.isdir(path):
            # Prefer ER_IROM1.bin when multiple binaries are generated.
            preferred = os.path.join(path, 'ER_IROM1.bin')
            if os.path.isfile(preferred):
                return os.path.getsize(preferred)
            max_size = 0
            for n in os.listdir(path):
                f = os.path.join(path, n)
                if os.path.isfile(f):
                    max_size = max(max_size, os.path.getsize(f))
            return max_size if max_size > 0 else None
        return None

    for img in imgs_info:
        img_name = img.get('name', '')
        if img_name:
            enabled_images.add(img_name)
        if img_name == 'bootloader' and 'binary' in img:
            try:
                sz = _calc_binary_size(img['binary'][0], img_name)
                if sz:
                    bootloader_size = sz
                    image_sizes['bootloader'] = int(sz)
            except:
                pass
        elif img_name == 'main' and 'binary' in img:
            try:
                sz = _calc_binary_size(img['binary'][0], img_name)
                if sz:
                    main_size = sz
                    image_sizes['main'] = int(sz)
            except:
                pass
        elif 'binary' in img:
            try:
                sz = _calc_binary_size(img['binary'][0], img_name or 'main')
                if sz:
                    image_sizes[img_name] = int(sz)
            except:
                pass

    # 生成 ftab.bin
    ftab_binary = gen_ftab.generate_ftab_binary(
        ptab_obj,
        chip_config,
        bootloader_size,
        main_size,
        image_sizes=image_sizes,
        enabled_images=enabled_images,
    )

    # 写入文件
    with open(target_file, 'wb') as f:
        f.write(ftab_binary)

    logging.info(f"Generated ftab.bin: {target_file} ({len(ftab_binary)} bytes)")
       

def FileCopyBuild(target, source, env):
    src_file = str(source[0])
    target_file = str(target[0])
    shutil.copy(src_file, target_file)

def ModifyFileCopyTargets(target, source, env):
    target = []
    for s in source:
        s = os.path.basename(str(s))
        t = os.path.splitext(s)[0]
        t = t + "_copy" + os.path.splitext(s)[1]
        target.append(t)
    
    return target, source

def AddCustomImg(name, hex = None, bin = None):
    img = {"name": name, "program_hex": hex, "program_binary": bin}
    CustomImgList.append(img)

def GetCustomImgList():
    return CustomImgList

def GenDownloadScript(main_env):
    import sdk_resource
    import rtconfig
    
    if rtconfig.ARCH=='sim':
        return

    PrintEnvList()

    if GetDepend("BOARD_RAMRUN_ENABLED"):
        return

    dependent_files = []
    for env in EnvList:
        # embedded project may contain multiple bin, but only IROM1.bin is embedded, others should be downloaded manually
        #if IsEmbeddedProjEnv(env):
        #    continue
        dependent_files.append(env['target'])
        dependent_files.append(env['program_binary'])        
        dependent_files.append(env['program_hex'])
    if 'PARTITION_TABLE' in main_env:
        dependent_files.append(main_env['PARTITION_TABLE'])
    elif not GetDepend('USING_PARTITION_TABLE'):
        logging.warning("Partition table is not used.")
    if 'FTAB_BIN' in main_env:
        dependent_files.append(main_env['FTAB_BIN'])

    for img in CustomImgList:
        if img['program_binary']:
            dependent_files.append(img['program_binary'])        
        if img['program_hex']:
            dependent_files.append(img['program_hex'])        
    target = main_env.DownloadScript(dependent_files)
    #Depends(target, dependent_files)

def DownloadScriptBuild(target, source, env):
    import sdk_resource

    sdk_resource.BuildJLinkLoadScript(env)

def FileSystemBuild(source, env):
    if GetDepend('RT_USING_DFS_ELMFAT'):
        target=os.path.join(env['build_dir'],'fs_root.bin')
        source_list=[]
        for r,d,f in os.walk(source):
            for file in f:
                source_list+=[os.path.join(r,file)]                
        Depends(target,source_list)
        SIFLI_SDK = os.getenv('SIFLI_SDK')
        if GetDepend('RT_USING_MTD_NAND'):
            MKIMG_PATH = os.path.join(SIFLI_SDK, f"tools/mkfatimg/mkfatimg_nand/Release/mkfatimg{env['tool_suffix']}")
            page_size=2048
        elif GetDepend('RT_USING_MTD_NOR'):
            MKIMG_PATH = os.path.join(SIFLI_SDK, f"tools/mkfatimg/mkfatimg{env['tool_suffix']}")
            page_size=4096
        else:
            MKIMG_PATH = os.path.join(SIFLI_SDK, f"tools/mkfatimg/mkfatimg{env['tool_suffix']}")
            page_size=4096

        env['fs_root']=source
        env['fs_mkimg']=MKIMG_PATH
        env['page_size']=page_size
        env.FileSystem(target,source_list)           
        return target
    else:   # TODO: Add for other filesystem.
        return None
       
    
def ModifyDownloadScriptTargets(target, source, env):
    target = [os.path.join(env['build_dir'], 'download.bat'), 
              os.path.join(env['build_dir'], 'download.jlink')]
          
    return target, source

def MergeRtconfig(rtconfig1, rtconfig2):
    for var in dir(rtconfig2):
        if not var.startswith('_') and not var.islower():
            setattr(rtconfig1, var, getattr(rtconfig2, var))

def _get_link_template_path(link_source):
    link_source = str(link_source or '')
    if link_source.endswith('.sct'):
        return link_source + '.jinja2'
    base, _ = os.path.splitext(link_source)
    return base + '.jinja2'


_LINK_SCRIPT_ENV_VARS = (
    '$SIFLI_SDK_PATH',
    '$SIFLI_SDK',
    '$SDK_ROOT',
    '$BSP_ROOT',
    '$BOARD_ROOT',
)


def _normalize_link_script_path(path):
    return str(path or '').replace('\\', '/')


def _get_link_script_board_root():
    try:
        board = GetBoardName()
        if board:
            board_path1, _ = GetBoardPath(board)
            return _normalize_link_script_path(board_path1)
    except Exception:
        pass
    return ''


def _replace_link_script_env_vars(script, bsp_root=''):
    sdk_root = os.getenv('SIFLI_SDK') or os.getenv('SIFLI_SDK_PATH') or ''
    replacements = [
        ('$SIFLI_SDK_PATH', _normalize_link_script_path(sdk_root)),
        ('$SIFLI_SDK', _normalize_link_script_path(sdk_root)),
        ('$SDK_ROOT', _normalize_link_script_path(sdk_root)),
        ('$BSP_ROOT', _normalize_link_script_path(bsp_root)),
        ('$BOARD_ROOT', _get_link_script_board_root()),
    ]
    for name, value in replacements:
        script = script.replace(name, value)
    return script


def _link_script_has_env_vars(script):
    return any(name in script for name in _LINK_SCRIPT_ENV_VARS)


def _current_project_uses_ptab_v3(bsp_root=''):
    if not GetDepend('USING_PARTITION_TABLE'):
        return False

    try:
        import rtconfig
        import ptab as ptab_module
    except Exception:
        return False

    project_root = bsp_root or Dir('#').abspath
    ptab_path = getattr(rtconfig, 'PARTITION_TABLE', None)
    ptab_obj = None
    try:
        board = GetBoardName(getattr(rtconfig, 'CORE', None))
        if board:
            board_path1, _ = GetBoardPath(board)
            prepared = ptab_module.prepare_project_ptab(
                project_root,
                board,
                getattr(rtconfig, 'CHIP', '').lower(),
                board_path=board_path1,
                emit_summary=False,
                validate=False,
            )
            ptab_obj = prepared.get('ptab_obj')
        else:
            if not ptab_path:
                link_dir = os.path.join(project_root, 'linker_scripts')
                yaml_path = os.path.join(link_dir, 'ptab.yaml')
                json_path = os.path.join(link_dir, 'ptab.json')
                if os.path.exists(yaml_path):
                    ptab_path = yaml_path
                elif os.path.exists(json_path):
                    ptab_path = json_path
            if ptab_path:
                ptab_obj = ptab_module.load_ptab(ptab_path, fatal=False)
    except Exception:
        return False

    return bool(ptab_obj and ptab_obj.is_v3())


def _get_chip_link_fallback(link_source, env):
    import rtconfig

    link_source = str(link_source or '')
    ext = os.path.splitext(link_source)[1].lower()
    if ext not in ('.lds', '.sct'):
        return None

    sifli_sdk = os.getenv('SIFLI_SDK')
    if not sifli_sdk:
        return None

    if GetDepend('SOC_SF32LB52X'):
        chip = 'sf32lb52x'
    elif GetDepend('SOC_SF32LB55X'):
        chip = 'sf32lb55x'
    elif GetDepend('SOC_SF32LB56X'):
        chip = 'sf32lb56x'
    elif GetDepend('SOC_SF32LB58X'):
        chip = 'sf32lb58x'
    else:
        return None

    core = _infer_build_core(env, getattr(rtconfig, 'CORE', 'HCPU')).lower()
    if ext == '.lds':
        fallback = os.path.join(sifli_sdk, 'drivers', 'cmsis', chip, 'Templates', 'gcc', core, 'link' + ext)
    else:
        fallback = os.path.join(sifli_sdk, 'drivers', 'cmsis', chip, 'Templates', 'arm', core, 'link' + ext)

    return fallback if os.path.exists(fallback) else None


def _select_link_script_base(candidates, ext):
    for base, desc in candidates:
        legacy_path = base + ext
        template_path = _get_link_template_path(legacy_path)
        if os.path.exists(legacy_path) or os.path.exists(template_path):
            logging.debug('Use {} link file: {}'.format(desc, base))
            return base, (base if os.path.exists(template_path) else None)
    return None, None


def GetLinkScript(proj_path,board,chip,core):
    board_path1, board_path2 = GetBoardPath(board)
    CC_TOOLS = os.getenv('RTT_CC')
    link_script=None
    link_script_template=None
    chip = chip.lower()
    core = core.lower()
    if CC_TOOLS=='keil' or CC_TOOLS=='gcc':
        ext = '.sct' if CC_TOOLS == 'keil' else '.lds'
        SIFLI_SDK = os.getenv('SIFLI_SDK')
        if CC_TOOLS == 'keil':
            chip_base = os.path.join(SIFLI_SDK, 'drivers/cmsis/{}/Templates/arm/{}/link'.format(chip, core))
        else:
            chip_base = os.path.join(SIFLI_SDK, 'drivers/cmsis/{}/Templates/gcc/{}/link'.format(chip, core))

        candidates = [
            (os.path.join(proj_path, board, 'link'), 'project board'),
            (os.path.join(proj_path, chip + '/link'), 'project chip'),
            (os.path.join(proj_path, 'link'), 'project'),
            (os.path.join(board_path2, 'link'), 'board'),
            (chip_base, 'chip'),
        ]
        link_script, link_script_template = _select_link_script_base(candidates, ext)
    return link_script,link_script_template

def AddChildProj(proj_name, proj_path, img_embedded=False, shared_option=None, core=None):
    import rtconfig
    global BuildOptions
    global Env

    try:
        AddOption('--compiledb',
                    dest = 'compiledb',
                    action = 'store_true',
                    default = False,
                    help = 'Generate DB')        
    except:
        pass

    if GetOption('compiledb'):
        return     

    parent_env = Env

    logging.debug("\n======================")
    logging.debug("Add child proj: {}".format(proj_name))
    
    board = GetBoardName(core)
    logging.debug('Child board: {}'.format(board))
    if board:
        board_path1, board_path2 = GetBoardPath(board)
        logging.debug("Load {}".format(os.path.join(board_path2, 'rtconfig.py')))

        proj_rtconfig = (lambda spec: (spec.loader.exec_module(mod := importlib.util.module_from_spec(spec)) or mod))(importlib.util.spec_from_file_location(proj_name, os.path.join(board_path2, 'rtconfig.py')))
        proj_rtconfig2 = ( lambda spec: (spec.loader.exec_module(mod := importlib.util.module_from_spec(spec)) or mod))(importlib.util.spec_from_file_location(proj_name, os.path.join(proj_path, 'rtconfig.py')))
        MergeRtconfig(proj_rtconfig, proj_rtconfig2)
    else:    
        proj_rtconfig = (lambda spec: (spec.loader.exec_module(mod := importlib.util.module_from_spec(spec)) or mod))(importlib.util.spec_from_file_location(proj_name, os.path.join(proj_path, 'rtconfig.py')))

    parent_name = ParentProjStack[-1]['name']
    if (len(ParentProjStack) == 1):
        parent_output_dir = rtconfig.OUTPUT_DIR
    else:
        parent_output_dir = ParentProjStack[-1]['rtconfig'].OUTPUT_DIR
    full_proj_name = '.'.join([ParentProjStack[-1]['name'], proj_name])
        
    # output to parent build dir
    proj_rtconfig.OUTPUT_DIR = os.path.join(parent_output_dir, proj_name).replace('\\', '/')
    if board:
        # as env is not created before SifliEnv, save CORE in rtconfig.py for board selection
        proj_rtconfig.CORE = 'HCPU'
        if board.endswith('_lcpu'):
            proj_rtconfig.CORE = 'LCPU'
        elif board.endswith('_acpu'):
            proj_rtconfig.CORE = 'ACPU'
        proj_rtconfig.LINK_SCRIPT, proj_rtconfig.LINK_SCRIPT_TEMPLATE=GetLinkScript(proj_path,board,proj_rtconfig.CHIP,proj_rtconfig.CORE)
    else:    
        proj_rtconfig.LINK_SCRIPT = os.path.join(proj_path, proj_rtconfig.LINK_SCRIPT)

    proj_rtconfig.TARGET_NAME = proj_name

    child_list = []
    ParentProjStack.append({'name': full_proj_name, 'rtconfig': proj_rtconfig, 'child_list': child_list})
    
    # backup current BuildOptions
    build_options_backup = dict(BuildOptions)
    # clear old rtconfig
    rtconfig_backup = []
    for var in dir(rtconfig):
        if not var.startswith('_') and not var.islower():
            rtconfig_backup.append({'name': var, 'value': getattr(rtconfig, var)})
            delattr(rtconfig, var)
       
    # sync rtconfig with child project rtconfig
    for var in dir(proj_rtconfig):
        if not var.startswith('_') and not var.islower():
            setattr(rtconfig, var, getattr(proj_rtconfig, var))

    child_builder = None
    has_custom_sconstruct_py = os.path.isfile(os.path.join(proj_path, 'SConstruct.py'))
    if has_custom_sconstruct_py:
        child_builder = (lambda spec: (spec.loader.exec_module(mod := importlib.util.module_from_spec(spec)) or mod))(importlib.util.spec_from_file_location(proj_name, os.path.join(proj_path, 'SConstruct.py')))
        child_builder.create_env(proj_path)
        
    else:
        SifliEnv(proj_path)

    proj_env = Environment(tools = ['mingw'],
        AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
        CC = rtconfig.CC, CFLAGS = rtconfig.CFLAGS,
        CXX = rtconfig.CXX, CXXFLAGS = rtconfig.CXXFLAGS,
        AR = rtconfig.AR, ARFLAGS = '-rc', LIBPATH=['.'],
        RANLIB = rtconfig.RANLIB,
        LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS)
            
    proj_env.PrependENVPath('PATH', rtconfig.EXEC_PATH)
        
    proj_env['build_dir'] = proj_rtconfig.OUTPUT_DIR
    proj_env['BSP_ROOT'] = os.path.abspath(proj_path)
    proj_env['is_child_proj'] = True
    proj_env['IMG_EMBEDDED'] = img_embedded
    proj_env['name'] = proj_name
    proj_env['full_name'] = full_proj_name
    proj_env['parent'] = parent_name
    if shared_option  and 'PARTITION_TABLE' in shared_option:
        proj_env['PARTITION_TABLE'] = os.path.abspath(shared_option['PARTITION_TABLE'])

    if has_custom_sconstruct_py:
        child_builder.build(proj_env)
    else:
        # prepare building environment
        objs = PrepareBuilding(proj_env)

        # make a building
        DoBuilding(os.path.join(proj_env['build_dir'], rtconfig.TARGET_NAME + '.' + rtconfig.TARGET_EXT), objs)


    if img_embedded:
        ChildProjList.append({'name': proj_name, 'binary': proj_env['program_binary'], 'build_dir': proj_env['build_dir'], 'parent': parent_name})

    # restore old rtconfig, delete added var and load rtconfig of parent project
    for var in dir(rtconfig):
        if not var.startswith('_') and not var.islower():
            delattr(rtconfig, var)

    for var in rtconfig_backup:
        setattr(rtconfig, var['name'], var['value'])
    #imp.reload(rtconfig)
    
    ParentProjStack.pop()

    # restore old BuildOptions
    BuildOptions = build_options_backup
    Env = parent_env

    return proj_env
    
def CheckChildProjPresent(proj_name):
    is_present = False
    for child_proj in ChildProjList:
        if proj_name == child_proj['name']:
            is_present = True
            break
            
    return is_present        

def BuildOptionUpdate(BuildOptions,BSP_Root):
    global Env
    import rtconfig
    PreProcessor = PatchedPreProcessor()
    
    if GetBoardName():
        f = open(os.path.join(rtconfig.OUTPUT_DIR, 'rtconfig.h'), 'r', encoding='utf-8')
    else:     
        if BSP_Root:
            f = open(os.path.join(BSP_Root, 'rtconfig.h'), 'r', encoding='utf-8')
        else:
            f = open('rtconfig.h', 'r', encoding='utf-8')

    contents = f.read()
    f.close()
    PreProcessor.process_contents(contents)
    options = PreProcessor.cpp_namespace
    BuildOptions.update(options)
    
    ## TODO: not used for now
    board=os.getenv("BOARD")
    if GetDepend(['BSP_DEFAULT_CONFIG']):
        PreProcessor = PatchedPreProcessor()
        if GetDepend(['BF0_HCPU']):
            f = open(board+'/hcpu/board_config.h', 'r')
        else:
            f = open(board+'/lcpu/board_config.h', 'r')
        contents = f.read()
        f.close()
        PreProcessor.process_contents(contents)
        options = PreProcessor.cpp_namespace    
        BuildOptions.update(options)

def GetCurrentEnv():
    return Env

def GetEnvList():
    return EnvList

def PrintEnvList():
    print("========")
    print("Multi-Project Info")
    for env in EnvList:
        print("--------")
        print("{:<15} {}".format("full_name", env['full_name']))
        print("{:<15} {}".format("parent", env['parent']))        
        print("{:<15} {}".format("bsp_root", env['BSP_ROOT']))
        print("{:<15} {}".format("build_dir", env['build_dir']))
        print("{:<15} {}".format("link_script", env['LINK_SCRIPT_SRC']))
        if "PARTITION_TABLE" in env:
            ptab = env['PARTITION_TABLE']
        else:
            ptab = "None"    
        print("{:<15} {}".format("ptab", ptab)) 
        if IsChildProjEnv(env):
            print("{:<15} {}".format("embedded:", env['IMG_EMBEDDED']))
    print("========")
    
def IsChildProjEnv(env=None):
    if env is None:
        env = Env
    if 'is_child_proj' in env:
        return env['is_child_proj']
    else:
        return False

def IsEmbeddedProjEnv(env=None):
    if env is None:
        env = Env
    if 'IMG_EMBEDDED' in env:
        return env['IMG_EMBEDDED']
    else:
        return False

def AddExternalComponents(deps_file):
    conandeps = SConscript(deps_file)
    objs = []
    build_vdir = Env['build_dir']
    for path in conandeps["conandeps"]["LIBPATH"]:
        if os.path.isfile(os.path.join(path, 'SConscript')):
            pkg_name = os.path.normpath(path).split(os.sep)[-2]
            objs += SConscript(os.path.join(path, 'SConscript'), variant_dir=f"{build_vdir}/sf-pkgs/{pkg_name}", duplicate=0)
    return objs


def EnableInlineCommandDisplay(env):
    if GetOption('verbose'):
        return
    if env.get('_inline_command_display_enabled'):
        return
    if not getattr(sys.stdout, 'isatty', lambda: False)():
        return

    inline_prefixes = ('CC ', 'CXX ', 'AS ', 'AR ')
    stream = sys.stdout
    lock = threading.Lock()
    current_line_width = 0
    line_open = False

    def finish_current_line():
        nonlocal current_line_width, line_open

        with lock:
            if not line_open:
                return

            stream.write('\n')
            stream.flush()
            current_line_width = 0
            line_open = False

    def print_cmd_line(cmd, target, source, env):
        nonlocal current_line_width, line_open

        message = str(cmd)

        with lock:
            if message.startswith(inline_prefixes):
                padding = max(0, current_line_width - len(message))
                stream.write('\r{}{}'.format(message, ' ' * padding))
                stream.flush()
                current_line_width = len(message)
                line_open = True
                return

            if line_open:
                stream.write('\n')
                current_line_width = 0
                line_open = False

            stream.write(message + '\n')
            stream.flush()

    env['_inline_command_display_enabled'] = True
    env['PRINT_CMD_LINE_FUNC'] = print_cmd_line
    atexit.register(finish_current_line)



def PrepareBuilding(env, has_libcpu=False, remove_components=[], buildlib=None):
    import rtconfig
    import platform

    global BuildOptions
    global Projects
    global Env
    global Rtt_Root

    if env is None:
        if rtconfig.ARCH != "sim":
            env = Environment(tools = ['mingw'],
                AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
                CC = rtconfig.CC, CFLAGS = rtconfig.CFLAGS,
                CXX = rtconfig.CXX, CXXFLAGS = rtconfig.CXXFLAGS,
                AR = rtconfig.AR, ARFLAGS = '-rc',
                RANLIB = rtconfig.RANLIB,
                LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS)
            env.PrependENVPath('PATH', rtconfig.EXEC_PATH)
        else:
            env = Environment(TARGET_ARCH='x86',
                AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
                CC = rtconfig.CC, CFLAGS = rtconfig.CFLAGS,
                CXX = rtconfig.CXX, CXXFLAGS = rtconfig.CXXFLAGS,
                AR = rtconfig.AR, ARFLAGS = '',
                LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS)
            env.PrependENVPath('PATH', 'X:/bin/Hostx64/x64/')

    if 'name' not in env:
        env['name'] = 'main'
        env['full_name'] = 'main'
        env['parent'] = ''
    
    if hasattr(rtconfig, 'JLINK_DEVICE'):
        env['JLINK_DEVICE'] = rtconfig.JLINK_DEVICE
    if hasattr(rtconfig, 'LINK_SCRIPT_SRC'):
        env['LINK_SCRIPT_SRC'] = rtconfig.LINK_SCRIPT_SRC

    platform_name = platform.system()
    if platform_name == 'Windows':
        tool_suffix = '.exe'
    elif platform_name == 'Linux': 
        tool_suffix = '_linux'
    elif platform_name == 'Darwin':
        tool_suffix = '_mac'
    else:
        raise ValueError('Unsupported platform: {}'.format(platform_name))

    env['tool_suffix'] = tool_suffix

    # Clear projects for new project
    Projects = []

    if rtconfig.PLATFORM == 'cl':
        TARGET = 'rtthread-win32.' + rtconfig.TARGET_EXT

        libs = Split('''
        winmm
        kernel32
        gdi32
        winspool
        comdlg32
        advapi32
        shell32
        ole32
        oleaut32
        uuid
        odbc32
        odbccp32
        libucrtd
        ''')
        definitions = Split('''
        WIN32
        _DEBUG
        _CONSOLE
        MSVC
        ''')
        env.Append(CCFLAGS=rtconfig.CFLAGS)
        env.Append(LINKFLAGS=rtconfig.LFLAGS)
        env['LIBS']=libs
        env['CPPDEFINES']=definitions
    elif rtconfig.PLATFORM == 'mingw':
        libs = Split('''
            winmm
            kernel32
            gdi32
            winspool
            comdlg32
            advapi32
            shell32
            ole32
            oleaut32
            uuid
            odbc32
            odbccp32
            libucrtd
            ''')
        TARGET = 'rtthread-win32.' + rtconfig.TARGET_EXT
        env = Environment(tools = ['mingw'],
            AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
            CC = rtconfig.CC, CCFLAGS = rtconfig.CFLAGS,
            AR = rtconfig.AR, ARFLAGS = '-rc',
            LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS)
        env['LIBS']=libs
        env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

    # ===== Add option to SCons =====
    option_added = False
    try:
        AddOption('--dist',
                          dest = 'make-dist',
                          action = 'store_true',
                          default = False,
                          help = 'make distribution')

    except:
        option_added = True
        
    if not option_added:    
        
        AddOption('--dist-strip',
        
                          dest = 'make-dist-strip',
                          action = 'store_true',
                          default = False,
                          help = 'make distribution and strip useless files')
        AddOption('--cscope',
                          dest = 'cscope',
                          action = 'store_true',
                          default = False,
                          help = 'Build Cscope cross reference database. Requires cscope installed.')
        AddOption('--clang-analyzer',
                          dest = 'clang-analyzer',
                          action = 'store_true',
                          default = False,
                          help = 'Perform static analyze with Clang-analyzer. ' + \
                               'Requires Clang installed.\n' + \
                               'It is recommended to use with scan-build like this:\n' + \
                               '`scan-build scons --clang-analyzer`\n' + \
                               'If things goes well, scan-build will instruct you to invoke scan-view.')
        AddOption('--buildlib',
                          dest = 'buildlib',
                          type = 'string',
                          default=buildlib,
                          help = 'building library of a component')
        AddOption('--cleanlib',
                          dest = 'cleanlib',
                          action = 'store_true',
                          default = False,
                          help = 'clean up the library by --buildlib')
        AddOption('--target',
                          dest = 'target',
                          type = 'string',
                          help = 'set target project: mdk/mdk4/mdk5/iar/vs/vsc/ua/cdk/ses/makefile/eclipse/si/json')
        AddOption('--genconfig',
                    dest = 'genconfig',
                    action = 'store_true',
                    default = False,
                    help = 'Generate .config from rtconfig.h')
        AddOption('--useconfig',
                    dest = 'useconfig',
                    type = 'string',
                    help = 'make rtconfig.h from config file.')
        try:
            AddOption('--verbose',
                        dest = 'verbose',
                        action = 'store_true',
                        default = False,
                        help = 'print verbose information during build')
        except:
            pass
        AddOption('--no-rt',
                    dest = 'no_rt',
                    action = 'store_true',
                    default = False,
                    help = 'Do not include RT-Thread')
        AddOption('--no_cc',
                    dest = 'no_cc',
                    action = 'store_true',
                    default = False,
                    help = 'Do not compile')
  

    try:
        AddOption('--compiledb',
            dest = 'compiledb',
            action = 'store_true',
            default = False,
            help = 'Generate DB')   
    except:
        pass

    Env = env
    EnvList.append(env)

    (v_major, v_minor, v_rev) = env._get_major_minor_revision(SCons.__version__)
    
    if (v_major >= 4):
        # compilation database is supported since SCons 4.x  
        env.Tool('compilation_db')
        cdb = env.CompilationDatabase(os.path.join(rtconfig.OUTPUT_DIR, 'compile_commands.json'))
        Alias('cdb', cdb)

    # make an absolute root directory
    SIFLI_SDK=os.getenv('SIFLI_SDK')
    Rtt_Root = os.path.join(SIFLI_SDK, 'rtos/rtthread')

    # set RTT_ROOT in ENV
    Env['RTT_ROOT'] = Rtt_Root
    # set BSP_ROOT in ENV
    if 'BSP_ROOT' not in Env:
        Env['BSP_ROOT'] = Dir('#').abspath
    logging.debug("bsp root: {}".format(Env['BSP_ROOT']))

    sys.path = sys.path + [os.path.join(Rtt_Root, 'tools')]

    # {target_name:(CROSS_TOOL, PLATFORM)}
    tgt_dict = {'mdk':('keil', 'armcc'),
                'mdk4':('keil', 'armcc'),
                'mdk5':('keil', 'armcc'),
                'iar':('iar', 'iar'),
                'vs':('msvc', 'cl'),
                'vs2012':('msvc', 'cl'),
                'vsc' : ('gcc', 'gcc'),
                'cb':('keil', 'armcc'),
                'ua':('gcc', 'gcc'),
                'cdk':('gcc', 'gcc'),
                'makefile':('gcc', 'gcc'),
                'eclipse':('gcc', 'gcc'),
                'ses' : ('gcc', 'gcc')}
    tgt_name = GetOption('target')

    if tgt_name:
        # --target will change the toolchain settings which clang-analyzer is
        # depend on
        if GetOption('clang-analyzer'):
            logging.error ('--clang-analyzer cannot be used with --target')
            sys.exit(1)

        SetOption('no_exec', 1)
        
        #try:
        #    rtconfig.CROSS_TOOL, rtconfig.PLATFORM = tgt_dict[tgt_name]
            # replace the 'RTT_CC' to 'CROSS_TOOL'
        #    os.environ['RTT_CC'] = rtconfig.CROSS_TOOL
        #    utils.ReloadModule(rtconfig)
        #except KeyError:
        #    print ('Unknow target: '+ tgt_name+'. Avaible targets: ' +', '.join(tgt_dict.keys()))
        #    sys.exit(1)
    elif (GetDepend('RT_USING_NEWLIB') == False and GetDepend('RT_USING_NOLIBC') == False) \
        and rtconfig.PLATFORM == 'gcc':
        AddDepend('RT_USING_MINILIBC')


    # auto change the 'RTT_EXEC_PATH' when 'rtconfig.EXEC_PATH' get failed
    # if not os.path.exists(rtconfig.EXEC_PATH):
    #     if 'RTT_EXEC_PATH' in os.environ:
    #         # del the 'RTT_EXEC_PATH' and using the 'EXEC_PATH' setting on rtconfig.py
    #         del os.environ['RTT_EXEC_PATH']
    #         utils.ReloadModule(rtconfig)

    # add compability with Keil MDK 4.6 which changes the directory of armcc.exe
    if rtconfig.PLATFORM == 'armcc':
        # Solve windows command line limit issue
        if not GetOption('target'):    
            env["TEMPFILE"] = SCons.Platform.TempFileMunge
            env["LINKCOM"] = "${TEMPFILE('%s','$LINKCOMSTR')}"%env['LINKCOM']
            if hasattr(SCons.Platform.TempFileMunge, 'version'):
                env["CCCOM"] = "${TEMPFILE('%s','$CCCOMSTR')}"%env['CCCOM']
                env["CXXCOM"] = "${TEMPFILE('%s','$CXXCOMSTR')}"%env['CXXCOM']
        if not os.path.isfile(os.path.join(rtconfig.EXEC_PATH, 'armcc.exe')):
            if rtconfig.EXEC_PATH.find('bin40') > 0:
                rtconfig.EXEC_PATH = rtconfig.EXEC_PATH.replace('bin40', 'armcc/bin')
                Env['LINKFLAGS'] = Env['LINKFLAGS'].replace('RV31', 'armcc')

        # reset AR command flags
        env['ARCOM'] = '$AR --create $TARGET $SOURCES'
        env['LIBPREFIX'] = ''
        env['LIBSUFFIX'] = '.lib'
        env['LIBLINKPREFIX'] = ''
        env['LIBLINKSUFFIX'] = '.lib'
        env['LIBDIRPREFIX'] = '--userlibpath '
        env['TEMPFILEPREFIX'] = '--via='  

        # add cppdefine in linkflags        
        if 'CPPDEFINES' in env:
            predefines = ''
            for item in set(env['CPPDEFINES']):
                if '__FILE__' not in item:
                    predefines += ' --predefine="-D{}"'.format(item)            
            if 'LINKFLAGS' not in env:
                env['LINKFLAGS'] = ''
            env['LINKFLAGS'] += predefines
        

    elif rtconfig.PLATFORM == 'iar':
        env['LIBPREFIX'] = ''
        env['LIBSUFFIX'] = '.a'
        env['LIBLINKPREFIX'] = ''
        env['LIBLINKSUFFIX'] = '.a'
        env['LIBDIRPREFIX'] = '--search '
        
    elif rtconfig.PLATFORM == 'gcc':
        # Solve windows command line limit issue
        if not GetOption('target'):    
            def expand_sources(target, source, env, for_signature):
                slist = [str(a).replace('\\','\\\\') for a in source]
                return ' '.join(slist)

            def expand_target(target, source, env, for_signature):
                tlist = [str(a).replace('\\','\\\\') for a in target]
                return ' '.join(tlist)

            def expand_ldir(target, source, env, for_signature):
                if 'LIBPATH' in env and len(env['LIBPATH']) > 0:
                    slist = [str(a).replace('\\','\\\\') for a in env['LIBPATH']]
                    ldir = '-L' + ' -L'.join(slist)
                else:
                    ldir = ''

                return ldir

            env['EXPANDED_SOURCES'] = expand_sources
            env['EXPANDED_TARGETS'] = expand_target            
            env['EXPANDED_LDIR'] = expand_ldir
            env["TEMPFILE"] = SCons.Platform.TempFileMunge
            #env["LINKCOM"] = "${TEMPFILE('%s','$LINKCOMSTR')}"%env['LINKCOM']   
            env.setdefault('LINKFLAGS_POST', '') 
            env["LINKCOM"] = "${TEMPFILE('$LINK -o $EXPANDED_TARGETS $LINKFLAGS $__RPATH $EXPANDED_SOURCES $EXPANDED_LDIR -Wl,--start-group $LINKFLAGS_POST $_LIBFLAGS -Wl,--end-group ','$LINKCOMSTR')}"  
            #if hasattr(SCons.Platform.TempFileMunge, 'version'):
            #    env["CCCOM"] = "${TEMPFILE('%s','$CCCOMSTR')}"%env['CCCOM']
            #    env["CXXCOM"] = "${TEMPFILE('%s','$CXXCOMSTR')}"%env['CXXCOM']
        env['TEMPFILEPREFIX'] = '@' 
        
    # patch for win32 spawn
    if env['PLATFORM'] == 'win32':
        win32_spawn = Win32Spawn()
        win32_spawn.env = env
        env['SPAWN'] = win32_spawn.spawn

    EnableInlineCommandDisplay(env)

    if os.getenv("LEGACY_ENV"):
        # make Keil toolchain is availabe by subprocess
        if env['PLATFORM'] == 'win32':
            os.environ['PATH'] = rtconfig.EXEC_PATH + ";" + os.environ['PATH']
        else:
            os.environ['PATH'] = rtconfig.EXEC_PATH + ":" + os.environ['PATH']
    
    # add program path
    env.PrependENVPath('PATH', rtconfig.EXEC_PATH)
    # add rtconfig.h/BSP path into Kernel group
    # DefineGroup("Kernel", [], [], CPPPATH=[str(Dir('#').abspath)])
    # Dir('#') points to where SConstruct locates, so it cannot differentiate parent and child project root directory
    # So use 'BSP_ROOT' to indicate the actual root directory of parent and child project
    if GetBoardName():
        path = [rtconfig.OUTPUT_DIR]
        # board_path1, board_path2 = GetBoardPath(GetBoardName())
        # path += [board_path1] 
    else:
        path = [Env['BSP_ROOT']]

    DefineGroup("Kernel", [], [], CPPPATH=path)

    # register ptab tool
    env.Tool('ptab', toolpath=[os.path.dirname(__file__)])


    # add font converter builder
    font_convert_action = SCons.Action.Action(FontConvertBuild, 'ConvertFont $TARGET')
    bld = Builder(action = font_convert_action, suffix = '.c', src_suffix = '.ttf', emitter = ModifyFontConvertTargets)
    Env.Append(BUILDERS = {"FontConvert": bld})
    Env.AddMethod(ConvertFont, "ConvertFont")

    # add library build action
    act = SCons.Action.Action(BuildLibInstallAction, 'Install compiled library... $TARGET')
    bld = Builder(action = act)
    Env.Append(BUILDERS = {'BuildLib': bld})
    
    # add image builder
    img_file_action = SCons.Action.Action(ImgFileBuilder, 'GenImgFile $TARGET')
    bld = Builder(action = img_file_action, suffix = '.tmp.c', src_suffix = '.c')
    Env.Append(BUILDERS = {"ImgFile": bld})
    
    bin_file_action = SCons.Action.Action(BinFileBuilder, 'GenBinFile $TARGET')
    bin_bld = Builder(action = bin_file_action)
    Env.Append(BUILDERS = {"BinFile": bin_bld})

    Env.AddMethod(ImgResource, "ImgResource")

    # add font builder
    font_file_action = SCons.Action.Action(FontFileBuild, 'GenFontFile $TARGET')
    bld = Builder(action = font_file_action, suffix = '.c', src_suffix = '.ttf', prefix = 'lvsf_font_', emitter = ModifyFontTargets)
    Env.Append(BUILDERS = {"FontFile": bld})

    # add lang builder
    lang_action = SCons.Action.Action(LangBuild, 'Generating langpack ...')
    bld = Builder(action = lang_action, src_suffix = '.json', emitter = ModifyLangTargets)
    Env.Append(BUILDERS = {"Lang": bld})

    # /*--------- Builtin/External Language-Pack Extension Begin ---------*/
    # add workbook-driven lang builder
    lang_excel_action = SCons.Action.Action(LangExcelBuild, 'Generating langpack from xlsx ...')
    bld = Builder(action = lang_excel_action, src_suffix = '.xlsx', emitter = ModifyLangExcelTargets)
    Env.Append(BUILDERS = {"LangExcel": bld})

    # add external language-pack builder
    ext_lang_action = SCons.Action.Action(ExternalLangPackBuild, 'Generating external langpack ...')
    bld = Builder(action = ext_lang_action, src_suffix = '.json', emitter = ModifyExternalLangPackTargets)
    Env.Append(BUILDERS = {"ExternalLangPack": bld})

    # add workbook-driven external language-pack builder
    ext_lang_excel_action = SCons.Action.Action(ExternalLangPackExcelBuild, 'Generating external langpack from xlsx ...')
    bld = Builder(action = ext_lang_excel_action, src_suffix = '.xlsx', emitter = ModifyExternalLangPackExcelTargets)
    Env.Append(BUILDERS = {"ExternalLangPackExcel": bld})
    # /*---------- Builtin/External Language-Pack Extension End ----------*/
    
    # add rom library builder
    rom_lib_action = SCons.Action.Action(RomLibBuild, 'GenRomLib $TARGET')
    bld = Builder(action = rom_lib_action, suffix = env['LIBSUFFIX'], src_suffix = '.sym')
    Env.Append(BUILDERS = {"RomLib": bld})    
    
    # add ProgramBinary builder
    bin_action = SCons.Action.Action(ProgramBinaryBuild, 'Generating $TARGET ...')
    bld = Builder(action = bin_action, suffix = '.bin', emitter = ModifyProgramBinaryTargets)
    Env.Append(BUILDERS = {"ProgramBinary": bld})
    
    # add ProgramHex builder
    hex_action = SCons.Action.Action(ProgramHexBuild, 'Generating $TARGET ...')
    bld = Builder(action = hex_action, suffix = '.hex', emitter = ModifyProgramHexTargets)
    Env.Append(BUILDERS = {"ProgramHex": bld})

    # add ProgramAsm builder
    asm_action = SCons.Action.Action(ProgramAsmBuild, 'Generating $TARGET ...')
    bld = Builder(action = asm_action, suffix = '.asm')
    Env.Append(BUILDERS = {"ProgramAsm": bld})

    # add EmbeddedImgCFile builder
    img_cfile_action = SCons.Action.Action(EmbeddedImgCFileBuild, 'Generating $TARGET ...')
    bld = Builder(action = img_cfile_action, suffix = '.c')
    Env.Append(BUILDERS = {"EmbeddedCFile": bld})

    # add FtabCFile builder
    ftab_cfile_action = SCons.Action.Action(FtabCFileBuild, 'Generating $TARGET ...')
    bld = Builder(action = ftab_cfile_action, suffix = '.c')
    Env.Append(BUILDERS = {"FtabCFile": bld})

    # add FtabBin builder (for ptab v3 - direct binary generation)
    ftab_bin_action = SCons.Action.Action(FtabBinBuild, 'Generating $TARGET ...')
    bld = Builder(action = ftab_bin_action, suffix = '.bin')
    Env.Append(BUILDERS = {"FtabBin": bld})

    # add DownloadScript builder
    download_script_action = SCons.Action.Action(DownloadScriptBuild, 'Generating $TARGET ...')
    bld = Builder(action = download_script_action, emitter = ModifyDownloadScriptTargets)
    Env.Append(BUILDERS = {"DownloadScript": bld})
    
    file_copy_action = SCons.Action.Action(FileCopyBuild, 'Generating $TARGET ...')
    bld = Builder(action = file_copy_action, emitter = ModifyFileCopyTargets) #ModifyFileCopyTargets
    Env.Append(BUILDERS = {"CopyFile": bld})    

    # add lds builder
    lds_action = SCons.Action.Action(LdsBuild, 'Generating $TARGET ...')
    bld = Builder(action = lds_action, emitter = ModifyLdsTargets)
    Env.Append(BUILDERS = {"LdsFile": bld})

    # add file system builder
    fs_action = SCons.Action.Action(FsBuild, 'Generating $TARGET ...')
    bld = Builder(action = fs_action)
    Env.Append(BUILDERS = {"FileSystem": bld})

    # parse rtconfig.h to get used component
    BuildOptions={}
    BuildOptionUpdate(BuildOptions,Env['BSP_ROOT'])
    
    if GetOption('clang-analyzer'):
        # perform what scan-build does
        env.Replace(
                CC   = 'ccc-analyzer',
                CXX  = 'c++-analyzer',
                # skip as and link
                LINK = 'true',
                AS   = 'true',)
        env["ENV"].update(x for x in os.environ.items() if x[0].startswith("CCC_"))
        # only check, don't compile. ccc-analyzer use CCC_CC as the CC.
        # fsyntax-only will give us some additional warning messages
        env['ENV']['CCC_CC']  = 'clang'
        env.Append(CFLAGS=['-fsyntax-only', '-Wall', '-Wno-invalid-source-encoding'])
        env['ENV']['CCC_CXX'] = 'clang++'
        env.Append(CXXFLAGS=['-fsyntax-only', '-Wall', '-Wno-invalid-source-encoding'])
        # remove the POST_ACTION as it will cause meaningless errors(file not
        # found or something like that).
        rtconfig.POST_ACTION = ''

    if GetOption('compiledb'):
        pass
        # env.Replace(
        # CC   = 'clang',
        # CXX  = 'clang++',
        # # skip as and link
        # LINK = 'true',
        # AS   = 'true',)    

    # generate cconfig.h file
    GenCconfigFile(env, BuildOptions)

    # auto append '_REENT_SMALL' when using newlib 'nano.specs' option
    if rtconfig.PLATFORM == 'gcc' and str(env['LINKFLAGS']).find('nano.specs') != -1:
        env.AppendUnique(CPPDEFINES = ['_REENT_SMALL'])

    if GetOption('genconfig'):
        from genconf import genconfig
        genconfig()
        exit(0)

    if not option_added:
        AddOption('--menuconfig',
                    dest = 'menuconfig',
                    action = 'store_true',
                    default = False,
                    help = 'make menuconfig for RT-Thread BSP')
    if GetOption('menuconfig'):
        board = f"--board={GetOption('board')}"
        global BOARD_SEARCH_PATH
        board_search_path = f"--board_search_path={BOARD_SEARCH_PATH}" if 'BOARD_SEARCH_PATH' in globals() else ''
        subprocess.run([sys.executable, os.path.join(SIFLI_SDK, 'tools',"kconfig" , 'menuconfig.py'), board, board_search_path], check=True)
        exit(0)

    if not option_added:
        AddOption('--pyconfig',
                    dest = 'pyconfig',
                    action = 'store_true',
                    default = False,
                    help = 'make menuconfig for RT-Thread BSP')
        AddOption('--pyconfig-silent',
                    dest = 'pyconfig_silent',
                    action = 'store_true',
                    default = False,
                    help = 'Don`t show pyconfig window')

    if GetOption('pyconfig_silent'):    
        from menuconfig import pyconfig_silent

        pyconfig_silent(Rtt_Root)
        exit(0)
    elif GetOption('pyconfig'):
        from menuconfig import pyconfig

        pyconfig(Rtt_Root)
        exit(0)

    configfn = GetOption('useconfig')
    if configfn:
        from menuconfig import mk_rtconfig
        mk_rtconfig(configfn)
        exit(0)


    if not GetOption('verbose'):
        # override the default verbose command string
        env.Replace(
            ARCOMSTR = 'AR $TARGET',
            ASCOMSTR = 'AS $TARGET',
            ASPPCOMSTR = 'AS $TARGET',
            CCCOMSTR = 'CC $TARGET',
            CXXCOMSTR = 'CXX $TARGET',
            LINKCOMSTR = 'LINK $TARGET'
        )

    # fix the linker for C++
    if GetDepend('RT_USING_CPLUSPLUS') or GetDepend('USING_CPLUSPLUS'):
        if env['LINK'].find('gcc') != -1:
            env['LINK'] = env['LINK'].replace('gcc', 'g++')

    # we need to seperate the variant_dir for BSPs and the kernels. BSPs could
    # have their own components etc. If they point to the same folder, SCons
    # would find the wrong source code to compile.
    if 'build_dir' in env:
        bsp_vdir = env['build_dir']
    else:
        bsp_vdir = rtconfig.OUTPUT_DIR  #'build'
        env['build_dir'] = bsp_vdir
    logging.debug('bsp_vdir: {}'.format(bsp_vdir))
    env['build_dir'] = bsp_vdir
    env['BUILD_DIR_FULL_PATH'] = os.path.abspath(env['build_dir'])

    kernel_vdir = os.path.join(bsp_vdir, 'sifli_sdk/rtos/kernel')    

    # board build script
    objs = SConscript(os.path.join(Env['BSP_ROOT'], 'SConscript'), variant_dir=bsp_vdir, duplicate=0)

    # embed child binary in parent project
    if len(ChildProjList) > 0:
        t = []
        for child_proj in ChildProjList:
            if child_proj['parent'] == env['full_name']:
                cfile_path = os.path.join(bsp_vdir, os.path.join(child_proj['name'], child_proj['name']))
                t += Env.EmbeddedCFile(cfile_path, child_proj['binary'])
        objs += DefineGroup('child_proj', t, depend = [])
    
    if GetOption('no_rt') or not GetDepend(['BSP_USING_RTTHREAD']):    
        logging.debug("No rtthread included in build")
    else:
        # include kernel

        objs.extend(SConscript(Rtt_Root + '/src/SConscript', variant_dir=kernel_vdir + '/src', duplicate=0))
        # include libcpu
        if not has_libcpu:
            objs.extend(SConscript(Rtt_Root + '/libcpu/SConscript',
                        variant_dir=kernel_vdir + '/libcpu', duplicate=0))

        # include components
        objs.extend(SConscript(Rtt_Root + '/components/SConscript',
                               variant_dir=kernel_vdir + '/components',
                               duplicate=0,
                               exports='remove_components'))

    # Add rt-thread online packages
    if os.path.isfile(os.path.join(Env['BSP_ROOT'], 'packages/SConscript')):
        objs.extend(SConscript(os.path.join(Env['BSP_ROOT'], 'packages/SConscript'), variant_dir=bsp_vdir + '/rt-pkgs', duplicate=0))

    # Add external components
    if os.path.isfile(os.path.join(Env['BSP_ROOT'], 'sf-pkgs/SConscript_conandeps')):
        objs.extend(AddExternalComponents(os.path.join(Env['BSP_ROOT'], 'sf-pkgs/SConscript_conandeps')))


    if IsChildProjEnv(env) and 'CPPPATH' in env:
        child_cpppath = env['CPPPATH']
        if not isinstance(child_cpppath, (list, tuple)):
            child_cpppath = [child_cpppath]
        parent_root = os.path.abspath(str(Dir('#')))
        child_cpppath_abs = [os.path.abspath(str(path)) for path in child_cpppath]
        if parent_root in child_cpppath_abs:
            logging.debug('Root path of parent project: {}'.format(parent_root))
            logging.debug('Search paths of child project: {}'.format(env['CPPPATH']))
            raise ValueError('Root path of parent project cannot be in search paths of child project')

    return objs


# get custom_mem_map.h source file path
def GetCustomMemMapSrc(bsp_root, build_dir, chip, board):
    file_path = os.path.join(bsp_root, board)
    custom_mem_map_file_name = 'custom_mem_map.h'
    file_path = os.path.join(file_path, custom_mem_map_file_name)
    if os.path.exists(file_path):
        return file_path

    file_path = os.path.join(bsp_root, chip)
    custom_mem_map_file_name = 'custom_mem_map.h'
    file_path = os.path.join(file_path, custom_mem_map_file_name)
    if os.path.exists(file_path):
        return file_path        

    file_path = os.path.join(bsp_root, custom_mem_map_file_name)
    if os.path.exists(file_path):
        return file_path            
      
    path1, board_path = GetBoardPath(board)
    file_path = os.path.join(board_path, custom_mem_map_file_name)
    if os.path.exists(file_path):
        return file_path

    return None


def InitBuild(bsp_root, build_dir, board):
    import rtconfig

    if not os.path.exists(build_dir):
       os.makedirs(build_dir)

    # create Kconfig
    s = ''
    s += 'source "$SIFLI_SDK/Kconfig.v2"\n'
    s += 'source "$SIFLI_SDK/customer/boards/Kconfig.v2"\n'
    path1, board_path = GetBoardPath(board)
    board_path += "/Kconfig.board"
    s += 'source "{}"\n'.format(board_path)
    if not bsp_root:
        bsp_root = Dir('#').abspath

    # create .config and rtconfig.h    
    # kconfiglib doesn't recognize backslash
    bsp_root = bsp_root.replace('\\', '/')
    s += 'osource "{}/sf-pkgs/Kconfig.conandeps"\n'.format(bsp_root)
    s += 'source "{}/Kconfig.proj"'.format(bsp_root)
    f = open(os.path.join(build_dir, 'Kconfig'), 'w')
    try:
        f.write(s)
    finally:
        f.close()

    SIFLI_SDK = os.getenv('SIFLI_SDK')
    KCONFIG_PATH = os.path.join(SIFLI_SDK, "tools/kconfig/kconfig.py")


    board_path = board_path.replace("$SIFLI_SDK", SIFLI_SDK)
    board_path = os.path.dirname(board_path)   
    
    # Use command line bconf 
    board_conf=GetOption('bconf')
    if board_conf=='board.conf':
        try:
            import proj
            if hasattr(proj,'BCONF'):
                board_conf=proj.BCONF
        except:
            pass
    
    if not os.path.isfile(os.path.join(board_path,board_conf)):
        logging.debug(os.path.join(board_path,board_conf)+ ' does not exist, use board.conf')
        board_conf=os.path.join(board_path, 'board.conf')
    else:
        board_conf=os.path.join(board_path, board_conf)
    conf_list = [ board_conf, 
                 os.path.join(bsp_root, 'proj.conf')]

    # Add chip specific config
    proj_chip_conf = os.path.join(bsp_root, rtconfig.CHIP.lower() + '/' + 'proj.conf')
    if os.path.exists(proj_chip_conf):
        conf_list += [proj_chip_conf]

    # Add board specific config
    proj_board_conf = os.path.join(bsp_root, board + '/' + 'proj.conf')             
    if os.path.exists(proj_board_conf):
        conf_list += [proj_board_conf]

    # Remove rtconfig.h to avoid read error as the file is in encrypted state and cannot be read correctly in some environment
    # if os.path.isfile(os.path.join(build_dir, "rtconfig.h")):
    #    os.remove(os.path.join(build_dir, "rtconfig.h"))

    if (is_verbose()):
        retcode = subprocess.call(['python', KCONFIG_PATH, '--handwritten-input-configs', '--verbose', os.path.join(build_dir, 'Kconfig'),
                         os.path.join(build_dir, '.config'), os.path.join(build_dir, "rtconfig.h"), 
                         os.path.join(build_dir, "kconfiglist")] + conf_list)
    else:
        retcode = subprocess.call(['python', KCONFIG_PATH, '--handwritten-input-configs', os.path.join(build_dir, 'Kconfig'),
                         os.path.join(build_dir, '.config'), os.path.join(build_dir, "rtconfig.h"),
                         os.path.join(build_dir, "kconfiglist")] + conf_list)
    assert retcode == 0, "Fail to generate .config and rtconfig.h"

    if os.path.isfile('rtconfig_project.h'):
        shutil.copy('rtconfig_project.h', os.path.join(build_dir, "rtconfig_project.h"))

    src = GetCustomMemMapSrc(bsp_root, build_dir, rtconfig.CHIP.lower(), board)
    if src:
        logging.debug("Copy custom_mem_map.h")
        logging.debug(" from {}".format(src))
        logging.debug(" to   {}".format(build_dir))
        shutil.copy(src, build_dir)


def PrepareModuleBuilding(env, root_directory, bsp_directory):
    import rtconfig
    import platform

    global BuildOptions
    global Env
    global Rtt_Root

    # patch for win32 spawn
    if env['PLATFORM'] == 'win32':
        win32_spawn = Win32Spawn()
        win32_spawn.env = env
        env['SPAWN'] = win32_spawn.spawn

    EnableInlineCommandDisplay(env)

    Env = env
    Rtt_Root = root_directory

    # parse bsp rtconfig.h to get used component
    PreProcessor = PatchedPreProcessor()
    f = open(bsp_directory + '/rtconfig.h', 'r', encoding='utf-8')
    contents = f.read()
    f.close()
    PreProcessor.process_contents(contents)
    BuildOptions = PreProcessor.cpp_namespace

    # add build/clean library option for library checking
    try:
        AddOption('--buildlib',
                dest='buildlib',
                type='string',
                help='building library of a component')
        AddOption('--cleanlib',
                dest='cleanlib',
                action='store_true',
                default=False,
                help='clean up the library by --buildlib')
        AddOption('--no_cc',
                dest = 'no_cc',
                action = 'store_true',
                default = False,
                help = 'Do not compile')
    except:
        pass

    platform_name = platform.system()
    if platform_name == 'Windows':
        tool_suffix = '.exe'
    elif platform_name == 'Linux': 
        tool_suffix = '_linux'
    elif platform_name == 'Darwin':
        tool_suffix = '_mac'
    else:
        raise ValueError('Unsupported platform: {}'.format(platform_name))

    env['tool_suffix'] = tool_suffix

    # add program path
    env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

    # add image builder
    img_file_action = SCons.Action.Action(ImgFileBuilder, 'GenImgFile $TARGET')
    bld = Builder(action = img_file_action, suffix = '.c', src_suffix = '.png')
    Env.Append(BUILDERS = {"ImgFile": bld})
    Env.AddMethod(ImgResource, "ImgResource")

    # add font builder
    font_file_action = SCons.Action.Action(FontFileBuild, 'GenFontFile $TARGET')
    bld = Builder(action = font_file_action, suffix = '.c', src_suffix = '.ttf', prefix = 'lvsf_font_', emitter = ModifyFontTargets)
    Env.Append(BUILDERS = {"FontFile": bld})

    # add lang builder
    lang_action = SCons.Action.Action(LangBuild, 'Generating langpack ...')
    bld = Builder(action = lang_action, src_suffix = '.json', emitter = ModifyLangTargets)
    Env.Append(BUILDERS = {"Lang": bld})

    # /*--------- Builtin/External Language-Pack Extension Begin ---------*/
    # add workbook-driven lang builder
    lang_excel_action = SCons.Action.Action(LangExcelBuild, 'Generating langpack from xlsx ...')
    bld = Builder(action = lang_excel_action, src_suffix = '.xlsx', emitter = ModifyLangExcelTargets)
    Env.Append(BUILDERS = {"LangExcel": bld})

    # add external language-pack builder
    ext_lang_action = SCons.Action.Action(ExternalLangPackBuild, 'Generating external langpack ...')
    bld = Builder(action = ext_lang_action, src_suffix = '.json', emitter = ModifyExternalLangPackTargets)
    Env.Append(BUILDERS = {"ExternalLangPack": bld})

    # add workbook-driven external language-pack builder
    ext_lang_excel_action = SCons.Action.Action(ExternalLangPackExcelBuild, 'Generating external langpack from xlsx ...')
    bld = Builder(action = ext_lang_excel_action, src_suffix = '.xlsx', emitter = ModifyExternalLangPackExcelTargets)
    Env.Append(BUILDERS = {"ExternalLangPackExcel": bld})
    # /*---------- Builtin/External Language-Pack Extension End ----------*/

def GetConfigValue(name):
    assert type(name) == str, 'GetConfigValue: only string parameter is valid'
    try:
        return BuildOptions[name]
    except:
        return ''

def GetDepend(depend):
    building = True
    if type(depend) == type('str'):
        if not depend in BuildOptions or BuildOptions[depend] == 0:
            building = False
        elif BuildOptions[depend] != '':
            return BuildOptions[depend]

        return building

    # for list type depend
    for item in depend:
        if item != '':
            if not item in BuildOptions or BuildOptions[item] == 0:
                building = False

    return building

def LocalOptions(config_filename):
    from SCons.Script import SCons

    # parse wiced_config.h to get used component
    PreProcessor = SCons.cpp.PreProcessor()

    f = open(config_filename, 'r', encoding='utf-8')
    contents = f.read()
    f.close()

    PreProcessor.process_contents(contents)
    local_options = PreProcessor.cpp_namespace

    return local_options

def GetLocalDepend(options, depend):
    building = True
    if type(depend) == type('str'):
        if not depend in options or options[depend] == 0:
            building = False
        elif options[depend] != '':
            return options[depend]

        return building

    # for list type depend
    for item in depend:
        if item != '':
            if not item in options or options[item] == 0:
                building = False

    return building

def AddDepend(option):
    BuildOptions[option] = 1

def MergeGroup(src_group, group):
    src_group['src'] = src_group['src'] + group['src']
    if 'CCFLAGS' in group:
        if 'CCFLAGS' in src_group:
            src_group['CCFLAGS'] = src_group['CCFLAGS'] + group['CCFLAGS']
        else:
            src_group['CCFLAGS'] = group['CCFLAGS']
    if 'CPPPATH' in group:
        if 'CPPPATH' in src_group:
            src_group['CPPPATH'] = src_group['CPPPATH'] + group['CPPPATH']
        else:
            src_group['CPPPATH'] = group['CPPPATH']
    if 'CPPDEFINES' in group:
        if 'CPPDEFINES' in src_group:
            src_group['CPPDEFINES'] = src_group['CPPDEFINES'] + group['CPPDEFINES']
        else:
            src_group['CPPDEFINES'] = group['CPPDEFINES']
    if 'ASFLAGS' in group:
        if 'ASFLAGS' in src_group:
            src_group['ASFLAGS'] = src_group['ASFLAGS'] + group['ASFLAGS']
        else:
            src_group['ASFLAGS'] = group['ASFLAGS']

    # for local CCFLAGS/CPPPATH/CPPDEFINES
    if 'LOCAL_CCFLAGS' in group:
        if 'LOCAL_CCFLAGS' in src_group:
            src_group['LOCAL_CCFLAGS'] = src_group['LOCAL_CCFLAGS'] + group['LOCAL_CCFLAGS']
        else:
            src_group['LOCAL_CCFLAGS'] = group['LOCAL_CCFLAGS']
    if 'LOCAL_CPPPATH' in group:
        if 'LOCAL_CPPPATH' in src_group:
            src_group['LOCAL_CPPPATH'] = src_group['LOCAL_CPPPATH'] + group['LOCAL_CPPPATH']
        else:
            src_group['LOCAL_CPPPATH'] = group['LOCAL_CPPPATH']
    if 'LOCAL_CPPDEFINES' in group:
        if 'LOCAL_CPPDEFINES' in src_group:
            src_group['LOCAL_CPPDEFINES'] = src_group['LOCAL_CPPDEFINES'] + group['LOCAL_CPPDEFINES']
        else:
            src_group['LOCAL_CPPDEFINES'] = group['LOCAL_CPPDEFINES']

    if 'LINKFLAGS' in group:
        if 'LINKFLAGS' in src_group:
            src_group['LINKFLAGS'] = src_group['LINKFLAGS'] + group['LINKFLAGS']
        else:
            src_group['LINKFLAGS'] = group['LINKFLAGS']
    if 'LIBS' in group:
        if 'LIBS' in src_group:
            src_group['LIBS'] = src_group['LIBS'] + group['LIBS']
        else:
            src_group['LIBS'] = group['LIBS']
    if 'LIBPATH' in group:
        if 'LIBPATH' in src_group:
            src_group['LIBPATH'] = src_group['LIBPATH'] + group['LIBPATH']
        else:
            src_group['LIBPATH'] = group['LIBPATH']
    if 'LOCAL_ASFLAGS' in group:
        if 'LOCAL_ASFLAGS' in src_group:
            src_group['LOCAL_ASFLAGS'] = src_group['LOCAL_ASFLAGS'] + group['LOCAL_ASFLAGS']
        else:
            src_group['LOCAL_ASFLAGS'] = group['LOCAL_ASFLAGS']

def DefineGroup(name, src, depend, **parameters):
    global Env
    if not GetDepend(depend):
        return []

    # find exist group and get path of group
    group_path = ''
    for g in Projects:
        if g['name'] == name:
            group_path = g['path']
    if group_path == '':
        group_path = GetCurrentDir()

    group = parameters
    group['name'] = name
    group['path'] = group_path
    if type(src) == type([]):
        group['src'] = File(src)
    else:
        group['src'] = src

    if 'CCFLAGS' in group:
        # when CFLAGS and CCFLAGS are concatenated, no preceding space in CCFLAGS is allowed
        old_value = "{}".format(Env['CCFLAGS'])
        if '' != old_value:
            Env.AppendUnique(CCFLAGS = group['CCFLAGS'])
        else:
            Env.AppendUnique(CCFLAGS = group['CCFLAGS'].strip())
    if 'CPPPATH' in group:
        paths = []
        for item in group['CPPPATH']:
            paths.append(os.path.abspath(item))
        group['CPPPATH'] = paths
        Env.AppendUnique(CPPPATH = group['CPPPATH'])
    if 'CPPDEFINES' in group:
        Env.AppendUnique(CPPDEFINES = group['CPPDEFINES'])

        import rtconfig
        # TODO, other toolchain?
        if rtconfig.PLATFORM == 'armcc':
            # add cppdefine in linkflags
            if 'LINKFLAGS' not in group:
                group['LINKFLAGS'] = ''
            
            predefines = ''
            for item in set(group['CPPDEFINES']):
                if '__FILE__' not in item:
                    predefines += ' --predefine="-D{}"'.format(item)            
        
            group['LINKFLAGS'] += predefines    
        
    if 'LINKFLAGS' in group:
        Env.AppendUnique(LINKFLAGS = group['LINKFLAGS'])
        
    if 'ASFLAGS' in group:
        Env.AppendUnique(ASFLAGS = group['ASFLAGS'])
    if 'LOCAL_CPPPATH' in group:
        paths = []
        for item in group['LOCAL_CPPPATH']:
            paths.append(os.path.abspath(item))
        group['LOCAL_CPPPATH'] = paths

    import rtconfig
    if rtconfig.PLATFORM == 'gcc':
        if 'CCFLAGS' in group:
            group['CCFLAGS'] = utils.GCCC99Patch(group['CCFLAGS'])
        if 'LOCAL_CCFLAGS' in group:
            group['LOCAL_CCFLAGS'] = utils.GCCC99Patch(group['LOCAL_CCFLAGS'])

    # check whether to clean up library
    if GetOption('cleanlib') and os.path.exists(os.path.join(group['path'], GroupLibFullName(name, Env))):
        if group['src'] != []:
            logging.debug ('Remove library:'+ GroupLibFullName(name, Env))
            fn = os.path.join(group['path'], GroupLibFullName(name, Env))
            if os.path.exists(fn):
                os.unlink(fn)

    if 'LIBS' in group:
        Env.AppendUnique(LIBS = group['LIBS'])
    if 'LIBPATH' in group:
        Env.AppendUnique(LIBPATH = group['LIBPATH'])

    # check whether to build group library
    if 'LIBRARY' in group:
        objs = Env.Library(name, group['src'])
    else:
        # only add source
        objs = group['src']

    if 'INSTALL_PATH' in group:
        group['path']=group['INSTALL_PATH']

    # merge group
    for g in Projects:
        if g['name'] == name:
            # merge to this group
            MergeGroup(g, group)
            return objs

    # add a new group
    Projects.append(group)

    return objs

def GetCurrentDir():
    conscript = File('SConscript')
    fn = conscript.rfile()
    name = fn.name
    path = os.path.dirname(fn.abspath)
    return path

PREBUILDING = []
def RegisterPreBuildingAction(act):
    global PREBUILDING
    assert callable(act), 'Could only register callable objects. %s received' % repr(act)
    PREBUILDING.append(act)

def PreBuilding():
    global PREBUILDING
    for a in PREBUILDING:
        a()

def GroupLibName(name, env=None):
    import rtconfig
    if rtconfig.PLATFORM == 'armcc':
        return name + '_rvds'
    elif rtconfig.PLATFORM == 'gcc':
        return name + '_gcc'
    elif rtconfig.PLATFORM == 'cl':
        return name + '_msvc'

    return name

def GroupLibFullName(name, env):
    s = GroupLibName(name, env)
    if not s.startswith(env['LIBPREFIX']):
        s = env['LIBPREFIX'] + s
    return s + env['LIBSUFFIX']

def BuildLibInstallAction(target, source, env):
    import rtconfig
    lib_name = GetOption('buildlib')
    for Group in Projects:
        if Group['name'] == lib_name:
            lib_name = GroupLibFullName(Group['name'], env)
            dst_name = os.path.join(Group['path'], lib_name)
            lib_name = os.path.join(os.path.dirname(str(source[0])), lib_name)
            logging.info ('Copy '+lib_name+' => ' + dst_name)
            do_copy_file(lib_name, dst_name)
            try:
                os.system(rtconfig.POST_ACTION)
            except:
                pass
            #os.system("postbuild.bat")
            break

def DoBuilding(target, objects):
    import rtconfig

    # merge all objects into one list
    def one_list(l):
        lst = []
        for item in l:
            if type(item) == type([]):
                lst += one_list(item)
            else:
                lst.append(item)
        return lst

    # handle local group
    def local_group(group, objects):
        if 'LOCAL_CCFLAGS' in group or 'LOCAL_CPPPATH' in group or 'LOCAL_CPPDEFINES' in group or 'LOCAL_ASFLAGS' in group:
            CCFLAGS = Env.get('CCFLAGS', '') + group.get('LOCAL_CCFLAGS', '')
            CPPPATH = Env.get('CPPPATH', ['']) + group.get('LOCAL_CPPPATH', [''])
            CPPDEFINES = list(Env.get('CPPDEFINES', [''])) + group.get('LOCAL_CPPDEFINES', [''])
            ASFLAGS = Env.get('ASFLAGS', '') + group.get('LOCAL_ASFLAGS', '')

            for source in group['src']:
                objects.append(Env.Object(source, CCFLAGS = CCFLAGS, ASFLAGS = ASFLAGS,
                    CPPPATH = CPPPATH, CPPDEFINES = CPPDEFINES))

            return True

        return False

    objects = one_list(objects)
    program = None

    if rtconfig.CROSS_TOOL == 'keil':
        for group in Projects:
            if 'Kernel' == group['name']:
                # only add cpppath of Kernel group to get the path of `rtconfig.h`, avoid too long command line parameters
                asm_include_paths = ' ' + ' '.join(['-I{}'.format(path.replace('\\', '/')) for path in group['CPPPATH']]) + ' '
                Env.AppendUnique(ASFLAGS = asm_include_paths)
                break
        
    # check whether special buildlib option
    lib_name = GetOption('buildlib')
    if lib_name:
        objects = [] # remove all of objects
        # build library with special component
        for Group in Projects:
            if Group['name'] == lib_name:
                lib_name = GroupLibName(Group['name'], Env)
                if not local_group(Group, objects):
                    objects = Env.Object(Group['src'])

                lib_name = os.path.join(os.path.dirname(target), lib_name)
                program = Env.Library(lib_name, objects)

                # add library copy action
                Env.BuildLib(lib_name, program)

                break
    else:
        # remove source files with local flags setting
        for group in Projects:
            if 'LOCAL_CCFLAGS' in group or 'LOCAL_CPPPATH' in group or 'LOCAL_CPPDEFINES' in group:
                for source in group['src']:
                    for obj in objects:
                        if source.abspath == obj.abspath or (len(obj.sources) > 0 and source.abspath == obj.sources[0].abspath):
                            objects.remove(obj)

        try:
            patch = GetOption('patch')
        except:
            patch = None
        if (patch==None):
            # re-add the source files to the objects
            for group in Projects:
                local_group(group, objects)
        program = Env.Program(target, objects)

    import rtconfig
    try:
        logging.debug(rtconfig.PRE_ACTION)
        Env.AddPreAction(target, rtconfig.PRE_ACTION)
    except:
        logging.debug("No pre action")
    EndBuilding(target, program)

def GenTargetProject(program = None):

    if GetOption('target') == 'mdk':
        from keil import MDKProject
        from keil import MDK4Project
        from keil import MDK5Project

        template = os.path.isfile('template.Uv2')
        if template:
            MDKProject('project.Uv2', Projects)
        else:
            template = os.path.isfile('template.uvproj')
            if template:
                MDK4Project('project.uvproj', Projects)
            else:
                template = os.path.isfile('template.uvprojx')
                if template:
                    MDK5Project('project.uvprojx', Projects)
                else:
                    logging.warning ('No template project file found.')

    if GetOption('target') == 'mdk4':
        from keil import MDK4Project
        MDK4Project('project.uvproj', Projects)

    if GetOption('target') == 'mdk5':
        from keil import MDK5Project
        MDK5Project('project.uvprojx', Projects)

    if GetOption('target') == 'iar':
        from iar import IARProject
        IARProject('project.ewp', Projects)

    if GetOption('target') == 'vs':
        from vs import VSProject
        VSProject('project.vcproj', Projects, program)

    if GetOption('target') == 'vs2012':
        from vs2012 import VS2012Project
        VS2012Project('project.vcxproj', Projects, program)

    if GetOption('target') == 'vs2017':
        from vs2017 import VS2017Project
        VS2017Project('project.vcxproj', Projects, program)

    if GetOption('target') == 'cb':
        from codeblocks import CBProject
        CBProject('project.cbp', Projects, program)

    if GetOption('target') == 'ua':
        from ua import PrepareUA
        PrepareUA(Projects, Rtt_Root, str(Dir('#')))

    if GetOption('target') == 'vsc':
        from vsc import GenerateVSCode
        GenerateVSCode(Env)

    if GetOption('target') == 'cdk':
        from cdk import CDKProject
        CDKProject('project.cdkproj', Projects)

    if GetOption('target') == 'ses':
        from ses import SESProject
        SESProject(Env)

    if GetOption('target') == 'makefile':
        from makefile import TargetMakefile
        TargetMakefile(Env)

    if GetOption('target') == 'eclipse':
        from eclipse import TargetEclipse
        TargetEclipse(Env)

    if GetOption('target') == 'si':
        from sourceinsight import TargetSI
        TargetSI(Env)

    if GetOption('target') == 'json':
        global _json_export_registered
        if not _json_export_registered:
            main_env = Env

            def export_project_json_at_exit():
                from json_export import TargetJSON
                TargetJSON(main_env, EnvList)

            atexit.register(export_project_json_at_exit)
            _json_export_registered = True

def GenCppdefineFiles():
    build_dir = Env['build_dir']    
    
    if 'CPPDEFINES' in Env:
        CPPDEFINES = Env['CPPDEFINES']
    else:
        CPPDEFINES = []
    CPPDEFINES = [i for i in CPPDEFINES]  

    for group in Projects:
        if 'CPPDEFINES' in group and group['CPPDEFINES']:
            if CPPDEFINES:
                CPPDEFINES += group['CPPDEFINES']
            else:
                CPPDEFINES = group['CPPDEFINES']      
    f = open(os.path.join(build_dir, "cppdefines.txt"), 'w')
    f.write(',\n'.join(set(CPPDEFINES)))
    f.close()


def EndBuilding(target, program = None):
    import rtconfig

    need_exit = False

    Env['target']  = program
    Env['project'] = Projects

    if hasattr(rtconfig, 'BSP_LIBRARY_TYPE'):
        Env['bsp_lib_type'] = rtconfig.BSP_LIBRARY_TYPE

    if hasattr(rtconfig, 'dist_handle'):
        Env['dist_handle'] = rtconfig.dist_handle

    Env.AddPostAction(target, rtconfig.POST_ACTION)
    # Add addition clean files
    Clean(target, 'cconfig.h')
    Clean(target, 'rtua.py')
    Clean(target, 'rtua.pyc')

    if GetOption('target') and not IsChildProjEnv():
        GenTargetProject(program)

    BSP_ROOT = Env['BSP_ROOT']
    if GetOption('make-dist') and program != None:
        from mkdist import MkDist
        MkDist(program, BSP_ROOT, Rtt_Root, Env)
    if GetOption('make-dist-strip') and program != None:
        from mkdist import MkDist_Strip
        MkDist_Strip(program, BSP_ROOT, Rtt_Root, Env)
        need_exit = True
    if GetOption('cscope'):
        from cscope import CscopeDatabase
        CscopeDatabase(Projects)

    # if not GetOption('help') and not GetOption('target'):
    #     if not os.path.exists(rtconfig.EXEC_PATH) and not GetDepend('BSP_USING_PC_SIMULATOR'):
    #         logging.error("Error: the toolchain path (" + rtconfig.EXEC_PATH + ") is not exist, please check 'EXEC_PATH' in path or rtconfig.py.")
    #         exit(1)

    if need_exit:
        exit(0)
        
    if not GetOption('buildlib') and not rtconfig.ARCH=='sim':
        if rtconfig.PLATFORM == 'armcc' or rtconfig.PLATFORM == 'gcc':
            program_binary = Env.ProgramBinary(program)
            Env['program_binary'] = program_binary
        program_hex = Env.ProgramHex(program)   
        Env['program_hex'] = program_hex
        program_asm = Env.ProgramAsm(program)   
        GenCppdefineFiles()

        link_file = None
        if rtconfig.CROSS_TOOL == 'gcc':
            link_file = Env.LdsFile([File(rtconfig.LINK_SCRIPT_SRC + '.lds')])
        elif rtconfig.CROSS_TOOL == 'keil':
            generated_link = Path(rtconfig.OUTPUT_DIR, 'link_copy').as_posix()
            if rtconfig.LINK_SCRIPT != generated_link:
                link_file = File(rtconfig.LINK_SCRIPT + '.sct')
                if "PTAB_HEADER" in Env:
                    Depends(program, Env['PTAB_HEADER'])
            else:
                link_file = Env.LdsFile([File(rtconfig.LINK_SCRIPT_SRC + '.sct')])

        if link_file:
            Depends(program, link_file)
            if not (rtconfig.CROSS_TOOL == 'keil' and rtconfig.LINK_SCRIPT != Path(rtconfig.OUTPUT_DIR, 'link_copy').as_posix()):
                # always build link script as it would not get rebuilt when header file changes
                AlwaysBuild(link_file)
            if "ROM_SYM" in Env and Env['ROM_SYM']:
                Depends(program, Env['ROM_SYM'])
            
        # Register sdk_size.py to run once at program exit after all builds complete
        # Only register for main project (not child projects) to get consolidated report
        if not IsChildProjEnv() and rtconfig.CROSS_TOOL == 'gcc':
            global _sdk_size_registered, _main_build_dir
            if not _sdk_size_registered:
                _main_build_dir = Env['build_dir']
                _sdk_size_registered = True
                
                def run_sdk_size_analysis_at_exit():
                    # Only run if build was successful
                    if GetBuildFailures():
                        return
                    SIFLI_SDK = os.getenv('SIFLI_SDK')
                    sdk_size_script = os.path.join(SIFLI_SDK, 'tools', 'sdk_size', 'sdk_size.py')
                    if os.path.exists(sdk_size_script) and _main_build_dir:
                        print("\n" + "="*80)
                        print("Memory Usage Analysis")
                        print("="*80)
                        try:
                            cmd = [sys.executable, sdk_size_script, _main_build_dir]
                            result = subprocess.run(cmd)
                            print("="*80 + "\n")
                            if result.returncode != 0:
                                logging.warning("sdk_size.py returned non-zero exit code: " + str(result.returncode))
                        except Exception as e:
                            logging.warning(f"Failed to run sdk_size.py: {e}")
                
                atexit.register(run_sdk_size_analysis_at_exit)

def SrcRemove(src, remove):
    if not src:
        return

    src_bak = src[:]

    if type(remove) == type('str'):
        if os.path.isabs(remove):
            remove = os.path.relpath(remove, GetCurrentDir())
        remove = os.path.normpath(remove)

        for item in src_bak:
            if type(item) == type('str'):
                item_str = item
            else:
                item_str = item.rstr()

            if os.path.isabs(item_str):
                item_str = os.path.relpath(item_str, GetCurrentDir())
            item_str = os.path.normpath(item_str)

            if item_str == remove:
                src.remove(item)
    else:
        for remove_item in remove:
            remove_str = str(remove_item)
            if os.path.isabs(remove_str):
                remove_str = os.path.relpath(remove_str, GetCurrentDir())
            remove_str = os.path.normpath(remove_str)

            for item in src_bak:
                if type(item) == type('str'):
                    item_str = item
                else:
                    item_str = item.rstr()

                if os.path.isabs(item_str):
                    item_str = os.path.relpath(item_str, GetCurrentDir())
                item_str = os.path.normpath(item_str)

                if item_str == remove_str:
                    src.remove(item)

def GetVersion():
    import SCons.cpp
    import string

    rtdef = os.path.join(Rtt_Root, 'include', 'rtdef.h')

    # parse rtdef.h to get RT-Thread version
    prepcessor = PatchedPreProcessor()
    f = open(rtdef, 'r')
    contents = f.read()
    f.close()
    prepcessor.process_contents(contents)
    def_ns = prepcessor.cpp_namespace

    version = int(filter(lambda ch: ch in '0123456789.', def_ns['RT_VERSION']))
    subversion = int(filter(lambda ch: ch in '0123456789.', def_ns['RT_SUBVERSION']))

    if 'RT_REVISION' in def_ns:
        revision = int(filter(lambda ch: ch in '0123456789.', def_ns['RT_REVISION']))
        return '%d.%d.%d' % (version, subversion, revision)

    return '0.%d.%d' % (version, subversion)

def GlobSubDir(sub_dir, ext_name):
    import os
    import glob

    def glob_source(sub_dir, ext_name):
        list = os.listdir(sub_dir)
        src = glob.glob(os.path.join(sub_dir, ext_name))

        for item in list:
            full_subdir = os.path.join(sub_dir, item)
            if os.path.isdir(full_subdir):
                src += glob_source(full_subdir, ext_name)
        return src

    dst = []
    src = glob_source(sub_dir, ext_name)
    for item in src:
        dst.append(os.path.relpath(item, sub_dir))
    return dst

def PackageSConscript(package):
    from package import BuildPackage

    return BuildPackage(package)

def SifliMsvcEnv(cpu):
    import rtconfig
    
    rtconfig.PLATFORM= 'cl'
    rtconfig.CPU='win32'

    rtconfig.PREFIX = ''
    rtconfig.TARGET_EXT = 'exe'
    rtconfig.AS = rtconfig.PREFIX + 'cl'
    rtconfig.CC = rtconfig.PREFIX + 'cl'
    rtconfig.CXX = rtconfig.PREFIX + 'cl'
    rtconfig.AR = rtconfig.PREFIX + 'lib'
    rtconfig.RANLIB = ''
    rtconfig.LINK = rtconfig.PREFIX + 'link'
       
    rtconfig.AFLAGS = ''
    rtconfig.CFLAGS = ''
    rtconfig.LFLAGS = ''

    #if BUILD == 'debug':
    #    CFLAGS += ' /MTd'
    #    LFLAGS += ' /DEBUG'
    #else:
    #    CFLAGS += ' /MT'
    #    LFLAGS += ''
    rtconfig.CFLAGS += ' /MT'
    
    rtconfig.CFLAGS += ' /Zi /Od /W3 /WL /D_Win32 /wd4828 /FS /utf-8 /nologo /we4013'        
    rtconfig.LFLAGS += ' /SUBSYSTEM:CONSOLE /MACHINE:X86 /INCREMENTAL:NO /nologo '
    rtconfig.LFLAGS += '/PDB:"{}\\{}.pdb" /DEBUG /ignore:4099 '.format(rtconfig.OUTPUT_DIR, rtconfig.TARGET_NAME)

    rtconfig.CXXFLAGS = rtconfig.CFLAGS

    rtconfig.CPATH = ''
    rtconfig.LPATH = ''

    rtconfig.POST_ACTION = ''   

    rtconfig.CFLAGS += ' /IX:\\include /IY:\\ucrt /IY:\\um /IY:\\shared '
    rtconfig.LFLAGS += ' /LIBPATH:L:\\ucrt\\x86  /LIBPATH:L:\\um\\x86 /LIBPATH:X:\\lib\\x86 user32.lib '    
    rtconfig.EXEC_PATH = 'X:/bin/Hostx64/x86/'

    os.system(os.path.join(os.getenv('SIFLI_SDK'), "msvc_setup.bat"))

def SifliIarEnv(cpu):
    import rtconfig

    rtconfig.PLATFORM= 'iar'
    rtconfig.CROSS_TOOL= 'iar'
    
    # toolchains
    rtconfig.CC = 'iccarm'
    rtconfig.CXX = 'iccarm'
    rtconfig.AS = 'iasmarm'
    rtconfig.AR = 'iarchive'
    rtconfig.RANLIB = ''
    rtconfig.LINK = 'ilinkarm'
    rtconfig.TARGET_EXT = 'out'

    DEVICE = '-Dewarm'

    rtconfig.CFLAGS = DEVICE
    rtconfig.CFLAGS += ' --diag_suppress Pa050'
    #rtconfig.CFLAGS += ' --no_cse'
    #rtconfig.CFLAGS += ' --no_unroll'
    #rtconfig.CFLAGS += ' --no_inline'
    #rtconfig.CFLAGS += ' --no_code_motion'
    #rtconfig.CFLAGS += ' --no_tbaa'
    #rtconfig.CFLAGS += ' --no_clustering'
    #rtconfig.CFLAGS += ' --no_scheduling'
    rtconfig.CFLAGS += ' --endian=little'
    rtconfig.CFLAGS += ' --cpu={}.no_se'.format(cpu)
    rtconfig.CFLAGS += ' -e'
    rtconfig.CFLAGS += ' --fpu=VFPv5_sp'
    rtconfig.CFLAGS += ' --dlib_config "' + rtconfig.EXEC_PATH + '/arm/INC/c/DLib_Config_Normal.h"'
    rtconfig.CFLAGS += ' --silent'
    rtconfig.CXXFLAGS = rtconfig.CFLAGS
    rtconfig.CCFLAGS =  '' #rtconfig.CFLAGS

    rtconfig.AFLAGS = DEVICE
    rtconfig.AFLAGS += ' -s+'
    rtconfig.AFLAGS += ' -w+'
    rtconfig.AFLAGS += ' -r'
    rtconfig.AFLAGS += ' --cpu {}.no_se'.format(cpu)
    rtconfig.AFLAGS += ' --fpu VFPv5_sp'
    rtconfig.AFLAGS += ' -S'

    #if BUILD == 'debug':
    #    CFLAGS += ' --debug'
    #    CFLAGS += ' -On'
    #else:
    #    CFLAGS += ' -Oh'
    rtconfig.CFLAGS += ' -Ohz'
    
    rtconfig.LFLAGS = ['--config', rtconfig.LINK_SCRIPT + '.icf']
    rtconfig.LFLAGS += ['--entry', '__iar_program_start']
    rtconfig.LFLAGS += ['--map', '{}.map'.format(rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME)]
    rtconfig.LFLAGS += ['--diag_suppress', 'Lt009', '--vfe']

    rtconfig.EXEC_PATH = rtconfig.EXEC_PATH + '/arm/bin/'
    rtconfig.POST_ACTION = 'ielftool --ihex $TARGET {}.hex\n'.format(rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME)
    #rtconfig.POST_ACTION = ''
    
    #env.Replace(CCCOM = ['$CC $CCFLAGS $CPPFLAGS $_CPPDEFFLAGS $_CPPINCFLAGS -o $TARGET $SOURCES'])
    #env.Replace(ARFLAGS = [''])
    #env.Replace(LINKCOM = env["LINKCOM"] + ' --map rt-thread.map')

def GetKeilMcpu():
    if GetDepend('SOC_SF32LB55X'):
        mcpu = 'cortex-m33'
    elif GetDepend('SOC_SF32LB56X'):
        mcpu = 'cortex-m33+cdecp1'
    elif GetDepend('SOC_SF32LB58X'):
        mcpu = 'cortex-m33+cdecp1'
    elif GetDepend('SOC_SF32LB52X'):  
        if GetDepend("BF0_LCPU"):
            mcpu = 'cortex-m33+nodsp'
        else:
            mcpu = 'cortex-m33+cdecp1'
    elif GetDepend('SOC_SF32LB57X'):  
        if GetDepend("BF0_LCPU"):
            mcpu = 'cortex-m33+nodsp'
        else:
            mcpu = 'cortex-m33+cdecp1'
    else:
        raise Exception("Unknown chip series")
    
    return mcpu

# ref.: https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html
# -mcpu=cortex-m33+cdecp1 is not supported by GCC14, -mcpu=star-mc1+cdecp1 neither, so we use the combination of -mtune and -march
# -mtune=cortex-m33 doesn't enable dsp and fp by default, so need to enable them by -march
def GetMtune():
    if GetDepend('SOC_SF32LB55X'):
        mtune = 'cortex-m33'
    elif GetDepend('SOC_SF32LB56X'):
        mtune = 'cortex-m33'
    elif GetDepend('SOC_SF32LB58X'):
        mtune = 'cortex-m33'
    elif GetDepend('SOC_SF32LB52X'):  
        mtune = 'cortex-m33'
    elif GetDepend('SOC_SF32LB57X'):  
        mtune = 'cortex-m33'
    else:
        raise Exception("Unknown chip series")
    
    return ' -mtune=' + mtune

def GetMarch():
    if GetDepend('SOC_SF32LB55X'):
        march = 'armv8-m.main+dsp+fp'
    elif GetDepend('SOC_SF32LB56X'):
        march = 'armv8-m.main+dsp+fp+cdecp1'
    elif GetDepend('SOC_SF32LB58X'):
        march = 'armv8-m.main+dsp+fp+cdecp1'
    elif GetDepend('SOC_SF32LB52X'):  
        if GetDepend("BF0_LCPU"):
            march = 'armv8-m.main'
        else:
            march = 'armv8-m.main+dsp+fp+cdecp1'
    elif GetDepend('SOC_SF32LB57X'):  
        if GetDepend("BF0_LCPU"):
            march = 'armv8-m.main'
        else:
            march = 'armv8-m.main+dsp+fp+cdecp1'
    else:
        raise Exception("Unknown chip series")
    
    return ' -march=' + march

def SifliGccEnv(cpu):
    import rtconfig
    
    rtconfig.PLATFORM= 'gcc'
    rtconfig.CROSS_TOOL= 'gcc'

    # toolchains
    if rtconfig.ARCH=='arm':
        rtconfig.PREFIX = 'arm-none-eabi-'
        DEVICE = GetMtune() + GetMarch() + ' -mthumb -ffunction-sections -fdata-sections'
    elif rtconfig.ARCH=='risc-v':
        rtconfig.PREFIX = 'riscv64-unknown-elf-'
        DEVICE = ' -march=rv32ima_zca_zcb_zcf_zcmp_zcmt_xxlcz -mabi=ilp32'

    rtconfig.CC = rtconfig.PREFIX + 'gcc'
    rtconfig.AS = rtconfig.PREFIX + 'gcc'
    rtconfig.AR = rtconfig.PREFIX + 'ar'
    rtconfig.RANLIB = rtconfig.PREFIX + 'ranlib'
    rtconfig.CXX = rtconfig.PREFIX + 'g++'
    rtconfig.LINK = rtconfig.PREFIX + 'gcc'
    rtconfig.STRIP = rtconfig.PREFIX + 'strip'
    rtconfig.TARGET_EXT = 'elf'
    rtconfig.SIZE = rtconfig.PREFIX + 'size'
    rtconfig.OBJDUMP = rtconfig.PREFIX + 'objdump'
    rtconfig.OBJCPY = rtconfig.PREFIX + 'objcopy'

    if GetDepend('CPU_HAS_NO_DSP_FP'):
        no_dsp_fp = True
    else:
        no_dsp_fp = False


    SIFLI_SDK = os.getenv('SIFLI_SDK')
    if rtconfig.ARCH=='arm':
        if not no_dsp_fp:
            rtconfig.CFLAGS = DEVICE + ' -mfpu=fpv5-sp-d16 -mfloat-abi=hard'
        else:
            rtconfig.CFLAGS = DEVICE + ' -mfloat-abi=soft'
    rtconfig.CFLAGS += ' -funsigned-char -fshort-enums -fshort-wchar'
    # We don't need to delete the SDK prefix now, as this would make debugging inconvenient.
    # rtconfig.CFLAGS += f' -ffile-prefix-map={SIFLI_SDK}=./'
    rtconfig.CFLAGS += ' -mlittle-endian -gdwarf-3 -Wno-packed -Wno-missing-noreturn -Wno-sign-conversion -Wno-unused-macros -Wnull-dereference'
    rtconfig.CFLAGS += ' -fno-unwind-tables -fno-exceptions'
    rtconfig.CFLAGS += ' -fno-common -fno-strict-aliasing'
    
    if hasattr(rtconfig, 'OPT_LEVEL'):
        rtconfig.CFLAGS += ' ' + rtconfig.OPT_LEVEL
    elif GetConfigValue("OPT_LEVEL") != "":
        rtconfig.CFLAGS += ' ' + GetConfigValue("OPT_LEVEL").replace('"', '')
    else:
        rtconfig.CFLAGS += ' -Os' 

    rtconfig.CXXFLAGS = rtconfig.CFLAGS
    if no_dsp_fp:
        rtconfig.CXXFLAGS += ' -fno-exceptions -fno-rtti'
    rtconfig.CCFLAGS =  rtconfig.CFLAGS + ' -std=c99 -Wno-missing-prototypes'
    rtconfig.AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp'
    if rtconfig.ARCH=='arm':
        rtconfig.AFLAGS += ' -Wa,-mimplicit-it=thumb'
        if not no_dsp_fp:
            rtconfig.AFLAGS += ' -mfpu=fpv5-sp-d16 -mfloat-abi=hard'
        else:
            rtconfig.AFLAGS += ' -mfloat-abi=soft'
    elif rtconfig.ARCH=='risc-v':
        rtconfig.CFLAGS += ' -ffunction-sections -fdata-sections -fno-common'
        if GetDepend('LTO_SUPPORT'):
            rtconfig.CFLAGS += ' -flto'
            rtconfig.AFLAGS += ' -ffat-lto-objects'
        
    
    rtconfig.LFLAGS = rtconfig.CFLAGS.strip().split()
    #  ['-mcpu=Cortex-M33', '-mthumb', '-ffunction-sections', '-fdata-sections']
    #rtconfig.LFLAGS += '-std=c99 -mfpu=fpv5-sp-d16 -mfloat-abi=hard'.split()
    if not hasattr(rtconfig, 'TARGET_NAME'):
        rtconfig.TARGET_NAME = 'rtthread'
    if rtconfig.ARCH=='arm':
        rtconfig.LFLAGS += ['-Wl,--no-wchar-size-warning,--gc-sections,-Map={}.map,-cref,-u,Reset_Handler'.format(rtconfig.OUTPUT_DIR + '/' + rtconfig.TARGET_NAME)]
    else:
        rtconfig.LFLAGS += ['-Wl,--undefined=g_patch_type,--undefined=_calloc_r,--undefined=_realloc_r,--undefined=bt_sco_data_handle_callback,--undefined=ble_boot,--gc-sections,-Map={}.map,-cref,-u,Reset_Handler'.format(rtconfig.OUTPUT_DIR + '/' + rtconfig.TARGET_NAME)]

    rtconfig.LINK_SCRIPT = rtconfig.OUTPUT_DIR + '/link_copy'

    rtconfig.LFLAGS += ['-T',  rtconfig.LINK_SCRIPT + '.lds']

    rtconfig.LFLAGS += ['-specs=nosys.specs']
    if ((not GetDepend('BSP_USING_RTTHREAD'))
            or (hasattr(rtconfig, 'USE_MICROLIB') and rtconfig.USE_MICROLIB) 
            or GetDepend("USING_MICROLIB")):

        rtconfig.LFLAGS += ['-specs=nano.specs']

    rtconfig.LFLAGS += ['-eentry']

    # Not support CUSTOM_LFLAGS as it depends on compiler
    # if hasattr(rtconfig, 'CUSTOM_LFLAGS') and rtconfig.CUSTOM_LFLAGS:
    #     if not GetDepend('ROM_ATTR'):
    #         rtconfig.CUSTOM_LFLAGS = rtconfig.CUSTOM_LFLAGS.replace('rom.sym', '')
    #     rtconfig.LFLAGS += rtconfig.CUSTOM_LFLAGS.split()

    rtconfig.CPATH = ''
    rtconfig.LPATH = ''
    
    if not hasattr(rtconfig, 'POST_ACTION'):
        rtconfig.POST_ACTION = ''

    #rtconfig.POST_ACTION += rtconfig.OBJCPY + ' -O binary $TARGET {}.bin\n'.format(rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME) + rtconfig.SIZE + ' $TARGET \n'
    #rtconfig.POST_ACTION += rtconfig.OBJCPY + ' -O ihex $TARGET {}.hex\n'.format(rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME)

    rtconfig.M_CFLAGS = rtconfig.CFLAGS + ' -mlong-calls -fPIC '
    rtconfig.M_LFLAGS = DEVICE + ' -Wl,--gc-sections,-z,max-page-size=0x4 ' +\
                                    '-shared -fPIC -nostartfiles -nostdlib -static-libgcc '
    rtconfig.M_POST_ACTION = rtconfig.STRIP + ' -R .hash $TARGET\n' + rtconfig.SIZE + ' $TARGET \n'    

def SifliKeilVersion():
    keil_ini = os.getenv('RTT_EXEC_PATH')+'/TOOLS.ini'
    try:
        f_keil=open(keil_ini,"r")
        for line in f_keil:
            line_s=re.split("=|\n", line)
            if line_s[0]=="VERSION":
                return line_s[1]
        return 0
    except:
        return 0

def SifliKeilEnv(cpu, BSP_ROOT=''):
    import rtconfig
    # toolchains
    rtconfig.PLATFORM= 'armcc'
    rtconfig.CROSS_TOOL= 'keil'
    rtconfig.CC = 'armclang'
    rtconfig.CXX = 'armclang'
    rtconfig.AS = 'armasm'
    rtconfig.AR = 'armar'
    rtconfig.RANLIB = ''
    rtconfig.LINK = 'armlink'
    rtconfig.TARGET_EXT = 'axf'
    
    if GetDepend('CPU_HAS_NO_DSP_FP'):
        no_dsp_fp = True
    else:
        no_dsp_fp = False
    
    use_link_template = bool(
        hasattr(rtconfig, 'LINK_SCRIPT_TEMPLATE')
        and rtconfig.LINK_SCRIPT_TEMPLATE
        and _current_project_uses_ptab_v3(BSP_ROOT)
    )
    if not use_link_template:
        link_script_path = rtconfig.LINK_SCRIPT_SRC + '.sct'
        with open(link_script_path, 'r', encoding='utf-8', errors='ignore') as f:
            script = f.read()
        if _link_script_has_env_vars(script):
            new_file_path = Path(rtconfig.OUTPUT_DIR, os.path.basename(rtconfig.LINK_SCRIPT_SRC) + '.sct')
            if not os.path.exists(rtconfig.OUTPUT_DIR):
                logging.debug('sct dir:{}'.format(rtconfig.OUTPUT_DIR))
                Execute(Mkdir(rtconfig.OUTPUT_DIR))
            with open(new_file_path, 'w', encoding='utf-8', newline='\n') as f:
                f.write(_replace_link_script_env_vars(script, BSP_ROOT))
            rtconfig.LINK_SCRIPT = new_file_path.with_suffix('').as_posix()
        else:
            rtconfig.LINK_SCRIPT = rtconfig.LINK_SCRIPT_SRC
    else:
        # Keil links against a generated scatter file in the build dir when the
        # selected source is rendered from a Jinja2 template.
        rtconfig.LINK_SCRIPT = Path(rtconfig.OUTPUT_DIR, 'link_copy').as_posix()
    
    rtconfig.keil_version=SifliKeilVersion()
    logging.debug("Keil version %s"%(rtconfig.keil_version))
    DEVICE=''
    rtconfig.AFLAGS=''
    rtconfig.CFLAGS=''
    if cpu=='cortex-m33':
        if not no_dsp_fp:
            rtconfig.AFLAGS+= '  --fpu=FPv5-SP --cpreproc_opts=-mfpu=fpv5-sp-d16 --cpreproc_opts=-mfloat-abi=hard --cpreproc_opts=-DARMCM33_DSP_FP --cpreproc --cpreproc_opts=--target=arm-arm-none-eabi --cpreproc_opts=-mfloat-abi=hard '
            rtconfig.CFLAGS+= ' -DARMCM33_DSP_FP '        
            DEVICE += ' -mfpu=fpv5-sp-d16 -mfloat-abi=hard '   
            asm_cpu = cpu        
        else:
            rtconfig.AFLAGS+= '  --fpu=SoftVFP --cpreproc_opts=-mfpu=none --cpreproc_opts=-mfloat-abi=soft --cpreproc_opts=-DARMCM33 --cpreproc --cpreproc_opts=--target=arm-arm-none-eabi --cpreproc_opts=-mfloat-abi=soft '
            rtconfig.CFLAGS+= ' -DARMCM33 '        
            DEVICE += ' -mfpu=none -mfloat-abi=soft '   
            asm_cpu = cpu + '.no_dsp'

        cpu = GetKeilMcpu()
    else:
        assert false, "Unknown cpu: {}".format(cpu)
    
    rtconfig.CFLAGS += ' -mcpu=' + cpu +  DEVICE + ' -c -ffunction-sections --target=arm-arm-none-eabi'
    rtconfig.CFLAGS += ' -fno-rtti -funsigned-char -fshort-enums -fshort-wchar'
	# -Werror 
    rtconfig.CFLAGS += ' -mlittle-endian -gdwarf-3 -Wno-builtin-macro-redefined -Wno-pointer-sign -Wno-typedef-redefinition '
    rtconfig.CFLAGS += ' -mno-outline '

    rtconfig.CFLAGS += ' -I' + rtconfig.EXEC_PATH + '/ARM/ARMCLANG/include'
    rtconfig.CFLAGS += ' -D__UVISION_VERSION="532" '
    
    rtconfig.AFLAGS += ' --cpu='+ asm_cpu + ' --cpreproc_opts=-mcpu=' + cpu +' --li -g '
    rtconfig.AFLAGS += ' --cpreproc_opts=-D__UVISION_VERSION="532" '
    rtconfig.AFLAGS += ' --diag_suppress=A1609 '

    rtconfig.CXXFLAGS = rtconfig.CFLAGS + ' -xc++ -fno-exceptions ' 
    rtconfig.CXXFLAGS += ' -I' + rtconfig.EXEC_PATH + '/ARM/ARMCLANG/include/libcxx'
    rtconfig.CCFLAGS =  rtconfig.CFLAGS
    rtconfig.CFLAGS = rtconfig.CFLAGS + ' -xc -std=c99 '
    rtconfig.LFLAGS = ' --cpu=' + asm_cpu 
    if no_dsp_fp:
        rtconfig.LFLAGS += ' --fpu=SoftVFP'    
    rtconfig.LFLAGS += ' --strict --scatter '+ rtconfig.LINK_SCRIPT+ '.sct --summary_stderr --info summarysizes --map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --any_contingency '
    
    rtconfig.LFLAGS += ' --list ' + os.path.join(rtconfig.OUTPUT_DIR, rtconfig.TARGET_NAME + '.map') + ' '
    if not GetDepend('RT_USING_CPLUSPLUS'):
        rtconfig.LFLAGS += ' --symdefs=' + os.path.join(rtconfig.OUTPUT_DIR, rtconfig.TARGET_NAME + '.symdefs') + ' '
    rtconfig.LFLAGS += ' --libpath=' + rtconfig.EXEC_PATH + '/ARM/ARMCLANG/lib'
    if ((not GetDepend('BSP_USING_RTTHREAD'))
            or (hasattr(rtconfig, 'USE_MICROLIB') and rtconfig.USE_MICROLIB) 
            or GetDepend("USING_MICROLIB")):

        rtconfig.CFLAGS += ' -D__MICROLIB '
        rtconfig.LFLAGS += ' --library_type=microlib '
    
    if hasattr(rtconfig, 'CUSTOM_LFLAGS') and rtconfig.CUSTOM_LFLAGS:
        if not GetDepend('ROM_ATTR'):
            rtconfig.CUSTOM_LFLAGS = rtconfig.CUSTOM_LFLAGS.replace('rom.sym', '')
        rtconfig.LFLAGS += ' ' + rtconfig.CUSTOM_LFLAGS
    rtconfig.EXEC_PATH += '/ARM/ARMCLANG/bin/'
    
    if GetDepend('LTO_SUPPORT'):
        rtconfig.CFLAGS += ' -flto'
        rtconfig.LFLAGS += ' --lto'

    if (not is_verbose()):
        rtconfig.LFLAGS +=' --diag_suppress=L6314 --diag_suppress=L6329'
       
    #if BUILD == 'debug':
    #    CFLAGS += ' -g -Oz'
    #    AFLAGS += ' -g'
    #else:
    #   CFLAGS += ' -O2'
    if hasattr(rtconfig, 'OPT_LEVEL'):
        rtconfig.CFLAGS += ' ' + rtconfig.OPT_LEVEL
    elif GetConfigValue("OPT_LEVEL") != "":
        rtconfig.CFLAGS += ' ' + GetConfigValue("OPT_LEVEL").replace('"', '')
    else:
        rtconfig.CFLAGS += ' -Oz'    
    
    if hasattr(rtconfig, 'WERROR') and rtconfig.WERROR:
        rtconfig.CFLAGS += ' -Werror'
    else:
        rtconfig.CFLAGS += ' -Wunused '

    try: 
        rtconfig.POST_ACTION
    except:
        rtconfig.POST_ACTION = ''
        #if GetDepend("SOC_SF32LB58X"):
        #    rtconfig.POST_ACTION = 'fromelf --bin $TARGET --output ' + rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME + '.bin \nfromelf -z $TARGET \nfromelf --cpu=8-M.Main --coproc1=cde --text -c $TARGET --output '+rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME + '.asm \n'
        #else:
        #    rtconfig.POST_ACTION = 'fromelf --bin $TARGET --output ' + rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME + '.bin \nfromelf -z $TARGET \nfromelf --text -c $TARGET --output '+rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME + '.asm \n'        
        #rtconfig.POST_ACTION += 'fromelf --i32 $TARGET --output ' + rtconfig.OUTPUT_DIR + rtconfig.TARGET_NAME + '.hex \n'


# load rtconfig.py in board folder
def LoadRtconfig(board):
    import rtconfig
    
    board_path1, board_path2 = GetBoardPath(board)
    if not os.path.exists(board_path1):
        logging.error('Board path "{}" not found'.format(board_path1))
        exit(1)

    if not os.path.exists(board_path2):
        logging.error('Board path "{}"" not found'.format(board_path2))
        exit(1)

    proj_rtconfig = (lambda spec: (spec.loader.exec_module(mod := importlib.util.module_from_spec(spec)) or mod))(importlib.util.spec_from_file_location('main', os.path.join(board_path2, 'rtconfig.py')))
    MergeRtconfig(proj_rtconfig, rtconfig)
        
    proj_rtconfig.OUTPUT_DIR = 'build_' + board + '/'
    proj_rtconfig.LINK_SCRIPT, proj_rtconfig.LINK_SCRIPT_TEMPLATE=GetLinkScript('.',board,proj_rtconfig.CHIP,proj_rtconfig.CORE.lower())
    proj_rtconfig.TARGET_NAME = "main"
    
    # clear old rtconfig
    for var in dir(rtconfig):
        if not var.startswith('_') and not var.islower():
            delattr(rtconfig, var)
       
    # sync rtconfig with board rtconfig
    for var in dir(proj_rtconfig):
        if not var.startswith('_') and not var.islower():
            setattr(rtconfig, var, getattr(proj_rtconfig, var))

    if '_lcpu' in board:
        rtconfig.CORE = "LCPU"
    elif '_acpu' in board:
        rtconfig.CORE = "ACPU"
    else:
        rtconfig.CORE = "HCPU"   

    return proj_rtconfig


# Get board full path tuple by board name, the first path is parent folder, the second path is hcpu or lcpu folder
# For single core board, both path are same
# core argument could override the core suffix of board name
def GetBoardPath(board):
    if board is None:
        return None

    SIFLI_SDK = os.getenv('SIFLI_SDK')
    if '_hcpu' in board:
        board_path = board.replace('_hcpu', '')
        subfolder = 'hcpu'
    elif '_lcpu' in board:
        board_path = board.replace('_lcpu', '')       
        subfolder = 'lcpu'
    elif '_acpu' in board:
        board_path = board.replace('_acpu', '')       
        subfolder = 'acpu'
    else:
        # default as hcpu
        board_path = board.replace('_hcpu', '')
        subfolder = 'hcpu'


    board_root = os.path.join(SIFLI_SDK, 'customer/boards')
    if (BOARD_SEARCH_PATH is not None) and os.path.exists(os.path.join(BOARD_SEARCH_PATH, board_path)):
        board_root = BOARD_SEARCH_PATH
    board_path1 = os.path.join(board_root, board_path).replace('\\', '/')
    board_path2 = os.path.join(board_path1, subfolder).replace('\\', '/')

    return (board_path1, board_path2)

def GetCoreType(board):
    if board.endswith('_lcpu'):
        core = 'LCPU'
    elif board.endswith('_hcpu'):
        core = 'HCPU'
    elif board.endswith('_acpu'):
        core = 'ACPU'
    else:
        core = 'HCPU'

    return core    

def GetBoardName(core=None):
    try:
        board = GetOption('board')
    except Exception as e: 
        board = None

    if board is not None:
        if not '_lcpu' in board and not '_hcpu' in board and not '_acpu' in board:
            #default set to HCPU
            board += '_hcpu'
        board_core = GetCoreType(board)
        if core and board_core and board_core != core:
            if board.endswith('_' + board_core.lower()):
                board = board[:-len('_' + board_core.lower())]
            board += '_' + core.lower()

    return board 


def IsInitBuild():
    if GetOption("init_build"):
        return True
    else:
        return False    


def PrepareEnv(board=None):
    import rtconfig
    global BuildOptions

    try:
        AddOption('--board',
                    dest = 'board',
                    type = 'string',
                    default=board,
                    help = 'board name')            
        AddOption('--board_search_path',
                    dest = 'board_search_path',
                    type = 'string',
                    default=None,
                    help = 'board search path in addition to sdk board path')            
        AddOption('--bconf',
                    dest = 'bconf',
                    type = 'string',
                    default = "board.conf",
                    help = 'board configuration')            
        AddOption('--init-build',
                    dest = 'init_build',
                    action = 'store_true',
                    default = False,
                    help = 'Init build')     
        AddOption('--verbose',
                    dest = 'verbose',
                    action = 'store_true',
                    default = False,
                    help = 'print verbose information during build')                    
    except:
        pass

    if GetOption('verbose'):
        VERBOSE=1
        Export('VERBOSE')
        logging.basicConfig(format='%(message)s',level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(message)s',level=logging.INFO)
    board = GetBoardName()
    logging.info("Board: {}".format(board))

    if GetOption('board_search_path'):
        global BOARD_SEARCH_PATH
        BOARD_SEARCH_PATH = os.path.abspath(GetOption('board_search_path'))
        logging.info(f"Board search path: {BOARD_SEARCH_PATH}")

    if board:
        LoadRtconfig(board)
        InitBuild(None, rtconfig.OUTPUT_DIR, board)
        # construct BuildOptions
        BuildOptions = {}
        BuildOptionUpdate(BuildOptions, None)
            

def _IsPcSimulatorBuild():
    try:
        board = GetBoardName()
    except Exception:
        board = None

    is_pc_board = board in ('pc_hcpu', 'pc_lcpu')
    try:
        if GetDepend('BSP_USING_PC_SIMULATOR'):
            return True
        if GetDepend('SOC_SIMULATOR') and is_pc_board:
            return True
    except Exception:
        pass

    return is_pc_board


def AddBootLoader(SIFLI_SDK, chip):
    if not chip and _IsPcSimulatorBuild():
        return

    if GetDepend("BOARD_RAMRUN_ENABLED"):
        return
    
    # Add bootloader project
    proj_path = None
    proj_name = 'bootloader'
    if "SF32LB56X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/boot_loader/project/sf32lb56x_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB52X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/boot_loader/project/butterflmicro/ram_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB58X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/boot_loader/project/sf32lb58x_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB57X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/boot_loader/project/sf32lb57x/ram_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB55X" == chip:
        # 55x has no bootloader    
        pass 
    else:
        assert False, "Unknown chip: {}".format(chip)    

def AddFTAB(SIFLI_SDK, chip, env=None):
    """Add ftab subproject based on ptab version
    
    - v3 format: Skip subproject, ftab.bin generated by script
    - v1/v2 format: Add ftab subproject for compilation
    """
    if not chip and _IsPcSimulatorBuild():
        return

    if GetDepend("BOARD_RAMRUN_ENABLED"):
        return

    import ptab as ptab_module
    
    if env is None:
        try:
            env = GetCurrentEnv()
        except Exception:
            env = None

    # Try to get ptab path from environment
    ptab_path = None
    if env and 'PARTITION_TABLE' in env:
        ptab_path = env['PARTITION_TABLE']
    
    # Check if this is v3 format
    if ptab_path and os.path.exists(ptab_path):
        ptab_obj = ptab_module.load_ptab(ptab_path, fatal=False)
        if ptab_obj and ptab_obj.is_v3():
            # v3 format: generate ftab.bin directly after all child projects are registered.
            if env:
                imgs_info = []
                target_list = []
                for e in GetEnvList():
                    # embedded project binaries are embedded into parent image
                    if IsChildProjEnv(e) and e.get('IMG_EMBEDDED'):
                        continue
                    proj_name = e['name'] if IsChildProjEnv(e) else 'main'
                    if 'program_binary' in e:
                        imgs_info.append({"name": proj_name, "binary": e['program_binary']})
                    if 'target' in e:
                        target_list += e['target']
                    if 'program_binary' in e:
                        target_list += e['program_binary']
                    if 'program_hex' in e:
                        target_list += e['program_hex']

                output_path = env.get('FTAB_BIN_PATH') or os.path.join(env['BUILD_DIR_FULL_PATH'], 'ftab.bin')
                ftab_bin = env.FtabBin(output_path, File(ptab_path), IMGS_INFO=imgs_info)
                Depends(ftab_bin, target_list)
                env['FTAB_BIN'] = ftab_bin
            logging.info(f"ftab.bin will be generated by script (no subproject) for chip: {chip} (ptab v3)")
            return
    
    # v1/v2 format: Add ftab subproject
    logging.info(f"Adding ftab subproject for chip: {chip} (ptab v1/v2)")
    proj_path = None
    proj_name = 'ftab'
    if "SF32LB56X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/flash_table/sf32lb56x_common_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB52X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/flash_table/sf32lb52x_common_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB58X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/flash_table/sf32lb58x_common_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB55X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/flash_table/sf32lb55x_common_v2')
        AddChildProj(proj_name, proj_path, False)
    elif "SF32LB57X" == chip:
        proj_path = os.path.join(SIFLI_SDK, 'example/flash_table/sf32lb57x_common_v2')
        AddChildProj(proj_name, proj_path, False)        
    else:
        assert False, "Unknown chip: {}".format(chip)    

def AddDFU(SIFLI_SDK):
    proj_path = None
    proj_name = 'dfu'
    proj_path = os.path.join(SIFLI_SDK, 'example/dfu/project')
    AddChildProj(proj_name, proj_path, False)

def AddDFU_PAN(SIFLI_SDK):
    proj_path = None
    proj_name = 'dfu'
    proj_path = os.path.join(SIFLI_SDK, 'example/dfu_pan/project')
    AddChildProj(proj_name, proj_path, False)

def AddDFU_PAN_V2(SIFLI_SDK):
    proj_path = None
    proj_name = 'dfu'
    proj_path = os.path.join(SIFLI_SDK, 'example/dfu_v2/bt_pan/loader/project')
    AddChildProj(proj_name, proj_path, False)

def AddDFU_CDC(SIFLI_SDK):
    proj_path = None
    proj_name = 'dfu'
    proj_path = os.path.join(SIFLI_SDK, 'example/dfu_v2/cdc/loader/project')
    AddChildProj(proj_name, proj_path, False)

def AddDFU_BLE(SIFLI_SDK):
    proj_path = None
    proj_name = 'dfu'
    proj_path = os.path.join(SIFLI_SDK, 'example/dfu_v2/ble/loader/project')
    AddChildProj(proj_name, proj_path, False)

def AddLCPU(SIFLI_SDK, chip,target_file=None):
    if "SF32LB56X" == chip or "SF32LB52X" == chip or "SF32LB58X" == chip:
        proj_path = None
        proj_path = os.path.join(SIFLI_SDK, 'example/ble/lcpu_general/project/common')
        lcpu_proj_name = 'lcpu'
        return AddChildProj(lcpu_proj_name, proj_path, True, core="LCPU")
    elif "SF32LB55X" == chip:
        import rtconfig
        if target_file != None:
            if "SF32LB551" == rtconfig.PACKAGE:
                shutil.copy(os.path.join(SIFLI_SDK, 'example/rom_bin/lcpu_general_ble_img/lcpu_lb551.c'), target_file)
            elif "SF32LB551_A3" == rtconfig.PACKAGE:
                shutil.copy(os.path.join(SIFLI_SDK, 'example/rom_bin/lcpu_general_ble_img/lcpu_lb551_a3.c'), target_file)
            elif "SF32LB555"==rtconfig.PACKAGE:
                shutil.copy(os.path.join(SIFLI_SDK, 'example/rom_bin/lcpu_general_ble_img/lcpu_lb555.c'), target_file)
            elif "SF32LB555_A3"==rtconfig.PACKAGE:
                shutil.copy(os.path.join(SIFLI_SDK, 'example/rom_bin/lcpu_general_ble_img/lcpu_lb555_a3.c'), target_file)
            elif "SS6600"==rtconfig.PACKAGE:
                shutil.copy(os.path.join(SIFLI_SDK, 'example/rom_bin/lcpu_general_ble_img/lcpu_general_6600.c'), target_file)
            elif "SS6600_A3"==rtconfig.PACKAGE:
                shutil.copy(os.path.join(SIFLI_SDK, 'example/rom_bin/lcpu_general_ble_img/lcpu_general_6600_a3.c'), target_file)
        return None   
# For parent project, BSP_Root should be None
def SifliEnv(BSP_Root = None):
    import rtconfig
    global BuildOptions
    
    if BSP_Root is None:
        logging.debug("\n======================")        
        logging.debug("Main project start")

    logging.debug("\n----------------------")
    if hasattr(rtconfig, 'CORE'):
        core = rtconfig.CORE
    else:
        core = None    

    board = GetBoardName(core)
    if board and (BSP_Root != None): # main project has called InitBuild in PrepareEnv
        logging.debug("Init build {} for output: {}".format(board, rtconfig.OUTPUT_DIR))
        InitBuild(BSP_Root, rtconfig.OUTPUT_DIR, board)
    elif board is None:
        if not os.path.exists(rtconfig.OUTPUT_DIR):
            logging.debug("create output dir {} first for old fashion build".format(rtconfig.OUTPUT_DIR))
            os.makedirs(rtconfig.OUTPUT_DIR)

    #  Clean BuildOptions
    BuildOptions = {}
    
    if BSP_Root is None:
        logging.debug("parent build dir: {}".format(rtconfig.OUTPUT_DIR))
        
    SIFLI_SDK = os.getenv('SIFLI_SDK')

    try:
        f = open(SIFLI_SDK + '/.version', 'r')
        for line in f:
            line=line[:-1]
            fields=line.split('=')
            if (fields[0]=='MAJOR'):
                MAJOR=int(fields[1])
            elif (fields[0]=='MINOR'):
                MINOR=int(fields[1])
            elif (fields[0]=='REV'):
                REV=int(fields[1])
        f.close()
        rtconfig.sifli_version=(MAJOR<<24)+(MINOR<<16)+REV
    except:
        logging.error("Cannot get SDK version")
        exit()

    BuildOptionUpdate(BuildOptions, BSP_Root)
    if core:
        if GetDepend('BF0_ACPU') and core != "ACPU":
            raise ValueError('Conflict core type: {}'.format(core))
        elif GetDepend('BF0_HCPU') and core != "HCPU" and core != "ACPU":
            raise ValueError('Conflict core type: {}'.format(core))
        elif GetDepend('BF0_LCPU') and core != "LCPU":
            raise ValueError('Conflict core type: {}'.format(core))

    rtconfig.keil_version=0

    sifli_build=os.popen('cd {} && git rev-parse HEAD'.format(SIFLI_SDK)).read()
    if len(sifli_build) < 8:
        rtconfig.sifli_build = '00000000'
    else:
        rtconfig.sifli_build = sifli_build[0:8]
    logging.debug("Version %08x, Build %s" %(rtconfig.sifli_version,rtconfig.sifli_build)) 

    try:       
        cpu=GetDepend('CPU').replace('"','').lower()
        rtconfig.CPU=cpu
    except:
        #TODO: it's not appropriate to use SOC name 
        if GetDepend('BF0_HCPU'):
            cpu='Cortex-M33'
        elif GetDepend('BF0_LCPU'):
            cpu='Cortex-M33'
        elif GetDepend('BF0_ACPU'):
            cpu='Cortex-M33'
        elif GetDepend('BSP_USING_PC_SIMULATOR'):
            cpu='X86'
        else:
            cpu='Cortex-M33'
            logging.error("Undefined core, please select BF0_HCPU/BF0_LCPU/BF0_ACPU")
        
        cpu = cpu.lower()
        rtconfig.CPU=cpu

    if GetDepend('BSP_USING_PC_SIMULATOR'):
        rtconfig.ARCH='sim'
    else:
        rtconfig.ARCH='arm'

    if GetDepend("S_SLIM"):
        rtconfig.S_SLIM = True
        
    # bsp lib config
    rtconfig.BSP_LIBRARY_TYPE = None        

    rtconfig.EXEC_PATH = os.getenv('RTT_EXEC_PATH')

    if not rtconfig.ARCH=='sim':
        rtconfig.LINK_SCRIPT_SRC = rtconfig.LINK_SCRIPT
    
    CROSS_TOOL='gcc'
    if not BSP_Root:
        BSP_Root = Dir('#').abspath
    if os.getenv('RTT_CC'):
        CROSS_TOOL = os.getenv('RTT_CC')
    if rtconfig.ARCH=='sim':
        SifliMsvcEnv(cpu) 
    elif CROSS_TOOL == 'gcc':
        SifliGccEnv(cpu)        
    elif CROSS_TOOL == 'keil':
        SifliKeilEnv(cpu, BSP_Root)
    elif CROSS_TOOL == 'iar':
        SifliIarEnv(cpu)
        
    RTT_ROOT=os.path.join(SIFLI_SDK, 'rtos/rtthread')
    
    Export('RTT_ROOT')
    Export('rtconfig')
    Export('SIFLI_SDK')
