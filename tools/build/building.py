#
# File      : building.py
# This file is part of RT-Thread RTOS
# COPYRIGHT (C) 2006 - 2015, RT-Thread Development Team
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Change Logs:
# Date           Author       Notes
# 2015-01-20     Bernard      Add copyright information
# 2015-07-25     Bernard      Add LOCAL_CCFLAGS/LOCAL_CPPPATH/LOCAL_CPPDEFINES for
#                             group definition.
#

import importlib
import os
import re
import shutil
import string
import subprocess
import sys
import logging

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

def is_verbose():
    if (logging.root.level<=logging.DEBUG):
        return True
    else:
        return False

# maximum extended image number
MAX_EX_IMG_NUM = 5

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

def FontFileBuild(target, source, env):
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    FONT2C_PATH = os.path.join(SIFLI_SDK, f"tools/font2c/font2c{env['tool_suffix']}")
    filename = os.path.basename("{}".format(target[0]))
    subprocess.run([FONT2C_PATH, str(source[0])], check=True)
    shutil.move(filename, '{}'.format(target[0]))

def ModifyFontTargets(target, source, env):
    target_path = str(target[0])
    target = []
    if "tiny" in target_path:
        target_path = target_path.replace("lvsf_font", "lvsf_tiny_font")
    target.append(target_path)
            
    return target, source

def ImgResource(env, source, flags):
    target = []
    for s in source:
        t = env.ImgFile(s, FLAGS = flags)
        target.extend(t)
    
    return target

def LangBuild(target, source, env):
    import resource
    
    src_path = os.path.dirname(str(source[0]))
    dst_path = os.path.dirname(str(target[0]))
    resource.ns = True
    resource.default_language = env['DEFAULT_LANG']  
    resource.GenerateStrRes(src_path, dst_path)
    
def ModifyLangTargets(target, source, env):
    target = []
    for s in source:
        t = os.path.basename(str(s))
        t = os.path.splitext(t)[0]
        t = t + '.c'
        target.append(t)
            
    target.append('lang_pack.c')
    target.append('lang_pack.h')
    
    return target, source

def GetLangBuildObject(objs):
    objs_bak = objs[:]
    SrcRemove(objs_bak, 'lang_pack.h')
    return objs_bak

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

def ProgramBinaryBuild(target, source, env):
    import rtconfig
    program_file = str(source[0])
    program_filename = os.path.basename(program_file)
    program_name = os.path.splitext(program_filename)[0]
    target_path = os.path.dirname(program_file)
    bin_path = os.path.join(target_path, program_name + '.bin')
    
    if os.path.exists(bin_path):
        shutil.rmtree(bin_path)
    # TODO: only support keil and gcc
    if rtconfig.PLATFORM == 'armcc':
        subprocess.run(['fromelf', '--bin', str(source[0]), '--output', bin_path], check=True)
        if os.path.isdir(bin_path):
            # delete the folder to clean old files
            shutil.rmtree(bin_path)
            subprocess.run(['fromelf', '--bin', str(source[0]), '--output', bin_path], check=True)        
            dir_list = os.listdir(bin_path)
            for d in dir_list:
                if '.bin' not in d:
                    shutil.move(os.path.join(bin_path, d), os.path.join(bin_path, d + '.bin'))
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
            subprocess.run([rtconfig.OBJCPY, '-Obinary', str(source[0]), bin_path], check=True)
        else:
            os.mkdir(bin_path)
            exclude_ex_imgs = []
            for i in ex_imgs:
                ex_img_path = os.path.join(bin_path, 'ER_IROM{}.bin'.format(i))
                subprocess.run([rtconfig.OBJCPY, '-Obinary', '-j.rom{}'.format(i), str(source[0]), ex_img_path], check=True)
                exclude_ex_imgs += ['-R.rom{}'.format(i)]

            rom1_path = ex_img_path = os.path.join(bin_path, 'ER_IROM1.bin')
            subprocess.run([rtconfig.OBJCPY, '-Obinary'] + exclude_ex_imgs + [str(source[0]), rom1_path], check=True)


def ProgramHexBuild(target, source, env):
    program_file = str(source[0])
    program_filename = os.path.basename(program_file)
    program_name = os.path.splitext(program_filename)[0]
    target_path = os.path.dirname(program_file)
    hex_path = os.path.join(target_path, program_name + '.hex')
 
    if os.path.exists(hex_path):
        shutil.rmtree(hex_path)
    #TODO: only support keil and gcc
    import rtconfig
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
            subprocess.run([rtconfig.OBJCPY, '-O', 'ihex', str(source[0]), hex_path], check=True)
        else:
            os.mkdir(hex_path)
            exclude_ex_imgs = []
            for i in ex_imgs:
                ex_img_path = os.path.join(hex_path, 'ER_IROM{}.hex'.format(i))
                subprocess.run([rtconfig.OBJCPY, '-O', 'ihex', '-j.rom{}'.format(i), str(source[0]), ex_img_path], check=True)
                exclude_ex_imgs += ['-R.rom{}'.format(i)]

            rom1_path = ex_img_path = os.path.join(hex_path, 'ER_IROM1.hex')
            subprocess.run([rtconfig.OBJCPY, '-O', 'ihex'] + exclude_ex_imgs + [str(source[0]), rom1_path], check=True)


    else:    
        subprocess.run(['fromelf', '--i32', str(source[0]), '--output', hex_path], check=True)    
        if os.path.isdir(hex_path):
            # delete the folder to clean old files
            shutil.rmtree(hex_path)
            subprocess.run(['fromelf', '--i32', str(source[0]), '--output', hex_path], check=True)    
            dir_list = os.listdir(hex_path)
            for d in dir_list:
                if '.hex' not in d:
                    shutil.move(os.path.join(hex_path, d), os.path.join(hex_path, d + '.hex'))


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
    include_paths = ['-I{}'.format(path.replace('\\', '/')) for path in env['CPPPATH']]

    target_path = os.path.join(os.path.dirname(str(target[0])), 'link_copy.lds')

    p = subprocess.Popen([rtconfig.CC, '-E', '-P'] + include_paths + ['-x', 'c', str(source[0])], stdout=subprocess.PIPE)
    (result, error) = p.communicate()
    f = open(target_path, "wb")
    f.write(result)
    f.close()   

def FsBuild(target, source, env):
    import json
    import rtconfig
    
    f = open(env['PARTITION_TABLE'])
    try:
        mems = json.load(f)
    finally:
        f.close()
        
    found=0
    for mem in mems:
        mem_base = int(mem['base'], 0)
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            max_size = int(region['max_size'], 0)
            start_addr = mem_base + offset
            if "FS_REGION" in region['tags']:
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
    target = [os.path.join(env['build_dir'], 'link_copy.lds')]
    
    return target, source

def EmbeddedImgCFileBuild(target, source, env):
    target_path = os.path.dirname(str(target[0]))
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    GEN_SRC_PATH = os.path.join(SIFLI_SDK, "tools/patch/gen_src.py")
    s = str(source[0])
    if os.path.isdir(s):
        s = os.path.join(s, 'ER_IROM1.bin')
    if "acpu" in s:
        subprocess.run(['python', GEN_SRC_PATH, 'general', s, target_path, "acpu"], check=True)
        shutil.move(os.path.join(target_path, 'acpu_img.c'), str(target[0]))
    else:
        subprocess.run(['python', GEN_SRC_PATH, 'lcpu', s, target_path], check=True)
        shutil.move(os.path.join(target_path, 'lcpu_img.c'), str(target[0]))



def FtabCFileBuild(target, source, env):
    import resource
    src_file = str(source[0])
    target_file = str(target[0])
    if 'template' in os.path.splitext(src_file)[1]:
        shutil.copy(src_file, target_file)
    else:
        resource.GenFtabCFile(src_file, target_file, env['IMGS_INFO'])       

def FileCopyBuild(target, source, env):
    import resource
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
    import resource
    import rtconfig
    
    if rtconfig.ARCH=='sim':
        return

    PrintEnvList()

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

    for img in CustomImgList:
        if img['program_binary']:
            dependent_files.append(img['program_binary'])        
        if img['program_hex']:
            dependent_files.append(img['program_hex'])        
    target = main_env.DownloadScript(dependent_files)
    #Depends(target, dependent_files)

def DownloadScriptBuild(target, source, env):
    import resource

    resource.BuildJLinkLoadScript(env)

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

def GetLinkScript(proj_path,board,chip,core):
    board_path1, board_path2 = GetBoardPath(board)
    CC_TOOLS = os.getenv('RTT_CC')
    link_script=None
    link_script_template=None
    chip = chip.lower()
    core = core.lower()
    if CC_TOOLS=='keil' or CC_TOOLS=='gcc':
        if CC_TOOLS=='keil':
            ext='.sct'
        else:
            ext='.lds'
        custom_link_file_path = os.path.join(proj_path, board)
        custom_link_file_path = os.path.join(custom_link_file_path, 'link'+ext)
        if os.path.exists(custom_link_file_path):
            link_script = os.path.splitext(custom_link_file_path)[0]
            logging.debug('Use project board link file: {}'.format(link_script))
        elif os.path.exists(os.path.join(proj_path, chip + '/link'+ ext)):
            link_script = os.path.join(proj_path, chip + '/link')
            logging.debug('Use project chip link file: {}'.format(link_script))
        elif os.path.exists(os.path.join(proj_path, 'link'+ext)):
            link_script = os.path.join(proj_path, 'link')
            logging.debug('Use project link file: {}'.format(link_script))
        elif os.path.exists(os.path.join(board_path2, 'link'+ext)):    
            # no custom link file present, use link file defined by board
            link_script = os.path.join(board_path2, "link")
            logging.debug('Use board link file: {}'.format(link_script))
        else:
            SIFLI_SDK = os.getenv('SIFLI_SDK')
            if CC_TOOLS=='keil':
                link_script = os.path.join(SIFLI_SDK, "drivers/cmsis/{}/Templates/arm/{}/link".format(chip.lower(), core.lower()))
            else:
                link_script_template = os.path.join(SIFLI_SDK, "drivers/cmsis/{}/Templates/gcc/{}/link".format(chip.lower(), core.lower()))
                link_script = link_script_template
            logging.debug('Use chip link file: {}'.format(link_script))
    return link_script,link_script_template

def AddChildProj(proj_name, proj_path, img_embedded=False, shared_option=None, core=None):
    import rtconfig
    global BuildOptions

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
    proj_rtconfig.OUTPUT_DIR = os.path.join(parent_output_dir, proj_name)
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
    if os.path.isfile(os.path.join(proj_path, 'SConstruct.py')):
        child_builder = (lambda spec: (spec.loader.exec_module(mod := importlib.util.module_from_spec(spec)) or mod))(importlib.util.spec_from_file_location(proj_name, os.path.join(proj_path, 'SConstruct.py')))
        proj_env = child_builder.create_env(proj_path)
        
    else:
        SifliEnv(proj_path)

        proj_env = Environment(tools = ['mingw'],
            AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
            CC = rtconfig.CC, CFLAGS = rtconfig.CFLAGS,
            AR = rtconfig.AR, ARFLAGS = '-rc', LIBPATH=['.'],
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

    # prepare building environment
    objs = PrepareBuilding(proj_env)
    
    if 'CPPPATH' in proj_env and os.path.abspath(str(Dir('#'))) in proj_env['CPPPATH']:
        logging.debug('Root path of parent project: {}'.format(os.path.abspath(str(Dir('#')))))
        logging.debug('Search paths of child project: {}'.format(proj_env['CPPPATH']))
        raise ValueError('Root path of parent project cannot be in search paths of child project')

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
        f = open(os.path.join(rtconfig.OUTPUT_DIR, 'rtconfig.h'), 'r')
    else:     
        if BSP_Root:
            f = open(os.path.join(BSP_Root, 'rtconfig.h'), 'r')
        else:
            f = open('rtconfig.h', 'r')

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
                          help = 'set target project: mdk/mdk4/mdk5/iar/vs/vsc/ua/cdk/ses/makefile/eclipse/si')
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
                slist = [str(a).replace('\\','\\\\') for a in env['LIBPATH']]
                return '-L' + ' -L'.join(slist)

            env['EXPANDED_SOURCES'] = expand_sources
            env['EXPANDED_TARGETS'] = expand_target            
            env['EXPANDED_LDIR'] = expand_ldir
            env["TEMPFILE"] = SCons.Platform.TempFileMunge
            #env["LINKCOM"] = "${TEMPFILE('%s','$LINKCOMSTR')}"%env['LINKCOM']    
            env["LINKCOM"] = "${TEMPFILE('$LINK -o $EXPANDED_TARGETS $LINKFLAGS $__RPATH $EXPANDED_SOURCES $EXPANDED_LDIR $_LIBFLAGS','$LINKCOMSTR')}"  
            #if hasattr(SCons.Platform.TempFileMunge, 'version'):
            #    env["CCCOM"] = "${TEMPFILE('%s','$CCCOMSTR')}"%env['CCCOM']
            #    env["CXXCOM"] = "${TEMPFILE('%s','$CXXCOMSTR')}"%env['CXXCOM']
        env['TEMPFILEPREFIX'] = '@' 
        
    # patch for win32 spawn
    if env['PLATFORM'] == 'win32':
        win32_spawn = Win32Spawn()
        win32_spawn.env = env
        env['SPAWN'] = win32_spawn.spawn

    # if env['PLATFORM'] == 'win32':
    #     os.environ['PATH'] = rtconfig.EXEC_PATH + ";" + os.environ['PATH']
    # else:
    #     os.environ['PATH'] = rtconfig.EXEC_PATH + ":" + os.environ['PATH']
    
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

    # add library build action
    act = SCons.Action.Action(BuildLibInstallAction, 'Install compiled library... $TARGET')
    bld = Builder(action = act)
    Env.Append(BUILDERS = {'BuildLib': bld})
    
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
    
    # add rom library builder
    rom_lib_action = SCons.Action.Action(RomLibBuild, 'GenRomLib $TARGET')
    bld = Builder(action = rom_lib_action, suffix = env['LIBSUFFIX'], src_suffix = '.sym')
    Env.Append(BUILDERS = {"RomLib": bld})    
    
    # add ProgramBinary builder
    bin_action = SCons.Action.Action(ProgramBinaryBuild, 'Generating $TARGET ...')
    bld = Builder(action = bin_action, suffix = '.bin')
    Env.Append(BUILDERS = {"ProgramBinary": bld})
    
    # add ProgramHex builder
    hex_action = SCons.Action.Action(ProgramHexBuild, 'Generating $TARGET ...')
    bld = Builder(action = hex_action, suffix = '.hex')
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
        board_search_path = f"--board_search_path={GetOption('board_search_path')}"
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

    global BuildOptions
    global Env
    global Rtt_Root

    # patch for win32 spawn
    if env['PLATFORM'] == 'win32':
        win32_spawn = Win32Spawn()
        win32_spawn.env = env
        env['SPAWN'] = win32_spawn.spawn

    Env = env
    Rtt_Root = root_directory

    # parse bsp rtconfig.h to get used component
    PreProcessor = PatchedPreProcessor()
    f = open(bsp_directory + '/rtconfig.h', 'r')
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
    except:
        pass

    # add program path
    env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

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

    f = open(config_filename, 'r')
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

        if rtconfig.CROSS_TOOL == 'gcc':
            lds_file = Env.LdsFile([File(rtconfig.LINK_SCRIPT_SRC + '.lds')])
            Depends(program, lds_file)
            # always build lds file as it would not get rebuilt when header file changes
            AlwaysBuild(lds_file)
            if "ROM_SYM" in Env and Env['ROM_SYM']:
                Depends(program, Env['ROM_SYM'])

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

def SifliGccEnv(cpu):
    import rtconfig
    
    rtconfig.PLATFORM= 'gcc'
    rtconfig.CROSS_TOOL= 'gcc'

    # toolchains
    rtconfig.PREFIX = 'arm-none-eabi-'
    rtconfig.CC = rtconfig.PREFIX + 'gcc'
    rtconfig.AS = rtconfig.PREFIX + 'gcc'
    rtconfig.AR = rtconfig.PREFIX + 'ar'
    rtconfig.CXX = rtconfig.PREFIX + 'g++'
    rtconfig.LINK = rtconfig.PREFIX + 'gcc'
    rtconfig.STRIP = rtconfig.PREFIX + 'strip'
    rtconfig.TARGET_EXT = 'elf'
    rtconfig.SIZE = rtconfig.PREFIX + 'size'
    rtconfig.OBJDUMP = rtconfig.PREFIX + 'objdump'
    rtconfig.OBJCPY = rtconfig.PREFIX + 'objcopy'

    if GetDepend('CPU_HAS_NO_DSP_FP'):
        no_dsp_fp = True
        cpu += '+nodsp'
    else:
        no_dsp_fp = False

    DEVICE = ' -mcpu=' + cpu + ' -mthumb -ffunction-sections -fdata-sections'
    if not no_dsp_fp:
        rtconfig.CFLAGS = DEVICE + ' -mfpu=fpv5-sp-d16 -mfloat-abi=hard'
    else:
        rtconfig.CFLAGS = DEVICE + ' -mfloat-abi=soft'
    rtconfig.CFLAGS += ' -funsigned-char -fshort-enums -fshort-wchar'
    rtconfig.CFLAGS += ' -mlittle-endian -gdwarf-3 -Wno-packed -Wno-missing-noreturn -Wno-sign-conversion -Wno-unused-macros -Wnull-dereference'
    rtconfig.CFLAGS += ' -fno-unwind-tables -fno-exceptions'
    rtconfig.CFLAGS += ' -fno-common'
    
    rtconfig.CFLAGS += ' -Os -D_GNU_SOURCE'
    rtconfig.CXXFLAGS = rtconfig.CFLAGS
    if no_dsp_fp:
        rtconfig.CXXFLAGS += ' -fno-exceptions -fno-rtti'
    rtconfig.CCFLAGS =  rtconfig.CFLAGS + ' -std=c99 -Wno-missing-prototypes'
    rtconfig.AFLAGS = ' -c' + DEVICE
    if not no_dsp_fp:
        rtconfig.AFLAGS += ' -mfpu=fpv4-sp-d16 -mfloat-abi=hard'
    else:
        rtconfig.AFLAGS += ' -mfloat-abi=soft'

    rtconfig.AFLAGS += ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb '    
    
    rtconfig.LFLAGS = rtconfig.CFLAGS.strip().split()
    #  ['-mcpu=Cortex-M33', '-mthumb', '-ffunction-sections', '-fdata-sections']
    #rtconfig.LFLAGS += '-std=c99 -mfpu=fpv5-sp-d16 -mfloat-abi=hard'.split()
    if not hasattr(rtconfig, 'TARGET_NAME'):
        rtconfig.TARGET_NAME = 'rtthread'
    rtconfig.LFLAGS += ['-Wl,--no-wchar-size-warning,--gc-sections,-Map={}.map,-cref,-u,Reset_Handler'.format(rtconfig.OUTPUT_DIR + '/' + rtconfig.TARGET_NAME)]

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
    rtconfig.LINK = 'armlink'
    rtconfig.TARGET_EXT = 'axf'
    
    if GetDepend('CPU_HAS_NO_DSP_FP'):
        no_dsp_fp = True
    else:
        no_dsp_fp = False
    
    # Preproc link_script
    f = open(rtconfig.LINK_SCRIPT + '.sct', 'r')
    script = f.readlines()
    f.close()
    if '$SDK_ROOT' in script[0] or '$BSP_ROOT' in script[0] or '$BOARD_ROOT' in script[0]:
        script[0] = script[0].replace('$SDK_ROOT', os.getenv('SIFLI_SDK'))
        script[0] = script[0].replace('$BSP_ROOT', BSP_ROOT)
        # if GetBoardName():
        #     board_path1,board_path2 = GetBoardPath(GetBoardName())
        #     script[0] = script[0].replace('$BOARD_ROOT', board_path1)
        new_file_path = os.path.join(rtconfig.OUTPUT_DIR,  os.path.basename(rtconfig.LINK_SCRIPT ) + '.sct')
        if not os.path.exists(rtconfig.OUTPUT_DIR):
            logging.debug('sct dir:{}'.format(rtconfig.OUTPUT_DIR))
            Execute(Mkdir(rtconfig.OUTPUT_DIR))
        f = open(new_file_path, 'w')
        f.writelines(script)
        f.close()  
        rtconfig.LINK_SCRIPT = os.path.splitext(new_file_path)[0]
    
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
            cpu += "+nodsp"
    else:
        assert false, "Unknown cpu: {}".format(cpu)
    
    rtconfig.CFLAGS +=' -mcpu=' + cpu +  DEVICE + ' -c -ffunction-sections --target=arm-arm-none-eabi'
    rtconfig.CFLAGS += ' -fno-rtti -funsigned-char -fshort-enums -fshort-wchar'
	# -Werror 
    rtconfig.CFLAGS += ' -mlittle-endian -gdwarf-3 -Wno-builtin-macro-redefined -Wno-pointer-sign -Wno-typedef-redefinition '
    rtconfig.CFLAGS += ' -mno-outline '

    rtconfig.CFLAGS += ' -I' + rtconfig.EXEC_PATH + '/ARM/ARMCLANG/include'
    rtconfig.CFLAGS += ' -D__UVISION_VERSION="532" '
    
    rtconfig.AFLAGS += ' --cpu='+ asm_cpu + ' --cpreproc_opts=-mcpu=' + cpu +' --li -g '
    rtconfig.AFLAGS += ' --cpreproc_opts=-D__UVISION_VERSION="532" '
    rtconfig.AFLAGS += ' --diag_suppress=A1609 '

    rtconfig.CXXFLAGS = rtconfig.CFLAGS + ' -xc++ -std=c++20 -fno-exceptions ' 
    rtconfig.CXXFLAGS += ' -I' + rtconfig.EXEC_PATH + '/ARM/ARMCLANG/include/libcxx'
    rtconfig.CCFLAGS =  rtconfig.CFLAGS
    rtconfig.CFLAGS = rtconfig.CFLAGS + ' -xc -std=c99 '
    rtconfig.LFLAGS = ' --cpu=' + asm_cpu 
    if no_dsp_fp:
        rtconfig.LFLAGS = ' --fpu=SoftVFP'    
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
    board_search_path = os.getenv('BOARD_SEARCH_PATH')
    if (board_search_path is not None) and os.path.exists(os.path.join(board_search_path, board_path)):
        board_root = board_search_path
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


def PrepareEnv(board=None, board_search_path=None):
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
                    default=board_search_path,
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
        os.environ["BOARD_SEARCH_PATH"] = os.path.abspath(GetOption('board_search_path'))
        logging.info("Board search path: {}".format(os.getenv('BOARD_SEARCH_PATH')))

    if board:
        LoadRtconfig(board)
        InitBuild(None, rtconfig.OUTPUT_DIR, board)
        # construct BuildOptions
        BuildOptions = {}
        BuildOptionUpdate(BuildOptions, None)
            

def AddBootLoader(SIFLI_SDK, chip):
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

def AddFTAB(SIFLI_SDK, chip):
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

def AddDFU(SIFLI_SDK):
    proj_path = None
    proj_name = 'dfu'
    proj_path = os.path.join(SIFLI_SDK, 'example/dfu/project')
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
