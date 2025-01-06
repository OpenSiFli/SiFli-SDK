#
# File      : ua.py
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
#

import os
import sys
from utils import _make_path_relative

def PrefixPath(prefix, path):
    path = os.path.abspath(path)
    prefix = os.path.abspath(prefix)

    if sys.platform == 'win32':
        prefix = prefix.lower()
        path = path.lower()

    if path.startswith(prefix):
        return True
    
    return False

def PrepareUA(project, RTT_ROOT, BSP_ROOT):
    with open('rtua.py', 'w') as ua:
        # ua.write('import os\n')
        # ua.write('import sys\n')
        ua.write('\n')
        
        print RTT_ROOT
        
        CPPPATH = []
        CPPDEFINES = []

        for group in project:
            # get each include path
            if 'CPPPATH' in group and group['CPPPATH']:
                CPPPATH += group['CPPPATH']

            # get each group's definitions
            if 'CPPDEFINES' in group and group['CPPDEFINES']:
                CPPDEFINES += group['CPPDEFINES']

        if len(CPPPATH):
            # use absolute path 
            for i in range(len(CPPPATH)):
                CPPPATH[i] = os.path.abspath(CPPPATH[i])

            # remove repeat path
            paths = [i for i in set(CPPPATH)]
            CPPPATH = []
            for path in paths:
                if PrefixPath(RTT_ROOT, path):
                    CPPPATH += ['RTT_ROOT + "/%s",' % _make_path_relative(RTT_ROOT, path).replace('\\', '/')]
                
                elif PrefixPath(BSP_ROOT, path):
                    CPPPATH += ['BSP_ROOT + "/%s",' % _make_path_relative(BSP_ROOT, path).replace('\\', '/')]
                else:
                    CPPPATH += ['"%s",' % path.replace('\\', '/')]

            CPPPATH.sort()
            ua.write('def GetCPPPATH(BSP_ROOT, RTT_ROOT):\n')
            ua.write('\tCPPPATH=[\n')
            for path in CPPPATH:
                ua.write('\t\t%s\n' % path)
            ua.write('\t]\n\n')
            ua.write('\treturn CPPPATH\n\n')
        else:
            ua.write('def GetCPPPATH(BSP_ROOT, RTT_ROOT):\n')
            ua.write('\tCPPPATH=[]\n\n')
            ua.write('\treturn CPPPATH\n\n')

        if len(CPPDEFINES):
            CPPDEFINES = [i for i in set(CPPDEFINES)]

            ua.write('def GetCPPDEFINES():\n')
            ua.write('\tCPPDEFINES=%s\n' % str(CPPDEFINES))
            ua.write('\treturn CPPDEFINES\n\n')

        else:
            ua.write('def GetCPPDEFINES():\n')
            ua.write('\tCPPDEFINES=""\n\n')
            ua.write('\treturn CPPDEFINES\n\n')


def BuildEnv(BSP_ROOT, RTT_ROOT):
    if BSP_ROOT == None:
        if os.getenv('BSP_ROOT'):
            BSP_ROOT = os.getenv('BSP_ROOT')
        else:
            #print 'Please set BSP(board support package) directory!'
            #exit(-1)
            BSP_ROOT = '.'

    if not os.path.exists(BSP_ROOT):
        print 'No BSP(board support package) directory found!'
        exit(-1)

    #if (BSP_ROOT=='.'):
    #    BSP_ROOT='..\\'

    if RTT_ROOT == None:
        # get RTT_ROOT from BSP_ROOT
        sys.path = sys.path + [BSP_ROOT]
        try:
            import rtconfig
            SIFLI_SDK=os.getenv('SIFLI_SDK')
            RTT_ROOT = SIFLI_SDK + 'rtos/rtthread'
        except Exception as e:
            print 'Import rtconfig.py in BSP(board support package) failed.'
            print e
            exit(-1)

    global Rtt_Root
    global BSP_Root

    Rtt_Root = RTT_ROOT
    BSP_Root = BSP_ROOT

from building import *
def BuildLibrary(TARGET, SConscriptFile, VAR_PATH = '', BSP_ROOT = None, RTT_ROOT = None):
    global Env
    global Rtt_Root
    global BSP_Root
    
    # add comstr option
    try:
        AddOption('--verbose',
                    dest='verbose',
                    action='store_true',
                    default=False,
                    help='print verbose information during build')
    except:
        pass                


    # build application in host 
    #if BSP_ROOT == None and RTT_ROOT == None and not os.getenv('BSP_ROOT'):
    #    print("build host")
    #    BuildHostLibrary(TARGET, SConscriptFile)
    #    return


    #if RTT_ROOT == None and os.getenv('RTT_ROOT'):
    #    RTT_ROOT = os.getenv('RTT_ROOT')

    # handle BSP_ROOT and RTT_ROOT
    BuildEnv(BSP_ROOT, RTT_ROOT)

    sys.path = sys.path + [os.path.join(Rtt_Root, 'tools'), BSP_Root]
    # get configuration from BSP 
    import rtconfig 
    # Set default compile options
    SifliEnv(BSP_Root)

    from rtua import GetCPPPATH
    from rtua import GetCPPDEFINES
    from building import PrepareModuleBuilding

    linkflags = rtconfig.M_LFLAGS + ' -e 0'
    CPPPATH = GetCPPPATH(BSP_Root, Rtt_Root)

    if rtconfig.PLATFORM == 'cl': 
        Env = Environment(TARGET_ARCH='x86')
        Env.Append(CCFLAGS=rtconfig.M_CFLAGS)
        Env.Append(LINKFLAGS=rtconfig.M_LFLAGS)
        Env.Append(CPPPATH=CPPPATH)
        Env.Append(LIBS='rtthread', LIBPATH=BSP_Root)
        Env.Append(CPPDEFINES=GetCPPDEFINES() + ['RTT_IN_MODULE'])
        Env.PrependENVPath('PATH', rtconfig.EXEC_PATH)
    else:
        Env = Environment(tools = ['mingw'],
            AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
            CC = rtconfig.CC, CCFLAGS = rtconfig.M_CFLAGS,
            CPPDEFINES = GetCPPDEFINES(),
            CXX = rtconfig.CXX, AR = rtconfig.AR, ARFLAGS = '-rc',
            LINK = rtconfig.LINK, LINKFLAGS = linkflags,
            CPPPATH = CPPPATH)

    if not GetOption('verbose'):
        # override the default verbose command string
        Env.Replace(
            ARCOMSTR = 'AR $TARGET',
            ASCOMSTR = 'AS $TARGET',
            ASPPCOMSTR = 'AS $TARGET',
            CCCOMSTR = 'CC $TARGET',
            CXXCOMSTR = 'CXX $TARGET',
            LINKCOMSTR = 'LINK $TARGET'
        )

    Env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

    PrepareModuleBuilding(Env, Rtt_Root, BSP_Root)

    objs = SConscript(SConscriptFile, variant_dir="output/" + VAR_PATH, duplicate=0)
    
    # build program 
    if rtconfig.PLATFORM == 'cl':
        dll_target = TARGET.replace('.so', '.dll')
        target = Env.SharedLibrary(dll_target, objs)

        target = Command("$TARGET", dll_target, [Move(TARGET, dll_target)])
        # target = dll_target
    else:
        print('output/' + TARGET)
        target = Env.Program('output/' + TARGET, objs)
        target_nostrip = Env.Program('output/' + TARGET + '.nostrip', objs)

    if hasattr(rtconfig, 'M_POST_ACTION'):
        Env.AddPostAction(target, rtconfig.M_POST_ACTION)


