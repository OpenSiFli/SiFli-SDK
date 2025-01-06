import os
import sys

from utils import *
from utils import _make_path_relative
import glob
import rtconfig

def TargetSI(env):
    project = ProjectInfo(env)

    filelist=open("si_filelist.txt","w")
    BSP_ROOT = os.path.abspath(env['BSP_ROOT'])
    RTT_ROOT = os.path.abspath(env['RTT_ROOT'])

    match_bsp = False
    if BSP_ROOT.startswith(RTT_ROOT): 
        match_bsp = True

    Files   = project['FILES']
    Headers = project['HEADERS']
    CPPDEFINES = project['CPPDEFINES']

    paths = [os.path.normpath(i) for i in project['CPPPATH']]
    CPPPATH = []
    for path in paths:
        fn = os.path.normpath(path)
        if fn[-1]==',':
            fn=fn[1:-2]
        #print(fn)
        hlist=glob.glob(fn+"\\*.h")
        
        #print(hlist);
        for hfile in hlist:
            filelist.write(hfile+"\n");
            
        if match_bsp:
            if fn.startswith(BSP_ROOT):
                fn = '$(BSP_ROOT)' + fn.replace(BSP_ROOT, '')
            elif fn.startswith(RTT_ROOT):
                fn = '$(RTT_ROOT)' + fn.replace(RTT_ROOT, '')
        else:
            if fn.startswith(RTT_ROOT):
                fn = '$(RTT_ROOT)' + fn.replace(RTT_ROOT, '')
            elif fn.startswith(BSP_ROOT):
                fn = '$(BSP_ROOT)' + fn.replace(BSP_ROOT, '')

        CPPPATH.append(fn)

    path = ''
    paths = CPPPATH
    for item in paths:
        path += '\t-I%s \\\n' % item

    """
    defines = ''
    for item in project['CPPDEFINES']:
        defines += ' -D%s' % item
    make.write('DEFINES :=')
    make.write(defines)
    make.write('\n')
    """
    files = Files
    Files = []
    
    for file in files:
        fn = os.path.normpath(file)
        filelist.write(fn+"\n");
        if match_bsp:
            if fn.startswith(BSP_ROOT):
                fn = '$(BSP_ROOT)' + fn.replace(BSP_ROOT, '')
            elif fn.startswith(RTT_ROOT):
                fn = '$(RTT_ROOT)' + fn.replace(RTT_ROOT, '')
        else:
            if fn.startswith(RTT_ROOT):
                fn = '$(RTT_ROOT)' + fn.replace(RTT_ROOT, '')
            elif fn.startswith(BSP_ROOT):
                fn = '$(BSP_ROOT)' + fn.replace(BSP_ROOT, '')
    
        Files.append(fn)

    filelist.close()
    return
