import os
import rtconfig
from building import *

def create_env(proj_path = None):
    # Check SDK 
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    if not SIFLI_SDK:
        print("Please run set_env.bat in root folder of SIFLI SDK to set environment.")
        exit()

    if proj_path:
        acpu_proj_path = os.path.join(proj_path, '../acpu')
    else:
        acpu_proj_path = '../acpu'   
    acpu_proj_name = 'acpu'
    AddChildProj(acpu_proj_name, acpu_proj_path, False, core="ACPU")
    
    if proj_path is None:
        AddBootLoader(SIFLI_SDK, rtconfig.CHIP)
    
    # Set default compile options
    SifliEnv(proj_path)

    env = Environment(tools = ['mingw'],
        AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
        CC = rtconfig.CC, CCFLAGS = rtconfig.CFLAGS,
        AR = rtconfig.AR, ARFLAGS = '-rc',
        LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS)
    env.PrependENVPath('PATH', rtconfig.EXEC_PATH)
    return env

def build(env):
    # prepare building environment
    objs = PrepareBuilding(env)

    TARGET = os.path.join(env['build_dir'], rtconfig.TARGET_NAME + '.' + rtconfig.TARGET_EXT)
    # make a building
    DoBuilding(TARGET, objs)
