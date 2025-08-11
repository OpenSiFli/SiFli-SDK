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
        lcpu_proj_path = os.path.join(proj_path, '../lcpu')
    else:
        lcpu_proj_path = '../lcpu'   
    lcpu_proj_name = 'lcpu'
    AddChildProj(lcpu_proj_name, lcpu_proj_path, True, core="LCPU")
    
    if proj_path is None:
        AddBootLoader(SIFLI_SDK, rtconfig.CHIP)
    
    # Set default compile options
    SifliEnv(proj_path)


def build():
    # prepare building environment
    objs = PrepareBuilding(None)
    env = GetCurrentEnv()

    TARGET = os.path.join(env['build_dir'], rtconfig.TARGET_NAME + '.' + rtconfig.TARGET_EXT)
    # make a building
    DoBuilding(TARGET, objs)

    return env
