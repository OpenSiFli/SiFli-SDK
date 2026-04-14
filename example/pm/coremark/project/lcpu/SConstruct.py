import os
import rtconfig
from building import *

def create_env(proj_path = None):
    # Check SDK 
    SIFLI_SDK = os.getenv('SIFLI_SDK')
    if not SIFLI_SDK:
        print("Please run set_env.bat in root folder of SIFLI SDK to set environment.")
        exit()


    # Set default rtconfig.xxx, select from HCPU, LCPU and BCPU
    SifliEnv(proj_path)

def build():
    # prepare building environment
    objs = PrepareBuilding(None)
    env = GetCurrentEnv()

    if (rtconfig.CHIP != "SF32LB52X") and (rtconfig.CHIP != "SF32LB55X"):
        env.AppendUnique(CPPDEFINES = ['ee_printf=custom_printf'])

    TARGET = os.path.join(env['build_dir'], rtconfig.TARGET_NAME + '.' + rtconfig.TARGET_EXT)
    # make a building
    DoBuilding(TARGET, objs)
