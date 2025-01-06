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
    #os.system("prebuild.bat")
    ################################## change rtconfig.xxx to customize build ########################################
    # print (rtconfig.OUTPUT_DIR)

    #OUTPUT_DIR=rtconfig.OUTPUT_DIR
    #TARGET = OUTPUT_DIR + rtconfig.TARGET_NAME + '.' + rtconfig.TARGET_EXT
    #BIN_TARGET = OUTPUT_DIR + rtconfig.TARGET_NAME + '.bin'
    #rtconfig.POST_ACTION ='fromelf --bin $TARGET --output ' + OUTPUT_DIR + rtconfig.TARGET_NAME + '.bin \npython ../../../../../tools/patch/gen_src.py lcpu '+ BIN_TARGET + ' ../hcpu/src/ '
    if (rtconfig.CHIP != "SF32LB52X") and (rtconfig.CHIP != "SF32LB55X"):
        CPPDEFINES = ['ee_printf=custom_printf']
    else:
        CPPDEFINES = []

    env = Environment(tools = ['mingw'],
        AS = rtconfig.AS, ASFLAGS = rtconfig.AFLAGS,
        CC = rtconfig.CC, CCFLAGS = rtconfig.CFLAGS,
        AR = rtconfig.AR, ARFLAGS = '-rc',
        LINK = rtconfig.LINK, LINKFLAGS = rtconfig.LFLAGS, CPPDEFINES = CPPDEFINES)
    env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

    return env

def build(env):
    # prepare building environment
    objs = PrepareBuilding(env)

    TARGET = os.path.join(env['build_dir'], rtconfig.TARGET_NAME + '.' + rtconfig.TARGET_EXT)
    # make a building
    DoBuilding(TARGET, objs)
