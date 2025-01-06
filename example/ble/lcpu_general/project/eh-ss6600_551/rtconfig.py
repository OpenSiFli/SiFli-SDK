import os
import argparse

SIFLI_SDK=os.getenv('SIFLI_SDK')
#output folder
OUTPUT_DIR='build/'
TARGET_NAME = 'bf0_ap'
LINK_SCRIPT = '../../linker_scripts/link_lcpu_ram'
CUSTOM_LFLAGS = 'rom.sym'
USE_MICROLIB = True





    
