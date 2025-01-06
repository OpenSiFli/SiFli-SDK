from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import os.path
import re
import sys
import struct
import openpyxl
import textwrap

rename_rules=[
    #r"rt_hw_usart_init2",
    #r"rt_hw_serial_register",
    #r"libc_system_init",
    #r"rt_hw_serial_isr",
    #r"HAL_DBG_printf",
    #r"HAL_FACC_*",
]

def sym_parse(srcfile, dstfile):
    fp = open(srcfile, 'r')
    fp2 = open(dstfile, 'w')
    for line in fp:
        if (re.match(r"^#",line)):
            fp2.writelines(line)
            continue
        line = line.rstrip('\n')
        info = re.split(' ', line)
        renamed = 0
        for rule in rename_rules:
            if (re.match(rule,info[2])):
                info[2] = info[2]+"_rom"
                fp2.writelines(info[0]+" "+info[1]+" "+info[2]+'\n')
                renamed = 1
                break
        if (renamed == 0):
            fp2.writelines(line+'\n')
    fp.close()
    fp2.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Filter symbole file Usage ", 
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=textwrap.dedent('''\
            python filter.py --src=<filename> --dst=<output filename>
         '''))
    parser.add_argument(
        '--src',
        type=str,
        default='rom.sym',
        help='Symbol file')
    parser.add_argument(
        '--dst',
        type=str,
        default='bf0_ap.sym ',
        help='Symbol file after revise')
        
    FLAGS, unparsed = parser.parse_known_args()    
    sym_parse(FLAGS.src, FLAGS.dst)


