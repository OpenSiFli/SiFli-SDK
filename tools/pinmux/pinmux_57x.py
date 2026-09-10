from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import os.path
import sys
import struct
import openpyxl
import textwrap

def get_func_sel_num(title_row):
    if (len(title_row) > 8) and ("Function" in title_row[8]):
        func_sel_num = 16
    else:
        func_sel_num = 8    

    return func_sel_num   

def pinmux_parse_h(excel_file):
    wb=openpyxl.load_workbook(filename=excel_file)
    # funtions 
    ws=wb['Functions']
    data=ws.values
    data=list(data)
    print(
"""\
/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */\n\
""")
    print('#ifndef _BF0_PIN_CONST_H')
    print('#define _BF0_PIN_CONST_H')    
    print('#include "stdint.h"')    
    print('\n')
    print("/** @addtogroup PINMUX")
    print(" * @{")
    print(" */\n")

    for row in data:        
        if row[2]=='Function Name ':
            print('/** pin function */')
            print('typedef enum\n{\n\tPIN_FUNC_UNDEF,')
        elif (row[2] is not None) and (row[0] is None):
            print("\t/** {} */".format(row[2]))
            print('\t'+row[2]+',')
    print('\tPIN_FUNC_MAX,')
    print('} pin_function;')
    
    # HCPU pads
    ws=wb['HPSYS']
    data=ws.values
    data=list(data)
    started=0
    func_sel_num = 0
    for row in data:
        if (0 == func_sel_num) and (row is not None):
            if ("Function" in row[7]):
                func_sel_num = get_func_sel_num(row[7:])
            else:
                pass    

        if row[2]=='PAD_SA00':
            print('\n/** pin pad */')
            print('typedef enum \n{\n\tPIN_PAD_UNDEF_H,')
            started=1

        if (started==1) and (row is not None) and (row[2] is not None) and ("PAD" in row[2]):
            print("\t/** {} */".format(row[2]))
            print('\t'+row[2]+',')
    print('\tPIN_PAD_MAX_H,')

    print("/**   ")
    print(" * @}")
    print(" */\n")
    
    print('#endif')


def pinmux_parse_c(excel_file):
    wb=openpyxl.load_workbook(filename=excel_file)
    print(
"""\
/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */\n\
""")
    print('#include "bf0_hal_def.h"')
    print('#include "bf0_pin_const.h"')
    print('')

    ws=wb['HPSYS']
    data=ws.values
    data=list(data)
    started=0
    func_sel_num = 0
    func_start_col = -1
    for row in data:  
        if (0 ==  func_sel_num) and (row is not None):
            if ("Function" in row[7]):
                func_sel_num = get_func_sel_num(row[7:])
                func_start_col = 7
            else:
                pass    

        if (row[2]=='PAD_SA00'):
            started=1

            
    # Geneate each pad_xx_fsel_func_tbl
    started=0
    pad_cnt = 0
    for row in data:        
        if (row[2]=='PAD_SA00'):
            started=1
        if (started==1) and (row[2] is not None) and ("PAD" in row[2]):
            pad_cnt += 1
            pad_name = row[2].replace('PAD_', '')
            row=row[func_start_col:(func_start_col+func_sel_num)]
            print('/* PAD_{} */'.format(pad_name)) 
            print('const pin_fsel_function_t pad_{}_fsel_func_tbl[] = '.format(pad_name.lower()))
            print('{')
            for i in range(func_sel_num):
                if (i >= len(row)) or (row[i]==None) or ('#' == row[i][0]) or ('x' == row[i][0]) or ('' == row[i].strip()
                    or ('(' == row[i][0])):
                    pass
                elif 'I2C_UART' in row[i] or '_TIM' in row[i]:
                    pass
                else:
                    func_name = row[i].replace('!', '')
                    print('\t{{{}, {}}},'.format(i, func_name))
            print('\t{{{}, {}}},'.format(0, 'PIN_FUNC_UNDEF'))
            print('};\n')
   
    # Geneate each pad_fsel_func_tbls
    print('const pin_fsel_function_t *const pad_fsel_func_tbls[HPSYS_PAD_NUM] = ')
    print('{')
    for row in data:        
        if (row[2]=='PAD_SA00'):
            started=1
        if (started==1) and (row[2] is not None) and ("PAD" in row[2]):
            pad_name = row[2].replace('PAD_', '')
            row=row[func_start_col:(func_start_col+func_sel_num)]
            print('\tpad_{}_fsel_func_tbl,'.format(pad_name.lower()))
    print('};')

    # Geneate pin_function2_t enum
    started=0
    for row in data:        
        if (row[2]=='PAD_SA00'):
            print('typedef enum _pin_function2\n{')
            print(
"""\
\t/****************************************************************************
\t *  Dedicated pin function part
\t *  Function could be assigned to specific pad
\t ****************************************************************************/\
""")
            started=1
        if (started==1) and (row[2] is not None) and ("PAD" in row[2]):
            pad_name = row[2].replace('PAD_', '')
            row=row[func_start_col:(func_start_col+func_sel_num)]
            print('\n \t/* PAD_{} */'.format(pad_name)) 
            for i in range(func_sel_num):
                if (i >= len(row)) or (row[i]==None) or ('#' == row[i][0]) or ('x' == row[i][0]) or ('' == row[i].strip()
                    or ('(' == row[i][0])):
                    pass
                elif 'I2C_UART' in row[i] or '_TIM' in row[i] or 'SPI1' in row[i] or 'SPI2' in row[i] or 'I2S' in row[i] or 'PDM' in row[i]:
                    pass
                else:
                    func_name = row[i].replace('!', '')
                    print('\tPIN_FUNC_ENUM_DEF({}, {}, {}),'.format(pad_name, func_name, i))
   
    print('')
    print(
"""\
\t/****************************************************************************
\t * Matrix pin function part
\t * Function in the PIN_MATRIX_FUNC_LIST could be assigned to any pad
\t *****************************************************************************/

\t/* Function name is like: PAD_PA00_I2C1_SCL*/\
""")
    for i in range(58):
        print('\tFOREACH_FUNC(PIN_MATRIX_PINMUX_ENUM_DEF, PA{:02d}, PIN_MATRIX_FUNC_LIST),'.format(i))
    
    print('} pin_function2;')    



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="General Usage ", 
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=textwrap.dedent('''\
            python pinmux.py --excel=<filename> --output=<output folder>
         '''))
    parser.add_argument(
        '--excel',
        type=str,
        default='pinmux.xslx',
        help='Pinmux definition file')
    parser.add_argument(
        '--type',
        type=str,
        choices=["c", "h"], 
        default='h',
        help='Output file type')
        
    FLAGS, unparsed = parser.parse_known_args()    
    if (FLAGS.type=='c'):
        pinmux_parse_c(FLAGS.excel)
    else:
        pinmux_parse_h(FLAGS.excel)

        
