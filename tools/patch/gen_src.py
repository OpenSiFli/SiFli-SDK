# SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import re
import struct
import shutil

def print_file(file_name, fpout):
    fp_bin=open(file_name, 'rb')
    data=fp_bin.read(4)
    count=0
    while (len(data)>0):
        if len(data) != 4:
            data += ('\x00'*(4-len(data)))
        temp=struct.unpack("<L", data)
        if ((count%4)==0):
            fpout.write("\t")        
        fpout.write("0x%08X,"%(temp))
        count=count+1
        if ((count%4)==0):
            fpout.write("\n")
        data=fp_bin.read(4)    
    fp_bin.close()

def gen_lcpu_img(src,dest,rom=False):
    if not os.path.exists(os.path.dirname(dest)):
        os.makedirs(os.path.dirname(dest))
    if os.path.isdir(dest):
        fpout=open(os.path.join(dest, 'lcpu_img.c'),"w+")
    else:
        fpout=open(dest,"w+")    
    fpout.write("#include <stdint.h>\n")
    fpout.write("#include <string.h>\n")
    fpout.write("#include \"rtconfig.h\"\n")
    fpout.write("#include \"mem_map.h\"\n")
    fpout.write("#include \"register.h\"\n\n")
    if (rom==True):
        fpout.write("#undef HCPU_LCPU_CODE_START_ADDR \n")        
        fpout.write("#define HCPU_LCPU_CODE_START_ADDR (LCPU_ROM_CODE_START_ADDR+0x0B000000)\n")        
    if os.path.isdir(src):        
        fpout.write("const unsigned int g_lcpu_bin1[]= { \n")
        print_file(src+'/ER_IROM1', fpout)
        fpout.write("};\n")
        fpout.write("const unsigned int g_lcpu_bin2[]= { \n")
        print_file(src+'/ER_IROM2', fpout)
        fpout.write("};\n")
        fpout.write("uint32_t lcpu_ramcode_len()\n{\n")
        fpout.write("\treturn sizeof(g_lcpu_bin1)+sizeof(g_lcpu_bin2);\n")
        fpout.write("}\n")
        fpout.write("void lcpu_img_install()\n{\n")
        fpout.write("#if (LPSYS_RAM_CBUS_BASE < (LPSYS_SRAM_BASE - 0x20000000)) \n")
        fpout.write("\tif (sizeof(g_lcpu_bin1) <= LPSYS_ITCM_SIZE)\n")
        fpout.write("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin1, sizeof(g_lcpu_bin1));\n")
        fpout.write("\telse\n\t{\n")
        fpout.write("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin1, LPSYS_ITCM_SIZE);\n")
        fpout.write("\t\tmemcpy((void *)(LPSYS_SRAM_BASE), (uint8_t *)g_lcpu_bin1 + LPSYS_ITCM_SIZE, sizeof(g_lcpu_bin1) - LPSYS_ITCM_SIZE);\n")
        fpout.write("\t}\n#else\n")
        fpout.write("\tmemcpy((void*)(HCPU_LCPU_CODE_START_ADDR),g_lcpu_bin1,sizeof(g_lcpu_bin1));\n")
        fpout.write("#endif\n")
        fpout.write("\tmemcpy((void*)(LCPU_DTCM_ADDR_2_HCPU_ADDR(LPSYS_DTCM_BASE)),g_lcpu_bin2,sizeof(g_lcpu_bin2));\n")
        fpout.write("}\n")
    else:
        fpout.write("const unsigned int g_lcpu_bin[]= { \n")
        print_file(src, fpout)
        fpout.write("};\n")
        fpout.write("uint32_t lcpu_ramcode_len()\n{\n")
        fpout.write("\treturn sizeof(g_lcpu_bin);\n")
        fpout.write("}\n")		
        fpout.write("void lcpu_img_install()\n{\n")
        fpout.write("#if (LPSYS_RAM_CBUS_BASE < (LPSYS_SRAM_BASE - 0x20000000)) \n")
        fpout.write("\tif (sizeof(g_lcpu_bin) <= LPSYS_ITCM_SIZE)\n")
        fpout.write("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, sizeof(g_lcpu_bin));\n")
        fpout.write("\telse\n\t{\n")
        fpout.write("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, LPSYS_ITCM_SIZE);\n")
        fpout.write("\t\tmemcpy((void *)(LPSYS_SRAM_BASE), (uint8_t *)g_lcpu_bin + LPSYS_ITCM_SIZE, sizeof(g_lcpu_bin) - LPSYS_ITCM_SIZE);\n")
        fpout.write("\t}\n#else\n")
        fpout.write("\tmemcpy((void*)(HCPU_LCPU_CODE_START_ADDR),g_lcpu_bin,sizeof(g_lcpu_bin));\n")
        fpout.write("#endif\n}\n")
    fpout.close()


def gen_lcpu_img_mix(a1src, a3src, dest):
    if os.path.isdir(dest):
        fpout=open(os.path.join(dest, 'lcpu_img.c'),"w+")
    else:
        fpout=open(dest,"w+")    
    fpout.write('''
#include <stdint.h>
#include <string.h>
#include "mem_map.h"
#include "rtconfig.h"
#include "register.h"
#include "board.h"

''')
    
    fpout.write("const unsigned int g_lcpu_bin[]= { \n")
    print_file(a1src, fpout)
    fpout.write("};\n")
    fpout.write("const unsigned int g_lcpu_bin_a3[]= { \n")
    print_file(a3src, fpout)
    fpout.write("};\n")
     
    fpout.write('''
uint32_t lcpu_ramcode_len()
{
    uint32_t *pData = NULL;
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    
    if(rev_id == HAL_CHIP_REV_ID_A3)
    {
	    pData = (uint32_t *)&g_lcpu_bin_a3[0];
    }
    else
    {
	    pData = (uint32_t *)&g_lcpu_bin[0];
    }
	
	return pData[0];
}

void lcpu_img_install()
{
    uint32_t *pData = NULL;
    uint8_t *pBuf = NULL;
    uint8_t rev_id;
    int r;

    rev_id = __HAL_SYSCFG_GET_REVID();
    
    if(rev_id == HAL_CHIP_REV_ID_A3)
    {
        pData = (uint32_t *)&g_lcpu_bin_a3[0];
    }
    else
    {
        pData = (uint32_t *)&g_lcpu_bin[0];
    }
    
    pBuf = (uint8_t *)malloc(pData[0]);
	
    if(pBuf)
    {
        EZIP_DecodeConfigTypeDef config;
        EZIP_HandleTypeDef ezip_handle = {0};
		
        config.input = (uint8_t *)&pData[1];
        config.output = pBuf;
        config.start_x = 0;
        config.start_y = 0;
        config.width = 0;
        config.height = 0;
        config.work_mode = HAL_EZIP_MODE_GZIP;
        config.output_mode = HAL_EZIP_OUTPUT_AHB;
        ezip_handle.Instance = hwp_ezip;
		
		register rt_base_t ret;
	    ret = rt_hw_interrupt_disable();
        HAL_EZIP_Init(&ezip_handle);
        r = HAL_EZIP_Decode(&ezip_handle, &config);
		rt_hw_interrupt_enable(ret);
        RT_ASSERT(HAL_OK == r);

#if (LPSYS_RAM_CBUS_BASE < (LPSYS_SRAM_BASE - 0x20000000)) && !defined(SOC_BF_Z0)
        if (pData[0] <= LPSYS_ITCM_SIZE)
        {
            memcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), pBuf, pData[0]);
        }
        else
        {
            memcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), pBuf, LPSYS_ITCM_SIZE);
            memcpy((void *)(LPSYS_SRAM_BASE), &pBuf[LPSYS_ITCM_SIZE], pData[0]-LPSYS_ITCM_SIZE);
        }
#else
        memcpy((void*)(HCPU_LCPU_CODE_START_ADDR), pBuf, pData[0]);
#endif

        free(pBuf);
    }
    else
    {
        RT_ASSERT(0);
    }
}

''')
    fpout.close()

def gen_lcpu_img_xip(src,dest):
    if os.path.isdir(dest):
        fpout=open(os.path.join(dest, 'lcpu_img.c'),"w+")
    else:
        fpout=open(dest,"w+")    
    fpout.write("#include <stdint.h>\n")
    fpout.write("#include <string.h>\n")
    fpout.write("#include \"mem_map.h\"\n")
    fpout.write("#include \"rtconfig.h\"\n")
    fpout.write("#include \"register.h\"\n\n")
    if os.path.isdir(src):        
        fpout.write("const unsigned int g_lcpu_bin[]= { \n")
        print_file(src+'/ER_IROM0', fpout)
        fpout.write("};\n")
        fpout.write("uint32_t lcpu_ramcode_len()\n{\n")
        fpout.write("\treturn sizeof(g_lcpu_bin);\n")
        fpout.write("}\n")	
        fpout.write("void lcpu_img_install()\n{\n")
        fpout.write("#if (LPSYS_RAM_CBUS_BASE < 0x100000) \n")
        fpout.write("\tif (sizeof(g_lcpu_bin) <= LPSYS_ITCM_SIZE)\n")
        fpout.write("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, sizeof(g_lcpu_bin));\n")
        fpout.write("\telse\n\t{\n")
        fpout.write("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, LPSYS_ITCM_SIZE);\n")
        fpout.write("\t\tmemcpy((void *)(LPSYS_SRAM_BASE), (uint8_t *)g_lcpu_bin + LPSYS_ITCM_SIZE, sizeof(g_lcpu_bin) - LPSYS_ITCM_SIZE);\n")
        fpout.write("\t}\n#else\n")
        fpout.write("\tmemcpy((void*)(HCPU_LCPU_CODE_START_ADDR),g_lcpu_bin,sizeof(g_lcpu_bin));\n")
        fpout.write("#endif\n")
        fpout.write("}\n")
    else:
        assert False, "dir not found"
    
    fpout.close()

def gen_lcpu_patch(src,dest):
    fpout=open(dest +'lcpu_patch.c',"w+")
    fpout.write("#include <stdint.h>\n")
    fpout.write("#include <string.h>\n")
    fpout.write("#include \"bf0_hal.h\"\n")
    fpout.write("#include \"mem_map.h\"\n")
    fpout.write("#include \"register.h\"\n")
    fpout.write("#include \"bf0_hal_patch.h\"\n")
    fpout.write("#ifdef HAL_LCPU_PATCH_MODULE\n")
    fpout.write("const unsigned int g_lcpu_patch_list[]= { \n")
    print_file(src+'patch_list.bin', fpout)
    fpout.write("};\n")
    fpout.write("const unsigned int g_lcpu_patch_bin[]= { \n")
    print_file(src+'lcpu_rom_patch.bin', fpout)
    fpout.write("};\n")
    fpout.write("void lcpu_patch_install()\n{\n")
    fpout.write("#ifdef SOC_BF0_HCPU\n")
    fpout.write("\tmemset((void*)(LCPU_PATCH_START_ADDR_S),0,LCPU_PATCH_TOTAL_SIZE);\n")
    fpout.write("\tmemcpy((void*)(LCPU_PATCH_START_ADDR_S),g_lcpu_patch_bin,sizeof(g_lcpu_patch_bin));\n")
    fpout.write("#else\n")
    fpout.write("\tmemset((void*)(LCPU_PATCH_START_ADDR),0,LCPU_PATCH_TOTAL_SIZE);\n")
    fpout.write("\tmemcpy((void*)(LCPU_PATCH_START_ADDR),g_lcpu_patch_bin,sizeof(g_lcpu_patch_bin));\n")
    fpout.write("#endif\n")
    fpout.write("\tmemcpy((void*)(LCPU_PATCH_RECORD_ADDR),g_lcpu_patch_list,sizeof(g_lcpu_patch_list));\n")
    fpout.write("\tHAL_PATCH_install();\n}\n")
    fpout.write("#endif\n")
    fpout.close()
    
def gen_general_img(name, src, dest):
    if not os.path.exists(os.path.dirname(dest)):
        os.makedirs(os.path.dirname(dest))
    if os.path.isdir(dest):
        fpout=open(os.path.join(dest, '{}_img.c').format(name),"w+")
    else:
        fpout=open(dest,"w+")    
    fpout.write("#include <stdint.h>\n")
    fpout.write("#include <string.h>\n")
    fpout.write("#include \"mem_map.h\"\n")
    fpout.write("#include \"rtconfig.h\"\n")
    fpout.write("#include \"register.h\"\n\n")

    bin_var_name = "g_{}_bin".format(name)
    fpout.write("const unsigned int {}[]= {{ \n".format(bin_var_name))
    print_file(src, fpout)
    fpout.write("};\n")
    fpout.write("uint32_t {}_ramcode_len(void)\n{{\n".format(name))
    fpout.write("\treturn sizeof({});\n".format(bin_var_name))
    fpout.write("}\n")

    fpout.close()


if __name__ == '__main__':    
    if (sys.argv[1]=='lcpu'):
        gen_lcpu_img(sys.argv[2], sys.argv[3])
    if (sys.argv[1]=='lcpu_mix'):
        gen_lcpu_img_mix(sys.argv[2], sys.argv[3], sys.argv[4])        
    if (sys.argv[1]=='lcpu_rom'):
        gen_lcpu_img(sys.argv[2], sys.argv[3], True)
    if (sys.argv[1]=='lcpu_patch'):
        gen_lcpu_patch(sys.argv[2], sys.argv[3])
    if (sys.argv[1]=='lcpu_xip'):
        gen_lcpu_img_xip(sys.argv[2], sys.argv[3])
    if (sys.argv[1]=='general'):
        gen_general_img(sys.argv[4], sys.argv[2], sys.argv[3])