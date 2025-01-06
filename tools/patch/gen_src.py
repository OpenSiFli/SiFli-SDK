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
            print>>fpout, ("\t"),        
        print>>fpout, ("0x%08X,"%(temp)),
        count=count+1
        if ((count%4)==0):
            print>>fpout, ("\n"),
        data=fp_bin.read(4)    
    fp_bin.close()

def gen_lcpu_img(src,dest,rom=False):
    if not os.path.exists(os.path.dirname(dest)):
        os.makedirs(os.path.dirname(dest))
    if os.path.isdir(dest):
        fpout=open(os.path.join(dest, 'lcpu_img.c'),"w+")
    else:
        fpout=open(dest,"w+")    
    print >>fpout, ("#include <stdint.h>\n"),
    print >>fpout, ("#include <string.h>\n"),
    print >>fpout, ("#include \"mem_map.h\"\n"),
    print >>fpout, ("#include \"rtconfig.h\"\n"),
    print >>fpout, ("#include \"register.h\"\n\n"),
    if (rom==True):
        print >>fpout, ("#undef HCPU_LCPU_CODE_START_ADDR \n"),        
        print >>fpout, ("#define HCPU_LCPU_CODE_START_ADDR (LCPU_ROM_CODE_START_ADDR+0x0B000000)\n"),        
    if os.path.isdir(src):        
        print >>fpout, ("const unsigned int g_lcpu_bin1[]= { ")
        print_file(src+'/ER_IROM1', fpout)
        print >>fpout, ("};")
        print >>fpout, ("const unsigned int g_lcpu_bin2[]= { ")
        print_file(src+'/ER_IROM2', fpout)
        print >>fpout, ("};")
        print >>fpout, ("uint32_t lcpu_ramcode_len()\n{\n"),
        print >>fpout, ("\treturn sizeof(g_lcpu_bin1)+sizeof(g_lcpu_bin2);\n"),
        print >>fpout, ("}\n"),
        print >>fpout, ("void lcpu_img_install()\n{\n"),
        print >>fpout, ("#if (LPSYS_RAM_CBUS_BASE < (LPSYS_SRAM_BASE - 0x20000000)) \n"),
        print >>fpout, ("\tif (sizeof(g_lcpu_bin1) <= LPSYS_ITCM_SIZE)\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin1, sizeof(g_lcpu_bin1));\n"),
        print >>fpout, ("\telse\n\t{\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin1, LPSYS_ITCM_SIZE);\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LPSYS_SRAM_BASE), (uint8_t *)g_lcpu_bin1 + LPSYS_ITCM_SIZE, sizeof(g_lcpu_bin1) - LPSYS_ITCM_SIZE);\n"),
        print >>fpout, ("\t}\n#else\n"),
        print >>fpout, ("\tmemcpy((void*)(HCPU_LCPU_CODE_START_ADDR),g_lcpu_bin1,sizeof(g_lcpu_bin1));\n"),
        print >>fpout, ("#endif\n"),
        print >>fpout, ("\tmemcpy((void*)(LCPU_DTCM_ADDR_2_HCPU_ADDR(LPSYS_DTCM_BASE)),g_lcpu_bin2,sizeof(g_lcpu_bin2));\n"),
        print >>fpout, ("}\n"),
    else:
        print >>fpout, ("const unsigned int g_lcpu_bin[]= { ")
        print_file(src, fpout)
        print >>fpout, ("};")
        print >>fpout, ("uint32_t lcpu_ramcode_len()\n{\n"),
        print >>fpout, ("\treturn sizeof(g_lcpu_bin);\n"),
        print >>fpout, ("}\n"),		
        print >>fpout, ("void lcpu_img_install()\n{\n"),
        print >>fpout, ("#if (LPSYS_RAM_CBUS_BASE < (LPSYS_SRAM_BASE - 0x20000000)) \n"),
        print >>fpout, ("\tif (sizeof(g_lcpu_bin) <= LPSYS_ITCM_SIZE)\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, sizeof(g_lcpu_bin));\n"),
        print >>fpout, ("\telse\n\t{\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, LPSYS_ITCM_SIZE);\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LPSYS_SRAM_BASE), (uint8_t *)g_lcpu_bin + LPSYS_ITCM_SIZE, sizeof(g_lcpu_bin) - LPSYS_ITCM_SIZE);\n"),
        print >>fpout, ("\t}\n#else\n"),
        print >>fpout, ("\tmemcpy((void*)(HCPU_LCPU_CODE_START_ADDR),g_lcpu_bin,sizeof(g_lcpu_bin));\n"),
        print >>fpout, ("#endif\n}\n"),
    fpout.close()


def gen_lcpu_img_mix(a1src, a3src, dest):
    if os.path.isdir(dest):
        fpout=open(os.path.join(dest, 'lcpu_img.c'),"w+")
    else:
        fpout=open(dest,"w+")    
    print >>fpout, ('''
#include <stdint.h>
#include <string.h>
#include \"mem_map.h\"
#include \"rtconfig.h\"
#include \"register.h\"
#include \"board.h\"

''')
    
    print >>fpout, ("const unsigned int g_lcpu_bin[]= { ")
    print_file(a1src, fpout)
    print >>fpout, ("};")
    print >>fpout, ("const unsigned int g_lcpu_bin_a3[]= { ")
    print_file(a3src, fpout)
    print >>fpout, ("};")
     
    print >>fpout, ('''
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
    print >>fpout, ("#include <stdint.h>\n"),
    print >>fpout, ("#include <string.h>\n"),
    print >>fpout, ("#include \"mem_map.h\"\n"),
    print >>fpout, ("#include \"rtconfig.h\"\n"),
    print >>fpout, ("#include \"register.h\"\n\n"),
    if os.path.isdir(src):        
        print >>fpout, ("const unsigned int g_lcpu_bin[]= { ")
        print_file(src+'/ER_IROM0', fpout)
        print >>fpout, ("};")
        print >>fpout, ("uint32_t lcpu_ramcode_len()\n{\n"),
        print >>fpout, ("\treturn sizeof(g_lcpu_bin);\n"),
        print >>fpout, ("}\n"),	
        print >>fpout, ("void lcpu_img_install()\n{\n"),
        print >>fpout, ("#if (LPSYS_RAM_CBUS_BASE < 0x100000) \n"),
        print >>fpout, ("\tif (sizeof(g_lcpu_bin) <= LPSYS_ITCM_SIZE)\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, sizeof(g_lcpu_bin));\n"),
        print >>fpout, ("\telse\n\t{\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE)), g_lcpu_bin, LPSYS_ITCM_SIZE);\n"),
        print >>fpout, ("\t\tmemcpy((void *)(LPSYS_SRAM_BASE), (uint8_t *)g_lcpu_bin + LPSYS_ITCM_SIZE, sizeof(g_lcpu_bin) - LPSYS_ITCM_SIZE);\n"),
        print >>fpout, ("\t}\n#else\n"),
        print >>fpout, ("\tmemcpy((void*)(HCPU_LCPU_CODE_START_ADDR),g_lcpu_bin,sizeof(g_lcpu_bin));\n"),
        print >>fpout, ("#endif\n"),
        print >>fpout, ("}\n"),
    else:
        assert False, "dir not found"
    
    fpout.close()

def gen_lcpu_patch(src,dest):
    fpout=open(dest +'lcpu_patch.c',"w+")
    print >>fpout, ("#include <stdint.h>\n"),
    print >>fpout, ("#include <string.h>\n"),
    print >>fpout, ("#include \"bf0_hal.h\"\n"),
    print >>fpout, ("#include \"mem_map.h\"\n"),
    print >>fpout, ("#include \"register.h\"\n"),
    print >>fpout, ("#include \"bf0_hal_patch.h\"\n"),
    print >>fpout, ("#ifdef HAL_LCPU_PATCH_MODULE\n"),
    print >>fpout, ("const unsigned int g_lcpu_patch_list[]= { ")
    print_file(src+'patch_list.bin', fpout)
    print >>fpout, ("};\n")
    print >>fpout, ("const unsigned int g_lcpu_patch_bin[]= { ")
    print_file(src+'lcpu_rom_patch.bin', fpout)
    print >>fpout, ("};")
    print >>fpout, ("void lcpu_patch_install()\n{\n"),
    print >>fpout, ("#ifdef SOC_BF0_HCPU\n"),
    print >>fpout, ("\tmemset((void*)(LCPU_PATCH_START_ADDR_S),0,LCPU_PATCH_TOTAL_SIZE);\n"),
    print >>fpout, ("\tmemcpy((void*)(LCPU_PATCH_START_ADDR_S),g_lcpu_patch_bin,sizeof(g_lcpu_patch_bin));\n"),
    print >>fpout, ("#else\n"),
    print >>fpout, ("\tmemset((void*)(LCPU_PATCH_START_ADDR),0,LCPU_PATCH_TOTAL_SIZE);\n"),
    print >>fpout, ("\tmemcpy((void*)(LCPU_PATCH_START_ADDR),g_lcpu_patch_bin,sizeof(g_lcpu_patch_bin));\n"),
    print >>fpout, ("#endif\n"),
    print >>fpout, ("\tmemcpy((void*)(LCPU_PATCH_RECORD_ADDR),g_lcpu_patch_list,sizeof(g_lcpu_patch_list));\n"),
    print >>fpout, ("\tHAL_PATCH_install();\n}\n")
    print >>fpout, ("#endif\n"),
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
