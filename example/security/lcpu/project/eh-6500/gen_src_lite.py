import os
import sys
import re
import struct
import shutil

def print_file(file_name, fpout):
    fp_bin=open(file_name, 'rb')
    data=fp_bin.read(4)
    count=0
    while (len(data)==4):
        temp=struct.unpack("<L", data)
        if ((count%4)==0):
            print>>fpout, ("\t"),        
        print>>fpout, ("0x%08X,"%(temp)),
        count=count+1
        if ((count%4)==0):
            print>>fpout, ("\n"),
        data=fp_bin.read(4)    
    fp_bin.close()

def gen_bcpu_patch(src,dest):
    fpout=open(dest +'bcpu_patch.c',"w+")
    print >>fpout, ("#include <stdint.h>\n"),
    print >>fpout, ("#include <string.h>\n"),
    print >>fpout, ("#include \"bf0_hal.h\"\n"),
    print >>fpout, ("#include \"mem_map.h\"\n"),
    print >>fpout, ("#include \"register.h\"\n"),
    print >>fpout, ("#include \"bf0_hal_patch.h\"\n"),
    print >>fpout, ("#ifdef HAL_BCPU_PATCH_MODULE\n"),
    print >>fpout, ("const unsigned int g_bcpu_patch_list[]= { ")
    print >>fpout, ("#ifndef BCPU_PATCH_CUSTOMIZE_PATCH_LIST\n")
    print_file(src+'patch_list.bin', fpout)
    print >>fpout, ("\n#else\n")
    print >>fpout, ("#include BCPU_PATCH_CUSTOMIZE_PATCH_LIST\n")
    print >>fpout, ("#endif\n")
    print >>fpout, ("};\n")
    print >>fpout, ("const unsigned int g_bcpu_patch_bin[]= { ")
    print >>fpout, ("#ifndef BCPU_PATCH_CUSTOMIZE_PATCH_BIN\n")
    print_file(src+'bf0_cp_patch.bin', fpout)
    print >>fpout, ("\n#else\n")
    print >>fpout, ("#include BCPU_PATCH_CUSTOMIZE_PATCH_BIN\n")
    print >>fpout, ("#endif\n")
    print >>fpout, ("};")
    print >>fpout, ("void bcpu_patch_install()\n{\n"),
    print >>fpout, ("\tmemcpy((void*)(BCPU_PATCH_CODE_START_ADDR+BCPU2HCPU_OFFSET),g_bcpu_patch_bin,sizeof(g_bcpu_patch_bin));\n"),
    print >>fpout, ("\tmemset((void*)(BCPU_PATCH_RAM_START_ADDR+BCPU2HCPU_OFFSET),0,BCPU_PATCH_RAM_SIZE);\n"),
    print >>fpout, ("\tHAL_PATCH_install((struct patch_entry_desc *)&g_bcpu_patch_list[2],(sizeof(g_bcpu_patch_list) - 8)/sizeof(struct patch_entry_desc));\n}\n")
    print >>fpout, ("#endif\n"),
    fpout.close()

def gen_bcpu_header_file(src,dest, list_file, bin_file):
    fpout=open(dest + list_file,"w+")
    print_file(src+'patch_list.bin', fpout)
    fpout.close()
    fpout=open(dest + bin_file,"w+")
    print_file(src+'bf0_cp_patch.bin', fpout)
    fpout.close()


def gen_lcpu_img(src,dest,rom=False):
    if os.path.isdir(dest):
        fpout=open(os.path.join(dest, 'lcpu_img.c'),"w+")
    else:
        fpout=open(dest,"w+")    
    print >>fpout, ("#include <stdint.h>\n"),
    print >>fpout, ("#include <string.h>\n"),
    print >>fpout, ("#include \"mem_map.h\"\n"),
    print >>fpout, ("#include \"rtconfig.h\"\n"),
    print >>fpout, ("#include \"register.h\"\n\n"),
    print >>fpout, ("#ifdef BSP_USING_LCPU_IMG\n"),
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
        print >>fpout, ("#endif\n"),
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
        print >>fpout, ("#endif\n"),
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
    print >>fpout, ("#ifdef BSP_USING_LCPU_IMG\n"),
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
        print >>fpout, ("#endif\n"),
    else:
        assert False, "dir not found"
    
    fpout.close()


def gen_bcpu_img(src,dest):
    fpout=open(os.path.join(dest, 'bcpu_img.c'),"w+")
    print >>fpout, ("#include <stdint.h>\n"),
    print >>fpout, ("#include <string.h>\n"),
    print >>fpout, ("#include <rtconfig.h>\n"),
    print >>fpout, ("#include <register.h>\n"),
    print >>fpout, ("#include \"mem_map.h\"\n"),
    print >>fpout, ("const unsigned int g_bcpu_bin[]= { ")
    print_file(src, fpout)
    print >>fpout, ("};")
    print >>fpout, ("uint32_t bcpu_ramcode_len()\n{\n"),
    print >>fpout, ("\treturn sizeof(g_bcpu_bin);\n"),
    print >>fpout, ("}\n"),	
    print >>fpout, ("void bcpu_img_install()\n{\n"),
    print >>fpout, ("\tmemcpy((uint8_t*)(BCPU_ADDR_2_HCPU_ADDR(BLE_ROM_BASE)),g_bcpu_bin,sizeof(g_bcpu_bin));\n}\n")
    fpout.close()

def gen_lcpu_patch(src,dest):
    fpout=open(dest +'lcpu_lite_patch.c',"w+")
    print >>fpout, ("#include <stdint.h>\n"),
    print >>fpout, ("#include <string.h>\n"),
    print >>fpout, ("#ifdef APP_BSP_TEST\n"),
    print >>fpout, ("#include  <rtdevice.h>\n"),
    print >>fpout, ("#include \"drv_flash.h\"\n"),
    print >>fpout, ("#endif\n"),
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
    print >>fpout, ("#ifdef APP_BSP_TEST\n"),
    print >>fpout, ("\trt_flash_write(LCPU_PATCH_RECORD_ADDR,(const uint8_t*)g_lcpu_patch_list,sizeof(g_lcpu_patch_list));\n"),
    print >>fpout, ("#endif\n"),
    print >>fpout, ("#else\n"),
    print >>fpout, ("\tmemset((void*)(LCPU_PATCH_RAM_START_ADDR),0,LCPU_PATCH_RAM_SIZE);\n"),
    print >>fpout, ("#endif\n"),
    print >>fpout, ("\tHAL_PATCH_install();\n}\n")
    print >>fpout, ("#endif\n"),
    fpout.close()
    
if __name__ == '__main__':    
    if (sys.argv[1]=='bcpu'):
        gen_bcpu_patch(sys.argv[2], sys.argv[3])
    if (sys.argv[1]=='lcpu'):
        gen_lcpu_img(sys.argv[2], sys.argv[3])
    if (sys.argv[1]=='lcpu_rom'):
        gen_lcpu_img(sys.argv[2], sys.argv[3], True)
    if (sys.argv[1]=='bcpu2'):
        gen_bcpu_img(sys.argv[2], sys.argv[3])        
    if (sys.argv[1]=='lcpu_patch'):
        gen_lcpu_patch(sys.argv[2], sys.argv[3])
    if (sys.argv[1]=='bcpu_speicial'):
        gen_bcpu_header_file(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
    if (sys.argv[1]=='lcpu_xip'):
        gen_lcpu_img_xip(sys.argv[2], sys.argv[3])
