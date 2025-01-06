#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from bmp import *

# example:
# Show rgb565 454x454 framebuffer at 0x20027610
# python jlinkbin2bmp.py CORTEX-M33 rgb565 320 386 603d460c


if __name__ == '__main__':

    jlink_tgt_dev = sys.argv[1]
    cf = sys.argv[2]
    width=int(sys.argv[3])  #in pixel
    height=int(sys.argv[4])  #in pixel
    address=int(sys.argv[5],16)   #framebuffer addresss

    image = bmp(width, height)

    offset = 0


    #Dump bin through Jlink
    color_depth = image.bin_format_depth(cf)
    if color_depth == 0:
       raise Exception("Invalid format",cf)
    else:
       print ("color depth = %d"%color_depth)

    if color_depth > 7:
        fb_bytes=height*(width*color_depth/8)
    else:
        fb_bytes=height*((width+(8/color_depth) - 1)*color_depth/8)

    bin_file_name=('%08x.bin'%(address))

    os.system('echo connect > tmp.jlink')
    os.system('echo h >> tmp.jlink')
    os.system(('echo savebin %s %x %x >> tmp.jlink'%(bin_file_name,address,fb_bytes)))
    os.system('echo exit >> tmp.jlink')
    #os.system(('call jlink.exe -device %s -if SWD -speed 12000 -autoconnect 1 -CommandFile tmp.jlink'%(jlink_tgt_dev)))
    #os.system(('call jlink.exe -device %s -ip 127.0.0.1:19025 -if SWD -speed 1000 -autoconnect 1 -CommandFile tmp.jlink'%(jlink_tgt_dev)))
    os.system(('call jlink.exe -device CORTEX-M33 %s -autoconnect 1 -CommandFile tmp.jlink'%(jlink_tgt_dev)))

    #read bin file
    f = open(bin_file_name, "rb")
    try:
        org_data = f.read()
    finally:
        f.close()



    image.convert_bin2bmp(cf, org_data, offset)

    bmp_fname = ("%x_%dx%d_%s.bmp"%(address,width,height,cf))
    image.save_image(bmp_fname)

    os.system('del tmp.jlink')
    #os.system(bmp_fname)
    os.startfile(bmp_fname)