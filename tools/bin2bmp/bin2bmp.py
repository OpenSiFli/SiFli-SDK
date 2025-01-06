#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from bmp import *
 
# example:
# Show rgb565 454x454 framebuffer in hcpu_ram.bin at offset 0x2000
# python bin2bmp.py hcpu_ram.bin rgb565 454 454 2000


if __name__ == '__main__':

    bin_file_name = sys.argv[1]
    cf = sys.argv[2]
    width=int(sys.argv[3])  #in pixel
    height=int(sys.argv[4])  #in pixel

    image = bmp(width, height)

    if len(sys.argv) > 5:
        offset = int(sys.argv[5],16)   #offset
    else :
        offset = 0

    #read bin file
    f = open(bin_file_name, "rb")
    try:
        org_data = f.read()
    finally:
        f.close()

    image.convert_bin2bmp(cf, org_data, offset)

    bmp_fname = ("%s_offset0x%x_%dx%d_%s.bmp"%(bin_file_name,offset,width,height,cf))
    image.save_image(bmp_fname)
    #os.system(bmp_fname)
    os.startfile(bmp_fname)