#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys

if __name__ == '__main__':

    jlink_tgt_dev = sys.argv[1]
    address=int(sys.argv[2],16)   #ezip addresss
    size=int(sys.argv[3])   #ezip size


    #Dump bin through Jlink
    bin_file_name=('%08x.bin'%(address))

    os.system('echo connect > tmp.jlink')
    os.system('echo h >> tmp.jlink')
    os.system(('echo savebin %s %x %x >> tmp.jlink'%(bin_file_name,address,size)))
    os.system('echo exit >> tmp.jlink')
    os.system(('call jlink.exe -device CORTEX-M33 %s -autoconnect 1 -CommandFile tmp.jlink'%(jlink_tgt_dev)))
    os.system('del tmp.jlink')
    
    #Convert it to png file
    png_name = os.path.join(os.getcwd(), 'dst', '%08x.png' % (address))
    if os.path.exists(png_name):
        os.remove(png_name)

    os.system(('ezip -convert %s -spt 1 -dpt 0 -outdir dst -dec_off_no_header 0'%(bin_file_name)))

    
    #Open png image
    if os.path.exists(png_name):
        os.startfile(png_name)
    else:
        print('Error: Fail to convert %s.'%(bin_file_name))