import os
import sys
import re
import struct
import shutil

def print_file(file_name, fpout):
    fp_bin=open(file_name, 'rb')
    data=fp_bin.read(1)
    count=0
    while (1):
        temp=struct.unpack("<B", data)
        if ((count% 16)==0):
            print>>fpout, ("\t"),        
        print>>fpout, ("0x%02X,"%(temp)),
        count=count+1
        if ((count% 16)==0):
            print>>fpout, ("\n"),
        data=fp_bin.read(1)
        if (len(data)==0):
            print>>fpout, ("0x0\n"),
            break;        
    fp_bin.close()
    
def gen_hpp_img(src):
    (name,ext)=os.path.splitext(os.path.basename(src))
    name_u = name.upper()
    name_u = "LOTTIE_"+name_u+"_H"
    if len(os.path.dirname(src))>0:
        fpout=open(os.path.dirname(src) +'\\lottie_'+name+'.h',"w+")
    else:
        fpout=open('lottie_'+name+'.h',"w+")
    print >>fpout, ("#ifndef %s" %(name_u))
    print >>fpout, ("#define %s\n" %(name_u))
    print >>fpout, ("#include <stdint.h>\n")
    print >>fpout, ("static const uint8_t lottie_" + name + "_raw[]= { ")
    print_file(src, fpout)
    print >>fpout, ("};\n")
    print >>fpout, ("#endif")
    fpout.close()
    
if __name__ == '__main__':    
    gen_hpp_img(sys.argv[1])
