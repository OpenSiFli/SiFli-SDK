import sys
import re
import struct
import shutil

symbols=[]

tag=0x50544348

def read_sym(file):
    fp=open(file,'r')
    for line in fp:    
        info=line.split()
        if len(info)<3:
            continue
        if (info[2]=='Thumb' or info[2]=='Data' or info[2]=='Number') :
            global symbols
            symbols+=[[info[0],info[1]]]

def convert( aString ):
    if aString.startswith("0x") or aString.startswith("0X"):
        return int(aString,16)
    elif aString.startswith("0"):
        return int(aString,8)
    else:
        return int(aString)

def resolve_sym(symbol):
    global symbols
    for info in symbols:
        if (info[0]==symbol):
            r=convert(info[1])
            return r
    return convert(symbol)  

def generate_record(break_address, data, is_data,fp_bin):
    break_address&=0xfffffffc
    if (is_data==0):
        offset=data-break_address-4
        #print("offset=%x, data=%x, breeak_address=%x" %(offset, data, break_address))
        if (offset>0):
            s=0
        else:
            s=1
            
        #calculate imm10
        imm10=offset>>12
        data=(0xf<<12)+(imm10&0x3FF)+(0xD<<28)
        #print("imm10=%x"%(imm10))
        
        #calculate J1
        if ((offset&(1<<23))>0) :
            ii=1
        else: 
            ii=0
        #print("i1=%d"%(ii));
        ii=(1-ii)
        ii=ii^s
        data+=(ii<<29)
        #calculate J2
        if ((offset&(1<<22))>0) :
            ii=1
        else:
            ii=0
        #print("i2=%d"%(ii));
        ii=(1-ii) 
        ii=ii^s
        data+=(ii<<27)
        #calculate imm11
        imm11=(offset&0xfff)>>1;
        #print("imm11=%x"%(imm11))
        data+=(imm11<<16)
    record=struct.pack("<LL",break_address,data)
    return record

def generate_record2(break_address, data, is_data,fp_bin):
    break_address&=0xfffffffc
    if (is_data==0):
        offset=data-break_address-4
        #print("offset=%x, data=%x, breeak_address=%x" %(offset, data, break_address))
        if (offset>0):
            s=0
        else:
            s=1
            
        #calculate imm10
        imm10=offset>>12
        data=(0xf<<12)+(imm10&0x3ff)+(0x9<<28)
        #print("imm10=%x"%(imm10))
        
        #calculate J1
        if ((offset&(1<<23))>0) :
            ii=1
        else: 
            ii=0
        #print("i1=%d"%(ii));
        ii=(1-ii)
        ii=ii^s
        data+=(ii<<29)
        #calculate J2
        if ((offset&(1<<22))>0) :
            ii=1
        else:
            ii=0
        #print("i2=%d"%(ii));
        ii=(1-ii) 
        ii=ii^s
        data+=(ii<<27)
        #calculate imm11
        imm11=(offset&0xfff)>>1;
        #print("imm11=%x"%(imm11))
        data+=(imm11<<16)
    record=struct.pack("<LL",break_address,data)
    return record

try:   
    read_sym(sys.argv[1]+'bf0_ap_patch.map')
    read_sym(sys.argv[1]+'bf0_ap.map')
except:
    read_sym(sys.argv[1]+'lcpu_rom_patch.map')
    read_sym(sys.argv[1]+'lcpu_rom.map')
fp=open(sys.argv[1]+'patch_list.txt', 'r')
fp_bin=open(sys.argv[1]+'patch_list.bin', 'wb+')
data=[0,0]
records=[]
for line in fp:
    info=re.split(' ', line)
    data[0]=resolve_sym(info[0])
    data[0]+=convert(info[1])
    data[1]=resolve_sym(info[2])
    if ((data[0]) & 2):
        print('Warning: break address %s+%s=0x%x is not aligned to 4 bytes.' %(info[0],info[1],data[0]))
    if (info[3]=='0'):
        print('Break:0x%x BL to: 0x%x'  %(data[0]&0xfffffffc,data[1]))
        records+=generate_record(data[0], data[1], 0, fp_bin)
    elif (info[3]=='2'):
        print('Break:0x%x BW to: 0x%x'  %(data[0]&0xfffffffc,data[1]))
        records+=generate_record2(data[0], data[1], 0, fp_bin)
    else:
        print('Replace:0x%x with 0x%x'  %(data[0]&0xfffffffc,data[1]))
        records+=generate_record(data[0], data[1], 1, fp_bin)
        
fp_bin.write(struct.pack("<L",tag))
fp_bin.write(struct.pack("<L",len(records)))
for record in records:
    fp_bin.write(record)
fp_bin.close()
fp.close()
fp_bin.close()
        
    
