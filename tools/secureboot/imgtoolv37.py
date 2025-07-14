from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import os.path
import sys
import struct
import textwrap
import struct
import json5
import zlib
import os
import binascii
import array

from Cryptodome.PublicKey import RSA
from Cryptodome.Random import get_random_bytes
from Cryptodome.Cipher import AES, PKCS1_OAEP
from Cryptodome.Hash import SHA256
from Cryptodome.Signature import pkcs1_15
from Cryptodome.Util import Counter
from crccheck.crc import Crc32Mpeg2

DFU_FLAG_ENC      =1
DFU_FLAG_AUTO     =2
DFU_FLAG_SINGLE   =4
DFU_FLAG_COMPRESS =16

DFU_ID_CODE        = 0
DFU_ID_CODE_MIX    = 1
DFU_ID_OTA_MANAGER = 2
DFU_ID_DL          = 3

DFU_IMG_ID_HCPU    = 0
DFU_IMG_ID_LCPU    = 1
DFU_IMG_ID_PATCH   = 2
DFU_IMG_ID_RES     = 3
DFU_IMG_ID_FONT    = 4
DFU_IMG_ID_EX = 5
DFU_IMG_ID_OTA_MANAGER = 6
DFU_IMG_ID_TINY_FONT = 7
DFU_IMG_ID_CTRL = 10

DFU_COMPRESS_TYPE_GZIP = 0
DFU_COMPRESS_TYPE_ZLIB = 1


def print_hex(data):
    if (sys.version_info > (3, 0)):
        print(data.hex(), end='')
    else :
        print(data.encode('hex'), end='')

def to_bytes(i):
    if (sys.version_info > (3, 0)):
        return i.to_bytes(4,byteorder='little')
    else:
        return struct.pack("<I", i)
        
def from_bytes(data):
    if (sys.version_info > (3, 0)):
        return int.from_bytes(data,'little')
    else: 
        if (len(data)==4):
            return struct.unpack("<I", data)[0]
        else:
            return struct.unpack("<H", data)[0]
        
def encrypt_image_help(img, eimg) :
    file_key=open(FLAGS.key+".bin", "rb")
    recipient_key = file_key.read()
    
    #1. Generate AES Key - Encrypt the data with the AES session key
    session_key = get_random_bytes(32)
    
    #2. Align image to 16 bytes 
    data=open(img,"rb").read()
    data=bytearray(data)
    remain=(len(data)%16)
    if (remain>0) :
        remain=16-remain
        str=u'0'
        for i in range(0,remain):
            data.append(0)
                
    img_len=to_bytes(len(data))
    # 3. Generate Header
    # img_len(4 bytes)+bksize(2 bytes) + flags(2 bytes) + session_key(32 bytes) + signature(256 bytes) = 296 bytes
    bksize=struct.pack("<H", FLAGS.bksize)    
    flags=struct.pack("<H", FLAGS.flags)    
    header=img_len+bksize+flags+session_key

    # 4. Encrypt image
    i=0
    cnt_prefix=open(FLAGS.sigkey+"_hash.bin", "rb").read(8)
    cnt_prefix+=b'\x00'*4
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0)
    cipher_aes = AES.new(session_key, AES.MODE_CTR,counter=cnter)
    data3=bytearray(b'')
    cipherimage=bytearray(b'')
    while (i<len(data)):
        # 7.1 Generate Hash+offset, data block
        if (i+FLAGS.bksize<len(data)):
            data2=data[i:i+FLAGS.bksize]
            hash=SHA256.new(data[i:i+FLAGS.bksize])
            hash2=hash.digest()+to_bytes(i)
        else:
            data2=data[i:len(data)]
            hash=SHA256.new(data[i:len(data)])
            hash2=hash.digest()+to_bytes(i)
        # 7.2 encrypt data block
        ciphertext= cipher_aes.encrypt(data2)

        # 7.3 Save each encrypted block
        data3+=hash2
        data3+=ciphertext
        cipherimage+=ciphertext
        i+=len(data2)

    #5. Calculate hash and get signature
    if ((FLAGS.flags&1)==1):
        hash=SHA256.new(cipherimage)                                       # Hash encrypted image for integrity check
    else:
        hash=SHA256.new(data)                                              # Hash plain image for integrity check
    pri_key = RSA.import_key(open(FLAGS.sigkey+"_pri.pem").read())
    sign_rsa = pkcs1_15.new(pri_key)
    signature = sign_rsa.sign(hash)                                        # Sign the hash
    header+=signature

    #header+=FLAGS.ver.encode()
    new_ver = FLAGS.ver.ljust(20, '0')
    print(new_ver)
    #header+=bytearray(FLAGS.ver.encode())
    header+=bytearray(new_ver.encode())

    #6. Encrypt header
    hash=SHA256.new(header)
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0)
    cipher_core_aes = AES.new(recipient_key, AES.MODE_CTR, counter=cnter)
    enc_header = cipher_core_aes.encrypt(header)
    t_f = open('test.bin', "wb")
    t_f.write(header)
    t_f.write(enc_header)
    t_f.write(hash.digest())
    print("%x", header)
    print(enc_header)
    print(hash.digest())
    #7. Save to encrypted image file
    data2=hash.digest()+enc_header
    file_out=open(eimg, "wb")
    file_out.write(data2)
    file_out.write(data3)
    
def encrypt_image() :
    if os.path.isdir(FLAGS.img):
        if os.path.exists(FLAGS.eimg):
            print("Skip create, already exist:" + FLAGS.eimg)
        else:
            os.makedirs(FLAGS.eimg)
        for f in os.listdir(FLAGS.img):
            print(FLAGS.img+'/'+f)
            print(FLAGS.eimg+'/'+f+'_sec.bin')  
            encrypt_image_help(FLAGS.img+'/'+f, FLAGS.eimg+'/'+f+'_sec.bin')
    else:
        encrypt_image_help(FLAGS.img,FLAGS.eimg)


def decrypt_image_help(img,eimg) :
    #1. Get root key, Read signature hash, 1st 12 bytes as prefix for counter
    file_key=open(FLAGS.key+".bin", "rb")
    recipient_key = file_key.read()
    cnt_prefix=open(FLAGS.sigkey+"_hash.bin", "rb").read(8)
    cnt_prefix+='\x00'*4
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0);
    
    #2. Decode image header
    file_in=open(eimg, "rb")
    hash=file_in.read(32)
    enc_header=file_in.read(296)
    cipher_core_aes = AES.new(recipient_key, AES.MODE_CTR, counter=cnter)
    header = cipher_core_aes.decrypt(enc_header)
    
    #3.Decrypt the data with the AES session key
    img_len=from_bytes(header[:4])
    bksize=from_bytes(header[4:6])
    flags=from_bytes(header[6:8])
    session_key=header[8:40]
    signature=header[40:296]
    print_hex(signature)
    
    i=0    
    if (sys.version_info > (3, 0)):
        data=bytes('', encoding='UTF-8')
    else:
        data=bytes('')
        
    print_hex(session_key)
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0); 
    cipher_aes = AES.new(session_key, AES.MODE_CTR, counter=cnter)
    ciphertext=''
    while (i<img_len):
        hash=file_in.read(32)
        offset=file_in.read(4)
        if (i+bksize<img_len):
            ciphertext+=file_in.read(bksize)
            i+=bksize
        else:
            ciphertext+=file_in.read(img_len-i)
            i=img_len

    data=cipher_aes.decrypt(ciphertext)
    data=bytearray(data)    
    
    #4. Calculate Hash, and validate signature
    h = SHA256.new()
    if ((int(flags)&1)==1):
        print("Sign encrypted")
        h.update(ciphertext)
    else:
        print("Sign plain")
        h.update(data)
    print_hex(h.digest())
    data2=open(FLAGS.sigkey+"_pub.der", "rb").read()
    pub_key = RSA.import_key(data2)
    verify_rsa = pkcs1_15.new(pub_key)
    verify_rsa.verify(h,signature)
    
    #5. Save to image
    file_out=open(img, "wb")
    file_out.write(data)

def decrypt_image() :
    if os.path.isdir(FLAGS.eimg):
        if os.path.exists(FLAGS.img):
            print("Skip create, already exist:" + FLAGS.img)
        else:
            os.makedirs(FLAGS.img)
        for f in os.listdir(FLAGS.eimg):
            print(FLAGS.img+'/dec_'+f)
            print(FLAGS.eimg+'/'+f)  
            decrypt_image_help(FLAGS.img+'/dec_'+f, FLAGS.eimg+'/'+f)
    else:
        decrypt_image_help(FLAGS.img,FLAGS.eimg)
        
def compress_and_sign_image_help(img, eimg) :

    # 1. Align image to 16 bytes
    data = open(img, "rb").read()
    data = bytearray(data)

    if (FLAGS.align == 1):
        remain = (len(data) % 16)
        if (remain > 0):
            remain = 16 - remain
            str = u'0'
            for i in range(0, remain):
                data.append(0)

    img_len = to_bytes(len(data))

	# Generate header
    pksize=struct.pack("<I", FLAGS.pksize)
    header=img_len+pksize
    i = 0
	
	# Compress
    compresstext = b''
    print(len(data))
    while (i < len(data)):
        if (i+FLAGS.pksize<len(data)):
            data2=data[i:i+FLAGS.pksize]
        else:
            data2=data[i:len(data)]
        i += len(data2)
        data2 = zlib.compress(data2, 9)
        pklen = struct.pack('<I', len(data2))
        #pklen = to_bytes(pklen)
        print(len(data2))
        compresstext+= pklen + data2

    #header+=to_bytes(len(compresstext))
    img_text = header+compresstext
    hash=SHA256.new(data)
    pri_key = RSA.import_key(open(FLAGS.sigkey+"_pri.pem").read())
    sign_rsa = pkcs1_15.new(pri_key)
    signature = sign_rsa.sign(hash)
    file_out=open(eimg, "wb")
    file_out.write(signature)
    file_out.write(img_text)
	
def compress_and_sign_image() :
    if os.path.isdir(FLAGS.img):
        if os.path.exists(FLAGS.eimg):
            print("Skip create, already exist:" + FLAGS.eimg)
        else:
            os.makedirs(FLAGS.eimg)
        for f in os.listdir(FLAGS.img):
            print(FLAGS.img+'/'+f)
            print(FLAGS.eimg+'/'+f+'_compress.bin')  
            compress_and_sign_image_help(FLAGS.img+'/'+f, FLAGS.eimg+'/'+f+'_compress.bin')
    else:
        compress_and_sign_image_help(FLAGS.img,FLAGS.eimg)	
	
def gen_uid() :
    session_key = get_random_bytes(16)
    file_out = open(FLAGS.key+".bin", "wb")
    file_out.write(session_key)
    
def gen_key2() :
    key = RSA.generate(2048)
    private_key = key.export_key()
    file_out = open(FLAGS.sigkey+"_pri.pem", "wb")
    file_out.write(private_key)

    public_key = key.publickey().export_key()
    file_out = open(FLAGS.sigkey+"_pub.pem", "wb")
    file_out.write(public_key)    
      
def dump_sig():
    key = RSA.import_key(open(FLAGS.sigkey+"_pub.pem").read())
    #Encrypt the core key using root key
    data=key.export_key(format='DER')
    file_out=open(FLAGS.sigkey+"_pub.der","wb")
    file_out.write(data);
    h=SHA256.new()
    h.update(data)
    data=h.digest()    
    file_out=open(FLAGS.sigkey+"_hash.bin","wb")
    file_out.write(data);
    
def gen_root() :
    session_key = get_random_bytes(32) 
    file_out = open(FLAGS.key+".bin", "wb")
    file_out.write(session_key)
	
def enc_ftab() :
    #1. Open key
    file_in=open(FLAGS.key+".bin", "rb")
    key=file_in.read()
    
    #2. Open flash table json, and output file
    with open(FLAGS.table+'.json') as f:
        data = json5.load(f)
    file_out= open(FLAGS.table+"_enc.bin", "wb")      
    
    #3. Translate json file into binary.
    data2=''
    for x in data['tables']:
        data2+=struct.pack("<I",x['base'])
        data2+=struct.pack("<I",x['size'])
        data2+=struct.pack("<I",x['xip_base'])
        data2+=struct.pack("<I",x['flags'])
    #4. Calculate Hash
    hash=SHA256.new(data2)
    
    #5. Encrypt the core key using root key
    cnt_prefix=open(FLAGS.sigkey+"_hash.bin", "rb").read(8)    
    cnt_prefix+='\x00'*4
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0)
    cipher_core_aes = AES.new(key, AES.MODE_CTR, counter=cnter)
    data = cipher_core_aes.encrypt(data2)
 
    #6. Save to output file
    [ file_out.write(x) for x in (hash.digest(),data)]    
       
def dec_ftab() :
    #1. Open key
    file_in=open(FLAGS.key+".bin", "rb")
    key=file_in.read()
    
    #2. Open encrypted flash table and read back.
    enc_in = open(FLAGS.table+"_enc.bin", "rb")
    hash=enc_in.read(32)
    ciphertext=enc_in.read()
   
    #3. Decrypt the flash table
    cnt_prefix=open(FLAGS.sigkey+"_hash.bin", "rb").read(8)    
    cnt_prefix+='\x00'*4
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0)
    cipher_core_aes = AES.new(key, AES.MODE_CTR, counter=cnter)
    data = cipher_core_aes.decrypt(ciphertext)

    #4. Print out decrypted flash table and save.
    table={}
    table['name']='Flash table'
    table['num']=int(len(data)/16)
    table['tables'] = []
    i=0
    for k in range(table['num']):
        base,size,xip_base,flags=struct.unpack("<IIII",data[i:i+16])
        i+=16
        table['tables'].append({'base':base,'size':size,'xip_base':xip_base,'flags':flags})
    print(table)    
    file_out= open(FLAGS.table+"2.json", "wb")     
    json5.dump(table,file_out)

def ftab() :   
    with open(FLAGS.table+'.json') as f:
        data = json5.load(f)
    file_out= open(FLAGS.table+".bin", "wb")      
    
    data2=''
    for x in data['tables']:
        data2+=struct.pack("<I",x['base'])
        data2+=struct.pack("<I",x['size'])
        data2+=struct.pack("<I",x['xip_base'])
        data2+=struct.pack("<I",x['flags'])
    file_out.write(data2)
    print_hex(data2)
    
def rftab() :   
    file_in = open(FLAGS.table+".bin", "rb")
    data= file_in.read()

    table={}
    table['name']='Flash table'
    table['num']=int(len(data)/16)
    table['tables'] = []
    i=0
    for k in range(table['num']):
        base,size,xip_base,flags=struct.unpack("<IIII",data[i:i+16])
        i+=16
        table['tables'].append({'base':base,'size':size,'xip_base':xip_base,'flags':flags})
    file_out= open(FLAGS.table+"_dec.json", "wb")     
    json5.dump(table,file_out)

def print_data(lable,data):
    print("unsigned char " + lable + "[] = {")
    for i in range (0,len(data)):
        if (i%16==0):
            print("    ", end='')
        print("0x", end='')
        print_hex(data[i])
        if (i!=len(data)-1):
            print(",", end='')
        if ((i%16)==15 or i==len(data)-1):
            print("\n", end='')
    print("};\n")
    
def test_vec() :
    file_key=open(FLAGS.key+".bin", "rb")
    recipient_key = file_key.read()
    print_data("root_key", recipient_key)
    
    cnt_prefix=open(FLAGS.sigkey+"_hash.bin", "rb").read(8)
    print_data("nounce_prfix", cnt_prefix)
    cnter=Counter.new(64,prefix=cnt_prefix,initial_value=0);
    
    data=bytearray(256)
    for i in range(0,256):
        data[i]=iargparser
    print_data("plain_text", str(data))

    cipher_core_aes = AES.new(recipient_key, AES.MODE_CTR, counter=cnter)
    data = cipher_core_aes.encrypt(data)    
    print_data("encrypted_text",str(data))


def dfu_crc_cal(img):
    data = open(img, "rb").read()
    data = bytearray(data)
    crc_value = binascii.crc32(data)
    print(crc_value)
def dfu_packed_bin(img, eimg, session_key, is_enc=True):

    # 2. Align image to 16 bytes
    data = open(img, "rb").read()
    data = bytearray(data)
    if (FLAGS.align == 1):
        remain = (len(data) % 16)
        if remain > 0:
            remain = 16 - remain
            str = u'0'
            for i in range(0, remain):
                data.append(0)

    img_len = to_bytes(len(data))

    # 4. Encrypt image
    i = 0
    cnt_prefix = open(FLAGS.sigkey + "_hash.bin", "rb").read(8)
    cnt_prefix += b'\x00' * 4
    cnter = Counter.new(32, prefix=cnt_prefix, initial_value=0)
    cipher_aes = AES.new(session_key, AES.MODE_CTR, counter=cnter)
    data3 = bytearray(b'')
    cipherimage = bytearray(b'')
    while i < len(data):
        # 7.1 Generate Hash+offset, data block
        if i + FLAGS.bksize < len(data):
            data2 = data[i:i + FLAGS.bksize]
            hash = SHA256.new(data[i:i + FLAGS.bksize])
            hash2 = hash.digest() + to_bytes(i)
        else:
            data2 = data[i:len(data)]
            hash = SHA256.new(data[i:len(data)])
            hash2 = hash.digest() + to_bytes(i)
        # 7.2 encrypt data block
        if is_enc == 1:
            ciphertext = cipher_aes.encrypt(data2)
        else:
            ciphertext = data2

        # 7.3 Save each encrypted block
        data3 += hash2
        data3 += ciphertext
        cipherimage += ciphertext
        i += len(data2)

    # 5. Calculate hash and get signature
    if ((FLAGS.flags & DFU_FLAG_ENC) == DFU_FLAG_ENC):
        hash = SHA256.new(cipherimage)  # Hash encrypted image for integrity check
    else:
        hash = SHA256.new(data)  # Hash plain image for integrity check
    pri_key = RSA.import_key(open(FLAGS.sigkey + "_pri.pem").read())
    sign_rsa = pkcs1_15.new(pri_key)
    signature = sign_rsa.sign(hash)  # Sign the hash

    # 7. Save to encrypted image file
    file_out = open(eimg, "wb")
    file_out.write(data3)
    return [signature, img_len]


def dfu_compress_bin(img, eimg):
    # 1. Align image to 16 bytes
    data = open(img, "rb").read()
    data = bytearray(data)
    if (FLAGS.align == 1):
        remain = (len(data) % 16)
        if (remain > 0):
            remain = 16 - remain
            str = u'0'
            for i in range(0, remain):
                data.append(0)

    img_len = to_bytes(len(data))

    # Generate header
    pksize = struct.pack("<I", FLAGS.pksize)
    header = img_len + pksize
    i = 0

    # Compress
    compresstext = b''
    print(len(data))
    while (i < len(data)):
        if (i + FLAGS.pksize < len(data)):
            data2 = data[i:i + FLAGS.pksize]
        else:
            data2 = data[i:len(data)]
        i += len(data2)

        if (FLAGS.com_type == DFU_COMPRESS_TYPE_ZLIB):
            data2 = zlib.compress(data2, 9)
        elif (FLAGS.com_type == DFU_COMPRESS_TYPE_GZIP):
            temp_file = "temp_com.bin"
            with open (temp_file, "wb") as fi:
                fi.write(data2)

            main = "eZIP.exe -gzip " + temp_file + " -length -noheader"
            r_v = os.system(main)
            #print (r_v)

            with open (temp_file + ".gz", "r+b") as fo:
                data2 = fo.read()
        '''
        gzip_headerlen = 10 + 13
        data2 = data2[gzip_headerlen:len(data2)]
        
        origin_len_byte = to_bytes(origin_len)
        #add old length
        data2 = origin_len_byte + data2
        with open (r"output\t0_rgb888_f0\temp_com_after.bin", "wb") as fo:
            fo.write(data2)
        '''
        pklen = struct.pack('<I', len(data2))
        # pklen = to_bytes(pklen)
        print(len(data2))
        compresstext += pklen + data2

    # header+=to_bytes(len(compresstext))
    img_text = header + compresstext
    file_out = open(eimg, "wb")
    file_out.write(img_text)


def dfu_bin_generation(bin, flag, session_key):
    img = bin+'.bin'
    is_enc=flag & DFU_FLAG_ENC
    if (flag & DFU_FLAG_COMPRESS):
        out_img = 'com_'+img
        dfu_compress_bin(img, out_img)
        img = 'com_'+img
    if (flag & DFU_FLAG_ENC):
        out_img = 'enc'+img
    else :
        out_img = 'out' + img
    if (FLAGS.offline_img == 2):
        return 0
    return dfu_packed_bin(img, out_img, session_key, is_enc)




def dfu_generation() :
    if (FLAGS.offline_img != 2):
        file_key = open(FLAGS.key + ".bin", "rb")
        recipient_key = file_key.read()
    
        control_packet = struct.pack("<B", FLAGS.dfu_id)
        control_packet += struct.pack("<I", FLAGS.hw_ver)
        control_packet += struct.pack("<I", FLAGS.sdk_ver)
        control_packet += struct.pack("<I", FLAGS.fw_ver)
    
        img_count = struct.pack("<B", int(len(FLAGS.img_para)/3))
        bksize = struct.pack("<H", FLAGS.bksize)
    
        img_header = bksize+img_count
        #print(img_header)
        list_pair=[FLAGS.img_para[i:i+3] for i in range(0, len(FLAGS.img_para), 3)]
    
        print(list_pair)
        # 1. Generate AES Key - Encrypt the data with the AES session key
        session_key = get_random_bytes(32)
        #print("session_key")
        #for i in session_key:
            #print('%x'%i)
        control_packet += session_key
        for a in list_pair:
            img_bin = a[0]
            img_flag = int(a[1])
            img_id = struct.pack("<B", int(a[2]))
            #print(img_bin)
            #print(type(img_flag))
            [img_sig, img_len] = dfu_bin_generation(img_bin, img_flag, session_key)
            img_header += img_sig + img_len + struct.pack("<H", img_flag) + img_id
        img_header_len = struct.pack("<H", len(img_header))
        control_packet += img_header_len
        control_packet += img_header
        #print(control_packet)
        file_out = open('control_packet_value.txt', "wb")
        file_out.write(control_packet)
    
        #6. Encrypt header
        cnt_prefix = open(FLAGS.sigkey + "_hash.bin", "rb").read(8)
        cnt_prefix += b'\x00' * 4
        hash=SHA256.new(control_packet)
    
        cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0)
        cipher_core_aes = AES.new(recipient_key, AES.MODE_CTR, counter=cnter)
        enc_header = cipher_core_aes.encrypt(control_packet)
        
    
        file_out = open('enc_header.txt', "wb")
        file_out.write(enc_header)
        
        pri_key = RSA.import_key(open(FLAGS.sigkey+"_pri.pem").read())
        sign_rsa = pkcs1_15.new(pri_key)
        signature = sign_rsa.sign(hash)
    
        file_out = open('ctrl_packet.bin', "wb")
        file_out.write(hash.digest())
        file_out.write(enc_header)
        file_out.write(signature)
        file_out.close()
    else:
        list_pair=[FLAGS.img_para[i:i+3] for i in range(0, len(FLAGS.img_para), 3)]        
        session_key = get_random_bytes(32)
        for a in list_pair:
            img_bin = a[0]
            img_flag = int(a[1])
            img_id = struct.pack("<B", int(a[2]))
            #print(img_bin)
            #print(type(img_flag))
            dfu_bin_generation(img_bin, img_flag, session_key)
        print(list_pair)
    print(FLAGS.offline_img)
    if (FLAGS.offline_img == 0):
        return
    if (FLAGS.offline_img == 1): 
        ota_magic = [0x46, 0x43, 0x45, 0x53]
        file_offline = open('offline_install.bin', "wb")
        for byte_one in ota_magic:
            byte_str = struct.pack('B', byte_one)
            file_offline.write(byte_str)

        image_count = len(list_pair) + 1
        file_offline.write(struct.pack('B', image_count))
    
        info_len = image_count * 5
        file_offline.write(info_len.to_bytes(4, byteorder="little"))
        
    
        
        file_offline.write(DFU_IMG_ID_CTRL.to_bytes(1, byteorder="little"))
        ctrl_packet_len = (os.stat('ctrl_packet.bin')).st_size
        #print(ctrl_packet_len)
        file_offline.write(ctrl_packet_len.to_bytes(4, byteorder="little"))
        
        for a in list_pair:
            img_id = struct.pack("<B", int(a[2]))
            file_offline.write(img_id)
            file_name = "com_" + a[0] + ".bin"
            file_len = (os.stat(file_name)).st_size
            file_offline.write(file_len.to_bytes(4, byteorder="little"))
        
        file = open('ctrl_packet.bin', "rb")
        file_content = file.read();
        file_offline.write(file_content)
        file.close 
        
        for a in list_pair:
            file_name = "com_" + a[0] + ".bin"
            file = open(file_name, "rb")
            file_content = file.read();
            file_offline.write(file_content)
            file.close()   
            
        file_offline.close()
    elif (FLAGS.offline_img == 2):
        ota_magic = [0x46, 0x43, 0x45, 0x53]
        file_offline = open('offline_install.bin', "wb")
        for byte_one in ota_magic:
            byte_str = struct.pack('B', byte_one)
            file_offline.write(byte_str)
        
        ver = 1
        byte_str = struct.pack('B', ver)
        file_offline.write(byte_str)
 
        install_state = 0xFF
        byte_str = struct.pack('B', install_state)
        file_offline.write(byte_str)
        #file_offline.write(ver.to_bytes(2, byteorder="little"))

        image_count = len(list_pair)
        file_offline.write(image_count.to_bytes(2, byteorder="little"))
        
        crc_value = 0
        file_offline.write(crc_value.to_bytes(4, byteorder="little"))
        
        for a in list_pair:
            img_id = struct.pack("<B", int(a[2]))
            file_offline.write(img_id)
            flag = struct.pack("<B", int(a[1]))
            file_offline.write(flag)
            
            img_flag = int(a[1])
            if (img_flag == 16):
                file_name = "com_" + a[0] + ".bin"
                file_len = (os.stat(file_name)).st_size
                file_offline.write(file_len.to_bytes(4, byteorder="little"))
                #print("add file info com " + file_name + ", " + file_len)
            else:
                file_name = a[0] + ".bin"
                file_len = (os.stat(file_name)).st_size
                file_offline.write(file_len.to_bytes(4, byteorder="little"))
                #print("add file info " + file_name + ", " + file_len)

        
        file_data = bytes()
        for a in list_pair:
            img_flag = int(a[1])
            if (img_flag == 16):
                file_name = "com_" + a[0] + ".bin"
                file = open(file_name, "rb")
                file_content = file.read();
                file_data += file_content
                file_offline.write(file_content)
                file.close()
            else:
                file_name = a[0] + ".bin"
                file = open(file_name, "rb")
                file_content = file.read();
                file_offline.write(file_content)
                file_data += file_content
                file.close()
        
        crc_value = crc32mpeg2(file_data)
        print(crc_value)
        file_offline.seek(8)
        file_offline.write(crc_value.to_bytes(4, byteorder="little"))
        
        file_offline.close()
        file_offline = None
    
    #file_offline.write()
    
    #print(info_len.hex()) 
    #for a in list_pair:

def crc32mpeg2(buf, crc=0xffffffff):
    for val in buf:
        crc ^= val << 24
        for _ in range(8):
            crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
    return crc

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="General Usage ", 
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=textwrap.dedent('''\
         Note: Please do not use suffix for each key, for example, core01.pem, use core01 as keyname   
         Generate root key: python imgtool.py root --key=<key>
         Generate core key: python imgtool.py uid --key=<keyname> 
         Generate signature key: python imgtool.py gensig --key=<keyname> 
         Encrypt image:     python imgtool.py enc --key=<keyname> --img=<plain image> --eimg=<encrypted image> --bksize=<block size>
         Encrypt flashtable: python imgtool.py enctab --key=<keyname> --table=<flash table>
         NonEncrypt flashtable: python imgtool.py ftab --table=<flash table>
         
         Validate commands:
         Decrypt image:     python imgtool.py dec --key=<keyname> --img=<plain image> --eimg=<encrypted image> --bksize=<block size>  
         Decrypt flashtable: python imgtool.py dectab --key=<keyname> --table=<flash table>
         Print Non-ecnrypted: python imgtool.py rftab --table=<flash table>
         '''))

    parser.add_argument('action', choices=['enc', 'dec', 'uid', 'root','dumproot', 'enctab', 'dectab','ftab','rftab','gensig','dumpsig', 'test', 'compress', 'gen_dfu'], default='enc')
    parser.add_argument(
        '--img',
        type=str,
        default='sifli01.bin',
        help='Plain image')
    parser.add_argument(
        '--eimg',
        type=str,
        default='sifli01_e',
        help='Encrypted and signed image.')
    parser.add_argument(
        '--bksize',
        type=int,
        default=512,
        help='Block size in bytes to encrypt.')
    parser.add_argument(
        '--key',
        type=str,
        default='sifli01key',
        help='File for key.')
    parser.add_argument(
        '--sigkey',
        type=str,
        default='sifli01key',
        help='File for signature key in PEM format.')        
    parser.add_argument(
        '--table',
        type=str,
        default='ftab',
        help='Encrypted Flash table')
    parser.add_argument(
        '--flags',
        type=int,
        default=1,
        help='Image flags, 1:save encrypted')
    parser.add_argument(
        '--ver',
        type=str,
        default='sifli_0.1.0',
        help='Image version, length should less than 20 Byte')
    parser.add_argument(
        '--pksize',
        type=int,
        default=10240,
        help='Compress block size')
    parser.add_argument(
        '--img_para',
        type=str,
        default=10240,
        nargs='*',
        help='img parameter list')
    parser.add_argument(
        '--dfu_id',
        type=int,
        default=0,
        help='DFU ID')
    parser.add_argument(
        '--hw_ver',
        type=int,
        default=0,
        help='HW version requirement')
    parser.add_argument(
        '--sdk_ver',
        type=int,
        default=0,
        help='SDK version requirement')
    parser.add_argument(
        '--fw_ver',
        type=int,
        default=0,
        help='FW version')
    parser.add_argument(
        '--com_type',
        type=int,
        default=0,
        help='Compress type')
    parser.add_argument(
        '--offline_img',
        type=int,
        default=0,
        help='general offline install packet')
    parser.add_argument(
        '--align',
        type=int,
        default=1,
        help='make general file 16-byte alignment')
    FLAGS, unparsed = parser.parse_known_args()
    FLAGS, unparsed = parser.parse_known_args()
    FLAGS, unparsed = parser.parse_known_args()
    FLAGS, unparsed = parser.parse_known_args()
    print(FLAGS.action)
    if (FLAGS.action == 'uid'):
        gen_uid()
    if (FLAGS.action == 'enc'):
        encrypt_image()
    if (FLAGS.action == 'dec'):
        decrypt_image()
    if (FLAGS.action == 'root'):
        gen_root()
    if (FLAGS.action == 'enctab'):
        enc_ftab()    
    if (FLAGS.action == 'dectab'):
        dec_ftab()    
    if (FLAGS.action == 'ftab'):
        ftab()         
    if (FLAGS.action == 'rftab'):
        rftab()
    if (FLAGS.action == 'gensig'):
        gen_key2()
    if (FLAGS.action == 'dumpsig'):
        dump_sig()    
    if (FLAGS.action == 'test'):
        test_vec()   
    if (FLAGS.action == 'compress'):
        print("decompress")
        compress_and_sign_image()
    if (FLAGS.action == 'gen_dfu'):
        dfu_generation()
