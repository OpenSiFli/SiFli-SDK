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

from Crypto.PublicKey import RSA
from Crypto.Random import get_random_bytes
from Crypto.Cipher import AES, PKCS1_OAEP
from Crypto.Hash import SHA256
from Crypto.Signature import pkcs1_15
from Crypto.Util import Counter

IMG_BLK_SIZE = 512
HCPU_IMG_HD_OFFSET = 5120
SIG_PUB_KEY_OFFSET = 260
     
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
    #1. Generate image AES Key
    image_key=get_random_bytes(32)
    with open(eimg+'/'+"img_key.bin","wb") as file_imgkey:
        file_imgkey.write(image_key)
    #2. Encrypt the image AES key with root_key and uid,save it
    with open(FLAGS.key+".bin", "rb") as file_rootkey:
        root_key=file_rootkey.read()
    with open(FLAGS.uid+".bin", "rb") as file_uid:
        uid=file_uid.read()
    imgkey_aes=AES.new(root_key,AES.MODE_CBC,iv=uid)
    ciphertext_imgkey=imgkey_aes.encrypt(image_key)
    with open(eimg+'/'+"imgkey_cip.bin","wb") as file_imgkey_cip:
        file_imgkey_cip.write(ciphertext_imgkey)
    #3. Align image to 16 bytes 
    with open(img,"rb") as file_image:
        image=file_image.read()
    image=bytearray(image)
    remain=(len(image) % 16)
    if(remain>0) :
        remain=16-remain
        str=u'0'
        for i in range(0, remain):
            image.append(0)
    #4. Encrypt image
    with open(FLAGS.sigkey+"_hash.bin","rb") as file_sig_hash:
        cnt_prefix=file_sig_hash.read(8)
    cnt_prefix+='\x00'*4
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0)
    image_aes=AES.new(image_key,AES.MODE_CTR,counter=cnter) 
    ciphertext_image=image_aes.encrypt(image)
    with open(eimg+'/'+ eimg+'.bin',"wb") as file_sec_image:
        file_sec_image.write(ciphertext_image)
    #5. get image hash , signature image hash with private sigkey
    hash=SHA256.new(image)
    image_hash=hash.digest()
    with open(eimg+'/'+ 'image_hash.bin',"wb") as file_hash:
        file_hash.write(image_hash)
    with open(FLAGS.sigkey+"_pri.pem") as file_sigkey:
        sigkey_pri = file_sigkey.read()
    pri_key = RSA.import_key(sigkey_pri)
    sign_rsa = pkcs1_15.new(pri_key)
    signature = sign_rsa.sign(hash)
    with open(eimg+'/'+ 'image_hash_sign.bin',"wb") as file_hash_sign:
        file_hash_sign.write(signature)
    
def encrypt_image() :
    if os.path.exists(FLAGS.eimg):
        print("Skip create, already exist:"+FLAGS.eimg)
    else:
        os.makedirs(FLAGS.eimg)
    encrypt_image_help(FLAGS.img,FLAGS.eimg)

def decrypt_image_help(img,eimg) :
    #1. get root key, uid, decrypt image key
    with open(FLAGS.key+".bin", "rb") as file_rootkey:
        root_key=file_rootkey.read()
    with open(FLAGS.uid+".bin", "rb") as file_uid:
        uid=file_uid.read()
    with open(eimg+'/'+"imgkey_cip.bin", "rb") as file_imgkey_cip:
        imgkey_cip=file_imgkey_cip.read()
    imgkey_aes=AES.new(root_key,AES.MODE_CBC,iv=uid)
    imgkey_pla=imgkey_aes.decrypt(imgkey_cip)
    with open(eimg+'/'+"imgkey_pla.bin","wb") as file_imgkey_pla:
        file_imgkey_pla.write(imgkey_pla)
    #2. decrypt image with image key and sig_hash
    with open(eimg+'/'+ eimg+'.bin',"rb") as file_img_cip:
        image_cip=file_img_cip.read()
    with open(FLAGS.sigkey+"_hash.bin", "rb") as file_cnt:
        cnt_prefix=file_cnt.read(8)
    cnt_prefix+='\x00'*4
    cnter=Counter.new(32,prefix=cnt_prefix,initial_value=0)
    image_aes = AES.new(imgkey_pla, AES.MODE_CTR, counter=cnter)
    image_pla = image_aes.decrypt(image_cip)
    with open(eimg+'/'+'image_pla.bin',"wb") as file_img_pla:
        file_img_pla.write(image_pla)

def decrypt_image() :
    if os.path.exists(FLAGS.eimg):
        decrypt_image_help(FLAGS.img,FLAGS.eimg)
    else:
        print("Skip decrypt, not find:"+FLAGS.eimg)

def add_imgkey_to_ftab(ftab,eimg):
    #1.get ciphertext image key,get ciphertext image len, get sigkey_pub, image_hash_sign
    with open(eimg+'/'+"imgkey_cip.bin", "rb") as file_imgkey:
        imgkey_cip=file_imgkey.read()
    with open(eimg+'/'+"image_sec.bin", "rb") as file_image:
        image_cip=file_image.read()
    with open(FLAGS.sigkey + "_pub.der", "rb") as file_sigkey_pub:
        sigkey_pub=file_sigkey_pub.read()
    with open(eimg+'/'+ 'image_hash_sign.bin',"rb") as file_hash_sign:
        img_hash_sig=file_hash_sign.read()

    #2.get ftab, update image size and image key
    with open(ftab, "rb") as file_ftab:
        ftab_data=file_ftab.read()
    len1 = len(ftab_data)
    length = struct.pack("<I", len(image_cip))
    blksize = struct.pack("<H", IMG_BLK_SIZE)
    flags = struct.pack("<H", 1)
    img_hd_enc = length+blksize+flags+imgkey_cip+img_hash_sig
    len2 = len(img_hd_enc)
    ftab_data1 = ftab_data[0:SIG_PUB_KEY_OFFSET]+sigkey_pub+ftab_data[SIG_PUB_KEY_OFFSET+len(sigkey_pub):HCPU_IMG_HD_OFFSET] + img_hd_enc + ftab_data[HCPU_IMG_HD_OFFSET+len2:len1]
    with open(eimg+'/'+"ftab_sec.bin", "wb") as file_ftab1:
        file_ftab1.write(ftab_data1)

    '''
    ftab_data2 = open(eimg+'/'+"ftab_sec.bin", "rb").read()
    leng,blksi,fla=struct.unpack("<IHH",ftab_data2[5120:5128])
    print(leng)
    print(blksi)
    print(fla)
    print('sig_pub:\n')
    print_hex(sigkey_pub)
    print('\n')
    print_hex(ftab_data2[260:554])
    print('\n')
    print('imgkey_cip:\n')
    print_hex(imgkey_cip)
    print('\n')
    print_hex(ftab_data2[5128:5160])
    print('\n')
    print('img_hash_sig:\n')
    print_hex(img_hash_sig)
    print('\n')
    print_hex(ftab_data2[5160:5416])
    '''

def gen_ftab():
    if os.path.exists(FLAGS.eimg):
        add_imgkey_to_ftab(FLAGS.table,FLAGS.eimg)
    else:
        print("Skip gen_ftab, not find:"+FLAGS.eimg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="General Usage ", 
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog=textwrap.dedent('''\
         Encrypt image: python imgtool.py enc_image --key=<keyname> --uid=<uid> --img=<plain image> --eimg=<encrypted image> --sigkey=<sig key>
         Decrypt image: python imgtool.py dec_image --key=<keyname> --uid=<uid> --img=<plain image> --eimg=<encrypted image> --sigkey=<sig key>
         gen flashtable: python python imgtool.py gen_ftab --table=<flash table> --eimg=<encrypted image>'''))

    parser.add_argument('action', choices=['enc_image', 'dec_image', 'gen_ftab'], default='enc_image')
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
        '--uid',
        type=str,
        default='sifli01uid',
        help='UID')
        
    FLAGS, unparsed = parser.parse_known_args()
    if (FLAGS.action == 'enc_image'):
        encrypt_image()
    if (FLAGS.action == 'dec_image'):
        decrypt_image()
    if (FLAGS.action == 'gen_ftab'):
        gen_ftab()

