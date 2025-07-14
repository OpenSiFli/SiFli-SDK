# Secure Bootloader

The butterfli on-chip ROM contains a secure bootloader with the following features:
- UART download
- AES encryption for image file transmission, Flash can store encrypted images with hardware real-time decryption for execution, also supports plaintext images
- Encrypted storage of image keys, preventing execution on different butterfli chips even if encrypted images are copied
- RSA digital signature verification for image file integrity, preventing download of illegal image files
- PingPong download
- Install patches to modify Flash drivers

## Keys
The bootloader uses the following keys:
- rootkey: Stored in the non-readable area of efuse, used to encrypt image keys
- image key: Key used for image encryption/decryption, stored in Flash after encryption with rootkey
- Digital signature RSA public key: Used to decrypt and verify image identity and integrity
- UID: Chip ID, stored in the readable area of efuse, used as nonce parameter to encrypt image keys

Image encryption download process:
1. Write rootkey, digital signature public key hash, and UID to efuse (only once)
2. Download Flash configuration table and digital signature public key to Flash (only once)
3. Encrypt image using image key, calculate hash of encrypted data using SHA256, encrypt hash using digital signature private key to obtain digital signature,
package image header (encrypted with rootkey, containing image key and other information), encrypted image, and digital signature to generate final encrypted image
4. Transmit encrypted image, bootloader extracts image key from image header to decrypt image, after receiving, compare digital signature to confirm source and verify integrity, if valid, download is successful

## Boot Modes
The bootloader boot mode can be set through bootmode jumpers. If jumper is connected to 0, it's normal boot mode where bootloader reads flash to determine if user program can be booted. If jumper is connected to 1, it enters download mode waiting for PC to send download commands.
In normal boot mode, if no valid user image is found on flash, it automatically enters download mode.

## Memory Planning

**Default Flash Address Space Planning**
|Name             | Start Address  | End Address    | Size(bytes)| Description
|-----------------|----------------|----------------|------------|--------
|Flash Config Table| 0x10000000    | 0x10004FFF     | 20*1024    | |
|Calibration Table| 0x10005000     | 0x10006FFF     | 8*1024     | |
|Reserved         | 0x10007000     | 0x1000FFFF     | 36*1024    | |
|bootrom patch    | 0x10010000     | 0x1001FFFF     | 64*1024    | |
|User Code        | 0x10020000     | | | | 

The location and format of the Flash configuration table cannot be modified. The addresses and sizes of other table entries are specified by the Flash configuration table.

**Flash Configuration Table Format**
Name              | Start Offset   | End Offset     | Size(bytes)| Description
------------------|----------------|----------------|------------|------------
MAGIC             | 0x00000000     | 0x00000003     | 4          | 0x53454346
Flash Partition Table| 0x00000004  | 0x00000103     | 16*16      | 16 partitions
Digital Signature Public Key| 0x00000104| 0x00000229| 294        | RSA public key in DER format
Reserved          | 0x0000022A     | 0x00000FFF     | 3542       | |
Image Descriptor Table| 0x00001000 | 0x00002BFF     | 14*512     | 14 image descriptor entries
Image Index Table | 0x00002C00     | 0x00002C0F     | 4*4        | Points to active (currently running) image descriptor table entry

The Flash partition table consists of 16 partition information entries, each specifying the start address and size of the partition.

**Partition Table**
|Index       | Name                | Description
|------------|---------------------|------------
|0           | Flash Partition Table| | 
|1           | Calibration Table   | | 
|2           | LCPU image Ping area| | 
|3           | BCPU image Ping area| | 
|4           | HCPU image Ping area| | 
|5           | Flash Boot patch area| | 
|6           | LCPU image Pong area| | 
|7           | BCPU image Pong area| | 
|8           | HCPU image Pong area| | 
|9           | RAM Boot patch area | | 
|10          | HCPU image extension area 1| | 
|11          | HCPU image extension area 2| | 
|12          | LCPU image extension area 1| | 
|13          | LCPU image extension area 2| | 
|14          | Reserved            | | 
|15          | Reserved            | | 

**Partition Information Format**
Name              | Size(bytes)| Description
------------------|------------|------------
Partition Start Address| 4     | Start address of partition
Partition Size    | 4          | Partition size
Partition Execute Address| 4   | For image partitions, specifies the execution address of this partition
Flags             | 4          | Unused

The following table shows the default Flash configuration table. `struct sec_configuration` is the data structure name for the Flash configuration table.
```c
const struct sec_configuration sec_config =
{
    .magic = SEC_CONFIG_MAGIC,
    .ftab[0] = {.base = 0x10000000,      .size = 20*1024,      .xip_base = 0,          .flags = 0},
    .ftab[1] = {.base = 0x10005000,      .size = 8*1024,       .xip_base = 0,          .flags = 0},
    .ftab[4] = {.base = 0x10020000,      .size = 0x80000,      .xip_base = 0x10020000, .flags = 0},
    .ftab[5] = {.base = 0x10010000,      .size = 64*1024,      .xip_base = 0x10010000, .flags = 0},
    .ftab[8] = {.base = 0x10020000,      .size = 0x80000,      .xip_base = 0x10020000, .flags = 0},
    .ftab[9] = {.base = 0x20050000,      .size = 64*1024,      .xip_base = 0x20050000, .flags = 0},
    .imgs[0]  = {.length = 0xFFFFFFFF},                                       //LCPU ping
    .imgs[1]  = {.length = 0xFFFFFFFF},                                       //BCPU ping
    .imgs[2]  = {.length = 0x80000, .blksize = 512, .flags = DFU_FLAG_AUTO},  //HCPU ping
    .imgs[3]  = {.length = 0xFFFFFFFF},                                       //Flash boot patch
    .imgs[4]  = {.length = 0xFFFFFFFF},                                       //LCPU pong
    .imgs[5]  = {.length = 0xFFFFFFFF},                                       //BCPU pong
    .imgs[6]  = {.length = 0xFFFFFFFF},                                       //HCPU pong
    .imgs[7]  = {.length = 0xFFFFFFFF},                                       //RAM boot patch   
    .imgs[8]  = {.length = 0xFFFFFFFF},
    .imgs[9]  = {.length = 0xFFFFFFFF},
    .imgs[10] = {.length = 0xFFFFFFFF},
    .imgs[11] = {.length = 0xFFFFFFFF},
    .imgs[12] = {.length = 0xFFFFFFFF},
    .imgs[13] = {.length = 0xFFFFFFFF},
    .running_imgs[0] = (struct image_header_enc *)0xFFFFFFFF,
    .running_imgs[1] = (struct image_header_enc *)0xFFFFFFFF,
    .running_imgs[2] = (struct image_header_enc *) &sec_config.imgs[2],
    .running_imgs[3] = (struct image_header_enc *)0xFFFFFFFF,
};
```

The bootrom uses the RAM address space shown in the following table. When using bootrom to boot user programs to execute in RAM, address space conflicts must be avoided.

**Bootloader RAM Address Space**
Name             | Start Address  | End Address    | Size(bytes)| Description
-----------------|----------------|----------------|------------|--------
DATA             | 0x20040000     | 0x2004FFFF     | 64*1024    | Bootrom data
PATCH CODE       | 0x20050000     | 0x2005FFFF     | 64*1024    | Patch code
PATCH DATA       | 0x20060000     | 0x2006FFFF     | 64*1024    | Patch data

## Encrypted Image Generation and Download
### Download without bootloader
Use JTAG to download the default Flash configuration table provided by SDK to address `0x10000000`, then download unencrypted user image to address `0x10020000`. After reset, the bootloader can boot the user image to run. The user image is compiled using address `0x10020000`.

### Download with bootloader
Related tools are located in `$sdk_root/tools/secureboot`. The bootloader uses the chip's UART4 as the download serial port.

#### Generate Keys
Execute script `genkeys.bat` to generate UID, rootkey, digital signature keys and public key hash.
```bat
genkeys.bat sifli01
```

`sifli01` is the path for generating keys. After running, the following files will be generated:

Name         | Description       
-------------|------------
s01.bin      | rootkey    
uid.bin      | rootkey    
sig_hash.bin | Digital signature public key hash
sig_pri.pem  | Digital signature private key
sig_pub.pem  | Digital signature public key    
sig_pub.der  | Digital signature public key in DER format

#### Program Keys
Programming keys to efuse can only be executed once. After programming, the keys in efuse cannot be modified.

1. Set butterfli bootmode jumper to download mode
2. Enter bootloader download mode after reset
3. Run script `program_efuse.bat` to program keys. The first parameter is the path where keys are located, the second parameter is the serial port number
```bat
program_efuse.bat sifli01/ COM3
```
4. Reset the chip to ensure keys are loaded correctly

#### Download Flash Configuration Table
1. Enter bootloader download mode
2. Run script `factory_flash_default.bat` to download flash configuration table (`ftab\ftab_flash_default.json`) and digital signature public key. The first parameter is the path where keys are located, the second parameter is the serial port number
```bat
factory_flash_default.bat sifli01/ COM3
```

If the Flash configuration table has not changed, there is no need to download it again when updating the image.

#### Generate Image

##### Encrypted Image
Run `gen_sec_img.bat`. The first parameter is the path where keys are located, the second parameter is the image path to be encrypted. The generated encrypted image is named `image_sec.bin`
```bat
gen_sec_img.bat sifli01/ example.bin
```

##### Plaintext Image
Run `gen_plain_img.bat`. The first parameter is the path where keys are located, the second parameter is the image path to be encrypted. The generated encrypted image is named `image_plain.bin`
```bat
gen_plain_img.bat sifli01/ example.bin
```

#### Download Image
##### Encrypted Image
Run `download_sec_image.bat`. The specified port is the serial port used by the bootloader
```bat
download_sec_image.bat --port=COM3
```

#### Plaintext Image
Run `download_plain_image.bat`. The specified port is the serial port used by the bootloader
```bat
download_plain_image.bat --port=COM3
```