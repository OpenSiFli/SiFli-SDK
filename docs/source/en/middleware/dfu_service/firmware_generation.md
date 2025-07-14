# Upgrade Firmware Generation

SIFLI provides an exe program to generate upgrade packages, located at _$SDK_ROOT\\mcu\\tools\\secureboot\\dfu_bin_generate37.exe_

Since upgrade package verification requires encryption, if encrypted efuse has not been programmed, please directly use the bins in mcu\\tools\\secureboot\\sifli02\\ for encryption

## exe Usage Instructions:
- Command:
```bat
dfu_bin_generate.exe gen_dfu --img_para <bin_name, e.g.: img> <flag, e.g.: 17> <img_id, e.g.: 0> --key=<keyname, e.g.: s01> --sigkey=<sig bin, e.g.: sig> --dfu_id=<dfu_id, e.g.:1> --hw_ver=<hw version, e.g.: 51> --sdk_ver=<sdk_lowest_ver, e.g.: 7010> --fw_ver=<fw_ver, e.g.: 1001001>
```
- Command Description:
    - img_para: Bins to be upgraded, multiple bins and parameters can be specified at once. Its sub-parameters include:
        - bin_name: Name of the upgrade bin, without suffix
        - flag: Attributes of the upgrade bin, can specify the following attributes
        - DFU_FLAG_ENC  1: Upgrade bin will be stored in encrypted form on flash
        - DFU_FLAG_COMPRESS 16: Upgrade bin will be compressed
        - Img_id: Type of the upgrade bin
        - DFU_IMG_ID_HCPU  0: Code
        - DFU_IMG_ID_RES  3: Resources
        - DFU_IMG_ID_FONT  4: Font

        Examples:
        1.	Upgrade only code bin named app.bin: `--img_para app 16 0`
        2.	Upgrade code bin app.bin along with resource res.bin and font font.bin: `--img_para app 16 0 res 0 3 font 0 4`
    - key: Encryption key for upgrade package, by default use `mcu\tools\secureboot\sifli02\s01.bin`\
        Example: `--key=s01`
    - sigkey: Signature key for upgrade package verification, by default use `mcu\tools\secureboot\sifli02\sig_hash.bin`\
        Example: `--sigkey=sig`
    - dfu_id: Type of upgrade package
    - DFU_ID_CODE  0: Upgrade package contains only code
    - DFU_ID_MIX  1: Upgrade package contains resources or fonts in addition to code
    - hw_ver: Not yet enabled, please use --hw_ver=51 directly
    - sdk_ver: Minimum required version number on device side, format is xxyyzzzzz
        Example: `--sdk_ver=70001`, specifies minimum SDK version as 0.7.1
    - fw_ver: User project version number, determined by user\n

## Examples:
- Upgrade code bin only:
```bat
dfu_bin_generate37.exe gen_dfu --img_para app 16 0 --key=s01 --sigkey=sig --dfu_id=1 --hw_ver=51 --sdk_ver=7001 --fw_ver=1001001
```
- Upgrade code, resources and fonts:
```bat
dfu_bin_generate37.exe gen_dfu --img_para app 16 0 res 0 3 font 0 4 --key=s01 --sigkey=sig --dfu_id=1 --hw_ver=51 --sdk_ver=7001 --fw_ver=1001001
```

## Upgrade Package Composition

After executing the upgrade package script, a header file _ctrl_packet.bin_ will be generated. Each bin to be upgraded will have a corresponding bin, named according to whether it has compression and encryption:
- If compressed but not encrypted, prefix com_ will be added
- If encrypted, prefix enc_ will be added; if not encrypted, prefix out_ will be added