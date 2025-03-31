# 升级固件生成

SIFLI提供了exe程序来生成升级包，其位置在 _$SDK_ROOT\\mcu\\tools\\secureboot\\dfu_bin_generate37.exe_w

由于升级包的验证需要加密，如果没有烧写加密的efuse，请直接使用mcu\\tools\\secureboot\\sifli02\\下的bin进行加密

## exe使用说明：
- 命令：
```bat
dfu_bin_generate.exe gen_dfu --img_para <bin_name, etc: img> <flag, ect: 17> <img_id, ect: 0> --key=<keyname, ect: s01> --sigkey=<sig bin, ect: sig> --dfu_id=<dfu_id, ect:1> --hw_ver=<hw version, ect: 51> --sdk_ver=<sdk_lowest_ver, ect: 7010> --fw_ver=<fw_ver, ect: 1001001>
```
- 命令说明:
    - img_para: 需要升级的bin，可以一次指定多个bin及参数, 其子参数有:
        - bin_name, 升级bin的名字, 不需要带后缀
        - flag, 升级bin的属性，可以指定下列属性
        - DFU_FLAG_ENC  1, 升级bin会以加密形式存储在flash上
        - DFU_FLAG_COMPRESS 16，升级bin会被压缩
        - Img_id, 升级bin的类型
        - DFU_IMG_ID_HCPU  0，代码
        - DFU_IMG_ID_RES  3，资源
        - DFU_IMG_ID_FONT  4, 字体

        例：
        1.	单独升级叫app.bin的代码bin： `--img_para app 16 0`
        2.	升级app.bin的代码bin以及资源res.bin和字体font.bin: `--img_para app 16 0 res 0 3 font 0 4`
    - key: 升级包的加密Key，默认请使用`mcu\tools\secureboot\sifli02\s01.bin`\
        例： `--key=s01`
    - sigkey：升级包校验用的签名Key，默认请使用`mcu\tools\secureboot\sifli02\sig_hash.bin`\
        例：`--sigkey=sig`
    - dfu_id：升级包的类型
    - DFU_ID_CODE  0，升级包只包含代码
    - DFU_ID_MIX  1，升级包除代码外，还包含资源或字体
    - hw_ver: 还没有启用，请直接用--hw_ver=51
    - sdk_ver: 要求设备端最低的版本号，格式为xxyyzzzzz
        例：`--sdk_ver=70001`，指定最低SDK版本号为0.7.1
    - fw_ver: 用户工程版本号，由用户自行决定\n

## 示例：
- 只升级代码bin：
```bat
dfu_bin_generate37.exe gen_dfu --img_para app 16 0 --key=s01 --sigkey=sig --dfu_id=1 --hw_ver=51 --sdk_ver=7001 --fw_ver=1001001
```
- 升级代码、资源和字体：
```bat
dfu_bin_generate37.exe gen_dfu --img_para app 16 0 res 0 3 font 0 4 --key=s01 --sigkey=sig --dfu_id=1 --hw_ver=51 --sdk_ver=7001 --fw_ver=1001001
```

## 升级包组成

执行升级包脚本后，会生成一个 _ctrl_packet.bin_ 的header file，每个待升级的bin都会有个对应的bin，其命名根据是否有压缩和加密来决定：
- 如果有压缩没有加密，会加上com_的前缀
- 如果有加密，会加上enc_的前缀，如果没有加密，会加上out_的前缀


