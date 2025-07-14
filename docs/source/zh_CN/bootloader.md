# 安全引导加载

butterfli的片上ROM包含一个安全引导加载 （bootloader），其特性有:
- UART下载
- 使用AES加密传输image文件，Flash上可保存加密image，由硬件实时解密执行，也支持明文image
- 加密保存image密钥，即使复制了加密image，也无法在一颗不同的butterfli芯片上执行
- 使用RSA数字签名验证image文件完整性，防止下载非法的image文件
- PingPong下载
- 安装Patch，修改Flash驱动

## 密钥
bootloader使用的密钥有:
- rootkey: 保存于efuse的不可读区域，用于加密image密钥
- image密钥: image加解密使用的密钥，经rootkey加密后保存于Flash
- 数字签名RSA公钥: 用于解密验证image身份和完整性
- UID: 芯片ID，保存于efuse的可读区域，作为nonce参数加密image密钥

image的加密下载过程:
1. 将rootkey、数字签名公钥的hash和UID写入efuse （只需一次）
2. 下载Flash配置表和数字签名公钥到Flash （只需一次）
3. 使用image密钥对image进行加密，使用SHA256计算加密后数据的hash，使用数字签名私钥加密hash得到数字签名，
image header（使用rootkey加密，包含image密钥等信息）、加密image和数字签名三者打包生成最终的加密image
4. 传输加密image，bootloader从image header中提取image密钥对image进行解密，接收完成后比对数字签名确认来源并验证完整性，若合法则下载成功

## 启动模式
可以通过bootmode跳线设置bootloader的启动模式，若跳线接0，则为正常启动，bootloader读取flash判断是否可以引导用户程序，若跳线接1，则进入下载模式，等待PC发送下载命令，
在正常启动模式如果发现flash上没有有效的用户image，则自动进入下载模式


## Memory规划

**默认的Flash地址空间规划**
|名称             | 起始地址       | 结束地址       | 大小(字节) | 描述
|-----------------|----------------|----------------|------------|--------
|Flash配置表      | 0x10000000     | 0x10004FFF     | 20*1024    | |
|校准表           | 0x10005000     | 0x10006FFF     | 8*1024     | |
|保留             | 0x10007000     | 0x1000FFFF     | 36*1024    | |
|bootrom patch    | 0x10010000     | 0x1001FFFF     | 64*1024    | |
|User Code        | 0x10020000     | | | | 

其中Flash配置表的位置和格式不可修改，其他表项的地址和大小由Flash配置表指定


**Flash配置表格式**
名称              | 起始偏移       | 结束偏移       | 大小(字节) | 描述
------------------|----------------|----------------|------------|------------
MAGIC             | 0x00000000     | 0x00000003     | 4          | 0x53454346
Flash分区表       | 0x00000004     | 0x00000103     | 16*16      | 16个分区
数字签名公钥      | 0x00000104     | 0x00000229     | 294        | DER格式的RSA公钥
保留              | 0x0000022A     | 0x00000FFF     | 3542       | |
image描述表       | 0x00001000     | 0x00002BFF     | 14*512     | 14个image描述条目，
image索引表       | 0x00002C00     | 0x00002C0F     | 4*4        | 指向激活（当前运行 ）的image描述表条目




Flash分区表由16个分区信息组成，每个分区信息指定分区的起始地址和大小

**分区表**
|索引        | 名称                | 描述
|------------|---------------------|------------
|0           | Flash分区表         | | 
|1           | 校准表              | | 
|2           | LCPU image Ping区   | | 
|3           | BCPU image Ping区   | | 
|4           | HCPU image Ping区   | | 
|5           | Flash Boot patch区  | | 
|6           | LCPU image Pong区   | | 
|7           | BCPU image Pong区   | | 
|8           | HCPU image Pong区   | | 
|9           | RAM Boot patch区    | | 
|10          | HCPU image扩展区1   | | 
|11          | HCPU image扩展区2   | | 
|12          | LCPU image扩展区1   | | 
|13          | LCPU image扩展区2   | | 
|14          | 保留                | | 
|15          | 保留                | | 



**分区信息格式**
名称              | 大小(字节) | 描述
------------------|------------|------------
分区起始地址      | 4          | 分区的起始地址
分区大小          | 4          | 分区大小
分区执行地址      | 4          | 对于image分区，指定该分区的执行地址
标志              | 4          | 未使用




下表为默认的Flash配置表，`struct sec_configuration`是Flash配置表的数据结构名
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

Bootrom使用下表所示的RAM地址空间，当使用bootrom引导用户程序在RAM中执行时，需要避免地址空间冲突

**bootloader RAM地址空间**
名称             | 起始地址       | 结束地址       | 大小(字节) | 描述
-----------------|----------------|----------------|------------|--------
DATA             | 0x20040000     | 0x2004FFFF     | 64*1024    | Bootrom数据
PATCH CODE       | 0x20050000     | 0x2005FFFF     | 64*1024    | Patch代码
PATCH DATA       | 0x20060000     | 0x2006FFFF     | 64*1024    | Patch数据


## 加密image的生成与下载
### 不使用bootloader下载
使用JTAG下载SDK提供的默认Flash配置表到`0x10000000`地址，再下载未加密的用户image到地址`0x10020000`，复位后bootloader即可引导用户image运行，用户image使用`0x10020000`地址编译

### 使用bootloader下载
相关工具在`$sdk_root/tools/secureboot`下，bootloader使用芯片的UART4作为下载串口

#### 生成密钥
执行脚本`genkeys.bat`生成UID、rootkey、数字签名密钥和公钥hash，
```bat
genkeys.bat sifli01
```

`sifli01`为生成密钥的路径，运行后会生成如下几个文件

名称         | 描述       
-------------|------------
s01.bin      | rootkey    
uid.bin      | rootkey    
sig_hash.bin | 数字签名公钥hash
sig_pri.pem  | 数字签名私钥
sig_pub.pem  | 数字签名公钥    
sig_pub.der  | DER格式的数字签名公钥


#### 烧写密钥
将密钥烧写到efuse中只能执行一次，烧写后无法再修改efuse中的密钥

1. 将butterfli的bootmode跳线设为下载模式
2. 复位后进入bootloader下载模式
3. 运行脚本`program_efuse.bat`烧写密钥，第一个参数为密钥所在路径，第二个参数为串口号
```bat
program_efuse.bat sifli01/ COM3
```
4. 复位芯片，使得密钥被正确加载

#### 下载Flash配置表
1. 进入bootloader下载模式
2. 运行脚本`factory_flash_default.bat`下载flash配置表(`ftab\ftab_flash_default.json`)和数字签名公钥，第一个参数为密钥所在路径，第二个参数为串口号
```bat
factory_flash_default.bat sifli01/ COM3
```

如果Flash配置表没有变化，更新image时不需要再次下载

#### 生成image

##### 加密image
运行`gen_sec_img.bat`，第一个参数为密钥所在路径，第二个参数为待加密的image路径，生成的加密image名为`image_sec.bin`
```bat
gen_sec_img.bat sifli01/ example.bin
```

##### 明文image
运行`gen_plain_img.bat`，第一个参数为密钥所在路径，第二个参数为待加密的image路径，生成的加密image名为`image_plain.bin`
```bat
gen_plain_img.bat sifli01/ example.bin
```


#### 下载image
##### 加密image
运行`download_sec_image.bat`， 指定的port为bootloader所使用的串口
```bat
download_sec_image.bat --port=COM3
```


#### 明文image
运行`download_plain_image.bat`，指定的port为bootloader所使用的串口
```bat
download_plain_image.bat --port=COM3
```
