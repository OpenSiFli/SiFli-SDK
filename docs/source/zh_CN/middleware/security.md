# 基于52x平台的安全方案

基于52x平台的安全方案主要包含安全启动, 安全隔离，安全存储，安全升级四个部分。下面对安全启动和安全隔离做详细说明。

## 安全启动
安全启动是从软硬件层面对产品的部分系统组件进行保护，防止攻击者对系统关键部分进行读写破坏，以达到对产品的商业保密、知识产权保护的目的。
52x安全启动任务执行在bootloader中，bootloader启动的时候会从efuse中读取rootkey、uid，对image进行解密和数字签名校验，验证通过后跳转执行
image程序，校验失败会打印失败信息，进入异常处理。

### 安全启动的要求
- 应从不可变的代码开始执行启动。
- 应有对可信系统组件、可信应用组件或其他安全子系统的签名验证机制。
- 应有对可信系统组件、可信应用组件或其他安全子系统的完整性验证机制，防止非法篡改等。
- 应具备安全启动过程的异常处理机制。

### 安全启动密钥
安全启动使用的密钥和数据有：
- rootkey：保存于efuse的不可读区域BANK3中，用于加密image密钥。
- uid: 芯片id，保存于efuse的可读区域BANK0，作为nonce用于image key的解密。
- image key：随机产生的32字节数据，经rootkey加密后保存于ftab，用于image解密。
- RSA数字签名公钥：保存于ftab中，用于验证image身份和完整性。
- RSA公钥哈希值：rsa数字签名公钥的哈希值，前8字节保存于efuse可读区域BANK0，作为nonce用于image的解密以及验证RSA数字签名公钥完整性。
- image 哈希值数字签名：将image明文哈希计算，用RSA密钥对哈希值进行数字签名所得，保存于ftab中,用于验证image身份和完整性。
 
### 安全启动加载流程
1. 获取image header，通过flag判断是否是加密的image。
2. rsa公钥哈希校验，从ftab中读取rsa公钥，进行哈希计算得到哈希值，从efuse中读取sig_hash，两者进行比对校验，校验成功继续往下走，校验失败进入异常处理。
3. 判断是否在flash里执行image程序。
4. 如果在flash里运行，先从ftab中读取加密的image key，从efuse中读取rootkey和uid，对image key解密，再将解密后的imagekey配置到flash xip中。
5. 如果在ram中执行，先从ftab中读取加密的image key，从efuse中读取rootkey和uid，对image key解密，再将加密的image拷贝到ram中，使用image key
   对加密的image进行解密。
6. 对image的数字签名和完整性进行校验，校验成功跳转到执行image程序，校验失败进入异常处理。数字签名是通过对明文的image进行哈希计算，并将计算后的哈希值
   用rsa私钥进行数字签名而产生，签名后的值写入到ftab中。
#### 数字签名和完整性验证分为以下几个步骤：
- 将解密后的image做哈希计算，得到image哈希值。
- 从ftab中读取数字签名数据值及rsa数字签名公钥。
- 调用mbedtls库提供的verify接口，将image哈希值、数字签名数据值、数字签名公钥传入进行校验，返回值非零校验失败，返回值零校验成功。

### 安全启动流程图
![图 1：安全启动流程](../../assets/sf52x_seboot_launch_image.png)
![图 2：rsa公钥哈希校验](../../assets/sf52x_seboot_sigkey_hash.png)
![图 3：image解密](../../assets/sf52x_dec_image.png)
![图 4：image数字签名完整性校验](../../assets/sf52x_image_dig_sig_ver.png)

### 安全启动代码
代码路径： _$SDK_ROOT\example\boot_loader\project\butterflmicro\board_ \
secboot.c :image数字签名完整性验证，secboot异常处理。\
efuse.c :efuse读取，加密的imagekey和加密的image的解密。\
main.c :image拷贝加载。

rsa数字签名公钥哈希校验代码：
```c
int sifli_hash_calculate(uint8_t *in, uint32_t in_size, uint8_t *out, uint8_t algo)
{
    int last, i;

    if (!in || !in_size || !out || algo > 3)
        return -1;

    HAL_HASH_reset();
    HAL_HASH_init(NULL, algo, 0);

    if (in_size > SPLIT_THRESHOLD)
    {
        for (i = 0; i < in_size; i += SPLIT_THRESHOLD)
        {
            last = (i + SPLIT_THRESHOLD >= in_size) ? 1 : 0;
            if (i > 0)
            {
                HAL_HASH_reset();
                HAL_HASH_init((uint32_t *)out, algo, last ? i : 0);
            }
            HAL_HASH_run(&in[i], last ? in_size - i : SPLIT_THRESHOLD, last);
            HAL_HASH_result(out);
        }
        HAL_HASH_result(out);
    }
    else
    {
        HAL_HASH_run(in, in_size, 1);
        HAL_HASH_result(out);
    }

    return 0;
}

int sifli_hash_verify(uint8_t *data, uint32_t data_size, uint8_t *hash, uint32_t hash_size)
{
    uint8_t hash_out[32] = {0};

    if (!data || !hash)
        return -1;

    if (sifli_hash_calculate(data, data_size, hash_out, HASH_ALGO_SHA256))
        return -1;

    if (memcmp(hash_out, hash, hash_size))
        return -1;

    return 0;
}

int sifli_sigkey_pub_verify(uint8_t *sigkey, uint32_t key_size)
{
    uint32_t size = 0;

    uint8_t sigkey_hash[DFU_SIG_HASH_SIZE] = {0};
    size = sifli_hw_efuse_read(EFUSE_ID_SIG_HASH, sigkey_hash, DFU_SIG_HASH_SIZE);
    if (size == DFU_SIG_HASH_SIZE)
        return sifli_hash_verify(sigkey, key_size, sigkey_hash, DFU_SIG_HASH_SIZE);
    else
        return -1;
}
```

image key解密代码：
```c
int sifli_hw_dec_key(uint8_t *in_data, uint8_t *out_data, int size)
{
    uint8_t *uid;
    uint8_t *key = NULL;

    uid = &g_uid[0];
    sifli_hw_efuse_read(EFUSE_UID, uid, DFU_UID_SIZE);
    HAL_AES_init((uint32_t *)key, DFU_KEY_SIZE, (uint32_t *)uid, AES_MODE_CBC);
    HAL_AES_run(AES_DEC, in_data, out_data, DFU_KEY_SIZE);

    return 0;
}
```

image解密代码：
```c
int sifli_hw_dec(uint8_t *key, uint8_t *in_data, uint8_t *out_data, int size, uint32_t init_offset)
{
    uint32_t offset = 0;

    static uint8_t temp[AES_BLOCK_SIZE];
    memset(temp, 0, AES_BLOCK_SIZE);
    while (offset < size)
    {
        int len = (size - offset) < AES_BLOCK_SIZE ? (size - offset) : AES_BLOCK_SIZE;
        memcpy(temp, in_data + offset, len);
        HAL_AES_init((uint32_t *)key, DFU_KEY_SIZE, (uint32_t *)dfu_get_counter(init_offset + offset), AES_MODE_CTR);
        HAL_AES_run(AES_DEC, temp, out_data + offset, len);
        offset += len;
    }
    return offset;
}
```


image哈希数字签名完整性校验代码：
```c
int sifli_img_sig_hash_verify(uint8_t *img_hash_sig, uint8_t *sig_pub_key, uint8_t *image, uint32_t img_size)
{
    uint8_t img_hash[32] = {0};
    mbedtls_pk_context pk;

    /*1.calculate image hash*/
    if (sifli_hash_calculate(image, img_size, img_hash, HASH_ALGO_SHA256))
        return -1;
    
    /*2.verify image hash digital signature*/
    mbedtls_pk_init(&pk);
    if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE))
        return -1;
    mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
    if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, img_hash, DFU_IMG_HASH_SIZE, img_hash_sig, DFU_SIG_SIZE))
        return -1;

    return 0;
}
```

安全启动异常处理代码：
```c
void sifli_secboot_exception(uint8_t excpt)
{
    char *err = NULL;

    switch (excpt)
    {
    case SECBOOT_SIGKEY_PUB_ERR:
        err = "secboot sigkey pub err!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    case SECBOOT_IMG_HASH_SIG_ERR:
        err = "secboot img hash sig err!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    default:
        err = "secboot excpt null!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    }

    HAL_sw_breakpoint();
}
```

### 安全加密image生成
产生加密image的脚本路径：_$SDK_ROOT\\example\\security\\hcpu\\project\\eh-lb525\\secboot_

加密脚本默认使用目录secboot/sifli01下的密钥。
产生加密image使用的密钥和数据有：
- rootkey：32字节的密钥，由客户提供，用于加密image key。
- uid: 芯片id，芯片出厂时烧写到efuse BANK0中，在运行加密脚本之前需要先从efuse中读取uid，并保存成uid.bin文件，
  作为nonce用于image key的加密。
- image key：随机产生的32字节数据，加密前用于image的加密，加密后保存到ftab中。
- RSA数字签名私钥：由客户提供，用于对image 哈希值进行数字签名。
- RSA数字签名公钥：由客户提供，保存到ftab中。
- RSA数字签名公钥哈希值：对RSA数字签名公钥做哈希计算所得，前8字节最为nonce用于image的加密。

产生加密image需要用到的文件：
- _build/bf0_ap.bin_ 编译生成的明文未加密的image。
- _build/ftab/ftab.bin_ 编译生成的ftab。

工程目录下运行加密脚本：
```bat
./secboot/gen_sec_img.bat
```
脚本执行成功后会在secboot目录下创建目录image_sec目录，image_sec目录下保存着加密后的image（image_sec.bin）和新生成的ftab（ftab_sec.bin）。

工程目录下执行下载脚本：
```bat
./secboot/uart_download.bat
```
开发板按reset键，执行脚本执行`uart_download.bat`，执行成功后会将 _bootloader.bin_， _image_sec.bin_， _ftab_sec.bin_ 下载到flash中。


## 安全隔离
### sf52x安全隔离方案介绍
sf52x安全方案是基于secu管理模块设计，sf52x包含两个secu模块，secu1， secu2分别位于大核系统和小核系统，通过配置secu模块可以管理各个
master和slave外设的安全属性，这里的master是指具有主动发出访问请求的设备，如cpu、dma等，slave是接收访问请求的设备，如ptc、dma、efuse等。

sf52x安全方案将小核作为安全核，一直运行于安全状态，大核运行于安全和非安全两种状态，将NMI中断作为大核安全的入口，当需要运行安全任务的时候，
由大核向小核发送请求（进入安全状态），小核经过检查校验后，设置大核为安全状态，同时触发大核的NMI中断，大核进入NMI中断函数，访问安全代码区域，
安全任务运行完成后大核将自己设置为非安全状态，退出安全代码区域。当大核为非安全状态时是无法访问安全区域的代码的，也无法访问小核系统中的安全代码区域。

sf52x安全隔离流程可参考下图：
![图 5：sf52x安全隔离](../../assets/sf52x_security.png)

### sf52x安全方案软件介绍
sf52x安全方案代码是基于sdk2.1.4开发，代码路径在 _$SDK_ROOT\middleware\\security\\sf52x_ 目录下，分为hcpu和lcpu两部分，hcpu主要实现了大小核安全环境的配置，
大核安全代码的管理及安全代码的触发接口；lcpu主要负责大核是否进入安全状态的审核以及NMI中断的触发。

hcpu目录下 _security.c_ 中实现了大小核安全环境的建立，包含了配置大核安全代码区域，配置小核系统进入安全模式，大核退出安全模式的接口，
大小核安全共享数据的写入等。大核安全代码区域包含了两个部分，第一部分就是安全代码区，存放代码及数据；另外一部分是大小核安全数据的共享区，
这部分小核随时可以访问，大核只有处于安全状态下才可以访问。大核安全代码区域的大小可以配置，目前默认配置了100kb的ram空间，位于sram2上，
其中安全数据的共享区固定占用128字节，不需要修改大小。

_sec_entry.c_ 中实现了安全代码的中断入口，安全代码全部在NMI中断中运行，安全代码的接口也是添加到这个文件中，所有的安全代码包含此文件中的代码都会被链接到安全代码区域。
_sec_task.c_ 实现了安全任务的触发接口`sec_task_enter()`，hcpu在非安全模式下想要切换到安全模式运行安全代码需要调用此接口，此接口在多线程中调用是互斥的，
先获取到互斥量的线程优先运行安全区代码。此接口是阻塞式的调用，只有当安全代码运行完成后才会退出。

Lcpu目录下 _security.c_ 中实现了hcpu进入安全模式的仲裁，当收到hcpu进入安全模式的请求时，首先会检查hcpu vector寄存器和中断向量表中NMI中断函数入口的值是否被篡改，
如果被篡改过则会拒绝hcpu进入安全模式的请求，如果没有被篡改则会设置hcpu为安全模式，并触发hcpu的NMI中断。

encrypt_demo目录下是安全代码样例，简单实现了几个安全接口，比如随机数的产生，aes加密，哈希计算。

### sf52x安全隔离软件接口使用
#### 1.如何添加安全代码到安全保护区域
安全代码的添加可以参考 _$SDK_ROOT\\security\\sf52x\\hcpu\\sec_entry.c_ 中的55到57行，第一步根据安全代码接口的功能为每个接口定义SEC_ENTRY_IDX_XX，id号是枚举类型，
在sec_entry.h中定义。
![](../../assets/sf52x_sec_entry.png)

第二步编写安全入口函数，如int sec_entry_xxx(void *arg)，此函数里解析传递过来的参数，调用对应的安全代码接口。
![](../../assets/sf52x_sec_entry1.png)

第三步，修改链接文件，将添加的安全代码链接到安全保护区域。连接文件在目录_$SDK_ROOT\\example\\security\\hcpu\\project\\eh-lb525\\linker_scripts_ 中，打开 _link_flash.sct_ ，
找到NMI_SEC_AREA，将安全代码添加到段中。
![](../../assets/sf52x_sec_entry2.png)

#### 2.hcpu如何进入安全状态运行安全代码
Hcpu进入安全状态运行安全代码的接口在 _sec_task.c_ 中`int sec_task_enter(sec_task_list_t * sec_tls)`。接口的使用可以参考 _security_demo.c_ 中的`sec_demo1`，
首先要准备好安全接口运行所需要的参数，将安全接口的ID和对应的参数填写到`sec_task_list`中，注意要填写总共需要运行的接口数量`task_cnt`，`sec_task_list`中最多
可以填写32个接口，自己可扩展，最后调用`sec_task_enter()`，`sec_task_list`中的安全接口会按照填入的顺序依次执行。

注意安全代码的执行是在NMI中断中进行的，NMI中断的优先级比较高，不会被其他的中断打断，所以NMI中断的执行时间不宜过长，中断中最好不要加打印，不要调用睡眠，
长时间延迟的函数。

触发进入安全模式运行安全代码的例子：
```c
static struct trng_gen_arg trng;
static struct hash_encrypt_arg hash;
static struct aes_encrypt_arg aes;
static sec_task_list_t sec_tls;
/*exe sec task, contain SEC_ENTRY_ID1_T/SEC_ENTRY_ID2_T/SEC_ENTRY_ID3_T*/
static void sec_demo1(uint8_t argc, char **argv)
{
    /*1.generate 1kb random num*/
    rt_memset(&trng, 0, sizeof(struct trng_gen_arg));
    trng.random = rt_calloc(1, 1024);
    trng.random_num = 1024 / 4;
    /*prepare sec task list*/
    sec_tls.task[0].id = SEC_ENTRY_ID1_RTNG_GEN;
    sec_tls.task[0].arg = &trng;
    sec_tls.task_cnt = 1;
    /*exe sec task, it will block in this function until sec task exe done*/
    sec_task_enter(&sec_tls);
    if (trng.complete)
    {
        LOG_I("trng generate complete");
        for (int i = 0; i < 256; i++)
        {
            LOG_I("0x%x ", trng.random[i]);
        }
    }
    /*2.aes encrypt 1kb random number*/
    rt_memset(&aes, 0, sizeof(struct aes_encrypt_arg));
    aes.aes_mode = AES_MODE_CTR;
    aes.input = (uint8_t *)trng.random;
    aes.size = 1024;
    aes.output = rt_calloc(1, aes.size);

    sec_tls.task[0].id = SEC_ENTRY_ID3_AES_ENC;
    sec_tls.task[0].arg = &aes;
    sec_tls.task_cnt = 1;
    sec_task_enter(&sec_tls);
    if (aes.complete)
    {
        LOG_I("aes encrypt complete");
        for (int i = 0; i < 256; i++)
        {
            LOG_I("0x%x ", ((uint32_t *)aes.output)[i]);
        }
    }

    /*3.hash 1kb aes encrypt data*/
    rt_memset(&hash, 0, sizeof(struct hash_encrypt_arg));
    hash.input = (uint8_t *)aes.output;
    hash.in_size = 1024;
    hash.output = rt_calloc(1, HASH_SHA256_SIZE);
    hash.algo = HASH_ALGO_SHA256;

    sec_tls.task[0].id = SEC_ENTRY_ID2_HASH_ENC;
    sec_tls.task[0].arg = &hash;
    sec_tls.task_cnt = 1;
    sec_task_enter(&sec_tls);
    if (hash.complete)
    {
        LOG_I("hash encrypt complete");
        for (int i = 0; i < 8; i++)
        {
            LOG_I("0x%x ", ((uint32_t *)hash.output)[i]);
        }
    }

    /*4.write data to nand flash, Write 2kb at most at one time*/
    rt_nand_erase(FLASH_SEC_DATA_START_ADDR, 1024 * 128);
    rt_nand_write_page(FLASH_SEC_DATA_START_ADDR, (uint8_t *)trng.random, 1024, NULL, 0);
    rt_nand_write_page(FLASH_SEC_DATA_START_ADDR + SPI_NAND_PAGE_SIZE, aes.output, 1024, NULL, 0);
    rt_nand_write_page(FLASH_SEC_DATA_START_ADDR + SPI_NAND_PAGE_SIZE * 2, hash.output, 32, NULL, 0);

    /*5.read data from nand flash and check*/
    uint8_t *random_data = rt_calloc(1, 1024);
    rt_nand_read_page(FLASH_SEC_DATA_START_ADDR, random_data, 1024, NULL, 0);
    if (rt_memcmp((void *)trng.random, (void *)random_data, 1024))
    {
        LOG_E("read random_data compare err");
    }
    else
    {
        LOG_E("read random_data compare succ");
    }

    uint8_t *aes_data = rt_calloc(1, 1024);
    rt_nand_read_page(FLASH_SEC_DATA_START_ADDR + SPI_NAND_PAGE_SIZE, aes_data, 1024, NULL, 0);
    if (rt_memcmp((void *)aes.output, (void *)aes_data, 1024))
    {
        LOG_E("read aes_data compare err");
    }
    else
    {
        LOG_E("read aes_data compare succ");
    }

    uint8_t *hash_data = rt_calloc(1, 32);
    rt_nand_read_page(FLASH_SEC_DATA_START_ADDR + SPI_NAND_PAGE_SIZE * 2, hash_data, 32, NULL, 0);
    if (rt_memcmp((void *)hash.output, (void *)hash_data, 32))
    {
        LOG_E("read hash_data compare err");
    }
    else
    {
        LOG_E("read hash_data compare succ");
    }

    /*6.free buffer*/
    rt_free((void *)trng.random);
    rt_free((void *)aes.output);
    rt_free((void *)hash.output);
    rt_free(random_data);
    rt_free(aes_data);
    rt_free(hash_data);
}
```
