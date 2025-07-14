# AES

SIFLI 具有 AES 加密/解密的硬件实现。 它在 AES 函数上提供高性能，并用于引导加载程序和 DFU 来解密收到的图像。
它支持：<br>
- 加密和解密。 <br>
- AES 和 SM4 算法。 <br>
- 128,192 或 256 AES 长度。 <br>
- ECB、CTR 和 CBC AES 密码模式。 <br>

API 详情请参考 [AES](#hal-aes) 。

```{note}
AES 使用的输入输出内存不能是ITCM RAM或者Retention RAM, 地址请参考 [](/app_note/memory_usage.md)
```

## 使用 AES 加密/解密
请参考以下代码为例：

```c

ALIGN(4)                        // Make sure g_key and g_nounce_counter 4bytes aligned
static uint8_t g_key[32] =		// Key is defined by user, if use 256 AES length, it need to be 32 bytes
{
    0x3D, 0xA5, 0xA4, 0x98, 0x6E, 0x90, 0xA7, 0x90,
    0x1D, 0x97, 0x69, 0xAA, 0xF0, 0xDF, 0x32, 0xE4,
    0x55, 0xE9, 0xFC, 0xD6, 0x75, 0x60, 0xBD, 0x33,
    0x5E, 0x20, 0xD0, 0x78, 0x47, 0xF7, 0x8C, 0x4D,
};
static uint8_t g_nounce_counter[16]=
{
    0xf0, 0xd7, 0x77, 0x7f, 0x61, 0x6f, 0x7c, 0x89,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

uint8_t input_data[BUFFER_SIZE];
uint8_t output_data[BUFFER_SIZE];

// Get input_data , this is not described in this code pieces.

// Encryption
HAL_AES_init(g_key, 32, g_nounce_counter, AES_MODE_CTR);
HAL_AES_run_IT(true, input_data, output_data, BUFFER_SIZE);     // Run Async, interrupt will generate interrupt when done

rt_thread_delay(1000);		

// Decryption
HAL_AES_init(g_key, 32, g_nounce_counter, AES_MODE_CTR);        // Run Sync
HAL_AES_run(false, input_data, output_data, BUFFER_SIZE);       // Function will block until Decryption finish.

....

void AES_IRQHandler(void)
{
   printf("AES function finished");
   HAL_AES_IRQHandler();
}

```
## API参考
[](#hal-aes)


