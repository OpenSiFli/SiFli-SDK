# AES

SIFLI has hardware implementation of AES encryption/decryption. It provides high performance on AES functions and is used in bootloader and DFU to decrypt received images.
It supports:<br>
- Encryption and decryption. <br>
- AES and SM4 algorithms. <br>
- 128, 192 or 256 AES key length. <br>
- ECB, CTR and CBC AES cipher modes. <br>

For API details, please refer to [AES](#hal-aes).

```{note}
The input and output memory used by AES cannot be ITCM RAM or Retention RAM, please refer to [](/app_note/memory_usage.md) for addresses
```

## Using AES Encryption/Decryption
Please refer to the following code as an example:

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
## API Reference
[](#hal-aes)
