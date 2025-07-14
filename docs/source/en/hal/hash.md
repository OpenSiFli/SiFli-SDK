# Hash

The HASH engine is a computational accelerator for hash sequence algorithms. Users can select different hash algorithms to calculate hash values for specific data in memory. The HASH engine is faster than software algorithms and more flexible in configuration. Users can also implement multi-threaded HASH computation through custom initial vectors. The HASH engine supports the following algorithms:

- SHA1
- SHA224
- SHA256 
- SM3
 
## Using Hash HAL Driver
Hash supports both polling and interrupt modes, which can call different HASH interfaces.

Example of Hash HAL polling with block processing:
```c

// Output result stored in g_temp, length is 20 bytes for SHA1, 32 bytes for other algorithms.
uint8_t g_temp[32];

// Each block must be a multiple of 64, except the last block.
// Input array is g_hash_input, length is len, using SHA1 algorithm
#define SPLIT_THRESHOLD 64
    for (i=0;i<len;i+=SPLIT_THRESHOLD) {
        if (i==0)
            // First block
            HAL_HASH_init(NULL, HASH_ALGO_SHA1, 0);
        else
            // Continue previous calculation, load previous run result
            HAL_HASH_init((uint32_t*)g_temp, HASH_ALGO_SHA1, i);  
            
        if (i+SPLIT_THRESHOLD<len) {            
            LOG_I("AES run %d\n", i);
            status = HAL_HASH_run(&(g_hash_input[i]), SPLIT_THRESHOLD, 0);
        }
        else {
            // Last block
            LOG_I("AES run continue %d\n", i);
            status = HAL_HASH_run(&(g_hash_input[i]), len-i, 1);
        }
        if (HAL_OK != status) // Decryption sync
        {
            LOG_E("AES run (sync mode) error\n");
            uassert_true(status == HAL_OK);
        }
        HAL_HASH_result(g_temp);
    }
    // Result stored in g_temp
```

Example of Hash HAL using interrupt, completed in one pass:
```c

// Input array is g_hash_input, length is len, using SHA256 algorithm 

// Output result stored in g_temp, length is 20 bytes for SHA1, 32 bytes for other algorithms.
uint8_t g_temp[32];

// AES and HASH share one interrupt
void AES_IRQHandler(void)
{
    rt_interrupt_enter();
    
    // Check Hash completion interrupt source
    if (hwp_aes_acc->IRQ & HASH_DONE_STAT)
    {
        HAL_HASH_IRQHandler();
        // Hash completion signal
        rt_sem_release(g_aes_sem);
    }
    rt_interrupt_leave();
}

    ....
    // HASH module initialization
    HAL_HASH_init(NULL, HASH_ALGO_SHA256, 0);

    // Enable interrupt
    NVIC_EnableIRQ(AES_IRQn);                                               
    HAL_NVIC_SetPriority(AES_IRQn, 5, 0);

    status = HAL_HASH_run_IT(g_hash_input, len, 1);
    if (HAL_OK != status) // Encryption Asyn
    {
        LOG_E("AES run (async mode) error\n");
        uassert_true(status == HAL_OK);
    }
    // Wait for Hash completion signal
    rt_sem_take(g_aes_sem, RT_TICK_PER_SECOND);

    // Output Hash to g_temp, length depends on different Hash algorithms.
    HAL_HASH_result(g_temp);    
    
    ...
```
## API Reference
[](#hal-aes)
