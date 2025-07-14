# Hash

HASH 引擎是一个针对散列数列算法的运算加速器，用户可以选择不同的散列算法计算内存中特定数据的散列
值。HASH 引擎相比软件算法速度更快，配置也灵活，用户也可以通过自定义初始向量，实现多线程HASH 运
算。HASH 引擎支持算法包括

- SHA1
- SHA224
- SHA256 
- SM3
 
## 使用 Hash HAL 驱动程序
Hash可以支持使用轮询和中断模式，它们可以调用不同的HASH接口。

Hash HAL 轮询，分块处理使用的示例：
```c

//输出结果存储在g_temp中，长度SHA1是20字节， 其他算法32字节.
uint8_t g_temp[32];

// 每一块必须是64的整数倍，除了最后一块。
// 输入数组为g_hash_input，长度为len，使用SHA1算法
#define SPLIT_THRESHOLD 64
    for (i=0;i<len;i+=SPLIT_THRESHOLD) {
        if (i==0)
            //第一块
            HAL_HASH_init(NULL, HASH_ALGO_SHA1, 0);
        else
            //继续上次的计算，加载上次的运行结果
            HAL_HASH_init((uint32_t*)g_temp, HASH_ALGO_SHA1, i);  
            
        if (i+SPLIT_THRESHOLD<len) {            
            LOG_I("AES run %d\n", i);
            status = HAL_HASH_run(&(g_hash_input[i]), SPLIT_THRESHOLD, 0);
        }
        else {
            // 最后一块
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
    // 结果存储在 g_temp 中
```

Hash HAL 使用中断，一次完成的示例：
```c

//输入数组为g_hash_input，长度为len，使用SHA256算法 

//输出结果存储在g_temp中，长度SHA1是20字节， 其他算法32字节.
uint8_t g_temp[32];

// AES和HASH共享一个中断
void AES_IRQHandler(void)
{
    rt_interrupt_enter();
    
    // 判断Hash 完成的中断源
    if (hwp_aes_acc->IRQ & HASH_DONE_STAT)
    {
        HAL_HASH_IRQHandler();
        // Hash 完成信号
        rt_sem_release(g_aes_sem);
    }
    rt_interrupt_leave();
}

    ....
    // HASH模块初始化
    HAL_HASH_init(NULL, HASH_ALGO_SHA256, 0);

    // 使能中断
    NVIC_EnableIRQ(AES_IRQn);                                               
    HAL_NVIC_SetPriority(AES_IRQn, 5, 0);

    status = HAL_HASH_run_IT(g_hash_input, len, 1);
    if (HAL_OK != status) // Encryption Asyn
    {
        LOG_E("AES run (async mode) error\n");
        uassert_true(status == HAL_OK);
    }
    // 等待Hash完成的信号
    rt_sem_take(g_aes_sem, RT_TICK_PER_SECOND);

    // 输出Hash到g_temp, 长度根据不同Hash算法。
    HAL_HASH_result(g_temp);    
    
    ...
```
## API参考
[](#hal-aes)

