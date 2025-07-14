# 核间通信队列

核间通信队列（IPC Queue library）实现了双向核间通信功能, 它结合了[](/hal/mailbox.md) 的通知机制与软件循环buffer，使得核间通信类似于两颗芯片间的串口通信。
函数库最多支持4条queue同时工作，每条queue与一对收发buffer绑定(收发buffer是可选的，如果配置为NULL，则只提供中断通知功能)，按用户配置映射到对应的硬件通道，
平台支持的硬件通道号可以查看对应的 _ipc_hw_port.h_ 。

以HCPU与LCPU的通信为例，由 _ipc_hw_port.h_ 可知，0~7为HCPU与LCPU间的硬件通道，选择通道0，调用 `ipc_queue_init()`配置好收发buffer的地址和接收回调函数，
如果成功则返回一个有效的queue handle，然后调用`ipc_queue_open()`打开通道，之后就可以正常收发数据了。

调用`ipc_queue_write()`可以发送数据给对端，发送的数据会被写入配置好的`tx_buffer`，同时接收端会收到通知中断（接收端已经打开了对应相应的IPC queue）。
如果对端发送了数据，`ipc_queue_init()`注册的接收回调会被触发，可以调用`ipc_queue_read()`从`rx_buffer`里读取数据。


需要注意的是，HCPU配置的`rx_buffer`地址应与LCPU配置的`tx_buffer_alias`地址相同，即指向`tx_buffer`，HCPU配置的`tx_buffer_alias`则与LCPU的`rx_buffer`地址相同，
`tx_buffer`必需是发送方所在子系统的RAM地址，即HCPU需要在自己的RAM空间里分配一块buffer作为发送buffer，这样才能避免对端子系统还未被唤醒的情况下也能写入数据，`tx_buffer`由发送方管理。
因为接收端可能无法直接访问发送端的地址空间，所以使用`tx_buffer_alias`指定接收端可访问的`tx_buffer`对应的地址空间，两个地址实际指向同一块物理buffer。
`tx_buffer`与`rx_buffer`的关系见图1

![图1: rx buffer和tx_buffer的关系](../../assets/ipc_queue_mem.png)

函数库的架构如图2所示，API接口定义在 _ipc_queue.h_ 中，_ipc_queue.c_ 为平台无关的实现，_ipc_hw.c_ 封装了ButterFli的[](/hal/mailbox.md) 接口，为 _ipc_queue.c_ 提供中断服务，
但因为不同系列的ButterFli芯片的硬件通道配置不同，每个芯片有各自的 _ipc_hw_port.c_ ，在文件中定义变量`ipc_hw_obj`描述硬件通道，并实现中断处理函数。
_ipc_queue.c_ 提供接口 `ipc_queue_data_ind`让中断处理函数通知上层新数据的到来。

![图2: IPC queue](../../assets/ipc_queue_arch.png)

对于RT-Thread平台，可以使用 [ipc_queue_device](#middleware-ipc_queue_device) 提供的API，将IPC Queue封装成RT Device。

```{note}
 #ipc_queue_read 和 #ipc_queue_write 不是线程安全的，对于同一个queue，如果在多线程环境中调用`ipc_queue_read`或者`ipc_queue_write`读写数据，调用者需要做好互斥，避免重入。
但如果是访问不同的queue，可以不做互斥。例如，thread_1使用`ipc_queue_write`往queue_0中写入数据，thread_2使用`ipc_queue_write`往`queue_1`中写入数据，
因为操作的是不同的queue，并不需要做互斥保护，但如果两个线程都是往queue_0里写数据，调用`ipc_queue_write`前就需要做互斥了。
```


## 配置
选中SiFli Middleware->Enable IPC Queue Library使能函数库

![图3: IPC Queue Library 配置菜单](../../assets/ipc_queue_menu.png)

## 使用示例
以下的测试代码展示使用硬件通道0作为HCPU与LCPU的双向通信通道，HCPU的`0x20000000`地址作为发送buffer，LCPU的`0x20100000`地址作为LCPU的发送buffer，
`HCPU_ADDR_2_LCPU_ADDR`用于把HCPU地址转换到LCPU的地址空间，`LCPU_ADDR_2_HCPU_ADDR`类似。

## HCPU代码片段：
```c

static int32_t hcpu_rx_ind(ipc_queue_handle_t ipc_queue, size_t size)
{
}

void hcpu_queue_test(void)
{
    ipc_queue_cfg_t q_cfg;
    ipc_queue_handle_t q_handle;
    uint32_t buf[32];
    
    q_cfg.qid = 0;  /* 使用硬件通道0 */
    q_cfg.tx_buf_size = 0x100;
    q_cfg.tx_buf_addr = 0x20000000;
    q_cfg.tx_buf_addr_alias = HCPU_ADDR_2_LCPU_ADDR(0x20000000);
    q_cfg.rx_buf_addr = LCPU_ADDR_2_HCPU_ADDR(0x20100000);
    q_cfg.rx_ind = hcpu_rx_ind;
    q_cfg.user_data = 0;    

    q_handle = ipc_queue_init(&q_cfg);
    
    ipc_queue_open(q_handle);
    /* send data to LCPU */
    ipc_queue_write(q_handle, buf, sizeof(buf), 10);
}
```

## LCPU代码片段：
```c
uint8_t lcpu_rx_data_size;
static int32_t lcpu_rx_ind(ipc_queue_handle_t ipc_queue, size_t size)
{
    lcpu_rx_data_size = size;
}

void lcpu_queue_test(void)
{
    ipc_queue_cfg_t q_cfg;
    ipc_queue_handle_t q_handle;    
    uint8_t *buffer;
    
    q_cfg.qid = 0;   /* 使用硬件通道0 */
    q_cfg.tx_buf_size = 0x100;
    q_cfg.tx_buf_addr = 0x20100000;
    q_cfg.tx_buf_addr_alias = LCPU_ADDR_2_HCPU_ADDR(0x20100000);
    q_cfg.rx_buf_addr = HCPU_ADDR_2_LCPU_ADDR(0x20000000);
    q_cfg.rx_ind = lcpu_rx_ind;
    q_cfg.user_data = 0;    

    q_handle = ipc_queue_init(&q_cfg);
    ipc_queue_open(q_handle);
    
    while (0 == lcopu_rx_data_size)
    {
        /* wait for data coming */
    }
    
    buffer = malloc(lcpu_rx_data_size);
    /* read data */
    ipc_queue_read(q_handle, buffer, lcpu_rx_data_size);
    
    /* handle data */
    ...
    
    free(buffer)
}


```



## API参考
[](middleware-ipc_queue)\
[](middleware-ipc_queue_device)
