# EFUSE

`EFUSE HAL` 驱动程序提供高级 API 来编程和读取 efuse。
主要功能包括：
- 4个bank，每个bank有256bits。 
- bank 需要一个一个编程，多个bank 可以同时读取，但HAL 只支持每次读取一个bank。

## 使用 EFUSE HAL 驱动程序 
首先，只调用一次`HAL_EFUSE_Init`来初始化 efuse。 应在设置 sysclk/hclk/pclk 后调用它。 如果更新了 sysclk/hclk/pclk，则需要再次调用 `HAL_EFUSE_Init`来更新相关的时间寄存器。

使用`HAL_EFUSE_Write`使用指定数据对 efuse 进行编程，使用`HAL_EFUSE_Read`从 efuse 读取数据。 例如，

```c
{
    /* Initialize efuse*/
    HAL_EFUSE_Init();

    uint8_t write_data[4];
    write_data[0] = 1;
    write_data[1] = 2;
    write_data[2] = 3;
    write_data[3] = 4;
    /* write 4 bytes starting from bit32(bank0) */
    HAL_EFUSE_Write(32, write_data, sizeof(write_data));
    
    uint8_t read_data[4];
    memset(read_data, 0, sizeof(read_data));
    /* read 4bytes starting from bit32(bank0) */
    HAL_EFUSE_Read(32, read_data, sizeof(read_data));
}
```

## API参考
[](../api/hal/efuse.md)
