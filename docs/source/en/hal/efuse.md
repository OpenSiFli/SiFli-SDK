# EFUSE

`EFUSE HAL` driver provides high-level APIs to program and read efuse.
Main features include:
- 4 banks, each bank has 256 bits. 
- Banks need to be programmed one by one, multiple banks can be read simultaneously, but HAL only supports reading one bank at a time.

## Using EFUSE HAL Driver
First, call `HAL_EFUSE_Init` only once to initialize efuse. It should be called after setting sysclk/hclk/pclk. If sysclk/hclk/pclk are updated, `HAL_EFUSE_Init` needs to be called again to update related time registers.

Use `HAL_EFUSE_Write` to program efuse with specified data, use `HAL_EFUSE_Read` to read data from efuse. For example,

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

## API Reference
[](../api/hal/efuse.md)
