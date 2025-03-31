# CACHE

HAL CACHE模块提供`HCPU CACHE MISS`率的统计功能，可单独或同时统计ICACHE和DCACHE的MISS率。
还可以设置需要统计的地址范围，支持单独或同时统计以下地址区域：
- QSPI1_4，地址空间: 0x10000000~0x13FFFFFF
- QSPI2， 地址空间：0x64000000~0x67FFFFFF
- QSPI3， 地址空间: 0x68000000~0x6FFFFFFF
- OPSRAM， 地址空间: 0x60000000~0x63FFFFFF


详细的API说明参考 [CACHE](#hal-cache)


## 使用HAL CACHE

```c
void enable(void)
{
    /* Enable ICACHE and DACHE miss rate profiling, range is QSPI1/2/4 memory space */
    HAL_CACHE_Enable(HAL_CACHE_ICACHE_QSPI1_4 | HAL_CACHE_ICACHE_QSPI2, 
                     HAL_CACHE_DCACHE_QSPI1_4 | HAL_CACHE_DCACHE_QSPI2);

}
void read(void)
{
    float irate;
    float drate;
    /* read cache miss rate and reset counter */
    HAL_CACHE_GetMissRate(&irate, &drate, true);
}

void disable(void)
{
    /* Disable ICACHE and DCACHE miss rate profiling */
    HAL_CACHE_Disable();
}
```

## API参考
[](#hal-cache)