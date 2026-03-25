# CACHE

HAL CACHE module provides statistical functionality for `HCPU CACHE MISS` rate,
can individually or simultaneously count ICACHE and DCACHE miss rates. Can also
set address ranges for statistics, supports individual or simultaneous
statistics for the following address regions:
- QSPI1_4, address space: 0x10000000~0x13FFFFFF
- QSPI2, address space: 0x64000000~0x67FFFFFF
- QSPI3, address space: 0x68000000~0x6FFFFFFF
- OPSRAM, address space: 0x60000000~0x63FFFFFF

For detailed API documentation, refer to [CACHE](#hal-cache)


## Using HAL CACHE

```c
void enable(void)
{
    /* * Enable ICACHE and DCACHE miss rate profiling for the QSPI1/2/4 memory space * */
    HAL_CACHE_Enable(HAL_CACHE_ICACHE_QSPI1_4 | HAL_CACHE_ICACHE_QSPI2, 
                     HAL_CACHE_DCACHE_QSPI1_4 | HAL_CACHE_DCACHE_QSPI2);

}
void read(void)
{
    float irate;
    float drate;
    /* * Read the cache miss rate and reset the counters * */
    HAL_CACHE_GetMissRate(&amp;irate, &amp;drate, true);
}

void disable(void)
{
    /* * Disable ICACHE and DCACHE miss rate profiling * */
    HAL_CACHE_Disable();
}
```

## API Reference
[]
