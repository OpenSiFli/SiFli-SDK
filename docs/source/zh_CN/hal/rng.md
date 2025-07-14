# RNG

SiFli 芯片组内部的 RNG（随机数生成器）模块用于生成随机数种子和随机数。 建议用户使用自己的随机算法生成随机数以获得更好的随机性，同时使用硬件生成的随机数作为种子。


## 使用RNG

```c
{
    #include "bf0_hal.h"
    
    ...    
    RNG_HandleTypeDef   RngHandle;
    
    if (HAL_RNG_Init(&RngHandle) == HAL_OK) {                       // Initialize RNG module
        uint32_t value = 0;
        if (HAL_RNG_Generate(&RngHandle, &value,  1)== HAL_OK)      // Generate random seed
            printf("Generated Randome seed %d\n", value);
        if (HAL_RNG_Generate(&RngHandle, &value,  0)== HAL_OK)      // Generate random number, it is recommed to use this number as random seed to user random algorithm
            printf("Generated Randome number %d\n", value);            
    }
    ...
}    
```

## API参考
[](../api/hal/rng.md)

