# RNG

The RNG (Random Number Generator) module inside SiFli chipsets is used to generate random number seeds and random numbers. It is recommended that users use their own random algorithms to generate random numbers for better randomness, while using hardware-generated random numbers as seeds.

## Using RNG

```c
{
    #include "bf0_hal.h"
    
    ...    
    RNG_HandleTypeDef   RngHandle;
    
    if (HAL_RNG_Init(&RngHandle) == HAL_OK) {                       // Initialize RNG module
        uint32_t value = 0;
        if (HAL_RNG_Generate(&RngHandle, &value,  1)== HAL_OK)      // Generate random seed
            printf("Generated Random seed %d\n", value);
        if (HAL_RNG_Generate(&RngHandle, &value,  0)== HAL_OK)      // Generate random number, it is recommended to use this number as random seed to user random algorithm
            printf("Generated Random number %d\n", value);            
    }
    ...
}    
```

## API Reference
[](../api/hal/rng.md)
