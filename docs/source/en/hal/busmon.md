# BUSMON

BUSMON (Bus Monitor) module is used to monitor transmission behavior of various masters and slaves on the system AHB bus. BUSMON has 8 channels that can simultaneously select 8 groups of masters or slaves as observation targets, count the number of read/write operations performed by each observation target within their respective configured address spaces, and generate trigger signals to PTC module when the count reaches specific values.
The BUSMON module can be used to count block access frequency of external flash to find areas where cache frequently misses for optimization; it can also find who is modifying specific RAM addresses; it can also be used to observe memory bandwidth usage. Combined with PTC, BUSMON can generate interrupts when accessing specific address spaces, or trigger other peripherals when an observation target performs a specific number of bus operations, forming bus-peripheral hardware task chains.
BUSMON does not increase bus critical path length.

BUSMON main features:<br>	
- 8 independently configurable channels can work simultaneously <br>
- Supports all masters and slaves on the bus <br>
- Arbitrarily configurable bus address space <br>
- Can count read/write/read-write bus operations<br>
- 31-bit counter, 24-bit comparator<br>
- Count overflow can automatically reset and restart, overflow can be queried<br>
- 8 independent PTC trigger sources for channels<br>

## Using Bus Monitor

```c
    BUSMON_HandleTypeDef   BusmonHandle;
    volatile uint32_t temp;
    volatile uint32_t * p;
    
    // Initialize Bus Monitor
	{ 	
        BusmonHandle.Init.Channel = 1;                  // Channel 1
        BusmonHandle.Init.Flags = BUSMON_OPFLAG_READ;   // Monitor read activity only
        BusmonHandle.Init.SelFunc = HAL_BUSMON_HCPU_S;  // Monitor HCPU BUS SLAVE
        BusmonHandle.Init.Max = 0x60010000;             // Max address range
        BusmonHandle.Init.Min = 0x60000000;             // Min address range
        HAL_BUSMON_Init(&BusmonHandle);                 // Initialize the busmon
        HAL_BUSMON_Enable(&BusmonHandle, 1);            // Enable bus monitor
	}
    
    p=*(uint32_t*)0x60000000;                           // Read from PSRAM between 0x60000000-0x60010000 100 times                    
    for (int i=0;i<100;i++) {       
        temp=*p;
        p++;
    }
    HAL_BUSMON_GetCount(&BusmonHandle, (int32_t *)&temp); // temp is 100.   
    printf("Count=%d\n", temp);
```

## API Reference
[](../api/hal/busmon.md)
