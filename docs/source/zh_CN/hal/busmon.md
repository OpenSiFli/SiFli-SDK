# BUSMON

BUSMON(Bus Monitor )模块用于统计系统AHB总线上各个master和slave的传输行为。BUSMON共有8个通道，可以同时选出8组master或slave作为观察对象，统计每个观察对象分别在各自设定的地址空间内进行的读/写操作次数，并在统计值达到特定次数时产生触发信号输出至PTC模块。
BUSMON模块可以用来统计外接flash的分块访问频次，找到cache经常miss的区域进行优化；也可以查找RAM的特定地址是被谁改写；也可以用来观测memory的带宽占用情况。BUSMON与PTC结合，可以在访问特定地址空间时产生中断，或者在某个观察对象进行特定次数的总线操作时触发其他外设工作，构成总线-外设硬件任务链。
BUSMON不增加总线关键路径长度。

BUSMON主要特性：<br>	
- 8个独立配置的通道可同时工作 <br>
- 支持总线上所有master和slave <br>
- 任意配置的总线地址空间 <br>
- 可统计读/写/读写总线操作<br>
- 31位计数器，24位比较器<br>
- 计数溢出可自动复位重新开始，溢出可查询<br>
- 8个通道独立的PTC触发源<br>

## 使用Bus Monitor

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

## API参考
[](/api/hal/busmon.md)