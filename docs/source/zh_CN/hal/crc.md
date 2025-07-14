# CRC

CRC模块可用于进行特定位宽，任意生成多项式，任意初始值的CRC计算。数据最小输入单元为单字节，没有最长字节数限制。单周期即能够完成单字节输入的计算。数据输入全部完成后即可快速得到校验结果。支持输入数据倒转和输出数据倒转。支持不同有效位宽的输入数据。
- 7/8/16/32比特CRC计算，支持这些位宽所有主流格式
- 任意生成多项式
- 任意初始值
- 输入数据支持单字节/双字节/三字节/四字节有效位宽
- 输入数据按字节/双字节/四字节高低位比特倒转
- 输出数据高低位比特倒转

## 使用 CRC
以下是CRC的代码片段：

```c
{
    CRC_HandleTypeDef   CrcHandle;                      // CRC handle declaration
    CrcHandle.Instance = CRC;                           // Initialize CRC handle
    uint8_t g_test_data[]= {                            // Raw data
        1,2,3,4,5,6,7,8,9,10
    }

    HAL_CRC_Init(&CrcHandle);                           // Initialize CRC module
    HAL_CRC_Setmode(&CrcHandle, CRC_8_ITU);             // Set CRC mode to CRC-8/ITU standard
    uint32_t crc=HAL_CRC_Accumulate(&CrcHandle,         // Calculate CRC result for g_test_data
        &(g_test_data[offset]), sizeof(g_test_data));
    
}    
```

## 使用完全自定义的初始值和多项式

```c
{
    CRC_HandleTypeDef   CrcHandle;                        // CRC handle declaration
    CrcHandle.Instance = CRC;                             // Initialize CRC handle
    uint8_t g_test_data[]= {                              // Raw data
        1,2,3,4,5,6,7,8,9,10
    }
    uint32_t init = 0xFF;                                 // Initial value
    uint32_t poly = 0x1D;                                 // CRC polynomial

    HAL_CRC_Init(&CrcHandle);                             // Initialize CRC module
    HAL_CRC_Setmode_Customized(hcrc, init, poly, CRC_8);  // Set CRC mode to CRC-8 standard
    uint32_t crc=HAL_CRC_Accumulate(&CrcHandle,           // Calculate CRC result for g_test_data
        &(g_test_data[offset]), sizeof(g_test_data));

}
```


## API参考
[](../api/hal/crc.md)