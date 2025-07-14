# CRC

The CRC module can be used for CRC calculation with specific bit width, arbitrary generator polynomial, and arbitrary initial value. The minimum data input unit is a single byte, with no maximum byte limit. Single cycle can complete single-byte input calculation. CRC result can be obtained quickly after all data input is complete. Supports input data reversal and output data reversal. Supports input data of different valid bit widths.
- 7/8/16/32-bit CRC calculation, supports all mainstream formats for these bit widths
- Arbitrary generator polynomial
- Arbitrary initial value
- Input data supports single-byte/double-byte/three-byte/four-byte valid bit width
- Input data bit reversal by byte/double-byte/four-byte high/low bit
- Output data high/low bit reversal

## Using CRC
The following is a CRC code snippet:

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

## Using Fully Customized Initial Value and Polynomial

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

## API Reference
[](../api/hal/crc.md)