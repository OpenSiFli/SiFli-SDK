# CRC

The CRC module can be used for CRC calculation with specific bit width,
arbitrary generator polynomial, and arbitrary initial value. The minimum data
input unit is a single byte, with no maximum byte limit. Single cycle can
complete single-byte input calculation. CRC result can be obtained quickly after
all data input is complete. Supports input data reversal and output data
reversal. Supports input data of different valid bit widths.
- 7/8/16/32-bit CRC calculation, supports all mainstream formats for these bit
  widths
- Arbitrary generator polynomial
- Arbitrary initial value
- Input data supports single-byte/double-byte/three-byte/four-byte valid bit
  width
- Input data bit reversal by byte/double-byte/four-byte high/low bit
- Output data high/low bit reversal

## Using CRC
The following is a CRC code snippet:

```c
{
    CRC_HandleTypeDef   CrcHandle;                      // Declare CRC handle
    CrcHandle.Instance = CRC;                           // Assign CRC instance
    uint8_t g_test_data[]= {                            // Input data buffer
        1,2,3,4,5,6,7,8,9,10
    }

    HAL_CRC_Init(&amp;CrcHandle);                           // Initialize the CRC peripheral
    HAL_CRC_Setmode(&amp;CrcHandle, CRC_8_ITU);             // Configure for CRC-8/ITU standard
    uint32_t crc=HAL_CRC_Accumulate(&amp;CrcHandle,         // Compute CRC for g_test_data
        &amp;(g_test_data[offset]), sizeof(g_test_data));

}
```

## Using Fully Customized Initial Value and Polynomial

```c
{
    CRC_HandleTypeDef   CrcHandle;                        // Declare CRC handle
    CrcHandle.Instance = CRC;                             // Assign CRC instance
    uint8_t g_test_data[]= {                              // Input data buffer
        1,2,3,4,5,6,7,8,9,10
    }
    uint32_t init = 0xFF;                                 // Initial value
    uint32_t poly = 0x1D;                                 // Polynomial

    HAL_CRC_Init(&amp;CrcHandle);                             // Initialize the CRC peripheral
    HAL_CRC_Setmode_Customized(hcrc, init, poly, CRC_8);  // Configure for custom CRC-8 parameters
    uint32_t crc=HAL_CRC_Accumulate(&amp;CrcHandle,           // Compute CRC for g_test_data
        &amp;(g_test_data[offset]), sizeof(g_test_data));

}
```


## API Reference
[]
