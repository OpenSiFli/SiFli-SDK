# Simple FFT Example

## Supported Platforms
The example can run on the following development boards.
- sf32lb56-lcd_a128r12n1

## Example Overview

This example demonstrates the following basic FFT operations:
- Real FFT (Real FFT) - Convert real number sequences to frequency domain
- Complex FFT (Complex FFT) - Convert complex number sequences to frequency domain  
- Inverse FFT (Inverse FFT) - Convert frequency domain data back to time domain

## Example Usage

### Hardware Connection
This example does not require special hardware connections, just connect the development board to the computer.

### Expected Results

Under expected conditions, you will see the following output:

```log
=========================================
        FFT HAL Module Example          
=========================================
This example demonstrates:
- Real FFT operation
- Complex FFT operation
- Inverse FFT operation
=========================================

FFT Example: Using FFT1 (HCPU)
FFT initialization successful
=== Real FFT Test ===
Real FFT completed successfully
[0] = 0x0B73
[1] = 0x0000
[2] = 0x0349
[3] = 0x0641
[4] = 0xFE9D
[5] = 0xFBF7
[6] = 0xF7E2
[7] = 0x067A
[8] = 0xF961
...

=== Complex FFT Test ===
Complex FFT completed successfully
[0] = 0x0599
[1] = 0x077B
[2] = 0x0BA7
[3] = 0xFF15
[4] = 0x04C7
[5] = 0xF4D7
[6] = 0xFC33
[7] = 0xF75A
[8] = 0x000A
...

=== Inverse FFT Test ===
Inverse FFT completed successfully
[0] = 0x0599
[1] = 0x077B
[2] = 0x0BA7
[3] = 0xFF15
[4] = 0x04C7
[5] = 0xF4D7
[6] = 0xFC33
[7] = 0xF75A
[8] = 0x000A
...

=========================================
          FFT Example Complete          
=========================================
```

## Update Log
|Version  |Date    |Release Notes |
|:---  |:---    |:---    |
|1.0.0 |6/2025 |Initial version |
|      |        |        |