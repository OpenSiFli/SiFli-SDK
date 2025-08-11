# Simple FFT Example

Source Path: example/hal/fft/simple

## Supported Development Boards
This example can run on the following development boards:
- sf32lb56-lcd_a128r12n1

## Example Overview

This example demonstrates the following basic FFT operations:
- Real FFT - Converts real number sequences to frequency domain
- Complex FFT - Converts complex number sequences to frequency domain
- Inverse FFT - Converts frequency domain data back to time domain

## Usage

The following sections provide only absolutely necessary information. For complete steps on configuring SiFli-SDK and building/running projects, please refer to the [SiFli-SDK Quick Start Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)

### Hardware Requirements
This example requires no special hardware configuration. Simply connect the development board to your computer via USB.

## Example Output

Under normal circumstances, you will see the following output:

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

## Troubleshooting

For any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation

- [SiFli-SDK Official Documentation](https://docs.sifli.com/)

## Update History
| Version | Date    | Release Notes |
|:--------|:--------|:--------------|
| 1.0.0   | 6/2025  | Initial version |
|         |         |               |