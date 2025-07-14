# BTIM

BTIM (Basic Timer) is based on a 32-bit up counter for timing functionality. The counting clock is system pclk or external input signal, and can be prescaled by 1~65536 times. Timing results can be notified to the system through interrupts or DMA. BTIM contains master-slave mode interface for multi-level cascading, implementing multi-level counting or synchronous triggering functions.

## BTIM Main Features:
- 32-bit up auto-reload counter <br>
- 16-bit programmable (can be modified in real-time) prescaler, counter clock frequency division factor can be any value between 1~65536<br>
- Supports One Pulse Mode (OPM), automatically stops counter when counting is complete<br>
- Master-slave mode<br>
    - Supports interconnection with BTIM and GPT, can generate control signals as master device while being controlled by external input or other master devices as slave device<br>
    - Control modes include reset, trigger, gate, etc.<br>
    - Supports synchronous start, reset of multiple timers<br>
- Generate interrupt/DMA when counter overflow or initialization occurs (by software or internal/external trigger)<br>

SF32LB55X/56X/58X HCPU has two BTIMs: BTIM1 and BTIM2, LCPU has two BTIMs: BTIM3 and BTIM4.

For detailed BTIM interface, please refer to [TIM](#hal-tim)

## Using BTIM

The following BTIM usage is the same as GPT basic function usage, except setting Instance to BTIMx during initialization.
For specific usage, please refer to [](../hal/gpt.md)

## API Reference
[](#hal-tim)
