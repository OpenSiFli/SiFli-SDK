# CAN Example

Source Path: example/hal/can

## Supported Platforms
This example can run on the following development boards.
* sf32lb56-lcd

## Overview
* This example demonstrates how to use the chip's CAN controller to implement CAN communication functionality.
* Two development boards perform CAN communication to achieve data transmission and reception.

## Hardware Requirements
- SiFli development board with CAN support (such as sf32lb56-lcd or sf32lb58-lcd)
- Two CAN transceiver modules (such as TJA1050)
- At least two development boards for testing bidirectional communication
- Several Dupont wires

## Hardware Connection
* Before using CAN communication, you need to remove these two resistors from the board. See the diagram below for their specific locations.
![alt text](assets/remove_r.png)


Development Board | CAN Transceiver |
--|--
PA03 (8) | TX 
PA04 (9) | RX
5V(4)     | VCC
GND(6)    | GND

CAN Transceiver | CAN Transceiver
--|--
CHAN  | CHAN 
CHAL  | CHAL

* CAN transceiver (TJA1050)
![alt text](assets/TJA1050.png)
![alt text](assets/TJA1050_back.png)


### Compilation and Download
* Compile and burn the can_tx and can_rx projects separately to the corresponding boards
* Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb56-lcd_a128r12n1 -j32
```
`build_sf32lb56-lcd_a128r12n1_hcpu\download.bat`, program automatically downloaded via JLink:
```
build_sf32lb56-lcd_a128r12n1_hcpu/uart_download.bat (UART download)
```

## Example Output
If the example runs successfully, you will see the following output on the serial port:
* can_rx log output
```
                        
                       \ | /
01-13 17:07:26:864    - SiFli Corporation
01-13 17:07:26:865     / | \     build on Dec 25 2025, 2.4.0 build befba178
01-13 17:07:26:868     2020 - 2022 Copyright by SiFli team
01-13 17:07:26:869    mount /dev sucess
01-13 17:07:26:873    [I/drv.audprc] init 00 ADC_PATH_CFG0 0x924
01-13 17:07:26:874    [I/drv.audprc] HAL_AUDPRC_Init res 0
01-13 17:07:26:877    [I/drv] HAL_AUDCODEC_Init res 0
01-13 17:07:26:878    [32m[I/TOUCH] Regist touch screen driver, probe=1001aa4d [0m
01-13 17:07:26:882    call par CFG1(35bb)
01-13 17:07:26:883    fc 9, xtal 2000, pll 2038
01-13 17:07:26:887    call par CFG1(35bb)
01-13 17:07:26:889    fc 9, xtal 2000, pll 2038
01-13 17:07:26:890    CAN 2.0 start test
01-13 17:07:26:893    This board operates as the receiving end.
01-13 17:07:26:894    Switch to receiving mode and wait for data...
01-13 17:07:26:895    Received data: ID=0x123, DLC=8, Data: 0x11223344, 0x55667788
01-13 17:07:26:903    msh />
01-13 17:07:27:347    Send successful: ID=0x234, Data=0xAABBCCDD 0xEEFF0011 
01-13 17:07:27:846    Switch to receiving mode and wait for data...
01-13 17:07:27:851    Received data: ID=0x123, DLC=8, Data: 0x11223344, 0x55667788
01-13 17:07:28:347    Send successful: ID=0x234, Data=0xAABBCCDD 0xEEFF0011 
01-13 17:07:28:848    Switch to receiving mode and wait for data...
01-13 17:07:28:850    Received data: ID=0x123, DLC=8, Data: 0x11223344, 0x55667788
01-13 17:07:29:349    Send successful: ID=0x234, Data=0xAABBCCDD 0xEEFF0011 
01-13 17:07:29:849    Switch to receiving mode and wait for data...
01-13 17:07:29:850    Received data: ID=0x123, DLC=8, Data: 0x11223344, 0x55667788
01-13 17:07:30:350    Send successful: ID=0x234, Data=0xAABBCCDD 0xEEFF0011 
01-13 17:07:30:849    Switch to receiving mode and wait 
```
* can_tx log output
```
01-13 17:07:26:527     \ | /
01-13 17:07:26:530    - SiFli Corporation
01-13 17:07:26:532     / | \     build on Dec 25 2025, 2.4.0 build befba178
01-13 17:07:26:536     2020 - 2022 Copyright by SiFli team
01-13 17:07:26:539    mount /dev sucess
01-13 17:07:26:542    [I/drv.audprc] init 00 ADC_PATH_CFG0 0x924
01-13 17:07:26:546    [I/drv.audprc] HAL_AUDPRC_Init res 0
01-13 17:07:26:548    [I/drv] HAL_AUDCODEC_Init res 0
01-13 17:07:26:550    [32m[I/TOUCH] Regist touch screen driver, probe=1001aa51 [0m
01-13 17:07:26:552    call par CFG1(35bb)
01-13 17:07:26:553    fc 9, xtal 2000, pll 2026
01-13 17:07:26:555    call par CFG1(35bb)
01-13 17:07:26:556    fc 9, xtal 2000, pll 2026
01-13 17:07:26:557    CAN 2.0 start test.
01-13 17:07:26:558    This board operates as the sending end.
01-13 17:07:26:559    Send successful: ID=0x123, Data=0x11223344 0x55667788 
01-13 17:07:27:017    Switch to receiving mode and wait for data...
01-13 17:07:27:023    Received data: ID=0x123, DLC=0, Data: 0x11223344, 0x55667788
01-13 17:07:27:519    Send successful: ID=0x123, Data=0x11223344 0x55667788 
01-13 17:07:28:020    Switch to receiving mode and wait for data...
01-13 17:07:28:022    Received data: ID=0x234, DLC=8, Data: 0xAABBCCDD, 0xEEFF0011
01-13 17:07:28:521    Send successful: ID=0x123, Data=0x11223344 0x55667788 
01-13 17:07:29:022    Switch to receiving mode and wait for data...
01-13 17:07:29:024    Received data: ID=0x234, DLC=8, Data: 0xAABBCCDD, 0xEEFF0011
01-13 17:07:29:523    Send successful: ID=0x123, Data=0x11223344 0x55667788 
01-13 17:07:30:024    Switch to receiving mode and wait for data...
```
### CAN Read/Write Waveform
* CAN communication waveform
![alt text](assets/can_txrx.png)


### CAN Configuration Parameters
```c
static void CAN_Config(void)
{
  /* Initialize parameters */
  hcan.Instance = CAN1; 
  hcan.Init.Prescaler = 0x0b0b;        //Baud rate prescaler,1/48MHz/（11+1）=0.25us。1/(0.25us*10) = 400kbps
  
  /* Initialize CAN peripheral */
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    rt_kprintf("CAN initialization failed.\n");
    Error_Handler();
  }
  
  /* Configure CAN filter */
  CAN_FilterTypeDef can_filter_config;
  
  // Configure standard frame filter (ID=0x123)
  can_filter_config.FilterId = 0x123;
  can_filter_config.FilterMask = 0x7FF; // Exact match for standard ID
  can_filter_config.FilterBank = 0;
  can_filter_config.FilterActivation = ENABLE;
  can_filter_config.IDECheckEnable = ENABLE;
  can_filter_config.IDEValue = CAN_ID_STD; // Only accept standard frames  
  if (HAL_CAN_ConfigFilter(&hcan, &can_filter_config) != HAL_OK)//Configure filter
  {
    rt_kprintf("Failed to configure CAN filter.\n");
    Error_Handler();
  }
}
```
* First configure the prescaler for 400kbps
* Next configure the filter, `can_filter_config.FilterId` receives ID: 0x123 data frames
* Filter data frames through `can_filter_config.FilterMask` (0x7FF is the standard frame mask, 0x1FFFFFFF is the extended frame mask)
* Configure the filter number through `can_filter_config.FilterBank` (0~15)
* Control whether the filter is enabled through `can_filter_config.FilterActivation`
* Used to control IDE detection through `can_filter_config.IDECheckEnable`, which can determine whether it is an extended frame via IDE
* Used to control whether to receive standard frames or extended frames through `can_filter_config.IDEValue`

## Troubleshooting
* If the output doesn't match the logs or waveforms are inconsistent, please check the following:
    1. Check if the hardware wiring is correct and consistent with the connection instructions above
    2. Check if the UART resistors have been removed 
    
If you have any problems, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documents
- [SiFli-SDK Quick Start Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
```