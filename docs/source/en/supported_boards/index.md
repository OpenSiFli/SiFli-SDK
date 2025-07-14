# Supported Boards

Development board source code root directory: `customer/boards`

The source code path for each board is shown in the **Board Directory** column in the table. For example, the code for development board 551-HDK is in the `eh-lb551` directory. This directory name is also used as the board name when compiling.

For boards with multi-core chips, each board directory contains subdirectories organized by core. As shown in the figure below, the eh-lb551 directory has two subdirectories: `hcpu` and `lcpu`, which store the configuration code for the HCPU and LCPU of the SF32LB551 chip respectively. When using `scons --board=<board_name>` to compile a program for a specific board, if no core is specified, the HCPU configuration is used by default. To specify the LCPU configuration, add the `_lcpu` suffix. For example, both `scons --board=eh-lb551` and `scons --board=eh-lb551_hcpu` compile with the 551-HDK's HCPU configuration, and the generated image files are saved in the `build_eh-lb551_hcpu` directory, while `scons --board=eh-lb551_lcpu` compiles with the 551-HDK's LCPU configuration, and the generated image files are saved in the `build_eh-lb551_lcpu` directory.

For currently available development boards, we have created a naming convention, generally following the pattern `model-type_memory-type_display-interface`. Note that memory type and display interface may be omitted.

Memory type naming rules:

- A: Indicates the board uses SPI NAND memory, followed by the memory capacity in MB
- N: Indicates the board uses SPI NOR memory, followed by the memory capacity in MB
- R: Indicates the board uses SPI PSRAM memory, followed by the memory capacity in MB

For example, `a128r32n1` indicates 128MB SPI NAND memory, 32MB SPI PSRAM memory, and 1MB SPI NOR memory.

It may also represent specific chip specifications. Unless otherwise specified, external Flash is `16MB`. For example, for the 52 series, `52b` represents the SF32LB52B chip with built-in 4MB SPI NOR memory; `52j` represents the SF32LB52J chip with built-in 8MB SPI PSRAM memory and external 16MB SPI NOR memory.

Some typical examples:

- `sf32lb52-nano_52b`: Indicates the Nano version development board of model SF32LB52, using 4MB SPI NOR memory.
- `sf32lb56-lcd_a128r12n1`: Indicates the LCD version development board of model SF32LB56, using 128MB SPI NAND memory, 12MB SPI PSRAM memory, and 1MB SPI NOR memory.

```{image} ../../assets/folder.png
:scale: 70%
```

<!-- 
| left | center | right |
| :--- | :----: | ----: |
| a    | b      | c     | -->


## SF32LB55x Series

Name         |  Model        |    Board Directory   |    
-------------|---------------|----------------------|
551-HDK      | EH-SS6600A8   |   eh-lb551          | 
555-HDK      | EH-SF32LB555  |   eh-lb555          | 


The boards in the following table are no longer maintained and cannot be used for project compilation, but the directories are still retained
Name         |  Model        |    Board Directory      |    
-------------|---------------|-------------------------|
551-EVB      | EC-LB551      |   ec-lb551xxx          | 
555-EVB      | EC-LB555      |   ec-lb555xxx          | 
557-EVB      | EC-LB557      |   ec-lb557xxx          | 
6600-HDK     | EH-SS6600     |   eh-ss6600xxx         | 


## SF32LB58x Series

Abbreviation |  Model                    |    Board Directory   |    
-------------|---------------------------|----------------------|
583-EVB      | SF32LB58X_EVB_CORE(583)   |   ec-lb583          | 
585-EVB      | SF32LB58X_EVB_CORE(585)   |   ec-lb585          | 
587-EVB      | SF32LB58X_EVB_CORE(587)   |   ec-lb587          | 
LCD-A128R12N1-DSI | SF32LB58-DevKit-LCD |   sf32lb58-lcd_a128r12n1_dsi    |
LCD-N16R32N1-DPI | SF32LB58-DevKit-LCD |   sf32lb58-lcd_n16r32n1_dpi    |
LCD-N16R32N1-DSI | SF32LB58-DevKit-LCD |   sf32lb58-lcd_n16r32n1_dsi    |


## SF32LB56x Series

Abbreviation  |  Model                    |    Board Directory   |    
--------------|---------------------------|----------------------|
567-EVB       | EC-LB56XV(567)            |   ec-lb567          | 
561-HDK       | EH-SF32LB56XU(561)        |   eh-lb561          | 
563-HDK       | EH-SF32LB56XU(561)        |   eh-lb563          | 
6700-HDK      | EH-SF32LB56XU(6700)       |   eh-ss6700         | 
LCD-A128R12N1 | SF32LB56-DevKit-LCD |   sf32lb56-lcd_a128r12n1    |
LCD-N16R12N1 | SF32LB56-DevKit-LCD |   sf32lb56-lcd_n16r12n1    |


The boards in the following table are no longer maintained and cannot be used for project compilation, but the directories are still retained

Name         |  Model        |    Board Directory      |    
-------------|---------------|-------------------------|
561-EVB      | EC-LB561      |   ec-lb561xxx          | 
563-EVB      | EC-LB563      |   ec-lb563xxx          | 



## SF32LB52x Series

Abbreviation |  Model                    |    Board Directory   |    
-------------|---------------------------|----------------------|
520-HDK      | EH-SF32LB52X(520)         |   eh-lb520          | 
523-HDK      | EH-SF32LB52X(523)         |   eh-lb523          | 
525-HDK      | EH-SF32LB52X(525)         |   eh-lb525          | 
6500-HDK     | EH-SF32LB52X(6500)        |   eh-lb6500         | 
NANO-52b     | Nano(52b)                 |   sf32lb52-nano_52b | 
NANO-52j     | Nano(52j)                 |   sf32lb52-nano_52j | 
ULP(Huangshan Pi) | ULP(525)            |   sf32lb52-ulp      | 
LCD-525      | SF32LB52-DevKit-LCD(525)  |   sf32lb52-lcd_n16r8|
LCD-52b      | SF32LB52-DevKit-LCD(52b)  |   sf32lb52-lcd_52d  |