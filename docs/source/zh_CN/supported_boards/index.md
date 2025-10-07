# 支持的板子

开发板源代码根目录： `customer/boards`

每个板子的源码路径见表格中的**板子目录**列，比如开发板551-HDK的代码在`eh-lb551`目录下，该目录名也作为编译时指定的板子名称。
对于板载多核芯片的板子，每个板子目录下还会按内核建有子目录，如下图的eh-lb551目录下有`hcpu`和`lcpu`两个子目录，分别存放芯片SF32LB551的HCPU和LCPU的配置代码，当使用`scons --board=<board_name>`编译指定板子的程序时，如果不指定内核，默认使用HCPU的配置编译，如果想指定LCPU的配置，则需要加上`_lcpu`后缀，例如，`scons --board=eh-lb551`和`scons --board=eh-lb551_hcpu`两个编译命令都是以551-HDK的HCPU配置进行编译，生成的镜像文件保存在`build_eh-lb551_hcpu`目录下，而`scons --board=eh-lb551_lcpu`则以551-HDK的LCPU配置进行编译，生成的镜像文件保存在`build_eh-lb551_lcpu`目录下。

对于目前在售的开发板，我们创建了一套命名规则，一般是 `型号-类型_存储器类型_屏幕接口`。特别的，存储器类型和屏幕接口可能会被省略。

存储器的类型命名规则为：

- A：表示板子使用了SPI NAND存储器，后面的数字为存储器的容量，单位为MB
- N：表示板子使用了SPI NOR存储器，后面的数字为存储器的容量，单位为MB
- R：表示板子使用了SPI PSRAM存储器，后面的数字为存储器的容量，单位为MB
- E: 表示板子使用了eMMC存储器，后面的数字为存储器的容量，单位为GB

对于一个示例`a128r32n1`来说，就是存在128MB的SPI NAND存储器、32MB的SPI PSRAM存储器和1MB的SPI NOR存储器。

一些典型的示例：

- `sf32lb52-nano_n4`：表示型号为 SF32LB52 的 Nano 版本开发板，使用 4MB 的 SPI NOR 存储器。
- `sf32lb56-lcd_a128r12n1`：表示型号为 SF32LB56 的 LCD 版本开发板，使用 128MB 的 SPI NAND 存储器、12MB 的 SPI PSRAM 存储器和 1MB 的 SPI NOR 存储器。

```{image} ../../assets/folder.png
:scale: 70%
```

<!-- 
| left | center | right |
| :--- | :----: | ----: |
| a    | b      | c     | -->


## SF32LB55x系列

名称         |  型号          |    板子目录   |    
-------------|---------------|--------------|
551-HDK       | EH-SS6600A8   |   eh-lb551    | 
555-HDK       | EH-SF32LB555  |   eh-lb555    | 


下表的板子不再维护，无法用于工程编译，但目录仍旧保留
名称         |  型号          |    板子目录      |    
-------------|---------------|------------------|
551-EVB       | EC-LB551     |   ec-lb551xxx    | 
555-EVB       | EC-LB555     |   ec-lb555xxx    | 
557-EVB       | EC-LB557     |   ec-lb557xxx    | 
6600-HDK      | EH-SS6600    |   eh-ss6600xxx   | 


## SF32LB58x系列

简称         |  型号                      |    板子目录   |    
-------------|---------------------------|--------------|
583-EVB       | SF32LB58X_EVB_CORE(583)   |   ec-lb583    | 
585-EVB       | SF32LB58X_EVB_CORE(585)   |   ec-lb585    | 
587-EVB       | SF32LB58X_EVB_CORE(587)   |   [ec-lb587](boards/ec-lb587/doc/index.md)    | 
58-LCD_A128R32N1_DSI | SF32LB58-DevKit-LCD_A128R32N1_DSI |   [sf32lb58-lcd_a128r32n1_dsi](boards/sf32lb58-lcd_a128r32n1_dsi/doc/index.md)    |
58-LCD_N16R32N1_DPI | SF32LB58-DevKit-LCD_LCD_N16R32N1_DPI |   [sf32lb58-lcd_n16r32n1_dpi](boards/sf32lb58-lcd_n16r32n1_dpi/doc/index.md)    |
58-LCD_N16R32N1_DSI | SF32LB58-DevKit-LCD_N16R32N1_DSI |   [sf32lb58-lcd_n16r32n1_dsi](boards/sf32lb58-lcd_n16r32n1_dsi/doc/index.md)    |
58-LCD_N16R64N4 | SF32LB58-DevKit-LCD_N16R64N4 |   [sf32lb58-lcd-n16r64n4](boards/sf32lb58-lcd_n16r64n4/doc/index.md)    |

## SF32LB56x系列

简称          |  型号                      |    板子目录   |    
--------------|---------------------------|--------------|
567-EVB       | EC-LB56XV(567)            |   ec-lb567    | 
561-HDK       | EH-SF32LB56XU(561)        |   eh-lb561    | 
563-HDK       | EH-SF32LB56XU(561)        |   eh-lb563    | 
6700-HDK      | EH-SF32LB56XU(6700)       |   eh-ss6700   | 
56-LCD_A128R12N1 | SF32LB56-DevKit-LCD_A128R12N1 |   [sf32lb56-lcd_a128r12n1](boards/sf32lb56-lcd_a128r12n1/doc/index.md)    |
56-LCD_N16R12N1 | SF32LB56-DevKit-LCD_N16R12N1 |   [sf32lb56-lcd_n16r12n1](boards/sf32lb56-lcd_n16r12n1/doc/index.md)    |


下表的板子不再维护，无法用于工程编译，但目录仍旧保留

名称         |  型号          |    板子目录      |    
-------------|---------------|------------------|
561-EVB       | EC-LB561     |   ec-lb561xxx    | 
563-EVB       | EC-LB563     |   ec-lb563xxx    | 



## SF32LB52x系列

简称         |  型号                      |    板子目录   |    
-------------|---------------------------|--------------|
520-HDK      | EH-SF32LB52X(520)         |   eh-lb520    | 
523-HDK      | EH-SF32LB52X(523)         |   eh-lb523    | 
525-HDK      | EH-SF32LB52X(525)         |   eh-lb525    | 
6500-HDK     | EH-SF32LB52X(6500)        |   eh-lb6500   | 
NANO-N4      | SF32LB52-NANO_N4          |   sf32lb52-nano_n4 | 
NANO-N16R16  | SF32LB52-NANO_N16R16      |   sf32lb52-nano_n16r16 | 
黄山派(LCHSPI-ULP)     | SF32LB52-LCHSPI-ULP         |   sf32lb52-lchspi-ulp    |
52-LCD_N16R8 | SF32LB52-DevKit-LCD_N16R8 |   [sf32lb52-lcd_n16r8](boards/sf32lb52-lcd_n16r8/doc/index.md)    |
52-LCD_52D | SF32LB52-DevKit-LCD_52D |   [sf32lb52-lcd_52d](boards/sf32lb52-lcd_52d/doc/index.md)    |


```{toctree}
:titlesonly:
:glob:

boards/ec-lb587/doc/index
boards/ec-lb587_a2/doc/index
boards/sf32lb52-core_e8r16/doc/index
boards/sf32lb52-core_n16r16/doc/index
boards/sf32lb52-lcd_52d/doc/index
boards/sf32lb52-lcd_n16r8/doc/index.
boards/sf32lb52-lcd_n16r8_jdi/doc/index
boards/sf32lb56-lcd_a128r12n1/doc/index
boards/sf32lb56-lcd_n16r12n1/doc/index
boards/sf32lb58-lcd_a128r32n1_dsi/doc/index
boards/sf32lb58-lcd_n16r32n1_dpi/doc/index
boards/sf32lb58-lcd_n16r32n1_dsi/doc/index
boards/sf32lb58-lcd_n16r64n4/doc/index

```