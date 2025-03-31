# 支持的板子

开发板源代码根目录： `customer/boards`

每个板子的源码路径见表格中的**板子目录**列，比如开发板551-HDK的代码在`eh-lb551`目录下，该目录名也作为编译时指定的板子名称。
对于板载多核芯片的板子，每个板子目录下还会按内核建有子目录，如下图的eh-lb551目录下有`hcpu`和`lcpu`两个子目录，分别存放芯片SF32LB551的HCPU和LCPU的配置代码，当使用`scons --board=<board_name>`编译指定板子的程序时，如果不指定内核，默认使用HCPU的配置编译，如果想指定LCPU的配置，则需要加上`_lcpu`后缀，例如，`scons --board=eh-lb551`和`scons --board=eh-lb551_hcpu`两个编译命令都是以551-HDK的HCPU配置进行编译，生成的镜像文件保存在`build_eh-lb551_hcpu`目录下，而`scons --board=eh-lb551_lcpu`则以551-HDK的LCPU配置进行编译，生成的镜像文件保存在`build_eh-lb551_lcpu`目录下。

```{image} assets/folder.png
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
587-EVB       | SF32LB58X_EVB_CORE(587)   |   ec-lb587    | 
587-MOD       | SF32LB58-DevKit-LCD       |   em-lb587    | 


## SF32LB56x系列

简称          |  型号                      |    板子目录   |    
--------------|---------------------------|--------------|
567-EVB       | EC-LB56XV(567)            |   ec-lb567    | 
561-HDK       | EH-SF32LB56XU(561)        |   eh-lb561    | 
563-HDK       | EH-SF32LB56XU(561)        |   eh-lb563    | 
566-MOD       | SF32-LB5X-LCD(566)        |   em-lb566    | 
567-MOD       | SF32-LB5X-LCD(567)        |   em-lb567    | 
6700-HDK      | EH-SF32LB56XU(6700)       |   eh-ss6700   | 


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
525-MOD      | SF32LB52-DevKit-LCD       |   em-lb525    | 
6500-HDK     | EH-SF32LB52X(6500)        |   eh-lb6500   | 
