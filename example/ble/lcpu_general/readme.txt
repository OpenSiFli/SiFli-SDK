@使用指南

    @介绍：lcpu_general工程是为全系列提供模板化的LCPU image，以便于使用者只想在HCPU上运行并希望打开蓝牙功能时可直接使用该工程产生的LCPU image。
    
    @使用方式：该工程在SDK release时已经生成标准化LCPU image并放置在SDK\example\rom_bin\lcpu_general_ble_img\lcpu_img.c。 如果使用者希望修改该LCPU image可对本工程进行配置并重新编译。
               生产的lcpu_img.c会通过post_build.bat自动覆盖SDK\example\rom_bin\lcpu_general_ble_img\lcpu_img.c。