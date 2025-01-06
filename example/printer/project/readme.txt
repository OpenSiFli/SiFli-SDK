如果编译非52的board，需要menuconfig关闭USING_ADC_BUTTON
56的编译命令如下：
scons --lcpu="..\..\rom_bin\lcpu_general_ble_img\lcpu_img_56x_52.c" --simu --board=eh-lb561 -j8
58的编译命令如下：
scons --lcpu="..\..\rom_bin\lcpu_general_ble_img\lcpu_img_58x_52.c" --simu --board=ec-lb587 -j8