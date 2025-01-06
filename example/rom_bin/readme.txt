这个目录包括：

- lcpu_boot_loader 里面是LCPU的bootloader ROM 信息，其中 rom.sym用于用户LCPU的image连接使用LCPU ROM里面代码，开发LCPU的用户需要链接rom.sym
- lcpu_general_ble_img 里面是LCPU默认的代码，里面包含了BLE的启动，对于用户只使用BLE的基本功能，可以将lcpu_img.c加入用户HCPU工程，参考BLE里面的示例使用。

   