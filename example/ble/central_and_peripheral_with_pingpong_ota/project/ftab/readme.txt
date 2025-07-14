这是Flash Table的生成工程，通常用户不需要改动flash table.

如果用户改变了默认flash启动地址(0x10020000), 请参考修改ftab.c, 编译生成新的flash table.

如果用户不小心将启动flash 全部erase，则需要重新下载flash table.
