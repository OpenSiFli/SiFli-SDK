# vglite 演示程序

## 线性渐变+mask
固定显示一个path内部的线性渐变，执行10帧，间隔2秒。
打开console 运行命令`utest_run tc_vglite_0_0 1`
代码在tc_vg_linear_grad.c

## 字体渲染
显示2帧，一个小字体，一个大字体，执行1遍。
打开console 运行命令`utest_run tc_vglite_0_1 1`
代码在tc_vg_glyphs.c

## 旋转图片
旋转一个指针图片，执行3600帧，间隔160ms。
打开console 运行命令`utest_run tc_vglite_0_2 1`
代码在tc_vg_rotate.c


## 翻唱片效果
执行翻唱片的3D翻转效果，一直执行。
打开console 运行命令`utest_run tc_vglite_1_2 1`
代码在tc_vg_coverflow.c