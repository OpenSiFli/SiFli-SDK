# Javascript 支持

思澈SDK选用QuickJS作为JS的开发运行平台。QuickJS是一个轻量的可嵌入的 JavaScript 引擎，支持包括模块、异步生成器、proxies、BigInt 等功能在内的 ES2020 标准。QuickJS 
也选择性地支持数学扩展，例如大十进制浮点数（BigDecimal）、大二进制浮点数（BigFloat）和运算符重载。

SDK将QuickJS和思澈的应用程序框架以及表盘框架做了集成，可以和现有的以C语言为基础开发的应用和表盘共存。在watch_demo中，提供了JS的示例。其中 _SDK_ROOT/example/watch_demo/project/jsroot_ 里面的JS运行在开发版中，需要通过project里面的 _jsroot.bat_ 下载到开发板中运行，例如ec-lb555开发板，  _SDK_ROOT/example/watch_demo/project/ec-lb555/jsroot.bat_ 会将 _SDK_ROOT/example/watch_demo/project/jsroot_ 里面的所有内容打包为一个文件系统，下载到开发版中。 
_SDK_ROOT/example/watch_demo/project/simulator/disk_ 下面的JS时模拟器运行的JS代码，和开发版中的代码相同，但是使用的资源格式不一样, 模拟器不支持思澈芯片特有的EZIP格式。

## 1. QuickJS 的初始化
   目前QucikJS的初始化是在第一个JS的应用或者表盘启动的时候完成的，初始化由SDK内部应用\
   表盘框架JS适配模块调用，所有JS的应用\
   表盘共享同一个JS运行上下文环境。为了和LVGL的单线程环境匹配，QuickJS也是单线程运行在LVGL的线程中。
   SDK QuickJS 目前需要：
- 4K bytes 栈, 
- 512K的运行堆。具体使用多少堆的内存，取决于运行的应用和表盘。
- 210K 的基础库ROM
- 其他扩展库，包括应用框架，以及LVGL支持， 需要 10K ROM，6K RAM.

## 2. 应用框架和表盘框架支持
  在Quick JS标准的class支持之上，SDK添加了应用框架和表盘框架支持,当系统启动的时候，会自动扫描文件系统的根目录，如果：
### -发现目录以JA_开头，  JA_<app_name>
      这个是Javascript的应用，目录中main.js是应用的主程序，需要定义一个class，名字app_name，继承自app class，并注册在JS全局变量中。Thumb.bin是APP的图标。
    
### -发现目录以JW_开头，  JW_<watchface_name>
      这个是Javascript的表盘，目录中 _main.js_ 是表盘的主程序，需要定义一个class，名字watchface_name，继承自app class，并注册在JS全局变量中。
      
  ```{note} 表盘和APP JS有区别在constructor：
      ```java
      //这个表盘的 main.js应该在/JW_wf8目录下
      class wf8 extends app{  //定义表盘
          constructor() {
              super(1);   // 表盘需要设置参数1， 其他应用使用0(默认值)
          }
          ...
      }
      ```
  ```
## 3. 应用框架函数包装
   SDK 在lv 模块中，也包装了应用框架的gui_app_xxx 函数， 包括：<br>
  - \ref gui_app_cleanup <br>
  - \ref gui_app_cleanup_now <br>
  - \ref gui_app_exit <br>
  - \ref gui_app_get_clock_parent <br>
  - \ref gui_app_get_intent <br>
  - \ref gui_app_get_page_userdata <br>
  - \ref gui_app_get_running_apps <br>
  - \ref gui_app_goback <br>
  - \ref gui_app_goback_to_page <br>
  - \ref gui_app_init <br>
  - \ref gui_app_is_actived <br>
  - \ref gui_app_is_all_closed <br>
  - \ref gui_app_manual_goback_anim <br>
  - \ref gui_app_remove_page <br>
  - \ref gui_app_run <br>
  - \ref gui_app_run_now <br>
  - \ref gui_app_self_exit <br>
  - \ref gui_app_set_page_userdata <br>

## 4. LVGL 支持
   SDK提供了LVGL的JS支持，其中 
   - lv.obj
     lv.obj 是基础LVGL控件, 每一个成员函数和lv_obj_xxx对应，为了节省内存，SDK仅仅输出了部分lv_obj_xxx函数作为lv.obj的成员, 目前包括：
     - create <br>
     - align_to <br>
     - get_x <br>
     - get_y <br>
     - set_pos <br>
     - set_size <br>
     - set_local_font <br>
     - set_page_glue <br>
     - align <br>
     - get_height <br>
     - get_width <br>
     - add_flag <br>
     - move_foreground <br>
     - move_background <br>
     - delete  <br>
     此外， lv.obj还增加了：
     - set_obj <br>
       这个函数在继承自lv.obj的控件创建时，设置已经创建的lv obj(c语言指针)，这样基类就不需要重复创建了。
     - bind <br>
       这个函数把控件和一个已知数据类型绑定，当数据产生时，可以调用回掉
     - set_event_cb <br>
       这个函数在控件接收lv event时候，调用回掉函数。
   - 继承自lv.obj的LV 控件
     - img <br>
       由LVGL提供的图像控件
     - label <br>
       由LVGL提供的Label控件     
     - analogclk <br>
       思澈开发的模拟表盘控件
     - idximg <br>
       思澈开发的多图像切换控件，图像有共同的文件头，以00-99为文件名尾，进行循环切换。
     - lvsfapng <br>
       思澈特有的压缩图像动画格式，尽在58x芯片支持
     - lvsfbarcode <br>
       思澈开发的条码控件
     - lvsfcomp <br>
       思澈开发的苹果风格组合控件     
     - lvsfcorner <br>
       思澈开发的苹果风格四角控件     
     - lvsfcurve <br>
       思澈开发的弯曲字符串显示控件     
     - qrcode <br>
       LVGL提供的二维码控件
     - rlottie <br>
       思澈集成的lottie动画控件
   - 图片资源
       Javascript的代码中，可能需要使用一些图片。思澈提供特有的图像压缩格式，可以帮助用户节省flash控件。思澈芯片采用硬件加速解码，可以达到流畅的用户体验。
       SDK有工具将PNG文件转换为私有格式，工具在 /tools/png2ezip/ezip.exe。 <br>
       其中,用于: <br>
       - 模拟器: <br>
          - ezip -convert xxx\yyy.png -rgb565 -binfile 1 <br>
            转换单个文件，颜色格式RGB565
          - ezip -dir xxx -rgb565 -binfile 1 <br>
            转换目录中所有PNG文件，颜色格式RGB565
       - 开发板：
          - ezip -convert xxx\yyy.png -rgb565 -binfile 2 <br>
            转换单个文件，颜色格式RGB565
          - ezip -dir xxx -rgb565 -binfile 2 <br>
            转换目录中所有PNG文件，颜色格式RGB565
       生成的文件在当前目录下output子目录. 
       
## 5. 数据业务支持

SDK提供了LVGL的控件和数据业务的绑定，通过lv.obj的bind函数，可以在JS代码中，处理收到的数据。
底层C代码在收到数据的时候，可以通过对绑定的lv obj, 发送LV_EVENT_REFRESH事件，将数据作为参数，这样JS的回掉就会被调用处理。
数据格式的定义请参考 _$SDK_ROOT/external/quickjs/lvgl/gui_app_data.h_ , SDK会把C的结构转换为相应的JS object.

