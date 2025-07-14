# JavaScript Support

The SiFli SDK utilizes QuickJS as its JavaScript development runtime platform. QuickJS is a lightweight embeddable JavaScript engine that supports ES2020 specification features including modules, async generators, proxies, BigInt, and other functionalities. QuickJS also provides selective support for mathematical extensions such as arbitrary precision decimal floating-point numbers (BigDecimal), arbitrary precision binary floating-point numbers (BigFloat), and operator overloading.
The SDK integrates QuickJS with SiFli's application framework and watch face framework, enabling coexistence with existing C-based applications and watch faces. The watch_demo provides JavaScript examples. The JavaScript code located in _SDK_ROOT/example/watch_demo/project/jsroot_ runs on the development board and requires deployment to the development board via the jsroot.bat script in the project directory. For instance, for the ec-lb555 development board, _SDK_ROOT/example/watch_demo/project/ec-lb555/jsroot.bat packages all contents from _SDK_ROOT/example/watch_demo/project/jsroot into a filesystem image and downloads it to the development board.
The JavaScript code under _SDK_ROOT/example/watch_demo/project/simulator/disk is designed for simulator execution. While the codebase is identical to the development board version, it uses different resource formats since the simulator does not support the SiFli chip-specific EZIP compression format.

## 1. QuickJS initialization
Currently, the initialization of QucikJS is completed when the first JS application or dial starts. The initialization is performed by the internal SDK application\
The dial frame JS adaptation module is called, and all JS applications are applied
The dial shares the same JS runtime context environment. In order to match the single-threaded environment of LVGL, QuickJS is also running in the single-threaded environment of LVGL.
 SDK QuickJS currently requires:
- 4K byte stack,
- 512K running heap. The exact amount of memory used depends on the application and dial running.
- 210K base library ROM
- Other extensions, including the application framework, and LVGL support, require 10K ROM and 6K RAM.

## 2. Application framework and dial frame support
  In addition to the class support of Quick JS standard, the SDK adds application framework and dial frame support. When the system starts, it will automatically scan the root directory of the file system. If:
### -Found that the directory starts with JA_,  JA_<app_name>
      This is an application of Javascript. main.js in the directory is the main program of the application. You need to define a class named app_name, which inherits from the app class and is registered in the global JS variable. Thumb.bin is the APP icon.
    
### -Found that the directory starts with JW_，  JW_<watchface_name>
      This is the dial of Javascript. _main.js_ in the directory is the main program of the dial. You need to define a class named watchface_name, which inherits from the app class and is registered in the global JS variable.
      
  ```{note} Dial and APP JS is different in constructor：
      ```java
      //The main.js of this dial should be in the /JW_wf8 directory
      class wf8 extends app{  //Define the dial
          constructor() {
              super(1);   // Parameter 1 is required for the dial and 0 (default) is used for other applications
          }
          ...
      }
      ```
  ```
## 3. Application framework function packaging
   In the lv module, SDK also packages the gui_app_xxx function of the application framework, including:<br>
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

## 4. LVGL Support
   The SDK provides JS support for LVGL, which includes some things:
   - lv.obj
     lv.obj is the basic LVGL control, and each member function corresponds to lv_obj_xxx. In order to save memory, the SDK only outputs some lv_obj_xxx functions as members of lv.obj. At present, it includes the following contents:
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
     In addition, lv.obj has added:
     - set_obj <br>
       This function sets the created lv obj (c language pointer) when creating a control that inherits from lv.obj, so that the base class does not need to create it again.
     - bind <br>
       This function binds the control to a known data type and can call the callback function when the data is generated
     - set_event_cb <br>
       This function calls the callback function when the control receives lv event.
   - Inheriting from the lv.obj LV control
     - img <br>
       Image control provided by LVGL
     - label <br>
       The Label control provided by LVGL     
     - analogclk <br>
       The simulated dial control developed by SiFli
     - idximg <br>
       The multi-image switching control developed by SiChai has images with a common file header and a file name ending of 00-99 for cyclic switching.
     - lvsfapng <br>
       The unique compressed image animation format of Sicheng is supported by the 58x chip
     - lvsfbarcode <br>
       Barcode control developed by SiFli
     - lvsfcomp <br>
       A combination of controls with an Apple style developed by SiFli    
     - lvsfcorner <br>
       Sicheng's Apple-style four-corner control     
     - lvsfcurve <br>
       The curved string display control developed by SiFli    
     - qrcode <br>
       The LVGL provides a QR code control
     - rlottie <br>
       Sight integrated lottie animation controls
   - Image resource
       In the code of Javascript, some pictures may be needed. SiChui provides a special image compression format to help users save flash controls. SiChui chip uses hardware accelerated decoding to achieve smooth user experience.
       The SDK has a tool to convert PNG files to private formats. The tool is in /tools/png2ezip/ezip.exe. <br>
       Among them, the part used for the simulator is:<br>
       - Simulator: <br>
          - ezip -convert xxx\yyy.png -rgb565 -binfile 1 <br>
            Convert a single file in RGB565 color format
          - ezip -dir xxx -rgb565 -binfile 1 <br>
            Convert all PNG files in the directory to RGB565 color format
       - Development board：
          - ezip -convert xxx\yyy.png -rgb565 -binfile 2 <br>
            Convert a single file in RGB565 color format
          - ezip -dir xxx -rgb565 -binfile 2 <br>
            Convert all PNG files in the directory to RGB565 color format
       The generated file is in the output subdirectory of the current directory.
       
## 5. Data service support

SDK provides the binding of LVGL controls and data services. Through lv.obj's bind function, you can process the received data in JS code.
When the underlying C code receives the data, it can send the LV_EVENT_REFRESH event to the bound lv obj and send the data as a parameter, so that the JS callback will be called for processing.
For the definition of data format, please refer to _$SDK_ROOT/external/quickjs/lvgl/gui_app_data.h_. The SDK will convert the C structure into the corresponding JS object.

