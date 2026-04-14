# 视频文档
视频解码使用了ffmpeg引擎，需要配置ffmpeg，见external/ffmpeg/Kconfig，完全使用了软件解码，视频需要转换为h264或mjpeg简洁编码才能解码显示。也支持思澈自定义的ezip编码，使用硬件解码ezip。ezip不支持帧间压缩，就是每张图片用ezip压缩，再加上声音，还是使用mp4封装， 对外调用接口不变。
## 内存要求
代码大小:      310K bytes
全局变量:      1K bytes
动态申请内存:  3.3M byte 240 * 320 (包含3帧视频YUV缓冲). 
## 性能指标
- 最大播放尺寸在240 * 320左右可以达到30fps，再大了要降低帧率才行，也跟图像细节复杂度有关，细节太复杂，压缩率低，解码慢。
- 目前软件要求转换后的视频宽度是16的整数倍。(否则会花屏，因为ffmepg解码后每行会padding对齐到16的整数倍，得降低性能从ffmpeg里一行行复制数据才能正常显示）
## 支持的音视频codec格式
 - h264
 - h263
 - mp3
 - aac
 - mjpeg
 - ezip
 - opus
 - vorbis
## 支持的音视频封装格式
 - ogg
 - mp4
## 配置说明
需要在menuconfig中打开PKG_USING_FFMPEG才能支持ffmpeg，如果要支持mp4文件中的音频，还要打开PKG_USING_FFMPEG_AUDIO， 如果要支持网络视频，还要打开PKG_USING_FFMPEG_HTTP。
如果要支持mjpeg，要打开PKG_USING_FFMPEG_MJPEG（在58x芯片平台上才会硬件解码，软件解码不建议）  
![](./png/menuconfig_ffmpeg.png)

## 工具链
通常视频尺寸比较大，视频格式也比较复杂，需要转换后才能显示。
### ffmpeg 官方工具链
 可以到 https://ffmpeg.org 下载ffmpeg工具链，下载解压后包含：
  * 视频转换、裁剪、缩放工具 ffmpeg.exe
  * 视频格式查看工具 ffprobe.exe
  * 视频播放工具 ffplay.exe  

这个一般是本地调试用, ffprobe.exe teste.mp4的例子
![](./png/ffprobe.png)

上面high表示是高质量编码格式, 这个没法支持，要转为baseline。
本地转换命令是：
```python
#
ffmpeg -i input.mp4 -profile:v baseline -level 1.0 -r 30 -acodec mp3 -ar 44100 -ac 2 output.mp4
# -profile:v baseline -level 1.0表示转为baseline格式
# -r 30 表示帧率为30， 可根据图像大小适当降低以降低cpu占用率
# -acodec mp3表示mp3编码，也可以用-acodec aac编码为aac
# -ar 44100表示音频采样率
# -ac 2 表示双声道，可以用-ac 1用单声道减小视频文件大小
# 命令中的输出的文件扩展名定要用mp4，不然可能无法转换。
```
```
# 转为mjpeg格式
ffmpeg -i input.mp4 -c:v mjpeg -c:a copy -vf "colorspace=bt709:iall=bt470bg:range=pc" video_example.mp4
```

转换后可以用ffprobe.exe看看文件格式， 也可以用ffplay.exe播放看看。

### 思澈工具链
   在ffmpeg官方工具链基础上封装的图像转换工具， 使用方法在工具下有文档。 
   GraphicsTool https://wiki.sifli.com/tools/index.html
   如果要转换为ezip图像编码格式，只能使用这个工具。
   ezip格式无法用ffplay.exe播放，只能在板子播放。

## 视频转换
- 视频转换可以用上面提到的ffmpeg官方工具(不支持ezip)或思澈工具，官方工具使用可以ffmpeg -h或ffmpeg -h full查看帮助，也可以网络上搜索ffmpeg常用的命令。
- 要求转换后的宽度是16的整数倍，编码格式为baseline

## example
- 播放本地文件的例子：
原来disk目录下是个空文件，需要用转换后的baseline格式mp4文件替换。  
https://gitee.com/SiFli/sifli-sdk/tree/main/example/multimedia/lvgl/lvgl_v8_media

- 播放网络视频文件的例子：
需要打开手机BT pan，并共享网络和热点。  
https://gitee.com/SiFli/sifli-sdk/tree/main/example/multimedia/lvgl/streaming_media

## 重要数据结构

```c
typedef struct
{
    ffmpeg_src_e    src;          //当前必须是e_src_localfile
    int             fmt_ctx_flag; //ffmpeg AVFMT_FLAG_*  map, in avformat.h. notw used now, should be zero
    uint8_t         fmt;          //dest image format. shoul be IMG_DESC_FMT_* in media_dec.h
                                  //后面单独介绍如何显示获取的图像
    uint8_t         is_loop;      //audio loop again if set to 1
    uint8_t         audio_enable; //enable audio in media file if set to 1
    uint8_t         video_enable; //enable video in meida file if set to 1
    const char     *file_path;   /*可以是本地文件或网络路径或nand里的地址
                                  nand里地址的例子(使用时替换地址和长度)：
                                  nand://addr=0x63000abcd&len=0x12bce
                                  本地文件例子：
                                  /video/test.mp4
                                  网络例子1：
                                  http://xxxx.com/test.mp4
                                  网络例子2：
                                  https://xxxx.com/test.mp4
                                 */
    void *(*mem_malloc)(size_t size); //给ffmpeg用的申请内存的函数
    void (*mem_free)(void *rmem);     //给ffmpeg用的释放内存的函数
    int (*notify)(uint32_t user_data, ffmpeg_cmd_e cmd, uint32_t val); //通知回调

    /*以下已经废弃不用*/
    uint8_t        *avio_buffer;
    uint32_t       avio_buffer_size;
    media_free     pack_free;
} ffmpeg_config_t;
```

## 接口说明
API参考media_dec.h

1. int ffmpeg_open(ffmpeg_handle *return_hanlde, ffmpeg_config_t *cfg, uint32_t user_data);  
```
函数说明：
    打开一个文件或网络URL播放。
参数：
    return_hanlde: 返回一个句柄  
    cfg: 配置媒体信息， 参考ffmpeg_config_t  
    user_data: 这个在调用cfg里的notify()回调时会传递回去  
返回值:
    返回0成功，其他值失败
```
2. void ffmpeg_close(ffmpeg_handle hanlde);

```
函数说明：
    停止播放, 不能在notify()里调用。
参数：
    hanlde: ffmpeg_open()时获得的句柄  
```
3. void ffmpeg_pause(ffmpeg_handle hanlde);
```
函数说明：
    暂停播放, 不能在notify()里调用。
参数：
    hanlde: ffmpeg_open()时获得的句柄  
```
4. void ffmpeg_resume(ffmpeg_handle hanlde);
```
函数说明：
    恢复播放, 不能在notify()回调函数里调用。
参数：
    hanlde: ffmpeg_open()时获得的句柄  
```
5. void ffmpeg_seek(ffmpeg_handle hanlde, uint32_t second);
```
函数说明：
    定位到某个位置播放, 不能在notify()里调用。
参数：
    hanlde: ffmpeg_open()时获得的句柄  
    second：从视频开始位置偏移多少秒
```
6. void ffmpeg_audio_mute(ffmpeg_handle hanlde, bool is_mute);
```
函数说明：
    把音静音或不静音, 不能在notify()里调用。
参数：
    hanlde: ffmpeg_open()时获得的句柄  
    is_mute： 1 -- 静音
              0 -- 不静音
```
7. uint8_t *ffmpeg_get_first_ezip(const char *filename, uint32_t *w, uint32_t *h, uint32_t *psize);
```
函数说明：
    当视频是ezip编码，并且是本地文件时，获取视频里第一张图片，用完后需要用
    ffmpeg_eizp_release()释放内存。
参数：
    filename: 文件路径
    w： 返回宽度
    h： 返回高度
    psize: 返回内存大小
返回值：
    图片对应的内存，用完后需要用ffmpeg_eizp_release()释放内存。
```
8. uint8_t *ffmpeg_get_first_ezip_in_nand(const char *nand_address, uint32_t nand_size, uint32_t *w, uint32_t *h, uint32_t *psize);
```
函数说明：
    当视频是ezip编码，并且是本地nand里直接存放时，获取视频里第一张图片。
参数：
    nand_address：视频起始地址
    nand_size：视频大小
    w： 返回宽度
    h： 返回高度
    psize: 返回内存大小
返回值：
    图片对应的内存，用完后需要用ffmpeg_eizp_release()释放内存。
```
9. void ffmpeg_eizp_release(uint8_t *ezip);
```
函数说明：
    释放获取第一张图片时申请的内存。
参数：
    ezip: ffmpeg_get_first_ezip()或ffmpeg_get_first_ezip_in_nand()的返回值
```
10. int ffmpeg_get_video_info(ffmpeg_handle hanlde, uint32_t *video_width, uint32_t *video_height, video_info_t *info);  

```
函数说明：
    获得视频信息。
参数：
    hanlde: ffmpeg_open()时获得的句柄
    video_width： 返回宽度
    video_height： 返回高度
    info: 图片格式更多的信息，图片格式在显示时要用到，其中一个format表示图像格式, 里面的gpu_pic_fmt如果是ezip，则app要记录，直接拿ezip去显示
返回值：
    0 -- 成功
    其他值 -- 失败
```

11. bool ffmpeg_is_video_available(ffmpeg_handle hanlde);
```
函数说明：
    判断解码缓存里是不是有新的图片，获取图片前应该用这个查询下
参数：
    hanlde: ffmpeg_open()时获得的句柄
返回值：
    1 -- 有
    0 -- 没有
```

12. int ffmpeg_next_video_frame(ffmpeg_handle hanlde, uint8_t *data);
```
函数说明：
    从解码缓存里取走一个新的图片
参数：
    hanlde: ffmpeg_open()时获得的句柄
    data： 返回图片等信息， data虽然是uint8_t *data， 其实返回时是往里面存三个指针，
           返回时是把uint8_t *data当作uint8 **data来用的，不同format三个指针赋值如下, format在前面
           ffmpeg_get_video_info()的info说明那里有。
           A. format is e_sifli_fmt_ezip
                data[0] -- data addr
                data[1] -- data size
                data[2] -- IMG_DESC_FMT_EZIP
           C. format is e_sifli_fmt_yuv420p, app can use yuv directly for 52x/56x/57x chip
                data[0] -- y address
                data[1] -- u address
                data[2] -- v address
           D. others
                data[0] -- data addr
                data[1] -- unused
                data[2] -- unused
 
返回值：
    0 -- 成功，可以显示了
    其他值 -- 失败
```
13. ffmpeg_handle ffmpeg_player_status_get(void);

```
函数说明：
    检查是不是有视频在播放，非线程安全的，可能返回后实际结果又变了
参数：
    void
返回值：
    返回正在播放的句柄，如果没有在播放的就返回NULL
```
## 图像显示
ffmpeg_config_t里的fmt表示UI上要显示的格式。
如果视频编码是h264/h263格式， 内部解码出来是yuv格式。内部media_video_convert()会把解码出来的yuv转换为fmt对应的格式。
如果视频编码是ezip， 内部解码出来仅仅是把ezip数据拿出来。UI上获得ezip数据直接显示。
如果视频编码是mjpeg， 内部解码出来是IMG_DESC_FMT_ARGB8888。

应用程序创建image控件的格式要用media_dec.h里这几个宏， 不同平台支持不一样，这部分参考example去修改。

IMG_DESC_FMT
IMG_PIXEL_SIZE
IMG_LV_FMT

## 调试
目前解码只缓存了3帧图像，以音频播放为主，如果log里有drop字样，表示有丢帧了。
