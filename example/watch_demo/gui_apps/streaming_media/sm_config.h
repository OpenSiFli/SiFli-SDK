#ifndef __SM_CONFIG_H_
#define __SM_CONFIG_H_

#define AUDIO_USING_44100   1
#define ORDER_VIDEO_SIZE    1
//#define __DROP_VIDEO__     1
//#define __DROP_AUDIO__     1

#define DOWNLOAD_THRESHHOLD     (1024 * 256)
#define VIDEO_THRESHHOLD        (1024 * 32)

#if 1
    #define APPLYING_DEVICE_URL "http://api-cluster.iwhop.cn/ddyun/device/runAppReturnToken"
    #define RESET_DEVICE_URL    "http://api-cluster.iwhop.cn/ddyun/device/updateStatus"
#else
    #define APPLYING_DEVICE_URL "http://test.iwhopro.com/howeartest/ddyun/device/runAppReturnToken"
    #define RESET_DEVICE_URL    "http://test.iwhopro.com/howeartest/ddyun/device/updateStatus"
#endif

#endif
