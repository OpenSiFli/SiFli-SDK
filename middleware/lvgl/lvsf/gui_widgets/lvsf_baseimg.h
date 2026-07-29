/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_BASEIMG_H_
#define _LVSF_BASEIMG_H_

#ifdef __cplusplus
extern "C" {
#endif


/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "lvsf_conf_internal.h"

#if LVSF_USE_BASEIMG!=0

/*Testing of dependencies*/

#if LV_USE_IMG == 0
#error "lv_baseimg: lv_baseimg is required. Enable it in lv_conf.h (LV_USE_IMG  1) "
#endif
/*********************
 *      DEFINES
 *********************/

extern const lv_obj_class_t lv_baseimg_class;


/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 基础图像类型枚举
 * @note 定义了基础图像控件支持的多种显示模式
 */
enum
{
    BASEIMG_TYPE_ARRAY_INDEX,       //图组，序列模式，数值就是图片序号
    BASEIMG_TYPE_ARRAY_VALUE,       //图组，数值模式，一段数值对应一张图
    BASEIMG_TYPE_ARRAY_Q248,        //图组，数字模式，数据是Q24.8格式
    BASEIMG_TYPE_POINTER,           //指针，无级变化
    BASEIMG_TYPE_POINTER_GRID,      //跳针

    BASEIMG_TYPE_SEQUENCE = 0x0f,           //序列帧，持续的正向循环
    BASEIMG_TYPE_SEQUENCE_BACK = 0x1f,      //序列帧，持续的逆向循环
    BASEIMG_TYPE_SEQUENCE_CIRCLE = 0x10f,   //序列帧，持续的正逆向循环
};
typedef uint16_t lv_baseimg_type_t;

/**
 * @brief 基础图像状态枚举
 * @note 定义了序列帧播放过程中的各种状态
 */
enum
{
    BASEIMG_STATE_NULL,

    BASEIMG_STATE_PLAY_START,            //序列帧播放开始，停止或者刚开始触发
    BASEIMG_STATE_PLAY_STOP,             //序列帧播放停止，序号归零
    BASEIMG_STATE_PLAY_RESUME,           //序列帧播放恢复，暂停以后播放触发
    BASEIMG_STATE_PLAY_PAUSE,            //序列帧播放暂停，序号不变
    BASEIMG_STATE_PLAY_DONE,             //序列帧正向播放完成
    BASEIMG_STATE_PLAY_BACK_DONE,        //序列帧反向播放完成
};
typedef uint16_t lv_baseimg_state_t;

/**
 * @brief 基础图像源类型枚举
 * @note 定义了图像源的两种类型：内存描述符和文件
 */
enum
{
    BASEIMG_SRC_TYPE_DSC,
    BASEIMG_SRC_TYPE_FILE,
    BASEIMG_SRC_TYPE_SEQUENCE,
};
typedef uint8_t lv_baseimg_src_type_t;

/**
 * @brief 图像值表位置结构体
 */
typedef struct
{
    int16_t x, y;    /**<X坐标，Y坐标*/
} img_value_table_pos;

/**
 * @brief 图像值表值结构体
 */
typedef struct
{
    int32_t value;    /**<值*/
} img_value_table_value;

/**
 * @brief 图像值表位置和值结构体
 */
typedef struct
{
    int16_t x, y;    /**<X坐标，Y坐标*/
    int32_t value;   /**<值*/
} img_value_table_pos_value;


/**
 * @brief 图像值表联合体
 * @note 用于存储不同类型的值表数据
 */
typedef union
{
    char *pos;                              /**<位置指针*/
    img_value_table_pos *pos_table;         /**<位置表指针*/
    img_value_table_value *value_table;     /**<值表指针*/
    img_value_table_pos_value *pos_value_table; /**<位置和值表指针*/
} img_value_table;


/**
 * @brief 图像文件数据结构
 */
typedef struct
{
    uint32_t addr;    /**<地址*/
    uint32_t len;     /**<长度*/
} lv_img_file_data_t;

/**
 * @brief 图像文件结构
 */
typedef struct
{
    char file_path[64];             /**<文件路径*/
    lv_img_file_data_t *dsc_array;  /**<描述符数组*/
} lv_img_file_t;

/**
 * @brief 基础图像索引回调函数类型
 * @param baseimg 基础图像对象指针
 * @note 当基础图像设置到指定索引时调用
 */
typedef void (*lv_baseimg_index_cb)(lv_obj_t *baseimg);

/**
 * @brief 基础图像状态回调函数类型
 * @param baseimg 基础图像对象指针
 * @param state 图像状态
 * @note 当基础图像状态改变时调用
 */
typedef void (*lv_baseimg_state_cb)(lv_obj_t *baseimg, lv_baseimg_state_t state);

/**
 * @brief 基础图像数据结构
 */
typedef struct
{
    /*Ext. of ancestor*/
    lv_img_t bg;
    lv_img_dsc_t **dsc_array;       /**<描述符数组*/
    lv_img_file_t file;             /**<文件信息*/

    uint16_t    index_current;      /**<当前索引*/
    uint16_t    index_start;        /**<起始索引*/
    uint16_t    index_end;          /**<结束索引*/
    uint16_t    index_empty;        /**<空索引*/

    int16_t     loop;               /**<循环次数，-1表示无限循环*/
    uint16_t     loop_count;        /**<当前循环次数*/
    lv_baseimg_type_t    img_type;  /**<图像类型*/
    lv_baseimg_src_type_t  src_type; /**<源类型*/
    lv_baseimg_state_t state;       /**<状态*/

    uint16_t    index_cb_num;       /**<索引回调数量*/
    lv_baseimg_index_cb index_cb;   /**<索引回调函数*/
    lv_baseimg_state_cb state_cb;   /**<状态回调函数*/

    img_value_table value_table;    /**<值表*/
} lv_baseimg_t;


/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * @brief 基础图像刷新定时器回调函数
 * @param timer 定时器对象指针
 * @note 该函数通过定时器定期刷新基础图像的显示内容
 */
void lv_baseimg_refresh_timer(lv_timer_t *timer);


/**
 * @brief 创建基础图像对象
 * @param parent 父对象指针
 * @return 成功返回图像对象指针，失败返回NULL
 * @note 该函数创建一个lv_baseimg对象并初始化
 */
lv_obj_t *lv_baseimg_create(lv_obj_t *parent);


/**
 * @brief 设置状态
 * @param img 图像对象指针
 * @param state 状态值
 * @note 该函数设置基础图像的状态
 */
void lv_baseimg_set_state(lv_obj_t *img, lv_baseimg_state_t state);


/**
 * @brief 设置源数组
 * @param img 图像对象指针
 * @param dsc_array 描述符数组指针
 * @param index_star 起始索引
 * @param index_end 结束索引
 * @note 该函数设置图像的源数组
 */
void lv_baseimg_set_src_array(lv_obj_t *img, const lv_img_dsc_t **dsc_array,
                              int16_t index_star, int16_t index_end);


/**
 * @brief 设置源数组2
 * @param img 图像对象指针
 * @param file_path 文件路径
 * @param dsc_array 描述符数组指针
 * @param index_start 起始索引
 * @param index_end 结束索引
 * @note 该函数通过文件路径设置图像的源数组
 */
void lv_baseimg_set_src_array2(lv_obj_t *img, char *file_path, lv_img_file_data_t *dsc_array,
                               int16_t index_start, int16_t index_end);


/**
 * @brief 设置序列帧图片源和序号
 * @param img 基础图片对象指针
 * @param src 图片源
 * @param index_start 起始索引
 * @param index_end 结束索引
 * @note 该函数设置文件路径和图片描述符数组，用于从文件加载图片
 */
void lv_baseimg_set_src_array3(lv_obj_t *img, const void *src, int16_t index_start, int16_t index_end);

/**
 * @brief 设置状态回调函数
 * @param img 图像对象指针
 * @param cb 回调函数指针
 * @note 该函数设置状态改变时的回调函数
 */
void lv_baseimg_set_state_cb(lv_obj_t *img, lv_baseimg_state_cb cb);


/**
 * @brief 设置值表
 * @param img 图像对象指针
 * @param value_table 值表指针
 * @note 该函数设置值表用于数值到图像的映射
 */
void lv_baseimg_set_value_table(lv_obj_t *img, char *value_table);


/**
 * @brief 设置角度
 * @param img 图像对象指针
 * @param angle 角度值
 * @note 该函数设置图像的角度
 */
void lv_baseimg_set_angle(lv_obj_t *img, int16_t angle);


/**
 * @brief 设置缩放
 * @param img 图像对象指针
 * @param zoom 缩放值
 * @note 该函数设置图像的缩放比例
 */
void lv_baseimg_set_zoom(lv_obj_t *img, uint16_t zoom);


/**
 * @brief 设置起始索引
 * @param img 图像对象指针
 * @param index 起始索引
 * @note 该函数设置图像序列的起始索引
 */
void lv_baseimg_set_start_index(lv_obj_t *img, uint16_t index);


/**
 * @brief 设置结束索引
 * @param img 图像对象指针
 * @param index 结束索引
 * @note 该函数设置图像序列的结束索引
 */
void lv_baseimg_set_end_index(lv_obj_t *img, uint16_t index);


/**
 * @brief 设置当前索引
 * @param img 图像对象指针
 * @param index 当前索引
 * @note 该函数设置图像的当前索引
 */
void lv_baseimg_set_current_index(lv_obj_t *img, uint16_t index);


/**
 * @brief 设置图像类型
 * @param img 图像对象指针
 * @param img_type 图像类型
 * @note 该函数设置图像的显示类型
 */
void lv_baseimg_set_img_type(lv_obj_t *img, uint16_t img_type);


/**
 * @brief 设置空索引
 * @param img 图像对象指针
 * @param index 空索引
 * @note 该函数设置空图像的索引
 */
void lv_baseimg_set_empty_idx(lv_obj_t *img, uint16_t index);


/**
 * @brief 设置旋转中心X坐标
 * @param img 图像对象指针
 * @param x X坐标
 * @note 该函数设置图像旋转的中心点X坐标
 */
void lv_baseimg_set_pivot_X(lv_obj_t *img, int16_t x);


/**
 * @brief 设置旋转中心Y坐标
 * @param img 图像对象指针
 * @param y Y坐标
 * @note 该函数设置图像旋转的中心点Y坐标
 */
void lv_baseimg_set_pivot_y(lv_obj_t *img, int16_t y);


/**
 * @brief 设置循环次数
 * @param img 图像对象指针
 * @param loop 循环次数，-1表示无限循环
 * @note 该函数设置序列帧的循环次数
 */
void lv_baseimg_set_loop(lv_obj_t *img, int16_t loop);


/**
 * @brief 获取角度
 * @param img 图像对象指针
 * @return 返回角度值
 * @note 该函数获取图像的当前角度
 */
int16_t lv_baseimg_get_angle(lv_obj_t *img);


/**
 * @brief 获取缩放
 * @param img 图像对象指针
 * @return 返回缩放值
 * @note 该函数获取图像的当前缩放比例
 */
uint16_t lv_baseimg_get_zoom(lv_obj_t *img);


/**
 * @brief 获取起始索引
 * @param img 图像对象指针
 * @return 返回起始索引
 * @note 该函数获取图像序列的起始索引
 */
uint16_t lv_baseimg_get_start_index(lv_obj_t *img);


/**
 * @brief 获取结束索引
 * @param img 图像对象指针
 * @return 返回结束索引
 * @note 该函数获取图像序列的结束索引
 */
uint16_t lv_baseimg_get_end_index(lv_obj_t *img);


/**
 * @brief 获取当前索引
 * @param img 图像对象指针
 * @return 返回当前索引
 * @note 该函数获取图像的当前索引
 */
uint16_t lv_baseimg_get_current_index(lv_obj_t *img);


/**
 * @brief 获取图像类型
 * @param img 图像对象指针
 * @return 返回图像类型（lv_baseimg_type_t）
 * @note 该函数获取图像的显示类型
 */
uint16_t lv_baseimg_get_img_type(lv_obj_t *img);


/**
 * @brief 获取空索引
 * @param img 图像对象指针
 * @return 返回空索引
 * @note 该函数获取空图像的索引
 */
uint16_t lv_baseimg_get_empty_idx(lv_obj_t *img);


/**
 * @brief 获取旋转中心X坐标
 * @param img 图像对象指针
 * @return 返回旋转中心X坐标
 * @note 该函数获取图像旋转中心的X坐标
 */
int16_t lv_baseimg_get_pivot_x(lv_obj_t *img);


/**
 * @brief 获取旋转中心Y坐标
 * @param img 图像对象指针
 * @return 返回旋转中心Y坐标
 * @note 该函数获取图像旋转中心的Y坐标
 */
int16_t lv_baseimg_get_pivot_y(lv_obj_t *img);


/**********************
 *      MACROS
 **********************/
#endif /*LVSF_USE_BASEIMG*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*_LVSF_BASEIMG_H_*/
