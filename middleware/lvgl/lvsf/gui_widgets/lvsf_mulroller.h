/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_MULROLLER_H
#define _LVSF_MULROLLER_H

#ifdef __cplusplus
extern "C" {
#endif
/*********************
 *      INCLUDES
 *********************/

#include "lvsf_conf_internal.h"
#include "lvsf_obj_ext.h"



#if LVSF_USE_MULROLLER != 0

/*Testing of dependencies*/
#if LVSF_USE_BASELABEL == 0
#error "lvsf_baselabel: lvsf_baselabel is required. Enable it in lv_conf.h (LVSF_USE_BASELABEL  1) "
#endif
#if LVSF_USE_BASEIMG == 0
#error "lvsf_baseimg: lvsf_baseimg is required. Enable it in lv_conf.h (LVSF_USE_BASEIMG  1) "
#endif



/**********************
 *      TYPEDEFS
 **********************/

/**
 * @brief 多滚轮出现回调函数类型
 * @param obj 滚轮对象指针
 * @param idx 元素索引
 * @return 返回布尔值
 * @note 当滚轮元素移动到指定位置时调用该回调函数
 */
typedef bool (*lv_mulroller_appear_cb)(lv_obj_t *obj, int16_t idx);

/**
 * @brief 滚轮动画就绪回调函数类型
 * @note 当滚轮动画完成时调用该回调函数
 */
typedef void (*lv_mulroller_anim_ready_cb)();

/**
 * @brief 滚轮中间模式枚举
 */
enum
{
    MULROLLER_MIDDLE_MODE_NULL,
    MULROLLER_MIDDLE_MODE_STOP,         //动画停止时使用
    MULROLLER_MIDDLE_MODE_MOVING,       //动画运行时使用
};
typedef uint8_t lv_mulroller_middle_mode_t;

/**
 * @brief 滚轮移动类型枚举
 */
enum
{
    MULROLLER_MOVE_NULL,
    MULROLLER_MOVE_LIMIT,
    MULROLLER_MOVE_FREEDOM,
};
typedef uint8_t lv_mulroller_move_type;

/**
 * @brief 滚轮动画类型枚举
 */
enum
{
    /* internal */
    MULROLLER_ANIM_NULL,
    MULROLLER_ANIM_INERTANCE,
    MULROLLER_ANIM_LIMIT,           //无法移动，停止移动动画和对齐动画
    /* custom */
    MULROLLER_ANIM_ALIGN,
    MULROLLER_ANIM_START,
    MULROLLER_ANIM_END,
};
typedef uint8_t lv_mulroller_anim_type;

/**
 * @brief 滚轮颜色模式枚举
 */
enum
{
    MULROLLER_COLOR_NULL,           /**<无颜色变化*/
    MULROLLER_COLOR_POS,            /**<随位置改变颜色*/
    MULROLLER_COLOR_TIME,           /**<动画停止后随时间改变颜色*/
    MULROLLER_COLOR_MASK,           /**<在中间创建颜色掩码*/
    MULROLLER_COLOR_COVER,          /**<在中间创建新对象，颜色会被覆盖*/
    MULROLLER_COLOR_STOP,           /**<动画停止时改变颜色*/
};
typedef uint8_t lv_mulroller_color_mode_t;

/**
 * @brief 滚轮布局模式枚举
 */
enum
{
    /*all elements at the same plane */
    MULROLLER_LAYOUT_FLAT,          /**<所有元素在同一平面，大小相同*/
    MULROLLER_LAYOUT_MID,           /**<只有中间元素不同*/
    MULROLLER_LAYOUT_RHOMB,         /**<所有元素看起来像菱形*/

    /*some elements overlap*/
    MULROLLER_LAYOUT_OVERLAP,       /**<所有元素依次重叠，移动方向为左*/
};
typedef uint8_t lv_mulroller_layout_mode_t;

/**
 * @brief 滚轮不透明度模式枚举
 */
enum
{
    MULROLLER_OPA_NULL,             /**<无不透明度变化*/
    MULROLLER_OPA_MID,              /**<只有中间元素不同*/
    MULROLLER_OPA_GRAD,             /**<越边缘不透明度越高*/
};
typedef uint8_t lv_mulroller_opa_mode_t;

/**
 * @brief 滚轮循环模式枚举
 */
enum
{
    MULROLLER_CIRCLE_NORMAL, /**< 普通模式（滚轮在选项末尾结束）。 */
    MULROLLER_CIRCLE_INFINITE, /**< 无限模式（滚轮可以永远滚动）。 */
};
typedef uint8_t lv_mulroller_circle_mode_t;

/**
 * @brief 滚轮对齐模式枚举
 */
enum
{
    MULROLLER_ALIGN_LEFT = 0x00,   /**< 滚轮元素位置在左侧。 */
    MULROLLER_ALIGN_RIGHT = 0x01,   /**< 滚轮元素位置在右侧。 */
    MULROLLER_ALIGN_TOP = 0x10,   /**< 滚轮元素位置在顶部。 */
    MULROLLER_ALIGN_BOTTOM = 0x11,   /**< 滚轮元素位置在底部。 */
    MULROLLER_ALIGN_CENTER = 0xff,   /**< 滚轮元素位置在中心。 */
};
typedef uint8_t lv_mulroller_align_t;

/**
 * @brief 滚轮方向枚举
 */
enum
{
    MULROLLER_DIR_HOR = 0x00,   /**< 滚轮可以水平拖动。*/
    MULROLLER_DIR_VER = 0x10,   /**< 滚轮可以垂直拖动。*/
};
typedef uint8_t lv_mulroller_dir_t;

/**
 * @brief 滚轮对象类型枚举
 */
enum
{
    MULROLLER_TYPE_LABEL,           /**< 元素对象类型为标签。 */
    MULROLLER_TYPE_IMG,         /**< 元素对象类型为图像。 */
#if LVSF_USE_IMGARRAY
    MULROLLER_TYPE_IMGARRAY,         /**< 元素对象类型为图像数组。 */
#endif
    MULROLLER_TYPE_MODULE,          /**< 元素对象类型为模块。 */
};
typedef uint8_t lv_mulroller_obj_type_t;

/**
 * @brief 滚轮属性表结构体
 */
typedef struct
{
    const char      id[17];         /**< ID字符串 */
    lv_coord_t      x;              /**< X坐标 */
    lv_coord_t      y;              /**< Y坐标 */
    lv_coord_t      w;              /**< 宽度 */
    lv_coord_t      h;              /**< 高度 */
    lv_coord_t      interval;       /**< 元素之间的间隔 */
    lv_mulroller_dir_t     dir;     /**< 方向 */
    lv_mulroller_obj_type_t obj_type;  /**< 元素的对象类型 */
} lv_mulroller_attr_t;

/**
 * @brief 滚轮元素结构体
 */
typedef struct
{
    lv_obj_t *bg_obj;               /**< 背景对象 */
    lv_obj_t *child;                /**< 子对象 */
#if LV_OBJ_SNAPSHOT
    lv_obj_t *snapshot;             /**< 快照对象 */
    char *anim_buf;                 /**< 动画缓冲区 */
#endif
    lv_event_cb_t child_event_cb;   /**< 子对象事件回调 */
    uint32_t color;                 /**< 颜色 */
    uint16_t zoom;                  /**< 缩放 */
    int16_t idx;                    /**< 索引 */
    int8_t pos_idx;                 /**< 位置索引 */
    uint8_t opa;                    /**< 不透明度 */
} lv_mulroller_ele_t;

/**
 * @brief 多滚轮控件数据结构
 */
typedef struct
{
    lv_obj_t bg;                    /**< 背景对象 */
    lv_obj_t *high_light_bg;        /**< 高亮背景对象 */

    uint32_t init : 1;                            /**< 滚轮已初始化 */
    uint32_t anim_running : 1;                    /**< 动画正在运行 */
    uint32_t high_light : 1;                      /**< 中间元素显示高亮轮廓 */
    uint32_t custom_font : 1;                     /**< 使用自定义字体，因此这里不需要设置字体 */
    uint32_t move_type : 2;                       /**< lv_mulroller_move_type */
    uint32_t layout_mode : 2;                     /**< lv_mulroller_layout_mode_t */
    uint32_t opa_mode : 2;                        /**< lv_mulroller_opa_mode_t */
    uint32_t anim_type : 3;                       /**< lv_mulroller_anim_type */
    uint32_t color_mode : 3;                      /**< lv_mulroller_color_mode_t */
    uint32_t circle_mode : 1;                     /**< lv_mulroller_circle_mode_t */
    uint32_t obj_type : 2;                        /**< lv_mulroller_obj_type_t */
    uint32_t dir : 5;                             /**< lv_mulroller_dir_t */
    uint32_t align : 8;                           /**< lv_mulroller_align_t */

    int16_t max_c_num;     /**< 最小圆圈数 */
    int16_t min_c_num;     /**< 最大圆圈数 */
    int16_t ori_mid_idx;    /**< 滚轮创建时的原始中间索引 */
    lv_obj_t *obj_lt;             /**< 自定义左或上对象，当滚轮移动到左侧或顶部时显示 */
    lv_obj_t *obj_rb;             /**< 自定义右或下对象，当滚轮移动到右侧或底部时显示 */

    uint8_t                     middle_opa;         /**< 中间部分的不透明度 */
    uint8_t                     lateral_opa;        /**< 侧边部分的不透明度 */
    uint16_t                    middle_zoom;        /**< 中间元素的缩放。如果对象类型是标签，该值是字体大小 */
    uint16_t                    lateral_zoom;       /**< 侧边元素的缩放。如果对象类型是标签，该值是字体大小 */
    uint32_t                    middle_color;       /**< 中间部分的颜色 */
    uint32_t                    lateral_color;      /**< 侧边部分的颜色 */

    lv_coord_t                  interval;           /**< 元素之间的间隔 */
    lv_coord_t                  boundary_lt;        /**< 左或上边界的坐标 */
    lv_coord_t                  boundary_rb;        /**< 右或下边界的坐标 */
    lv_coord_t                  ori_pos;            /**< 位置 */
    lv_coord_t                  align_pos;          /**< 中间元素的位置 */
    lv_coord_t                  ele_area;           /**< 元素区域 */
    lv_coord_t                  ele_size;           /**< 元素大小 */

    float throw_scale;              //每个投掷像素转换为距离的缩放比例
    float wheel_scale;              //每个轮子变化转换为距离的缩放比例

    lv_coord_t                      offset_lt;      /**< 左上角偏移 */
    lv_coord_t                      offset_rb;      /**< 右下角偏移 */

    lv_mulroller_appear_cb       middle_cb;      /**< 当元素移动到中间时调用该函数 */
    lv_mulroller_appear_cb       middle_cb2;      /**< 当元素移动时调用该函数 */
    lv_mulroller_appear_cb       appear_cb;      /**< 当元素移动到侧面时调用该函数 */

    /*set by custom*/
    uint8_t                     ori_pos_idx;        /**< 原始对齐元素的位置ID */
    uint8_t                     ele_num;        /**< 实际滚动元素数量 */
    uint8_t                     ele_lt_idx;       /**< 左或上元素的索引 */
    uint8_t                     ele_rb_idx;       /**< 右或下元素的索引 */
    lv_mulroller_ele_t *ele_tab;       /**< 指向滚动元素表的指针 */

    int16_t mov_dis;                                //范围： 0 - ele_area;
    int16_t mov_dis_offset;                         //用于居中对齐
    int16_t mov_dis_anim_start;                     //范围： 0 - ele_area;

    uint32_t anim_value;                            /**< 动画值 */

    lv_mulroller_anim_ready_cb start_anim_cb;       /**< 动画开始回调 */
    lv_mulroller_anim_ready_cb end_anim_cb;         /**< 动画结束回调 */
    lv_mulroller_anim_ready_cb align_anim_cb;       /**< 对齐动画回调 */

#if LVSF_USING_ENCODER != 0
    lv_obj_t *encoder;                /**< 编码器对象 */
    lv_timer_t *encoder_timer;        /**< 编码器定时器 */
#endif

    const lv_mulroller_attr_t *attr_addr;    /**< 属性地址 */
} lv_mulroller_t;

/**
 * @brief 创建多滚轮对象
 * @param parent 父对象指针
 * @return 返回创建的多滚轮对象指针
 * @note 该函数创建一个lv_mulroller对象并初始化
 */
lv_obj_t *lv_mulroller_create(lv_obj_t *parent);


/**
 * @brief 创建动画
 * @param mulroller 多滚轮对象指针
 * @param type 动画类型
 * @param num 动画数量
 * @param time 动画时间
 * @param cb 动画就绪回调函数
 * @return 返回动画创建结果
 * @note 该函数为多滚轮创建动画
 */
bool lv_mulroller_create_anim(lv_obj_t *mulroller, lv_mulroller_anim_type type, int16_t num, uint32_t time, lv_mulroller_anim_ready_cb cb);


/**
 * @brief 删除动画
 * @param mulroller 多滚轮对象指针
 * @note 该函数删除多滚轮的动画
 */
void lv_mulroller_del_anim(lv_obj_t *mulroller);


/**
 * @brief 设置移动百分比
 * @param mulroller 多滚轮对象指针
 * @param percent 百分比
 * @param param 参数
 * @note 该函数设置多滚轮的移动百分比
 */
void lv_mulroller_set_moving_percent(lv_obj_t *mulroller, int16_t percent, int16_t param);


/**
 * @brief 对齐所有元素
 * @param mulroller 多滚轮对象指针
 * @param bg_obj 背景对象
 * @note 该函数对齐多滚轮的所有元素
 */
void lv_mulroller_align_all_element(lv_obj_t *mulroller, lv_obj_t *bg_obj);


/**
 * @brief 通过索引获取背景对象
 * @param mulroller 多滚轮对象指针
 * @param idx 背景对象索引
 * @return 返回背景对象指针
 * @note 该函数通过索引获取多滚轮的背景对象
 */
lv_obj_t *lv_mulroller_get_bg_obj(lv_obj_t *mulroller, uint8_t idx);


/**
 * @brief 获取中间索引
 * @param mulroller 多滚轮对象指针
 * @return 返回中间索引
 * @note 该函数获取多滚轮的中间元素索引
 */
int16_t lv_mulroller_get_mid_idx(lv_obj_t *mulroller);


/**
 * @brief 设置多滚轮的原始中间索引
 * @param mulroller 多滚轮对象指针
 * @param idx 原始中间索引
 * @note 该函数设置多滚轮初始化后的原始中间索引
 */
void lv_mulroller_set_ori_mid_idx(lv_obj_t *mulroller, int16_t idx);


/**
 * @brief 设置多滚轮的方向
 * @param mulroller 多滚轮对象指针
 * @param dir 方向（MULROLLER_DIR_HOR或MULROLLER_DIR_VER）
 * @note 该函数设置多滚轮的拖动方向
 */
void lv_mulroller_set_dir(lv_obj_t *mulroller, lv_mulroller_dir_t dir);


/**
 * @brief 设置多滚轮的元素对象类型
 * @param mulroller 多滚轮对象指针
 * @param obj_type 对象类型（MULROLLER_TYPE_LABEL或MULROLLER_TYPE_IMG或MULROLLER_TYPE_MODULE）
 * @note 该函数设置多滚轮元素的对象类型
 */
void lv_mulroller_set_obj_type(lv_obj_t *mulroller, lv_mulroller_obj_type_t obj_type);


/**
 * @brief 设置多滚轮的颜色模式
 * @param mulroller 多滚轮对象指针
 * @param color_mode 颜色模式（目前仅支持MULROLLER_COLOR_POS）
 * @note 该函数设置多滚轮在移动时改变元素颜色的模式
 */
void lv_mulroller_set_color_mode(lv_obj_t *mulroller, lv_mulroller_color_mode_t color_mode);


/**
 * @brief 设置多滚轮的布局模式
 * @param mulroller 多滚轮对象指针
 * @param layout_mode 布局模式（目前仅支持MULROLLER_LAYOUT_OVERLAP、MULROLLER_LAYOUT_MID和MULROLLER_LAYOUT_RHOMB）
 * @note 该函数设置多滚轮的布局模式
 */
void lv_mulroller_set_layout_mode(lv_obj_t *mulroller, lv_mulroller_layout_mode_t layout_mode);


/**
 * @brief 设置多滚轮的不透明度模式
 * @param mulroller 多滚轮对象指针
 * @param opa_mode 不透明度模式（MULROLLER_OPA_MID、MULROLLER_OPA_GRAD或MULROLLER_OPA_NULL）
 * @note 该函数设置多滚轮在移动时改变元素不透明度的模式
 */
void lv_mulroller_set_opa_mode(lv_obj_t *mulroller, lv_mulroller_opa_mode_t opa_mode);


/**
 * @brief 设置多滚轮的循环模式
 * @param mulroller 多滚轮对象指针
 * @param circle_mode 循环模式（MULROLLER_CIRCLE_NORMAL或MULROLLER_CIRCLE_INFINITE）
 * @note 该函数设置多滚轮的循环模式
 */
void lv_mulroller_set_circle_mode(lv_obj_t *mulroller, lv_mulroller_circle_mode_t circle_mode);


/**
 * @brief 设置多滚轮的循环范围
 * @param mulroller 多滚轮对象指针
 * @param min 最小数据索引
 * @param max 最大数据索引
 * @note 该函数设置多滚轮的循环范围（仅在循环模式为MULROLLER_CIRCLE_NORMAL时使用）
 */
void lv_mulroller_set_circle_range(lv_obj_t *mulroller, int16_t min, int16_t max);


/**
 * @brief 设置多滚轮的不透明度范围
 * @param mulroller 多滚轮对象指针
 * @param middle_opa 中间元素的不透明度
 * @param lateral_opa 侧边元素的不透明度
 * @note 该函数设置多滚轮的不透明度范围（仅在对象类型为MULROLLER_TYPE_LABEL或MULROLLER_TYPE_IMG时使用）
 */
void lv_mulroller_set_opa_range(lv_obj_t *mulroller, uint8_t middle_opa, uint8_t lateral_opa);


/**
 * @brief 设置多滚轮的缩放范围
 * @param mulroller 多滚轮对象指针
 * @param middle_zoom 中间元素的缩放
 * @param lateral_zoom 侧边元素的缩放
 * @note 该函数设置多滚轮的缩放范围（仅在对象类型为MULROLLER_TYPE_LABEL或MULROLLER_TYPE_IMG时使用）
 */
void lv_mulroller_set_zoom_range(lv_obj_t *mulroller, uint16_t middle_zoom, uint16_t lateral_zoom);


/**
 * @brief 设置多滚轮的颜色范围
 * @param mulroller 多滚轮对象指针
 * @param middle_color 中间元素的颜色
 * @param lateral_color 侧边元素的颜色
 * @note 该函数设置多滚轮的颜色范围（仅在对象类型为MULROLLER_TYPE_LABEL时使用）
 */
void lv_mulroller_set_color_range(lv_obj_t *mulroller, uint32_t middle_color, uint32_t lateral_color);


/**
 * @brief 设置多滚轮的元素对齐方式
 * @param mulroller 多滚轮对象指针
 * @param align 对齐类型（见lv_mulroller_align_t枚举）
 * @note 该函数设置多滚轮的元素对齐方式
 */
void lv_mulroller_set_align(lv_obj_t *mulroller, lv_mulroller_align_t align);


/**
 * @brief 设置多滚轮的元素高亮
 * @param mulroller 多滚轮对象指针
 * @param high_light 高亮标志
 * @note 该函数设置多滚轮的中间元素是否显示高亮轮廓
 */
void lv_mulroller_set_high_light(lv_obj_t *mulroller, bool high_light);


/**
 * @brief 设置多滚轮的元素间隔
 * @param mulroller 多滚轮对象指针
 * @param interval 元素间隔
 * @note 该函数设置多滚轮的元素之间的间隔
 */
void lv_mulroller_set_interval(lv_obj_t *mulroller, lv_coord_t interval);


#if LV_OBJ_SNAPSHOT
/**
 * @brief 设置多滚轮的快照功能
 * @param mulroller 多滚轮对象指针
 * @param en 快照功能使能
 * @note 该函数设置多滚轮的快照功能
 */
void lv_mulroller_set_snapshot(lv_obj_t *mulroller, bool en);
#endif


/**
 * @brief 设置多滚轮的偏移
 * @param mulroller 多滚轮对象指针
 * @param offset_lt 左上角偏移
 * @param offset_rb 右下角偏移
 * @note 该函数设置多滚轮的偏移位置
 */
void lv_mulroller_set_offset(lv_obj_t *mulroller, lv_coord_t offset_lt, lv_coord_t offset_rb);


/**
 * @brief 设置多滚轮的轮子缩放比例
 * @param mulroller 多滚轮对象指针
 * @param wheel_scale 轮子缩放比例
 * @note 该函数设置多滚轮的轮子缩放比例
 */
void lv_mulroller_set_wheel_scale(lv_obj_t *mulroller, float wheel_scale);


/**
 * @brief 设置多滚轮的投掷缩放比例
 * @param mulroller 多滚轮对象指针
 * @param throw_scale 投掷缩放比例
 * @note 该函数设置多滚轮的投掷缩放比例
 */
void lv_mulroller_set_throw_scale(lv_obj_t *mulroller, float throw_scale);


/**
 * @brief 设置多滚轮的自定义对象
 * @param mulroller 多滚轮对象指针
 * @param obj_lt 左或上自定义对象
 * @param obj_rb 右或下自定义对象
 * @note 该函数设置多滚轮的自定义对象
 */
void lv_mulroller_set_custom_obj(lv_obj_t *mulroller, lv_obj_t *obj_lt, lv_obj_t *obj_rb);


/**
 * @brief 使用自定义字体
 * @param mulroller 多滚轮对象指针
 * @param en 使能标志（true：多滚轮不会为标签设置字体和大小）
 * @note 该函数设置多滚轮是否使用自定义字体（仅在对象类型为MULROLLER_TYPE_LABEL时使用）
 */
void lv_mulroller_set_custom_font(lv_obj_t *mulroller, bool en);


/**
 * @brief 设置中间回调函数
 * @param mulroller 多滚轮对象指针
 * @param middle_cb 回调函数
 * @note 该函数设置多滚轮停止移动时调用的回调函数，通知当前中间索引
 */
void lv_mulroller_set_middle_cb(lv_obj_t *mulroller, lv_mulroller_appear_cb middle_cb);


/**
 * @brief 设置中间回调函数2
 * @param mulroller 多滚轮对象指针
 * @param middle_cb 回调函数
 * @note 该函数设置多滚轮移动时调用的回调函数
 */
void lv_mulroller_set_middle_cb2(lv_obj_t *mulroller, lv_mulroller_appear_cb middle_cb);


/**
 * @brief 设置出现回调函数
 * @param mulroller 多滚轮对象指针
 * @param appear_lt_cb 回调函数
 * @note 该函数设置多滚轮移动时元素改变位置时调用的回调函数，通知新索引
 */
void lv_mulroller_set_appear_cb(lv_obj_t *mulroller, lv_mulroller_appear_cb appear_lt_cb);


/**
 * @brief 扩展区域
 * @param mulroller 多滚轮对象指针
 * @param size 扩展大小
 * @note 该函数扩展多滚轮的区域（目前无用）
 */
void lv_mulroller_extend_area(lv_obj_t *mulroller, lv_coord_t size);


/**
 * @brief 创建掩码
 * @param mulroller 多滚轮对象指针
 * @param dir 方向
 * @param img_src 图像源
 * @note 该函数为多滚轮创建掩码（目前无用）
 */
void lv_mulroller_create_mask(lv_obj_t *mulroller, lv_mulroller_dir_t dir, const void *img_src);


/**
 * @brief 创建多滚轮的元素
 * @param mulroller 多滚轮对象指针
 * @param ele_num 元素数量
 * @param w 元素宽度
 * @param h 元素高度
 * @note 该函数创建多滚轮的元素，元素对象类型由obj_type决定
 */
void lv_mulroller_create_element(lv_obj_t *mulroller, uint8_t ele_num, lv_coord_t w, lv_coord_t h);


/**
 * @brief 设置元素
 * @param mulroller 多滚轮对象指针
 * @param ele 元素对象
 * @param ele_idx 元素索引
 * @param data_idx 数据索引
 * @note 该函数设置多滚轮的元素（仅在obj_type为MULROLLER_TYPE_MODULE时使用）
 */
void lv_mulroller_set_element(lv_obj_t *mulroller, lv_obj_t *ele, uint8_t ele_idx, uint8_t data_idx);


/**
 * @brief 绑定属性
 * @param mulroller 多滚轮对象指针
 * @param attr 属性表
 * @note 该函数绑定多滚轮的属性表
 */
void lv_mulroller_bind_attr(lv_obj_t *mulroller, const lv_mulroller_attr_t *attr);


/**
 * @brief 传输刷新
 * @param mulroller 多滚轮对象指针
 * @note 该函数刷新多滚轮的传输状态
 */
void lv_mulroller_trans_refresh(lv_obj_t *mulroller);


/**
 * @brief 验证多滚轮
 * @param mulroller 多滚轮对象指针
 * @note 该函数在所有设置完成后验证多滚轮
 */
void lv_mulroller_validate(lv_obj_t *mulroller);


#if LVSF_USING_ENCODER != 0
/**
 * @brief 启用编码器
 * @param mulroller 多滚轮对象指针
 * @param en 使能标志
 * @note 该函数启用多滚轮的编码器功能
 */
void lv_mulroller_encoder_enable(lv_obj_t *mulroller, bool en);
#endif /* LVSF_USING_ENCODER != 0 */



/**********************
 *      MACROS
 **********************/
#endif /*LVSF_USING_MULROLLER*/

#ifdef __cplusplus
} /* extern "C" */
#endif


#endif /* _LVSF_MULROLLER_H */
