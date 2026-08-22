# LVGL FreeType 字体

SiFli SDK 在 LVGL v8 上集成了 FreeType 字体适配层和 LVSF 字体管理器。应用通常将完整的 TTF 文件作为普通文件存放在已挂载的 RT-Thread DFS 文件系统中，再调用 `lvsf_font_load_ex()` 按路径加载，无需先把字体转换为 C 数组。字体管理器按像素字号创建并缓存 `lv_font_t`，供 LVGL 控件和样式使用。

矢量字体适合需要多个字号、较大字符集或运行时更换字体的界面。与预生成点阵字体相比，FreeType 需要额外的字体解析、字形渲染和缓存内存。

- 主要头文件：_middleware/lvgl/lvsf/lvsf_font.h_、_middleware/lvgl/lvsf/lvsf_font_manager.h_
- 注册描述符：_middleware/lvgl/lvsf/lvsf_ft_reg.h_
- 实现目录：_middleware/lvgl/lvsf/_、_external/freetype/_

```{note}
本文介绍 `LV_USING_FREETYPE_ENGINE` 对应的 SiFli 字体接口，仅适用于 LVGL v8。LVGL 上游的 `LV_USE_FREETYPE` 使用 `lv_ft_font_init()` 等接口，不包含 LVSF 字体管理器，两套接口不要混用。
```

## 系统组成

| 组件 | 职责 |
| --- | --- |
| FreeType | 打开字体源、读取字形轮廓并生成灰度字形位图 |
| SiFli 字体适配层 | 将一个字体源和一个像素字号封装为 LVGL 的 `lv_font_t` |
| LVSF 字体管理器 | 注册字体条目，创建、缓存、选择和释放受管字体对象 |
| LVGL | 在控件、样式和主题中保存并使用 `lv_font_t` 指针 |

字体管理器先保存“字体条目”，再为每个“字体条目、像素字号”组合创建一个 `lv_font_t`。同一组合在缓存有效期间返回同一个指针；清理缓存或卸载操作实际释放该对象后，原指针失效。

应用从字体管理器取得的是借用指针。不要释放该指针，也不要对它调用 `lv_freetype_font_deinit()` 或 `lv_freetype_set_font_size()`。应用可以设置 `fallback`，但必须遵守后文的引用和生命周期要求。

### 字体来源

对于已经使用文件系统的应用，运行时加载完整 TTF 文件是最直接的方式。编译时打包和运行时内存注册用于不依赖文件系统或需要长期驻留字体数据的场景。

字体条目的注册时间与 FreeType 打开字体的方式是两个不同概念。静态注册的条目既可以引用固件数组，也可以引用固定文件路径；运行时注册的描述符也可以引用内存数据。

本文所称“编译时打包”只表示字体源随固件链接；字体轮廓不会预先转换成 LVGL 点阵字体。无论字体源来自固件、文件还是运行时内存，具体字号的 `lv_font_t` 都由字体管理器在运行时创建，字形再由 FreeType 解析和渲染。

| 使用场景 | 注册接口 | 创建字号对象 | 字体源的有效期 | 可否用 `lvsf_font_unload_ex()` 注销 |
| --- | --- | --- | --- | --- |
| 文件系统中的完整 TTF 文件 | `lvsf_font_load_ex()` | 注册时立即尝试指定字号 | 文件在卸载前持续可读 | 是，但要求该路径此前没有注册为不可卸载条目 |
| 延迟注册的 DFS 字体文件 | `lvsf_font_register()`，`external=1` | 首次查找时创建 | 文件在卸载前持续可读 | 是，但要求该路径此前没有注册为不可卸载条目 |
| 编译时打包的字体数组 | `LVSF_FREETYPE_FONT_REGISTER()` | 默认条目可能在启动时预创建常用字号，其余按需创建 | 整个固件生命周期 | 否，只能清理字号缓存 |
| 编译时注册的固定文件路径 | `LVSF_FREETYPE_FONT_REGISTER()`，`font_lib_size=0` | 默认条目可能在启动时预创建常用字号，其余按需创建 | 文件在整个运行期持续可读 | 否，只能清理字号缓存 |
| 运行时内存描述符 | `lvsf_font_register_lib()` | 首次查找时创建 | 描述符和数据在字体管理器使用期间持续有效 | 否，只能清理字号缓存 |

`font_lib_size` 决定 FreeType 如何打开字体：

- `font_lib_size > 0`：`font_lib_data` 指向内存中的字体数据；
- `font_lib_size == 0`：`font_lib_data` 被解释为文件路径。

该判断与条目是否能由 `lvsf_font_unload_ex()` 注销无关。是否可注销由注册接口创建的条目类型决定。

对 `lvsf_font_load_ex()` 和 `lvsf_font_register()` 这两个文件注册接口，路径是条目的复用键：同一路径再次注册时直接复用第一次建立的条目，不修改其查找名和是否可卸载等来源属性。`lvsf_font_register_lib()` 和链接段注册不按路径去重——前者按描述符指针或名称复用，后者只按描述符指针去重——用它们注册一个已按文件注册过的路径，会产生共享同一路径的第二个条目。因此，同一字体路径不应混用链接段注册、`lvsf_font_register_lib()` 和运行时文件注册接口。

```{warning}
当前管理器建立链接段条目或 `lvsf_font_register_lib()` 条目时，即使 `font_lib_size > 0`，也会按 C 字符串语义从 `font_lib_data` 复制内容到条目的 `path` 字段。所有内存字体缓冲区都必须包含完整、可信的 TrueType 数据，并且在有效范围内可以安全读取到 `'\0'`。不要注册未校验、截断或短生命周期的下载缓冲区。这是当前实现限制，不是 FreeType 对内存字体的要求。
```

## 配置和初始化

### 启用组件

在 `menuconfig` 中进入 `Third-Party Components -> LittlevGL2RTT -> LVGL configuration -> SiFli extend`，启用 FreeType 字体引擎。

```none
CONFIG_LV_USING_FREETYPE_ENGINE=y
CONFIG_LV_USE_USER_DATA=y
```

从文件系统加载字体时，还需要启用 RT-Thread DFS 和实际存储介质对应的文件系统：

```none
CONFIG_RT_USING_DFS=y
```

FreeType 本身不依赖 `RT_USING_DFS_ELMFAT`；只有使用 ELM FAT 文件系统的工程才需要启用该配置。

当前字体适配层只注册了 TrueType 轮廓驱动。该限制适用于文件字体和内存字体；文件扩展名不能用于判断字体是否可用。含 CFF 轮廓的 OpenType 字体需要先转换为 TrueType 轮廓。

```{warning}
本文所述实现要求使用 LVGL v8 的 FreeType 路径。`PKG_SCHRIFT` 或 `USING_VGLITE` 与 `LV_USING_FREETYPE_ENGINE` 同时启用时无法完成构建：`lv_freetype.c` 的实现主体被条件编译排除，而字体管理器仍会编译并引用这些接口。不要将这些配置与本文接口组合使用。
```

### 初始化

标准 SDK 启动流程通过 `gui_lib_init()` 初始化 FreeType 和字体管理器，应用通常不需要再次初始化。

如果工程绕过了标准组件初始化流程，可以调用并检查返回值：

```c
static int init_font_manager(void)
{
    return lvsf_font_manager_init(0);
}
```

参数为 0 时，管理器使用 SDK 提供的 FreeType 缓存容量。重复调用不会重建已经初始化的 FreeType 缓存。

字体管理器的全局条目、字号缓存和 FreeType FTC 缓存没有内部互斥保护。GUI 启动后，应在 LVGL 线程中串行执行字体注册、获取、顺序调整、缓存清理和卸载操作，不得与渲染并发修改字体状态。

## 加载和注册字体

### 从文件系统加载完整 TTF 文件

这是使用 LVSF FreeType 字体的常用方式。工程将完整的 TTF 文件作为普通文件部署到所选存储介质，并挂载对应的 RT-Thread DFS 文件系统；字体不需要转换为 C 数组，也不需要链接进应用固件。文件如何进入存储介质由工程的资源部署流程决定，LVSF 只要求调用接口时文件已经可以通过 DFS 路径读取。

`lvsf_font_load_ex()` 将 DFS 路径交给 FreeType 打开字体。路径不是 LVGL 文件系统的盘符路径。

```c
#include "lvsf/lvsf_font.h"
#include "lvsf/lvsf_font_manager.h"

#define APP_FONT_PATH "/data/fonts/MyFont.ttf"
#define APP_FONT_NAME "MyFont"

static int load_app_font(void)
{
    uint16_t sizes[] = {16, 24, 32, 0};

    if (lvsf_font_load_ex(APP_FONT_PATH, sizes) != 1)
    {
        return -1;
    }

    return lvsf_font_get(APP_FONT_NAME, 24) != NULL ? 0 : -1;
}
```

`sizes` 必须以 0 结尾。传入 `NULL` 时只创建 `FONT_NORMAL` 对应的字号；`{0}` 不包含有效字号，加载会失败。

`lvsf_font_load_ex()` 返回 1 表示至少有一个请求字号创建成功。路径无效、条目注册失败或所有请求字号都创建失败时返回 -1。批量加载成功不表示每个字号都可用，应用仍应获取并检查实际需要的字号。

管理器使用路径最后一段去掉最后一个扩展名后的内容作为查找名。例如，`/data/fonts/MyFont.v2.ttf` 对应 `MyFont.v2`。名称区分大小写；不同路径如果生成相同名称，后注册的条目会被拒绝。同一路径再次加载时会复用原条目，并保留第一次注册时确定的名称和来源属性。

FreeType 缓存可能在后续重新打开字体文件，因此不能在 `lvsf_font_load_ex()` 返回后立即删除或替换该文件，也不能卸载字体所在的文件系统。字体文件应保持可读，直到相关字体条目成功卸载。

### 可选：延迟注册文件

`lvsf_font_register()` 可以先注册文件路径和选择属性，首次查找时再验证文件。该接口可以为尚未注册的路径指定查找名，也可以在创建字号前配置条目的启用状态和优先级。

```c
static const lvsf_font_config_t app_font_config =
{
    .name = "AppBody",
    .path = "/data/fonts/MyFont.ttf",
    .external = 1,
    .enabled = 1,
    .priority = 10,
};

static int register_app_font(void)
{
    return lvsf_font_register(&app_font_config);
}
```

对于尚未注册的路径，`external=1` 创建可由 `lvsf_font_unload_ex()` 注销的运行时文件条目。如果路径已经存在，接口会复用原条目，并保留其名称和来源属性。`external=0` 不会创建新的内存字体，只能配置已经存在的条目。`priority=0` 还会把该条目设为当前默认条目。

`lvsf_font_register_batch()` 按数组顺序逐项注册，遇到第一个错误就停止，之前成功的条目不会回滚。需要原子更新时，应用必须自行检查并处理已注册的条目。

### 可选：编译时打包字体

没有文件系统，或者字体数据必须随固件长期驻留时，可以将完整的 TrueType 文件转换为只读 C 数组。该方式只是改变字体数据的存放位置；FreeType 仍在运行时解析字体并渲染字形。

生成的头文件声明数组和实际字节数：

```c
/* app_font_data.h */
#include <stdint.h>

#define APP_FONT_DATA_SIZE 12345U
extern const uint8_t app_font_data[APP_FONT_DATA_SIZE];
```

再定义字体库描述符并注册到 `app_font` 链接段：

```c
#include <stdint.h>
#include "lvsf/lvsf_ft_reg.h"
#include "app_font_data.h"

const lv_font_freetype_lib_dsc_t AppFont_lib =
{
    .font_lib_size = APP_FONT_DATA_SIZE,
    .font_lib_data = (const char *)app_font_data,
    .font_lib_name = "AppFont",
};

LVSF_FREETYPE_FONT_REGISTER(AppFont);
```

`LVSF_FREETYPE_FONT_REGISTER(AppFont)` 要求存在具有外部链接的 `AppFont_lib`。标准启动流程会扫描链接段并建立字体条目。当前管理器使用宏参数 `AppFont` 生成查找名，因此应用使用 `lvsf_font_get("AppFont", size)` 获取字体；描述符中的 `font_lib_name` 当前不参与名称查找。

链接段字体条目默认启用，优先级为 0。扫描链接段时遇到的第一个尚未注册的字体库描述符成为初始内置条目和默认条目。链接段中的字号记录不限制当前管理器按需创建其它像素字号。

固件数组和描述符必须在整个运行期保持有效。`lvsf_font_unload_ex()` 不注销链接段条目。如果不再需要该字体的任何已创建字号，应先替换这些字号的全部引用，再调用 `lvsf_font_clear_cache("AppFont")`。当前没有只清理单个字号的接口。

### 可选：运行时注册内存字体

如果字体数据已经长期保存在内存中，可以在运行时注册描述符，而不使用链接段宏：

```c
#include <stdint.h>
#include "lvsf/lvsf_ft_reg.h"
#include "lvsf/lvsf_font_manager.h"
#include "app_font_data.h"

static const lv_font_freetype_lib_dsc_t app_memory_font_lib =
{
    .font_lib_size = APP_FONT_DATA_SIZE,
    .font_lib_data = (const char *)app_font_data,
    .font_lib_name = "MemoryFont",
};

static int register_memory_font(void)
{
    return lvsf_font_register_lib(&app_memory_font_lib, "MemoryFont", 10);
}
```

`lvsf_font_register_lib()` 只注册条目，不立即验证字体数据。首次查找并选中该条目时，管理器才打开字体并创建所需的字号对象。

管理器不接管描述符和字体数据的所有权，也不复制其完整内容，只保存原始指针。描述符及其数据不得位于函数栈上，也不能在清理字号缓存后立即释放，因为字体条目仍然存在，后续获取可能再次访问这些指针。当前没有按名称注销此类条目的公开接口。

`name` 应使用未注册过的唯一名称。当前接口会按描述符指针或名称复用已有条目；如果与运行时文件条目重名，已有条目的来源属性可能被保留，后续获取可能仍按文件路径处理。

注册时，管理器会按 `priority` 重插当前条目：它位于当前列表中第一个优先级数值更大的条目之前。该操作不会对整个列表重新排序。传入 0 还会把该条目设为当前默认条目；如果不希望改变默认字体，应使用非零优先级。

`font_lib_size == 0` 时也可以用 `lvsf_font_register_lib()` 保存文件路径，但该条目仍不能由 `lvsf_font_unload_ex()` 注销。普通运行时文件应使用 `lvsf_font_load_ex()` 或前述延迟注册接口。

## 获取和使用字体

### 按名称获取

需要指定具体的已注册字体时，优先使用 `lvsf_font_get()`：

```c
static int set_label_font(lv_obj_t *label)
{
    lv_font_t *font = lvsf_font_get("AppFont", 24);

    if (font == NULL)
    {
        return -1;
    }

    lv_obj_set_style_text_font(label, font,
                               LV_PART_MAIN | LV_STATE_DEFAULT);
    return 0;
}
```

字体不存在、字体源无法打开、格式不受支持或字号对象创建失败时，函数返回 `NULL`。显式按名称获取不检查条目的启用状态。

字号参数为 0 时，管理器使用 `FONT_NORMAL`。为了让样式和内存预算清晰，应用代码宜传入明确的像素字号。

### 按字号自动选择

`lvsf_get_font_from_size(size)` 不指定字体名称。管理器按当前条目顺序遍历已启用字体，返回第一个能创建目标字号的字体：

```c
static lv_font_t *get_font_for_size(uint16_t size)
{
    return lvsf_get_font_from_size(size);
}
```

如果所有已启用字体都失败，而当前默认条目处于禁用状态，管理器还会尝试一次该默认条目。因此，禁用默认字体前应先切换默认条目。

默认条目、条目顺序和启用状态相互独立：

- `lvsf_font_set_default(name)` 改变 `lvsf_font_get("default", size)` 使用的条目，但不会自动启用该字体，也不会把它移到顺序首位；
- `lvsf_font_set_enable(name, 0)` 只影响自动选择中遍历已启用条目的阶段，不阻止 `lvsf_font_get(name, size)`；
- `lvsf_font_set_priority(name, priority)` 修改指定条目的优先级，并按当前列表重插该条目；它不会对整个列表重新排序；
- `lvsf_font_set_order(names, count)` 将指定字体按数组顺序移动到条目表前部。

```c
static void select_app_fonts(void)
{
    char *font_order[] = {"AppFont", "BackupFont"};

    lvsf_font_set_enable("AppFont", 1);
    lvsf_font_set_order(font_order, 2);
}
```

名称 `"default"` 只应作为 `lvsf_font_get()` 的特殊查找名使用。其它管理接口对该名称的处理不一致；设置默认字体、启用状态、优先级或清理缓存时，应传入字体的实际注册名。

`LV_EXT_FONT_GET()` 面向 `LVSF_FONT_SMALL` 至 `LVSF_FONT_SUPER` 字号档位。参数 0 至 6 会先转换为对应的预设像素字号，而不是按 0 至 6 像素处理。需要任意像素字号时，使用 `lvsf_get_font_from_size()` 或 `lvsf_font_get()`。

### 设置回退字体

当前字体缺少某个字形时，LVGL 会沿 `fallback` 指针继续查找：

```c
static int set_app_font_fallback(void)
{
    lv_font_t *primary = lvsf_font_get("AppFont", 24);
    lv_font_t *fallback = lvsf_font_get("BackupFont", 24);

    if (primary == NULL || fallback == NULL || primary == fallback ||
            fallback->fallback != NULL)
    {
        return -1;
    }

    primary->fallback = fallback;
    return 0;
}
```

回退链不得形成环。仅检查 `primary != fallback` 不能排除间接环；设置前还要确认目标字体现有的回退链不会回到 `primary`。上面的简化代码只接受自身没有回退字体的目标。`fallback` 不带引用计数，目标字体必须比引用它的字体存活更久。卸载或清理目标字体前，应先替换所有回退引用。

字体管理器在释放受管字体时会清除其它受管字体中指向它的 `fallback`。应用自行创建的字体不在管理器的扫描范围内，其回退引用必须由应用清理。

当整条回退链都没有目标字形时，`LV_USE_FONT_PLACEHOLDER` 决定是否显示占位方框；未启用时，该字符没有前进宽度。

## 字体对象、缓存和内存

### 对象生命周期

受管 `lv_font_t` 指针只在对应字号缓存存在期间有效。以下操作可能使指针失效：

- `lvsf_font_clear_cache()` 或 `lvsf_font_clear_all_cache()` 成功释放字号对象；
- `lvsf_font_unload_ex()` 成功卸载运行时文件字体；
- 字体管理器或 FreeType 引擎整体关闭。

不要长期保存字体指针而不管理其来源和卸载时机。控件、共享样式、主题、回退链和应用自己的绘制描述符都可能保存字体指针。

### 缓存层次

| 缓存 | 行为 | 配置或清理方式 |
| --- | --- | --- |
| 管理器字号缓存 | 每个“字体条目、像素字号”保存一个 `lv_font_t`，没有独立的数量上限 | 替换引用后调用 `lvsf_font_clear_cache()` 或卸载文件字体 |
| FTC Face/Size 缓存 | 同一字体源的多个字号共享 Face，Face 和 Size 数量受配置限制 | `LV_FREETYPE_CACHE_FT_FACES`、`LV_FREETYPE_CACHE_FT_SIZES` |
| FTC 字形和字符映射缓存 | 按总容量淘汰，需要时重新生成 | 使用 SDK 缓存容量；应用通常无需直接操作 |

持续请求新的像素字号会持续增加管理器字号对象。增大 FTC Size 数量不能限制这部分增长。

`lvsf_font_clear_cache(name)` 尝试清理一个条目的所有字号对象，`lvsf_font_clear_all_cache()` 尝试清理全部条目。启用引用检查时，仍被显示对象或主题引用的字号对象会保留；这两个函数没有返回值，应用不能据此确认缓存已经清空。

### 相关配置

| 配置项 | 默认值 | 作用 |
| --- | --- | --- |
| `FREETYPE_TINY_FONT` / `FREETYPE_NORMAL_FONT` | Tiny | Tiny 使用预编译 FreeType 库，Normal 从仓库源码构建 |
| `LV_FREETYPE_CACHE_FT_FACES` | 0 | FTC 保留的 Face 数量；0 使用 FreeType 默认值 2 |
| `LV_FREETYPE_CACHE_FT_SIZES` | 0 | FTC 保留的 Size 数量；0 使用 FreeType 默认值 4 |
| `LVSF_FONT_MIN_FREE_HEAP` | 0 | 创建运行时文件字体对象后要求保留的系统堆字节数；0 表示不检查 |
| `LVSF_FONT_UNLOAD_REF_CHECK` | y | 释放字号对象前检查显示对象和主题中的字体引用 |

`LVSF_FONT_MIN_FREE_HEAP` 只检查 `external=1` 的运行时文件条目，并且只在 FreeType 使用系统堆时生效。链接段条目、`lvsf_font_register_lib()` 条目以及使用独立 SRAM/PSRAM FreeType 内存池的配置不执行该检查。

该堆余量检查在字体打开后执行，用于拒绝会使系统堆低于下限的字号对象。部分管理器元数据分配仍使用断言，因此该配置不是通用的内存不足恢复机制。

## 切换和卸载运行时文件字体

`lvsf_font_unload_ex()` 只处理运行时文件条目。链接段字体和 `lvsf_font_register_lib()` 注册的条目不能通过该接口注销。

运行时文件字体必须在 LVGL 线程中按以下顺序卸载：

1. 停止所有按名称获取待卸载字体的代码路径；
2. 如果它是当前默认条目，先调用 `lvsf_font_set_default()` 切换到其它已注册且可用的字体；
3. 调用 `lvsf_font_set_enable(name, 0)`，将其排除在自动选择对已启用条目的遍历之外；
4. 替换控件、共享样式、主题、回退链和应用数据中的全部字体引用；
5. 如果使用 `lv_obj_del_async()` 删除对象，等待对象实际删除；
6. 使用完整字体路径调用 `lvsf_font_unload_ex()`；
7. 卸载成功后不再访问旧字体指针。

下面的代码只处理一个控件，并假设调用方已经切换默认条目、清理其它引用：

```c
static int unload_app_font(lv_obj_t *label)
{
    if (lvsf_font_set_enable(APP_FONT_NAME, 0) != 0)
    {
        return -1;
    }

    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT,
                               LV_PART_MAIN | LV_STATE_DEFAULT);

    return lvsf_font_unload_ex(APP_FONT_PATH);
}
```

`lvsf_font_unload_ex()` 返回 -1 表示至少有一个匹配字号对象仍被引用并被保留；返回 0 表示没有对象因引用而保留，但也可能没有匹配到任何条目。

该接口还接受字体注册名、目录前缀和 `NULL`；`NULL` 会匹配所有运行时文件条目。应用应传入完整路径，避免扩大卸载范围。

`LVSF_FONT_UNLOAD_REF_CHECK` 默认启用。它检查显示对象和主题中的字体引用，但不能覆盖未挂到控件上的样式、`lv_spangroup` 中各 span 自己持有的样式、应用保存的绘制描述符，以及管理器之外字体的 `fallback`。引用检查是释放前的补充检查，不能替代应用主动替换引用。

```{warning}
不要用 `lvsf_font_manager_deinit()` 或 `lvsf_font_unload()` 代替单个字体卸载。当前全局清理流程只固定保留第一个内置条目，可能移除其它链接段字体，而且不会在第一个内置条目仍存在时重新扫描这些条目。正常应用应让 SDK 启动和关闭流程管理整个字体引擎。
```

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| `lvsf_font_load_ex()` 返回 -1 | 文件系统是否挂载、路径是否可读、字体是否为 TrueType 轮廓、名称是否冲突、字号数组是否有效、系统堆是否满足下限 |
| 批量加载返回 1，但某个字号获取失败 | 返回 1 只表示至少一个字号成功；逐个调用 `lvsf_font_get()` 检查目标字号 |
| `lvsf_font_get()` 返回 `NULL` | 注册名和大小写是否正确、描述符和数据是否仍有效、字体源是否可打开、内存是否足够 |
| 字符显示为方框或缺失 | 当前字体和回退链都缺少该字形；同时检查 `LV_USE_FONT_PLACEHOLDER` |
| 部分字符大小或风格不同 | 缺失字符由回退字体绘制，两个字体的度量或字形风格不同 |
| 持续切换新字号后堆占用增加 | 管理器会为每个新字号保留字体对象；替换引用后清理字号缓存 |
| 自动选择得到非预期字体 | 同时检查条目顺序、启用状态和默认条目；设置默认条目不会自动改变顺序 |
| `lvsf_font_unload_ex()` 返回 -1 | 仍有显示对象、主题或样式引用该字体；按上文卸载步骤替换全部引用后重试 |

## 常用接口

字体管理器函数声明在 _lvsf_font.h_ 和 _lvsf_font_manager.h_ 中，链接段注册宏声明在 _lvsf_ft_reg.h_ 中。这些接口仅在 `LV_USING_FREETYPE_ENGINE` 启用时可用。

| 接口 | 用途 | 关键语义 |
| --- | --- | --- |
| `lvsf_font_manager_init(cache_size)` | 自定义启动流程中初始化管理器 | 成功返回 0；标准 SDK 启动通常不需要调用 |
| `lvsf_font_load_ex(path, sizes)` | 注册 DFS 文件并立即创建字号 | 至少一个字号成功返回 1；路径、注册或全部字号创建失败时返回 -1 |
| `LVSF_FREETYPE_FONT_REGISTER(name)` | 将字体描述符注册到链接段 | 要求 `name_lib` 具有外部链接 |
| `lvsf_font_register_lib(lib, name, priority)` | 运行时注册内存或描述符字体 | 延迟验证；描述符和数据必须长期有效 |
| `lvsf_font_register(config)` | 延迟注册或配置字体条目 | 对未注册路径，`external=1` 创建运行时文件条目 |
| `lvsf_font_register_batch(configs, count)` | 依次注册多个配置 | 失败时不回滚此前成功项 |
| `lvsf_font_get(name, size)` | 按名称获取字体 | 返回借用指针；不检查启用状态 |
| `lvsf_get_font_from_size(size)` | 按当前顺序自动选择字体 | 优先遍历已启用条目 |
| `lvsf_font_set_enable(name, enable)` | 控制已启用条目的遍历阶段是否包含该字体 | 不阻止按名称获取；禁用默认条目仍可能被重试 |
| `lvsf_font_set_default(name)` | 设置当前默认条目 | 不改变启用状态和条目顺序 |
| `lvsf_font_set_priority(name, priority)` | 修改并重插指定字体 | 不会对整个条目列表重新排序 |
| `lvsf_font_set_order(names, count)` | 将指定字体移动到条目表前部 | 按数组顺序排列，无返回值 |
| `lvsf_font_unload_ex(path)` | 卸载匹配的运行时文件字体 | 应传完整路径；无匹配也可能返回 0 |
| `lvsf_font_clear_cache(name)` | 尝试清理一个条目的字号对象 | 无返回值；仍被引用的对象可能保留 |
| `lvsf_font_clear_all_cache()` | 尝试清理所有字号对象 | 调用前先替换全部外部引用 |

应用应优先使用字体管理器，不直接调用 `lv_freetype_*` 底层接口。底层接口不会同步字体管理器保存的条目、字号键和对象生命周期。

## 参考资料

- 字体管理器实现：_middleware/lvgl/lvsf/lvsf_font_manager.c_
- 字体适配层实现：_middleware/lvgl/lvsf/lv_freetype.c_
- [LVGL v8 字体文档](https://docs.lvgl.io/8.3/overview/font.html)
- [FreeType 参考文档](https://freetype.org/freetype2/docs/reference/ft2-index.html)
