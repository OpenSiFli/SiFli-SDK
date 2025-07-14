# 配置存储

## 介绍

  配置存储(`Shared Preferences`) 是Andorid平台上的一个轻量级的存储接口，是key-value形式的数据库，适用于应用的少量数据存储（如应用的配置信息）以及应用间共享配置信息。

- 内置信号量，线程安全
- 不支持中断内调用
- 支持整数、字符串、二进制数据块存取


## 使能模块

  menuconfig配置菜单路径：`SiFli Middleware->Enable share preference`
  
## Usage
  以下是闹钟应用获取和存储闹钟列表的示例：

```c

typedef struct{
    bool enable;           
    char   title[32];     //alarm name
    time_t time;          //time
    uint32 repeat;        //peroid repeat
}alarm_info_t;


void app_alarm_main(int argc, char **argv)
{
    rt_err_t res = RT_EOK;
    alarm_info_t alarm_list[16];

    int32_t list_len;

    /* Open an preference*/
    share_prefs_t *pref = share_prefs_open("alarm", SHAREPREFS_MODE_PRIVATE);

    /* Read alarm list*/
    list_len = share_prefs_get_int(pref, "list_len", -1);
    if(list_len > 0)
    {
        res = share_prefs_get_block(pref,"list", &alarm_list, list_len * sizeof(alarm_info_t));
        assert(res == SF_EOK);
    }

	/* User edit alarm*/
    ...


    /* Save alarm list*/
    if(list_len > 0)
    {
        res = share_prefs_set_block(pref,"list", &alarm_list, list_len * sizeof(alarm_info_t));
        assert(res == SF_EOK);
        res = share_prefs_set_int(pref, "list_len", list_len);
        assert(res == SF_EOK);
    }


    res = share_prefs_close(pref);
}
```



## API参考
[share_prefs.h](middleware-share_prefs)
