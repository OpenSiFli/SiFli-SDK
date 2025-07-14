
# Configuration Storage

## Introduction

Configuration Storage (`Shared Preferences`) is a lightweight storage interface on the Android platform. It is a key-value style database suitable for storing small amounts of application data (such as configuration information) and shared configuration information between applications.

- Built-in semaphore, thread-safe
- Does not support interrupt calls
- Supports integer, string, and binary data block access

## Enable the Module

The configuration menu path in `menuconfig` is: `SiFli Middleware -> Enable share preference`

## Usage
Below is an example of the alarm application obtaining and storing the alarm list:

```c
typedef struct{
    bool enable;           
    char   title[32];     // Alarm name
    time_t time;          // Time
    uint32 repeat;        // Period repeat
} alarm_info_t;

void app_alarm_main(int argc, char **argv)
{
    rt_err_t res = RT_EOK;
    alarm_info_t alarm_list[16];

    int32_t list_len;

    /* Open a preference */
    share_prefs_t *pref = share_prefs_open("alarm", SHAREPREFS_MODE_PRIVATE);

    /* Read alarm list */
    list_len = share_prefs_get_int(pref, "list_len", -1);
    if (list_len > 0)
    {
        res = share_prefs_get_block(pref, "list", &alarm_list, list_len * sizeof(alarm_info_t));
        assert(res == SF_EOK);
    }

    /* User edits alarm */
    ...

    /* Save alarm list */
    if (list_len > 0)
    {
        res = share_prefs_set_block(pref, "list", &alarm_list, list_len * sizeof(alarm_info_t));
        assert(res == SF_EOK);
        res = share_prefs_set_int(pref, "list_len", list_len);
        assert(res == SF_EOK);
    }

    res = share_prefs_close(pref);
}
```

## API Reference
[share_prefs.h](middleware-share_prefs)
