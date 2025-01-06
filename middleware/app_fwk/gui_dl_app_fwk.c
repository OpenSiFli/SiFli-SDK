#include <rtthread.h>

#ifdef DL_APP_SUPPORT

#include "gui_app_int.h"
#include <dfs_posix.h>
#ifdef RT_USING_MODULE
    #include "dlmodule.h"
    #include "dlfcn.h"
#endif
#include "log.h"


#define SYS_DL_APP_REG_FILE "/sys/dl_app.reg"
#define SYS_DL_APP_REG_TMP_FILE "/sys/dl_app.reg.tmp"

#define KEYWORD_ID  "id"
#define KEYWORD_DISPLAY_NAME  "display_name"
#define KEYWORD_ICON  "icon"
#define KEYWORD_EXE_FILE  "exe_file"

#if 1 //debug
    #define DL_APP_LOG LOG_I
    #define PRINT_SYS_REG_RECORD(r) DL_APP_LOG("{\nid=%s\ndir=%s\nname=%s\nicon=%s\nexe_file=%s\n}\n",(r)->id,(r)->dir,(r)->name,(r)->icon,(r)->exe_file)
#else
    #define DL_APP_LOG(...)
    define PRINT_SYS_REG_RECORD(r)
#endif
/**
 * copy file to a buffer
 * \n
 *
 * @return
 * \n
 * @see
 */
static char *create_file_buffer(const char *name)
{

    int fd, length = 0;
    char *p_file_buff = NULL;

    if (name)
    {
        fd = open(name, O_RDONLY, 0);

        if (fd >= 0)
        {
            length = lseek(fd, 0, SEEK_END);
            lseek(fd, 0, SEEK_SET);

            if (length > 0)
            {
                p_file_buff = (char *) rt_malloc(length + 1);
                if (p_file_buff)
                {
                    if (read(fd, p_file_buff, length) != length)
                    {
                        DL_APP_LOG("create_file_buffer failure,%d\n", length);

                        rt_free(p_file_buff);
                        p_file_buff = NULL;
                    }
                    else
                    {
                        p_file_buff[length] = '\0';
                    }
                }
            }

            close(fd);
        }
    }
    return p_file_buff;
}


static void destory_file_buffer(const char *p_buff)
{
    if (p_buff) rt_free((void *)p_buff);
}

/**
 * read data before comma from ppf_buff, meanwhile skip heading white space&comma
 * \n
 *
 * @param data
 * @param data_max_len
 * @return
 * @param ppf_buff
 * \n
 * @see
 */
static int sys_reg_table_read_1item_data(const char **ppf_buff, char *data, int data_max_len)
{
    int copy_len = 0;
    const char *ptr_fb = *ppf_buff;

    if ((NULL == ppf_buff) || (NULL == data) || (data_max_len < 2)) return 0;

    ptr_fb = *ppf_buff;
    if (NULL == ptr_fb) return 0;

    //skip heading white space and comma
    while ((' ' == *ptr_fb) || (',' == *ptr_fb) || ('\t' == *ptr_fb)) ptr_fb++;

    //copy data
    while ((*ptr_fb != '\0') && (*ptr_fb != '\r') && (*ptr_fb != '\n') && (*ptr_fb != ','))
    {
        if (copy_len < data_max_len - 1)
        {
            data[copy_len++] = *ptr_fb++;
        }
        else
        {
            //skip excess data
            ptr_fb++;
        }
    }

    //locate to next item data
    while ((',' == *ptr_fb) || ('\t' == *ptr_fb)) ptr_fb++;

    //set data string end
    data[copy_len] = '\0';

    *ppf_buff = ptr_fb;

    DL_APP_LOG("read_item:%s\n", data);
    return copy_len;
}


/**
 * read 1 registered_app_desc record form ppf_buff, meanwhile skip heading white space sysmbls
 * \n
 *
 * @param ppf_buff
 * @param record
 * @return
 * \n
 * @see
 */
static rt_err_t sys_reg_table_read_1record(const char **ppf_buff, dl_app_reg_desc_t *record)
{
    int r_len;
    const char *ptr_fb;
    rt_err_t rt_v = RT_EOK;

    if ((NULL == ppf_buff) || (NULL == record)) return RT_EINVAL;

    ptr_fb = *ppf_buff;
    if (NULL == ptr_fb) return RT_EINVAL;

    if ('\0' == *ptr_fb) return RT_EEMPTY;

    DL_APP_LOG("sys_reg_table_read_1record:\n{\n");

    // skip white space ahead of line
    while ((' ' == *ptr_fb) || ('\t' == *ptr_fb) || ('\r' == *ptr_fb) || ('\n' == *ptr_fb)) ptr_fb++;

    r_len = sys_reg_table_read_1item_data(&ptr_fb, record->id, sizeof(record->id));
    if (r_len <= 0) rt_v = RT_EEMPTY;
    r_len = sys_reg_table_read_1item_data(&ptr_fb, record->dir, sizeof(record->dir));
    if (r_len <= 0) rt_v = RT_EEMPTY;
    r_len = sys_reg_table_read_1item_data(&ptr_fb, record->name, sizeof(record->name));
    if (r_len <= 0) rt_v = RT_EEMPTY;
    r_len = sys_reg_table_read_1item_data(&ptr_fb, record->icon, sizeof(record->icon));
    if (r_len <= 0) rt_v = RT_EEMPTY;
    r_len = sys_reg_table_read_1item_data(&ptr_fb, record->exe_file, sizeof(record->exe_file));
    if (r_len <= 0) rt_v = RT_EEMPTY;

    // skip white space after line
    while ((' ' == *ptr_fb) || ('\t' == *ptr_fb) || ('\r' == *ptr_fb) || ('\n' == *ptr_fb)) ptr_fb++;
    DL_APP_LOG("\n}\n");

    *ppf_buff = ptr_fb;

    return rt_v;
}

/**
 * write record to fd
 * \n
 *
 * @param fd
 * @param record
 * @return
 * \n
 * @see
 */
static rt_err_t sys_reg_table_write_record(int fd, const dl_app_reg_desc_t *record)
{
    const char *write_buf;
    int write_len, result_len;
    DL_APP_LOG("sys_reg_table_write_record\n");

    if ((fd < 0) || (NULL == record)) return RT_EINVAL;

    write_buf = record->id;
    write_len = strlen(write_buf);
    result_len = write(fd, write_buf, write_len);
    if (result_len != write_len) return RT_EIO;
    if (1 != write(fd, ",", 1)) return RT_EIO;

    write_buf = record->dir;
    write_len = strlen(write_buf);
    result_len = write(fd, write_buf, write_len);
    if (result_len != write_len) return RT_EIO;
    if (1 != write(fd, ",", 1)) return RT_EIO;

    write_buf = record->name;
    write_len = strlen(write_buf);
    result_len = write(fd, write_buf, write_len);
    if (result_len != write_len) return RT_EIO;
    if (1 != write(fd, ",", 1)) return RT_EIO;

    write_buf = record->icon;
    write_len = strlen(write_buf);
    result_len = write(fd, write_buf, write_len);
    if (result_len != write_len) return RT_EIO;
    if (1 != write(fd, ",", 1)) return RT_EIO;

    write_buf = record->exe_file;
    write_len = strlen(write_buf);
    result_len = write(fd, write_buf, write_len);
    if (result_len != write_len) return RT_EIO;
    if (1 != write(fd, "\n", 1)) return RT_EIO;

    DL_APP_LOG("sys_reg_table_write_record OK\n");

    return RT_EOK;
}

static const char *sys_reg_table_find_record(const char **ppf_buff, const char *id)
{
    const char *ptr_ret;
    dl_app_reg_desc_t *record;

    if ((NULL == ppf_buff) || (NULL == id)) return NULL;

    DL_APP_LOG("sys_reg_table_find_record in file_buffer by id[%s]\n", id);

    if (NULL == *ppf_buff) return NULL;
    record = (dl_app_reg_desc_t *) rt_malloc(sizeof(dl_app_reg_desc_t));

    ptr_ret = NULL;
    while (**ppf_buff != '\0')
    {
        const char *ptr_tmp = *ppf_buff;

        if (RT_EOK == sys_reg_table_read_1record(ppf_buff, record))
        {
            if (0 == rt_strcmp(id, record->id))
            {
                DL_APP_LOG("record matched:\n");
                PRINT_SYS_REG_RECORD(record);
                ptr_ret = ptr_tmp;
                break;
            }
        }
    }

    if (record) rt_free(record);

    return ptr_ret;
}



/**
 * try to add a record in to sytem register file
 * \n
 *
 * @param record
 * @return
 * \n
 * @see
 */
static rt_err_t sys_reg_table_add_record(const dl_app_reg_desc_t *record)
{
    int fd, length = 0;
    const char *p_file_buff, *p_fb_head, *p_fb_end;
    rt_err_t rt_v = RT_EOK;

    DL_APP_LOG("sys_reg_table_add_record\n");

    p_fb_head = create_file_buffer(SYS_DL_APP_REG_FILE);
    p_file_buff = p_fb_head;
    if (NULL == p_file_buff)
    {
        DL_APP_LOG("First record, create new file\n");
        fd = open(SYS_DL_APP_REG_FILE, O_CREAT | O_WRONLY, 0);

        if (fd < 0)
        {
            DL_APP_LOG("create file failed:%s\n", SYS_DL_APP_REG_FILE);
            return RT_ERROR;
        }
        else
        {
            rt_v = sys_reg_table_write_record(fd, record);
        }
    }
    else
    {
        int fd_tmp;
        const char *p_dup_head, *p_dup_end;

        p_fb_end = p_fb_head + strlen(p_fb_head);

        p_dup_head = sys_reg_table_find_record(&p_file_buff, record->id);

        fd_tmp = open(SYS_DL_APP_REG_TMP_FILE, O_WRONLY | O_CREAT, 0);

        if (fd_tmp >= 0)
        {
            if (p_dup_head)
            {
                DL_APP_LOG("finded duplicated record\n");
                p_dup_end = p_file_buff;
            }
            else
            {
                p_dup_head = p_dup_end = p_fb_end;
                DL_APP_LOG("No duplicated record,append to the end of file\n");
            }

            do
            {
                int write_len, result_len;

                //write buffer before overwrite
                write_len = (p_dup_head - p_fb_head);
                if (write_len > 0)
                {
                    DL_APP_LOG("write records before\n");
                    result_len = write(fd_tmp, p_fb_head, write_len);
                    if (result_len != write_len)
                    {
                        rt_v = RT_EIO;
                        break;
                    }
                }

                DL_APP_LOG("overwrite old record\n");
                rt_v = sys_reg_table_write_record(fd_tmp, record);
                if (rt_v != RT_EOK) break;

                //write left buffer
                write_len = p_fb_end - p_dup_end;
                if (write_len > 0)
                {
                    DL_APP_LOG("write records behind\n");
                    result_len = write(fd_tmp, p_dup_end, write_len);
                    if (result_len != write_len)
                    {
                        rt_v = RT_EIO;
                        break;
                    }
                }
            }
            while (0);
            close(fd_tmp);


            if (unlink(SYS_DL_APP_REG_FILE) < 0)
            {
                DL_APP_LOG("delete file %s fail!\n", SYS_DL_APP_REG_FILE);
                rt_v = RT_EIO;
            }

            if (rename(SYS_DL_APP_REG_TMP_FILE, SYS_DL_APP_REG_FILE) < 0)
            {
                DL_APP_LOG("rename %s to %s fail\n", SYS_DL_APP_REG_TMP_FILE, SYS_DL_APP_REG_FILE);
                rt_v = RT_EIO;
            }
        }
        else
        {
            rt_v = RT_EIO;
        }
    }

_write_r_exit:
    if (p_fb_head) destory_file_buffer(p_fb_head);
    if (fd >= 0) close(fd);

    if (rt_v != RT_EOK)
        DL_APP_LOG("sys_reg_table_add_record error=%d\n", rt_v);

    return rt_v;
}

/**
 *
 * \n
 *
 * @param flags
 * @param p_app_desc
 * \n
 * @see
 */
static void *dl_app_open(dl_app_reg_desc_t *p_app_desc, int flags)
{
    char *fullpath;
    const char *def_path = p_app_desc->dir;
    struct rt_dlmodule *module;


    if (p_app_desc->exe_file[0] != '/') /* it's a relative path, prefix with MODULE_ROOT_DIR */
    {
        fullpath = rt_malloc(strlen(def_path) + strlen(p_app_desc->exe_file) + 2);

        /* join path and file name */
        rt_snprintf(fullpath, strlen(def_path) + strlen(p_app_desc->exe_file) + 2,
                    "%s/%s", def_path, p_app_desc->exe_file);
    }
    else
    {
        fullpath = (char *)p_app_desc->exe_file; /* absolute path, use it directly */
    }

#ifdef RT_USING_MODULE
    module = (struct rt_dlmodule *) dlopen(fullpath, flags);
#else
    module = NULL;
#endif

    if (fullpath != p_app_desc->exe_file)
    {
        rt_free(fullpath);
    }

    return (void *)module;

}


/**
 * open dl-app by id
 * \n
 *
 * @param flags
 * @param id
 * \n
 * @see
 */
void *gui_app_dlopen(const char *id, int flags)
{
    const char *p_file_buff, *p_fb_head;
    void *module_handler = NULL;
    dl_app_reg_desc_t *record;

    DL_APP_LOG("gui_app_dlopen %s,%d\n", id, flags);

    p_fb_head = create_file_buffer(SYS_DL_APP_REG_FILE);
    p_file_buff = p_fb_head;
    record = (dl_app_reg_desc_t *) rt_malloc(sizeof(dl_app_reg_desc_t));
    if (p_file_buff)
    {
        while (*p_file_buff != '\0')
        {
            if (RT_EOK == sys_reg_table_read_1record(&p_file_buff, record))
            {
                if (0 == rt_strcmp(id, record->id))
                {
                    DL_APP_LOG("find exe_file:%s/%s\n", record->dir, record->exe_file);
                    module_handler = dl_app_open(record, flags);
                    break;
                }
            }
        }
    }

    if (record) rt_free(record);
    if (p_fb_head) destory_file_buffer(p_fb_head);

    return module_handler;
}


void gui_app_dlclose(void *handle)
{
#ifdef RT_USING_MODULE
    dlclose(handle);
#endif
}

/**
 * read a valid equation like "keyword=aaa" form ppf_buff
 * \n
 *
 * @param keyword_buf
 * @param keyword_buf_len
 * @param ppf_buff
 * @return
 * @param value_buf
 * @param value_buf_len
 * \n
 * @see
 */
static rt_err_t dl_app_reg_read_next_cfg(const char **ppf_buff, char *keyword_buf, int keyword_buf_len,
        char *value_buf, int value_buf_len)
{
    rt_err_t rt_v = RT_EOK;
    int copy_len;
    const char *ptr_fb;

    if ((NULL == ppf_buff) || (NULL == keyword_buf) || (NULL == value_buf)
            || (keyword_buf_len < 2) || (value_buf_len < 2))

        return RT_EINVAL;

    ptr_fb = *ppf_buff;
    if (NULL == ptr_fb) return RT_EINVAL;

    // 1.skip white space ahead of line
    while ((' ' == *ptr_fb) || ('\t' == *ptr_fb) || ('\r' == *ptr_fb) || ('\n' == *ptr_fb)) ptr_fb++;


    // 2.read keyword
    copy_len = 0;
    while ((*ptr_fb != '\0') && (*ptr_fb != '\r') && (*ptr_fb != '\n') && (*ptr_fb != '='))
    {
        if (copy_len < keyword_buf_len - 1)
        {
            keyword_buf[copy_len++] = *ptr_fb++;
        }
        else
        {
            // keyword is too long
            return RT_ERROR;
        }
    }
    keyword_buf[copy_len] = '\0'; //set data string end

    // 3.check '=' symbol
    if (*ptr_fb != '=')
    {
        // not an '=' after keyword
        return RT_ERROR;
    }
    else
    {
        ptr_fb++;
    }

    // 4.read value
    copy_len = 0;
    while ((*ptr_fb != '\0') && (*ptr_fb != '\r') && (*ptr_fb != '\n'))
    {
        if (copy_len < value_buf_len - 1)
        {
            value_buf[copy_len++] = *ptr_fb++;
        }
        else
        {
            // keyword is too long
            return RT_ERROR;
        }
    }
    value_buf[copy_len] = '\0'; //set data string end


    *ppf_buff = ptr_fb;

    return rt_v;
}

/**
 * read a dl_app_reg_desc record from ppf_buff
 * \n
 *
 * @param ppf_buff
 * @param record
 * @return
 * \n
 * @see
 */
static rt_err_t dl_app_reg_read(const char **ppf_buff, dl_app_reg_desc_t *record)
{
    const char *ptr_fb;
    char keyword[32];
    char value[256];
    rt_err_t rt_v = RT_EOK;

    if ((NULL == ppf_buff) || (NULL == record)) return RT_EINVAL;

    ptr_fb = *ppf_buff;
    if (NULL == ptr_fb) return RT_EINVAL;

    DL_APP_LOG("dl_app_reg_read \n{\n");
    while (*ptr_fb != '\0')
    {
        int value_str_len;
        // read next cfg keyword & value
        if (RT_EOK != dl_app_reg_read_next_cfg(&ptr_fb, &keyword[0], sizeof(keyword), &value[0], sizeof(value)))
        {
            rt_v = RT_ERROR;
            continue;
        }

        value_str_len = strlen(&value[0]);
        //copy value to matched member
        if (0 == rt_strcmp(&keyword[0], KEYWORD_ID))
        {
            if (value_str_len + 1 > sizeof(record->id))
            {
                //too long value, report error
                rt_v = RT_ERROR;
                continue;
            }

            memcpy(record->id, &value[0], value_str_len + 1);
            DL_APP_LOG("id=%s\n", record->id);
        }
        else if (0 == rt_strcmp(&keyword[0], KEYWORD_DISPLAY_NAME))
        {
            if (value_str_len + 1 > sizeof(record->name))
            {
                //too long value, report error
                rt_v = RT_ERROR;
                continue;
            }

            memcpy(record->name, &value[0], value_str_len + 1);
            DL_APP_LOG("display name=%s\n", record->name);
        }
        else if (0 == rt_strcmp(&keyword[0], KEYWORD_ICON))
        {
            if (value_str_len + 1 > sizeof(record->icon))
            {
                //too long value, report error
                rt_v = RT_ERROR;
                continue;
            }

            memcpy(record->icon, &value[0], value_str_len + 1);
            DL_APP_LOG("icon=%s\n", record->icon);
        }
        else if (0 == rt_strcmp(&keyword[0], KEYWORD_EXE_FILE))
        {
            if (value_str_len + 1 > sizeof(record->exe_file))
            {
                //too long value, report error
                rt_v = RT_ERROR;
                continue;
            }

            memcpy(record->exe_file, &value[0], value_str_len + 1);
            DL_APP_LOG("exe_file=%s\n", record->exe_file);
        }
        else
        {
            DL_APP_LOG("unknow keyword=%s,value=%s\n", keyword, value);
        }
    }

    DL_APP_LOG("}\n");
    *ppf_buff = ptr_fb;

    return rt_v;
}

/**
 * register an dl-app by reg_file
 * \n
 *
 * @param reg_file - the path of directory which contains reg_file will be dl_app_reg_desc_t->dir
 * @return
 * \n
 * @see
 */
rt_err_t gui_dl_app_register(const char *reg_file)
{
    const char *p_file_buff, *p_fb_head;
    dl_app_reg_desc_t *record;
    rt_err_t rt_v = RT_EOK;

    if (!reg_file) return RT_EINVAL;

    DL_APP_LOG("gui_dl_app_register reg_file[%s]\n", reg_file);

    p_fb_head = create_file_buffer(reg_file);
    p_file_buff = p_fb_head;
    if (p_file_buff)
    {
        record = (dl_app_reg_desc_t *) rt_malloc(sizeof(dl_app_reg_desc_t));
        //decode register file to dl_app_reg_desc_t
        rt_v = dl_app_reg_read(&p_file_buff, record);
        if (RT_EOK == rt_v)
        {
            const char *ptr;
            int dir_strlen;
            //get register file direcory
            ptr = reg_file + strlen(reg_file);
            while (*ptr != '/') ptr--;
            dir_strlen = ptr - reg_file;

            // directory path too long, report error
            if (dir_strlen + 1 > sizeof(record->dir))
            {
                DL_APP_LOG("directory path too long: %s\n", reg_file);
                rt_v = RT_ERROR;
            }
            else
            {
                DL_APP_LOG("valid register file & directory path[");
                memcpy(record->dir, reg_file, dir_strlen);
                record->dir[dir_strlen] = '\0';
                DL_APP_LOG("%s]\n", record->dir);

                if (p_fb_head)
                {
                    destory_file_buffer(p_fb_head);
                    p_fb_head = NULL;
                }

                // add a record in system register table
                rt_v = sys_reg_table_add_record(record);

                if (RT_EOK == rt_v)
                    DL_APP_LOG("gui_dl_app_register %s DONE!\n", record->id);

            }
        }
        else
        {
            DL_APP_LOG("invalid register file\n");
        }
    }

    if (record) rt_free(record);

    if (p_fb_head) destory_file_buffer(p_fb_head);

    return rt_v;
}



const char *gui_dl_app_list_open(void)
{
    return create_file_buffer(SYS_DL_APP_REG_FILE);
}


rt_err_t gui_dl_app_list_get_next(const char **ppf_buff, dl_app_reg_desc_t *record)
{
    return sys_reg_table_read_1record(ppf_buff, record);
}

void gui_dl_app_list_close(const char *ptr_app)
{
    destory_file_buffer(ptr_app);
}

#ifdef FINSH_USING_MSH
#include <finsh.h>
static rt_err_t dlapp_register(int argc, char **argv)
{
#if 0
    char cmd_buf[FILE_PATH_MAX];
    rt_err_t res;
    int i, buf_len, str_len;

    if (argc < 2)
        return -RT_ERROR;

    buf_len = GUI_APP_CMD_MAX_LEN - 1;
    memset(cmd_buf, 0, sizeof(cmd_buf));

    for (i = 1; (i < argc) && (buf_len > 0); i++)
    {
        str_len = strlen(argv[i]) + 1/*1 space*/;
        if (buf_len < str_len)
        {
            break;
        }
        else
        {
            strcat(cmd_buf, " ");
            strcat(cmd_buf, argv[i]);
        }
        buf_len -= str_len;
    }
#endif


    return  gui_dl_app_register(argv[1]);
}
FINSH_FUNCTION_EXPORT(dlapp_register, register an dl - app);
MSH_CMD_EXPORT(dlapp_register, register an dl - app);

#endif

#endif /*DL_APP_SUPPORT*/
