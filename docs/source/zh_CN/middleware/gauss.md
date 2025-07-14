# 高斯模糊
我们的高斯模糊使用快速算法，在模糊程度比较高的情况下效果会比较接近标准的高斯模糊。\
另外考虑到内存占用和计算量的问题，目前的高斯计算最小半径和步进是25像素，后续将继续优化。


```{note}
高斯模糊 会用到NNACC以及EPIC，注意错开使用。
```

## 使用示例
    

```c

static void gauss_done_cbk(void)
{
    rt_sem_release(asyn_blur_sem);

}

static void main(int argc, char **argv)
{
    uint16_t radius = 30;

    BlurDataType blur_in, blur_out;

    uassert_true_ret(output_buf != NULL);


    /*
        输入数据： 输入格式颜色没有特比要求，EPIC能识别的即可。
                   尺寸不能超过EPIC混叠的最大限制（一般是1024x1024像素)
    */
    blur_in.data = (uint8_t *) MAINMENU_RGB565.data;
    blur_in.color_mode = MAINMENU_RGB565.format;
    blur_in.width = MAINMENU_RGB565.width;
    blur_in.height = MAINMENU_RGB565.height;

    /**
        输出数据：尺寸，格式可以和输入不一样。
                  bufffer可以和输入数据共用一个。
                  
                  输出格式仅支持RGB颜色
    */
    blur_out.data = (uint8_t *) output_buf;
    blur_out.color_mode = EPIC_OUTPUT_RGB565;
    blur_out.width = OUTPUT_WIDTH;
    blur_out.height = OUTPUT_HEIGHT;




    /*初始化参数，申请中间内存等*/
    void *p_gauss = gauss_init(&blur_in, &blur_out, radius);
    if (p_gauss)
    {
        /*执行模糊算法*/
        if (RT_EOK == gauss_start_IT(p_gauss, gauss_done_cbk))
        {
            if(RT_EOK == rt_sem_take(asyn_blur_sem, rt_tick_from_millisecond(100)))
            {
                /*完成，显示到屏幕*/
                LOG_I("Draw to lcd");
                output_to_lcd(output_buf);
            }
        }

        /*释放内存*/
        gauss_deinit(p_gauss);
    }



}

```

