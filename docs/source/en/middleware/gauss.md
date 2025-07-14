
# Gaussian Blur

Our Gaussian blur uses a fast algorithm, which performs closer to standard Gaussian blur when the blur degree is relatively high. Additionally, considering memory usage and computational load, the current Gaussian blur calculation has a minimum radius and step of 25 pixels. Further optimization will be carried out in the future.

```{note}
Gaussian blur utilizes NNACC and EPIC, so make sure to use them alternately.
```

## Usage Example

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
        Input Data: The color format is not particularly specified, as long as it's recognized by EPIC.
                    The size should not exceed the maximum aliasing limit of EPIC (usually 1024x1024 pixels).
    */
    blur_in.data = (uint8_t *) MAINMENU_RGB565.data;
    blur_in.color_mode = MAINMENU_RGB565.format;
    blur_in.width = MAINMENU_RGB565.width;
    blur_in.height = MAINMENU_RGB565.height;

    /**
        Output Data: The size and format can be different from the input.
                     The buffer can be shared with the input data.

                     The output format only supports RGB color.
    */
    blur_out.data = (uint8_t *) output_buf;
    blur_out.color_mode = EPIC_OUTPUT_RGB565;
    blur_out.width = OUTPUT_WIDTH;
    blur_out.height = OUTPUT_HEIGHT;

    /* Initialize parameters and allocate intermediate memory */
    void *p_gauss = gauss_init(&blur_in, &blur_out, radius);
    if (p_gauss)
    {
        /* Execute the blur algorithm */
        if (RT_EOK == gauss_start_IT(p_gauss, gauss_done_cbk))
        {
            if(RT_EOK == rt_sem_take(asyn_blur_sem, rt_tick_from_millisecond(100)))
            {
                /* Completed, display on the screen */
                LOG_I("Draw to lcd");
                output_to_lcd(output_buf);
            }
        }

        /* Release memory */
        gauss_deinit(p_gauss);
    }
}
```

