# Sensor添加指南

## 1. 准备工作 
当有新的传感器设备需要添加到思澈科技的SDK中时，首先需要确定传感器的功能类型和接口类型。<br/>
从功能划分上看，可以支持的传感器包括6轴（加速度、角速度），温度，气压，地磁，感光，GPS, 马达等。从接口上看，目前支持的包括I2C, SPI, UART。<br/>
在拿到设备的规格书后，先确定使用的接口，需要和硬件确认可以使用的接口已经使能并由管脚引出，另外需要注意电压和频率范围是否匹配。之后再根据规格书中功能的相关的描述，与使用者确定需要提供的接口等信息。<br/>


## 2. 代码准备 

目前外设代码都放在 _rtos/rthread/bsp/sifli/peripherals_ 下，当确定好接口和实现方案后，在该目录下添加新的目录.
同时需要修改 _rtos/rthread/bsp/sifli/peripherals/Kconfig_ 文件，添加menuconfig 项，包括传感器的控制开关宏，I2C/SPI等接口配置，开关、电源、中断等PIN的设置。<br/>
KCONFIG修改完成后，在新加的目录中完成代码实现。在代码中使用的接口名，PIN号，中断号等都可以在KCONFIG中的配置中获取。最后在该目录的SConscript文件中添加与kconfig开关宏的关联。<br/>
代码和配置完成后，在对应工程目录下menuconfig进入配置页面后，选择”Select board peripherals”,进入外设配置，找到新添加的模块，使能后进入下一级进行具体配置。<br/>

## 3.接口配置

以6轴传感器为例，Kconfig 中配置如下：
```c
menuconfig SENSOR_USING_6D
    bool "Enable 6D Sensor for Accelerator and Gyro"
	default n
    if SENSOR_USING_6D
        menuconfig ACC_USING_LSM6DSL
            bool "Enable Accelerator and Gyro LSM6DSL"
            select RT_USING_SENSOR
            default n
            if ACC_USING_LSM6DSL
                config LSM6DSL_USING_I2C
                int "LSM6DSL BUS type: 1 = I2C, 0 = SPI"
                default 0
                config LSM6DSL_BUS_NAME
                string "Sensor LSM6DSL BUS name"
                default "spi1"
		
                config LSM6DSL_INT_GPIO_BIT
                int "LSM6DSL Interrupt 1 PIN"
                default 97
                config LSM6DSL_INT2_GPIO_BIT
                int "LSM6DSL Interrupt 2 PIN"
                default 94

                config LSM_USING_AWT
                bool "Enable AWT fucntion"
                default y
                config LSM_USING_PEDO
                bool "Enable Pedometer fucntion"
                default y
		config LSM6DSL_UES_FIFO
                bool "LSM6DSL use fifo"
                default y
            endif	
    endif
```
其中宏SENSOR_USING_6D 作为系统是否使能6轴传感器的总开关，下面会列出支持的不同型号的传感器，包括LSM6DSL, SC7A20等，同一时间，最好只使能一个传感器，以免对外接口出现冲突。<br/>
以LSM6DSL为例，ACC_USING_LSM6DSL 为该传感器的开关; <br/>
LSM6DSL_USING_I2C 定义了接口类型，因为次传感器可以支持I2C接口，也可以支持SPI接口，所以需要在配置中定义好，如果传感器只支持一种接口，则没必要加这一项配置；<br/>
LSM6DSL_BUS_NAME 用来定义接口名字，该名字在RT-Thread sensor使用时，用来通过查找接口设备，具体使用参考代码实现；<br/>
LSM6DSL_INT_GPIO_BIT/LSM6DSL_INT2_GPIO_BIT 用来定义中断管脚（如果硬件有分配）； <br/>
LSM_USING_AWT/LSM_USING_PEDO/LSM6DSL_USE_FIFO 具体功能与传感器有关，这里不做介绍。<br/>
    
接口设备名使用示例：<br/>
通过`rt_i2c_bus_device_find`或者`rt_device_find`函数，可以查找到BUS设备，之后按照接口模式，分别调用BUS对应的读写函数，实现对设备的读写函数。而具体的设备的控制命令，需要参考设备规格书的设备地址，寄存器定义来实现。<br/>
```c
int LSM6DSL_I2C_Init()
{
    /* get i2c bus device */
    lsm6dsl_content.bus_handle = rt_i2c_bus_device_find(LSM6DSL_BUS_NAME);
    if (lsm6dsl_content.bus_handle)
    {
        LOG_D("Find i2c bus device %s\n", LSM6DSL_BUS_NAME);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", LSM6DSL_BUS_NAME);
        return -1;
    }

    return 0;
}

int32_t LSM_I2C_Write(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_size_t res;

    struct LSM6DSL_CONT_T *handle = (struct LSM6DSL_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
        uint16_t addr16 = (uint16_t)reg;
        res = rt_i2c_mem_write(handle->bus_handle, handle->dev_addr, addr16, 8, data, len);
        if (res > 0)
            return 0;
        else
            return -2;
    }

    return -3;
}

int32_t LSM_I2C_Read(void *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_size_t res;
    struct LSM6DSL_CONT_T *handle = (struct LSM6DSL_CONT_T *)ctx;

    if (handle && handle->bus_handle && data)
    {
        uint16_t addr16 = (uint16_t)reg;
        res = rt_i2c_mem_read(handle->bus_handle, handle->dev_addr, addr16, 8, data, len);
        if (res > 0)
            return 0;
        else
            return -2;
    }

    return -3;
}

```

中断PIN使用示例：<br/>
将中断PIN设置为输入模式`rt_pin_mode`； 
设置中断处理函数和中断产生的条件`rt_pin_attach_irq`，其中中断可以通过上升沿`PIN_IRQ_MODE_RISING`、下降沿`PIN_IRQ_MODE_FALLING`、双沿`PIN_IRQ_MODE_RISING_FALLING`以及高低电平产生`PIN_IRQ_MODE_HIGH_LEVEL` / `PIN_IRQ_MODE_LOW_LEVEL`；<br/>
使能或者关闭中断`rt_pin_irq_enable`。 中断处理函数的实现与具体传感器有关。<br/>
```c
int lsm6dsl_gpio_int_enable(void)
{
    struct rt_device_pin_mode m;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at LSM6DSL\n");
        return -1;
    }

    rt_device_open(device, RT_DEVICE_OFLAG_RDWR);

    // int pin cfg
    m.pin = LSM6DSL_INT_GPIO_BIT;
    m.mode = PIN_MODE_INPUT;
    rt_device_control(device, 0, &m);

    // enable LSM int
    rt_pin_mode(LSM6DSL_INT_GPIO_BIT, PIN_MODE_INPUT);
    rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING, lsm6dsl_int1_handle, (void *)(rt_uint32_t)m.pin);
    rt_pin_irq_enable(m.pin, 1);

    return 0;
}


```

## 4.注册SENSOR设备

在添加的设备目录中，添加文件sensor_xxx.c， 用来将新添加的传感器设备注册到RT-Thread 中的sensor 设备中，以便直接上层代码可以直接通过sensor device 控制对应的传感器设备。<br/>
这部分代码主要分两部分， 设备的注册以及控制回调实现：<br/>
下面代码以BMP280 气压与温度传感器为例：<br/>
填写设备基本信息，接口类型，数据范围等，之后通过rt_hw_sensor_register函数注册到sensro设备列表中，其中sensor_ops为操作回调函数定义。<br/>
设备的回调处理，主要包括数据读取，模式控制， 睡眠与唤醒 等。<br/>
```c

int rt_hw_bmp280_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_temp = RT_NULL, sensor_baro = RT_NULL;

    result = _bmp280_init();
    if (result != RT_EOK)
    {
        LOG_E("bmp280 init err code: %d", result);
        goto __exit;
    }

    /* temperature sensor register */
    {
        sensor_temp = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_temp == RT_NULL)
            return -1;

        sensor_temp->info.type       = RT_SENSOR_CLASS_TEMP;
        sensor_temp->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_temp->info.model      = "bmp280_temp";
        sensor_temp->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
        sensor_temp->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_temp->info.range_max  = 85;
        sensor_temp->info.range_min  = -40;
        sensor_temp->info.period_min = 5;

        rt_memcpy(&sensor_temp->config, cfg, sizeof(struct rt_sensor_config));
        sensor_temp->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_temp, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    /* barometer sensor register */
    {
        sensor_baro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_baro == RT_NULL)
            goto __exit;

        sensor_baro->info.type       = RT_SENSOR_CLASS_BARO;
        sensor_baro->info.vendor     = RT_SENSOR_VENDOR_BOSCH;
        sensor_baro->info.model      = "bmp280_bora";
        sensor_baro->info.unit       = RT_SENSOR_UNIT_PA;
        sensor_baro->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_baro->info.range_max  = 110000;
        sensor_baro->info.range_min  = 30000;
        sensor_baro->info.period_min = 5;

        rt_memcpy(&sensor_baro->config, cfg, sizeof(struct rt_sensor_config));
        sensor_baro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_baro, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (sensor_temp)
        rt_free(sensor_temp);
    if (sensor_baro)
        rt_free(sensor_baro);
    if (bmp_dev)
        rt_free(bmp_dev);
    return -RT_ERROR;
}

```

## 5.DEBUG 方法

在代码添加完成后，可以通过添加命令行的方式，进行sensor设备的基本功能验证。<br/>

添加 命令行函数`int cmd_xxx(int argc, char *argv[])`; 然后使用`FINSH_FUNCTION_EXPORT_ALIAS` 方法，将函数注册成finsh命令。<br/>
在命令行函数中应该包含一下几个方法：<br/>
Sensor 初始化—包括 接口初始化，设备地址检查；<br/>
寄存器读写—通过I2C/SPI等接口，实现SENSOR内部寄存器的读写操作。<br/>
Sensor 相关功能验证，这个与具体sensor的类型有关，比如气压传感器要能获取准确的气压，温度传感器获取温度，加速度传感器获取速度信息等。<br/>
下面是气压传感器BMP280 的命令行实现部分代码：<br/>

```c
#define DRV_BMP280_TEST

#ifdef DRV_BMP280_TEST
#include <string.h>

int cmd_bmpt(int argc, char *argv[])
{
    int32_t temp, pres, alti;
    if (argc < 2)
    {
        LOG_I("Invalid parameter!\n");
        return 1;
    }
    if (strcmp(argv[1], "-open") == 0)
    {
        uint8_t res = BMP280_Init();
        if (BMP280_RET_OK == res)
        {
            BMP280_open();
            LOG_I("Open bmp280 success\n");
        }
        else
            LOG_I("open bmp280 fail\n");
    }
    if (strcmp(argv[1], "-close") == 0)
    {
        BMP280_close();
        LOG_I("BMP280 closed\n");
    }
    if (strcmp(argv[1], "-r") == 0)
    {
        uint8_t rega = atoi(argv[2]) & 0xff;
        uint8_t value;
        BMP280_ReadReg(rega, 1, &value);
        LOG_I("Reg 0x%x value 0x%x\n", rega, value);
    }
    if (strcmp(argv[1], "-tpa") == 0)
    {
        temp = 0;
        pres = 0;
        alti = 0;
        BMP280_CalTemperatureAndPressureAndAltitude(&temp, &pres, &alti);
        LOG_I("Get temperature = %.1f\n", (float)temp / 10);
        LOG_I("Get pressure= %.2f\n", (float)pres / 100);
        LOG_I("Get altitude= %.2f\n", (float)alti / 100);
    }
    if (strcmp(argv[1], "-bps") == 0)
    {
        struct rt_i2c_configuration cfg;
        int bps = atoi(argv[2]);
        cfg.addr = 0;
        cfg.max_hz = bps;
        cfg.mode = 0;
        cfg.timeout = 5000;
        rt_i2c_configure(i2cbus, &cfg);
        LOG_I("Config BMP I2C speed to %d\n", bps);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_bmpt, __cmd_bmpt, Test driver bmp280);

#endif //DRV_BMP280_TEST

```


   

