
# Sensor Adding Guide

## 1. Preparation
When a new sensor device needs to be added to the SiFli Technology SDK, the first step is to determine the sensor's functionality type and interface type. <br/>
From a functionality perspective, supported sensors include 6-axis (acceleration, angular velocity), temperature, pressure, geomagnetic, light sensing, GPS, motor, etc. From an interface perspective, the currently supported types include I2C, SPI, and UART. <br/>
Once the device's datasheet is obtained, first determine the interface used, ensuring with hardware that the necessary interfaces are enabled and routed through the pins. Additionally, verify that the voltage and frequency ranges match. Then, based on the functional descriptions in the datasheet, confirm with the user the interfaces and other information required. <br/>

## 2. Code Preparation

Currently, peripheral code is placed under _rtos/rthread/bsp/sifli/peripherals_. After confirming the interface and implementation plan, a new directory is added under this path. 
Additionally, the _rtos/rthread/bsp/sifli/peripherals/Kconfig_ file needs to be modified to include menuconfig items, such as sensor control macros, I2C/SPI interface configuration, pin settings for switches, power, and interrupts. <br/>
Once the Kconfig modifications are complete, implement the code in the newly added directory. The interface names, pin numbers, interrupt numbers, etc., used in the code can be obtained from the configuration in Kconfig. Finally, link the Kconfig switch macros in the SConscript file for the directory. <br/>
After completing the code and configuration, go to the project directory's menuconfig configuration page. Choose "Select board peripherals" to enter the peripheral configuration. Find the newly added module, enable it, and proceed with further configuration. <br/>

## 3. Interface Configuration

For a 6-axis sensor, the Kconfig configuration is as follows:
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
                bool "Enable AWT function"
                default y
                config LSM_USING_PEDO
                bool "Enable Pedometer function"
                default y
                config LSM6DSL_USE_FIFO
                bool "LSM6DSL use FIFO"
                default y
            endif    
    endif
```
The macro `SENSOR_USING_6D` is the main switch for enabling the 6D sensor in the system. Below, the supported sensor models will be listed, such as LSM6DSL, SC7A20, etc. It's recommended to enable only one sensor at a time to avoid interface conflicts. <br/>
For LSM6DSL, `ACC_USING_LSM6DSL` is the switch for this specific sensor. <br/>
`LSM6DSL_USING_I2C` defines the interface type. Since this sensor supports both I2C and SPI, the configuration must specify which interface is used. If the sensor only supports one interface, this configuration is unnecessary. <br/>
`LSM6DSL_BUS_NAME` is used to define the interface name, which RT-Thread's sensor uses to find the interface device. For more details, refer to the code implementation. <br/>
`LSM6DSL_INT_GPIO_BIT/LSM6DSL_INT2_GPIO_BIT` define the interrupt pins (if allocated in hardware). <br/>
`LSM_USING_AWT/LSM_USING_PEDO/LSM6DSL_USE_FIFO` correspond to specific functionalities of the sensor and are not explained here. <br/>

Example of using interface devices: <br/>
Through the `rt_i2c_bus_device_find` or `rt_device_find` functions, you can find the bus device, then call the corresponding read/write functions of the bus according to the interface mode to read and write to the device. The specific control commands for the device need to be implemented based on the device's address and register definitions from the datasheet. <br/>
```c
int LSM6DSL_I2C_Init()
{
    /* get i2c bus device */
    lsm6dsl_content.bus_handle = rt_i2c_bus_device_find(LSM6DSL_BUS_NAME);
    if (lsm6dsl_content.bus_handle)
    {
        LOG_D("Find i2c bus device %s
", LSM6DSL_BUS_NAME);
    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail
", LSM6DSL_BUS_NAME);
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

Example of using interrupt PINs: <br/>
Set the interrupt PIN to input mode using `rt_pin_mode`; 
Set the interrupt handler and conditions for triggering the interrupt using `rt_pin_attach_irq`, where interrupts can be triggered on rising edge `PIN_IRQ_MODE_RISING`, falling edge `PIN_IRQ_MODE_FALLING`, both edges `PIN_IRQ_MODE_RISING_FALLING`, or high/low level `PIN_IRQ_MODE_HIGH_LEVEL` / `PIN_IRQ_MODE_LOW_LEVEL`; <br/>
Enable or disable interrupts using `rt_pin_irq_enable`. The interrupt handler implementation depends on the specific sensor. <br/>
```c
int lsm6dsl_gpio_int_enable(void)
{
    struct rt_device_pin_mode m;

    // get pin device
    rt_device_t device = rt_device_find("pin");
    if (!device)
    {
        LOG_E("GPIO pin device not found at LSM6DSL
");
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

## 4. Register SENSOR Device

In the added device directory, create a file `sensor_xxx.c` to register the new sensor device to the RT-Thread sensor device list, so that upper-level code can directly control the corresponding sensor device through the sensor device. <br/>
This part mainly consists of two parts: device registration and control callback implementation. <br/>
The code example is for the BMP280 barometric and temperature sensor: <br/>
Fill in the device information, interface type, data range, etc., then register it to the sensor device list using the `rt_hw_sensor_register` function. The `sensor_ops` defines the operation callbacks. <br/>
The callback handles mainly include data reading, mode control, sleep/wake-up, etc. <br/>
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

## 5. Debug Method

After adding the code, you can perform basic functionality verification of the sensor device by adding a command-line function. <br/>

Create a command function `int cmd_xxx(int argc, char *argv[])`, and use `FINSH_FUNCTION_EXPORT_ALIAS` to register the function as a Finsh command. <br/>
The command function should include the following methods: <br/>
Sensor initialization— including interface initialization and device address checking; <br/>
Register read/write—use I2C/SPI interfaces to perform read and write operations to the SENSOR's internal registers. <br/>
Sensor functionality verification—this depends on the specific sensor type. For example, a pressure sensor should be able to get the correct pressure, a temperature sensor should get the temperature, and an accelerometer should retrieve velocity information. <br/>
The following is part of the command-line implementation for the BMP280 pressure sensor: <br/>

```c
#define DRV_BMP280_TEST

#ifdef DRV_BMP280_TEST
#include <string.h>

int cmd_bmpt(int argc, char *argv[])
{
    int32_t temp, pres, alti;
    if (argc < 2)
    {
        LOG_I("Invalid parameter!");
        return 1;
    }
    if (strcmp(argv[1], "-open") == 0)
    {
        uint8_t res = BMP280_Init();
        if (BMP280_RET_OK == res)
        {
            BMP280_open();
            LOG_I("Open bmp280 success");
        }
        else
            LOG_I("open bmp280 fail");
    }
    if (strcmp(argv[1], "-close") == 0)
    {
        BMP280_close();
        LOG_I("BMP280 closed");
    }
    if (strcmp(argv[1], "-r") == 0)
    {
        uint8_t rega = atoi(argv[2]) & 0xff;
        uint8_t value;
        BMP280_ReadReg(rega, 1, &value);
        LOG_I("Reg 0x%x value 0x%x", rega, value);
    }
    if (strcmp(argv[1], "-tpa") == 0)
    {
        temp = 0;
        pres = 0;
        alti = 0;
        BMP280_CalTemperatureAndPressureAndAltitude(&temp, &pres, &alti);
        LOG_I("Get temperature = %.1f", (float)temp / 10);
        LOG_I("Get pressure= %.2f", (float)pres / 100);
        LOG_I("Get altitude= %.2f", (float)alti / 100);
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
        LOG_I("Config BMP I2C speed to %d", bps);
    }

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_bmpt, __cmd_bmpt, Test driver bmp280);

#endif //DRV_BMP280_TEST

```


