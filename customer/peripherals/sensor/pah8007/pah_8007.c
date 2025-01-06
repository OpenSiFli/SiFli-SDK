
// PAH8007 Function Support
#include "pah_8007.h"

// PAH8007 Algorithm
#include "pxialg.h"
#ifdef V30_ALG_STATIC_RAM
    #include "pxivar.h"
#else
    #ifdef V30_ALG_DYNAMIC_RAM
        #include "pxivar.h"
    #endif
#endif

typedef struct
{
    sRaw_Data_13Bytes hr_data;
    float MEMS_Data[3] ;
    pah_8007_ppg_data ppg_data;
    pah_8007_touch_data touch_data;
} ppg_mems_data_t;

#define FIFO_SIZE 8  //FIFO Size is dependent on the CPU performance
static ppg_mems_data_t hr_data_buf[FIFO_SIZE];
static int _read_index = 0 ;
static int _write_index = 0 ;
static ppg_mems_data_t ppg_mems_data_alg;

//============================================================================
// Private function
//============================================================================
// Power
pah_ret     pah_8007_power_toggle(pah_8007 *state, bool enable);
////////////////////////////////////////////////////////////////////////////////
static bool sensor_basic_verification(void);
static void process(uint64_t timestamp);
static void ppg_sensor_enable(void);
static void ppg_sensor_disable(void);
static void touch_sensor_enable(void);
static void touch_sensor_disable(void);
static void hr_algorithm_enable(void);
static void hr_algorithm_disable(void);
static void hr_algorithm_update(const pah_8007_ppg_data *ppg_data, uint8_t ppg_duration, const pah_8007_touch_data *touch_data);
static void report_touch_data(uint64_t timestamp, const pah_8007_touch_data *touch_data);
static void report_hr_data(const pah_8007_ppg_data *ppg_data, uint8_t ppg_duration, const pah_8007_touch_data *touch_data, int32_t ret, float hr, float *MEMS_Data, uint8_t ppg_frame_count);

static main_state   _main;

static float hr = 0.0f;
static int hr_update_count = 0;
void *hr_pBuffer = NULL;
#ifdef V30_ALG_STATIC_RAM
uint8_t hr_static_ram[7 * 1024] = {0};      // create an array space for HR algorithm
#endif
static uint8_t data = 0;

//============================================================================
// Private function
//============================================================================
static pah_ret ppg_convergence_toggle(pah_8007 *state, bool enable);
static pah_ret interrupt_configuration(pah_8007 *state);
static pah_ret ppg_reset_exposure(void);
static pah_ret touch_process(pah_8007 *state, pah_8007_touch_data *touch_result);
static pah_ret ppg_process(pah_8007 *state, pah_8007_ppg_data *ppg_result);
static pah_ret clear_interrupt(pah_8007 *state, bool has_ppg_interrupt, bool has_overflow);
static pah_ret ppg_converging(pah_8007 *state, uint16_t ppg_value);

static bool isFIFOEmpty(void);
static bool Push(ppg_mems_data_t *data); //Push return data must be true. If return false, please increase FIFO size.
static bool Pop(ppg_mems_data_t *data);

static void pah_8007_SWRESET(void);

//============================================================================
// Public function (driver)
//============================================================================
static void comm_write(uint8_t addr, uint8_t data)
{
    pah_comm_i2c_write(addr, &data);
}
#define comm_read( addr, data)                     pah_comm_i2c_read( addr, data, 1)
#define comm_burst_read( addr, data, read_size)    pah_comm_i2c_read( addr, data, read_size)

/*****************************
PPG Function
*****************************/
bool pah_sensor_init(void)
{
    memset(&_main, 0, sizeof(_main));

    if (!sensor_basic_verification())       // verify Sensor ID
    {
        return false;
    }

    DEBUG_PRINT("enable_ppg_convergence = %d\n", ENABLE_PPG_CONVERGENCE);
    if (ENABLE_PPG_CONVERGENCE)
    {
        DEBUG_PRINT("(t_weight_normal_mode, t_weight_fast_mode) = (%d, %d)\n", PPG_CONVERGENCE_T_WEIGHT_NORMAL_MODE, PPG_CONVERGENCE_T_WEIGHT_FAST_MODE);
        DEBUG_PRINT("(t_min, t_max) = (%d, %d)\n", PPG_CONVERGENCE_T_MIN, PPG_CONVERGENCE_T_MAX);
        DEBUG_PRINT("(raw_outer_high_bound, raw_inner_high_target) = (%d, %d)\n", PPG_CONVERGENCE_RAW_OUTER_HIGH_BOUND, PPG_CONVERGENCE_RAW_INNER_HIGH_TARGET);
        DEBUG_PRINT("(raw_inner_low_target, raw_outer_low_bound) = (%d, %d)\n", PPG_CONVERGENCE_RAW_INNER_LOW_TARGET, PPG_CONVERGENCE_RAW_OUTER_LOW_BOUND);
    }
    _main.pah_8007.is_powered = true ;

    pah_8007_SWRESET();     // SW reset

    // PAH_8007_REG_DEVICE_INIT
    comm_write(0xEC, 0xFF);
    comm_write(0x2B, 0x00);
    comm_write(0x0A, 0x00);
    comm_write(0x4B, 0x04);
    comm_write(0x05, 0x50);
    comm_write(0x0C, 0x20);
    comm_write(0x0D, 0x1A);
    comm_write(0x38, 0x01); // Lock ExpoTime
    comm_write(0x34, 0x00); // R_Exposure_ALS_F0 [7:0]
    comm_write(0x35, 0x02); // R_Exposure_ALS_F0 [11:8]
    comm_write(0x38, 0x00); // Update ExpoTime

    comm_read(ADDR_03, &data);
    PAH_CLEAR_BIT(data, ADDR_03_BIT_R_PS_EN);
    PAH_CLEAR_BIT(data, ADDR_03_BIT_R_ALS_EN);
    comm_write(ADDR_03, data);

    comm_read(ADDR_11, &data);
    PAH_CLEAR_BIT(data, ADDR_11_BIT_R_LED_CONTROL_OUT_EN);
    PAH_CLEAR_BIT(data, ADDR_11_BIT_R_PS_INTERRUPT_EN);
    comm_write(ADDR_11, data);

    pah_8007_power_toggle(&_main.pah_8007, false);

    return true;
}

void ppg_sensor_start(uint8_t work_mode)
{
    pah_8007_power_toggle(&_main.pah_8007, true);

    switch (work_mode)
    {
    case PPG_ONLY_MODE:
        ppg_sensor_enable();
        _main.ppg_only_is_enabled = 1 ;
        hr_algorithm_enable();
        break;

    case TOUCH_ONLY_MODE:
        touch_sensor_enable();
        _main.touch_only_is_enabled = 1 ;
        break;

    case AUTO_MODE:
        touch_sensor_enable();
        break;
    }
    _main.debug_print_header = true;
}

void pah_sensor_stop(void)
{
    hr_algorithm_disable();     // Close HR algorithm
    ppg_sensor_disable();       // Stop sensor
    touch_sensor_disable();
    pah_8007_power_toggle(&_main.pah_8007, false);
}

void ppg_sensor_task_interrupt(void)
{
    if (_main.has_interrupt)
    {
        _main.has_interrupt = false;
        _main.interrupt_timestamp = pah_get_tick_count();
        process(_main.interrupt_timestamp);
    }
}

void ppg_sensor_task_polling(void)
{
    _main.interrupt_timestamp = pah_get_tick_count();
    process(_main.interrupt_timestamp);
}

void ppg_interrupt_handle(void)
{
    _main.has_interrupt = true;
    //_main.interrupt_timestamp = pah_get_tick_count();
}


/*****************************
FW FIFO Function
*****************************/
/* Sensor alg calculate task */
#define HR_OUTPUT_UPDATE_COUNT  20

uint8_t pah8007_alg_task(void)
{
    if (_main.alg_status)
    {
        LOG_PRINT("_main.alg_status =%d\n", _main.alg_status);
        if (_main.alg_status/*Check_alg_enable()*/ == MSG_ALG_ENABLE)
        {
            hr_algorithm_enable();
            LOG_PRINT("hr_algorithm_enable\n");
        }
        else if (_main.alg_status == MSG_ALG_DISABLE)
        {
            hr_algorithm_disable();
            LOG_PRINT("hr_algorithm_disable\n");
        }
        _main.alg_status = MSG_ALG_NOCHANGE;
    }
    if (!isFIFOEmpty())
    {
        if (Pop(&ppg_mems_data_alg)) //Get data from FIFO
        {
            if (_main.hr_is_enabled)
            {
                // data entrance to HR algorithm
                int32_t ret = PxiAlg_Process_Expo((uint8_t *)&ppg_mems_data_alg.hr_data, ppg_mems_data_alg.MEMS_Data, ppg_mems_data_alg.ppg_data.exposure_time);

                hr_update_count++;
                if (hr_update_count == HR_OUTPUT_UPDATE_COUNT)
                {
                    PxiAlg_HrGet(&hr);      // interface to get calculated HR value
                    hr_update_count = 0;
                    // print raw data and calculated result for analysis
                    report_hr_data(&ppg_mems_data_alg.ppg_data, ppg_mems_data_alg.hr_data.time, &ppg_mems_data_alg.touch_data, ret, hr, ppg_mems_data_alg.MEMS_Data, ppg_mems_data_alg.hr_data.frame_count);
                    return (uint8_t)hr;
                }
            }
        }
    }
    return 0;
}

static bool isFIFOEmpty(void)
{
    return (_write_index == _read_index);
}

static bool Push(ppg_mems_data_t *data) //Push return data must be true. If return false, please increase FIFO size.
{
    int tmp = _write_index ;
    tmp++;
    if (tmp >= FIFO_SIZE)
        tmp = 0 ;
    if (tmp == _read_index)
        return false;
    hr_data_buf[tmp] = *data ;
    _write_index = tmp ;
    return true;
}

static bool Pop(ppg_mems_data_t *data)
{
    int tmp ;
    if (isFIFOEmpty())
        return false;
    tmp = _read_index + 1;
    if (tmp >= FIFO_SIZE)
        tmp = 0 ;
    _read_index = tmp ;
    *data = hr_data_buf[_read_index] ;
    return true;
}

static void hr_algorithm_enable(void)
{
    if (_main.hr_is_enabled)
        return;

    PxiAlg_Close();     // close HR algorithm
    int version = PxiAlg_Version();     // check algorithm version

#ifdef V30_ALG_DYNAMIC_RAM      /* dynamic allocate RAM for V30xxxx algorithm */
    uint32_t Query_Mem_Size = PxiAlg_Query_Mem_Size();      // check algorithm RAM request(in bytes)
    DEBUG_PRINT("HR_Version = %d, HR_Mem_Size = %d \n", version, Query_Mem_Size);

    hr_pBuffer = malloc(Query_Mem_Size);        // allocate specified RAM space for algorithm

    if (!PxiAlg_Open_Mem(hr_pBuffer))           // open algorithm
        DEBUG_PRINT("PxiAlg_Open_Mem fail, %p\n", hr_pBuffer);
#else
#ifdef V30_ALG_STATIC_RAM       /* static RAM for V30xxxx algorithm */
    uint32_t Query_Mem_Size = PxiAlg_Query_Mem_Size();      // check algorithm RAM request(in bytes)
    DEBUG_PRINT("HR_Version = %d, HR_Mem_Size = %d \n", version, Query_Mem_Size);

    if (sizeof(hr_static_ram) < Query_Mem_Size)     // check if the array size is big enough
    {
        DEBUG_PRINT("ERROR: size of hr_static_ram is %d bytes, smaller than algorithm required(%d bytes) \n", sizeof(hr_static_ram), Query_Mem_Size);
        return;
    }
    hr_pBuffer = hr_static_ram;

    if (!PxiAlg_Open_Mem(hr_pBuffer))           // open algorithm
        DEBUG_PRINT("PxiAlg_Open_Mem fail \n");
#else       /* static RAM for other version algorithm */
    DEBUG_PRINT("HR_Version = %d \n", version);
    PxiAlg_Open();      // open algorithm
#endif
#endif

    /* "PxiAlg_Setxxx" statements are settings for algorithm */
    PxiAlg_SetQualityThreshold(0.35f);
    PxiAlg_SetHRSmoothWeight(0.9f);
    PxiAlg_SetProcWindow(5.0f, 5.0f);
    PxiAlg_SetTimeoutLength(30.0f);
    PxiAlg_SetMemsScale(1);                 //MEMS +/-2G/4G/8G/16G: 0/1/2/3
    PxiAlg_SetSkipSampleNum(10);
    PxiAlg_SetMemsScaleBase(1, 0);          //MEMS +/-2G/4G/8G/16G: 0/1/2/3
    PxiAlg_SetForceOutputLink(0, 150, 40, 1, 0.9f);
    PxiAlg_SetNormalOutput(200, 40, 0);
    PxiAlg_SetForceRipple(3, 1);
    //PxiAlg_SetForceOutput(30, 120, 45);
    //PxiAlg_SetForceOutputTime(10);
    //PxiAlg_SetPackage(0, 1);

    _main.hr_is_enabled = true;
}

static void hr_algorithm_disable(void)
{
    if (!_main.hr_is_enabled)
        return;

    PxiAlg_Close();

#ifdef V30_ALG_DYNAMIC_RAM
    free(hr_pBuffer);
#endif

    hr = 0.0f;
    _main.hr_is_enabled = false;
}

static bool sensor_basic_verification(void)
{
    uint8_t product_id = 0;
    comm_read(ADDR_00_PRODUCT_ID, &product_id);
    if (product_id != PAH_8007_PRODUCT_ID)
    {
        pah_8007_power_toggle(&_main.pah_8007, true);
        comm_read(ADDR_00_PRODUCT_ID, &product_id);
    }
    LOG_PRINT("hr product_id = 0x%04X\n\n", product_id);
    if (product_id != PAH_8007_PRODUCT_ID)
    {
        LOG_PRINT("Failed to verify product_id\n\n");
        return false;
    }
    return true;
}

static void process(uint64_t timestamp)
{
    // Task
    uint8_t ppg_interrupt = 0;
    bool has_ppg_interrupt = false;
    memset(&_main.pah_8007.ppg_result, 0, sizeof(_main.pah_8007.ppg_result));
    memset(&_main.pah_8007.touch_result, 0, sizeof(_main.pah_8007.touch_result));

    // pah_delay_ms(1);
    touch_process(&_main.pah_8007, &_main.pah_8007.touch_result);
    comm_read(ADDR_09, &data);
    ppg_interrupt = PAH_CHECK_BIT(data, ADDR_09_BIT_ALS_INTERRUPT);
    has_ppg_interrupt = (ppg_interrupt > 0);
    if (has_ppg_interrupt > 0)
    {
        ppg_process(&_main.pah_8007, &_main.pah_8007.ppg_result);
    }
    clear_interrupt(&_main.pah_8007, has_ppg_interrupt, _main.pah_8007.ppg_result.has_overflow);

    pah_8007_touch_data touch_data;
    touch_data = _main.pah_8007.touch_result;
    if (_main.hr_is_enabled)
    {
        pah_8007_ppg_data ppg_data;
        ppg_data = _main.pah_8007.ppg_result;
        if (ppg_data.has_data)
        {
            uint64_t duration = timestamp - _main.ppg_prev_report_timestamp;
            if (!_main.ppg_has_prev_report_timestamp)
            {
                _main.ppg_has_prev_report_timestamp = true;
                duration = 50;
            }
            _main.ppg_prev_report_timestamp = timestamp;

            if (duration > UINT8_MAX)
            {
                LOG_PRINT("duration > UINT8_MAX !!!\n");
                return;
            }
            hr_algorithm_update(&ppg_data, (uint8_t)duration, &touch_data);
        }
    }
    if (_main.touch_is_enabled)
    {
        if (_main.touch_only_is_enabled == 1)
        {
            report_touch_data(timestamp, &touch_data);
        }
        else if (touch_data.has_touch_flag
                 && _main.touch_last_is_touched != touch_data.touch_flag)
        {
            _main.touch_last_is_touched = touch_data.touch_flag;
            report_touch_data(timestamp, &touch_data);

            bool is_touched = (touch_data.touch_flag >= 1);
            if ((is_touched) && (_main.touch_only_is_enabled == 0))
            {
                ppg_sensor_enable();
                _main.alg_status = MSG_ALG_ENABLE ;
            }
            else if ((!is_touched) && (_main.touch_only_is_enabled == 0))
            {
                LOG_PRINT("ppg_sensor_disable!!!\n");
                ppg_sensor_disable();
                _main.alg_status = MSG_ALG_DISABLE ;
            }
        }
    }
}

static void ppg_sensor_enable(void)
{
    if (_main.ppg_is_enabled)
        return;

    if (_main.pah_8007.ppg_state.is_enabled == true)
        return;

    // enable PPG channel
    comm_read(ADDR_03, &data);
    PAH_SET_BIT(data, ADDR_03_BIT_R_ALS_EN);
    comm_write(ADDR_03, data);

    // clear ALS_Interrupt
    comm_read(ADDR_09, &data);
    PAH_CLEAR_BIT(data, ADDR_09_BIT_ALS_INTERRUPT);
    comm_write(ADDR_09, data);

    //pah_8007_hal_write_clk_manual_toggle( true);
    comm_read(ADDR_EC, &data);
    PAH_SET_BIT(data, ADDR_EC_BIT_7_R_CLK_MANUAL);
    comm_write(ADDR_EC, data);

    ppg_reset_exposure();

    _main.pah_8007.ppg_state.is_enabled = true;
    interrupt_configuration(&_main.pah_8007);
    _main.pah_8007.ppg_state.is_converging = true;

    _main.debug_print_header = true;
    _main.ppg_has_prev_report_timestamp = false;
    _main.ppg_prev_report_timestamp = pah_get_tick_count();
    _main.ppg_frame_count = 0;

    _main.ppg_is_enabled = true;
    //LOG_PRINT("ppg_sensor_enable\n");
}

static void ppg_sensor_disable(void)
{
    if (!_main.ppg_is_enabled)
        return;

    if (_main.pah_8007.ppg_state.is_enabled == false)
        return;

    // disable PPG channel
    comm_read(ADDR_03, &data);
    PAH_CLEAR_BIT(data, ADDR_03_BIT_R_ALS_EN);
    comm_write(ADDR_03, data);

    _main.pah_8007.ppg_state.is_enabled = false;
    interrupt_configuration(&_main.pah_8007);
    _main.pah_8007.ppg_state.is_converging = false;

    _main.ppg_is_enabled = false;
}

static void touch_sensor_enable(void)
{
    if (_main.touch_is_enabled)
        return;
    if (_main.pah_8007.touch_state.is_enabled == true)
        return ;
    // PS_Interrupt_clear
    comm_read(ADDR_09, &data);
    PAH_CLEAR_BIT(data, ADDR_09_BIT_PS_INTERRUPT);
    comm_write(ADDR_09, data);

    _main.pah_8007.touch_state.is_enabled = true;
    _main.pah_8007.touch_state.approach_continuous_count = 0;
    interrupt_configuration(&_main.pah_8007);

    _main.touch_last_is_touched = -1;
    _main.touch_is_enabled = true;
}

static void touch_sensor_disable(void)
{
    if (!_main.touch_is_enabled)
        return;
    if (_main.pah_8007.touch_state.is_enabled == false)
        return ;
    _main.pah_8007.touch_state.is_enabled = false;
    _main.pah_8007.touch_state.approach_continuous_count = 0;
    interrupt_configuration(&_main.pah_8007);

    _main.touch_is_enabled = false;
}

static void hr_algorithm_update(const pah_8007_ppg_data *ppg_data, uint8_t ppg_duration, const pah_8007_touch_data *touch_data)
{
    ppg_mems_data_t ppg_mems_data;

    ppg_mems_data.hr_data.loss = (uint8_t)ppg_data->has_overflow + 1;
    ppg_mems_data.hr_data.rawData = ppg_data->data;
    ppg_mems_data.hr_data.non_zero1 = 1;
    ppg_mems_data.hr_data.non_zero2 = 2;
    ppg_mems_data.hr_data.non_zero3 = 3;
    ppg_mems_data.hr_data.frame_count = _main.ppg_frame_count;
    ppg_mems_data.hr_data.time = ppg_duration;
    ppg_mems_data.hr_data.rsv = 0;
    ppg_mems_data.hr_data.touch = (touch_data->touch_flag > 0 ? 0x80 : 0x00);
    ppg_mems_data.hr_data.rsv2 = 0;

    float mems_data[3] = { 0.0f, 0.0f, 0.0f };      // init GSensor raw data to all zero

    /* To get true GSensor raw data by enabling ReadGSensor() */
    //ReadGSensor(&mems_data[0], &mems_data[1], &mems_data[2]);
    ppg_mems_data.MEMS_Data[0] = mems_data[0];  // pass X_Axis raw data;
    ppg_mems_data.MEMS_Data[1] = mems_data[1];  // pass Y_Axis raw data;
    ppg_mems_data.MEMS_Data[2] = mems_data[2];  // pass Z_Axis raw data;

    pah_8007_ppg_data ppg_data_tmp = *ppg_data ;
    pah_8007_touch_data touch_data_tmp = *touch_data ;

    ppg_mems_data.ppg_data = ppg_data_tmp ;
    ppg_mems_data.touch_data = touch_data_tmp ;

    Push(&ppg_mems_data);
    ++_main.ppg_frame_count;
}

static void report_touch_data(uint64_t timestamp, const pah_8007_touch_data *touch_data)
{
    LOG_PRINT("TOUCH: time = { %llu }, touch_flag = { %d }, touch_value = { %d }\n", timestamp, touch_data->touch_flag, touch_data->data);
}

static void report_hr_data(const pah_8007_ppg_data *ppg_data, uint8_t ppg_duration, const pah_8007_touch_data *touch_data, int32_t ret, float hr, float *MEMS_Data, uint8_t ppg_frame_count)
{
    if (_main.debug_print_header == true)
    {
        LOG_PRINT("Frame Count, Time, Touch Flag, ALS_Lux_Sync, ALS_overflow_eco, PS_ONOFF_Sync, PS_Approach, ALS_Interrupt, R_Exposure_ALS_F0, Alg_ret, Alg_hr, MEMS_X, MEMS_Y, MEMS_Z\n");

        _main.debug_print_header = false;
    }

    LOG_PRINT("%3d, %3d, %3d, %5d, %3d, %3d, %3d, %3d, %3d, %3d, %f, %d, %d, %d\n", ppg_frame_count, \
              ppg_duration, touch_data->touch_flag, ppg_data->data, ppg_data->has_overflow, \
              touch_data->data, touch_data->approach, ppg_data->has_data, \
              ppg_data->exposure_time, ret, hr, (int)MEMS_Data[0], (int)(MEMS_Data[1]), (int)(MEMS_Data[2]));
}

//============================================================================
/*pah8007 driver*/
//============================================================================

pah_ret pah_8007_power_toggle(pah_8007 *state, bool enable)
{
    DEBUG_PRINT("%s(%d, %d)\r\n", __func__, state->is_powered, enable);
    if (enable)
    {
        if (state->is_powered == enable)
        {
            return PAH_RET_SUCCESS;
        }

        // wake up sensor
        // Read twice
        comm_read(ADDR_00_PRODUCT_ID, &data);
        comm_read(ADDR_00_PRODUCT_ID, &data);
        pah_delay_ms(100);

        // enable Touch channel
        comm_read(ADDR_03, &data);
        PAH_SET_BIT(data, ADDR_03_BIT_R_PS_EN);
        comm_write(ADDR_03, data);
    }
    else
    {
        //write_ps_toggle
        comm_read(ADDR_03, &data);
        PAH_CLEAR_BIT(data, ADDR_03_BIT_R_PS_EN);
        PAH_CLEAR_BIT(data, ADDR_03_BIT_R_ALS_EN);
        comm_write(ADDR_03, data);

        state->ppg_state.is_enabled = false;
        state->touch_state.is_enabled = false;
        interrupt_configuration(state);
        //pah_8007_hal_write_suspend_state( true);
        comm_read(ADDR_03, &data);
        PAH_SET_BIT(data, ADDR_03_BIT_SW_SUSPEND);
        comm_write(ADDR_03, data);
    }

    state->is_powered = enable;
    state->touch_state.approach = -1;
    state->touch_state.approach_continuous_count = 0;
    state->touch_state.touch_flag_is_ready = false;
    state->touch_state.touch_flag = -1;

    return PAH_RET_SUCCESS;
}

//============================================================================
// Private function
//============================================================================

static pah_ret ppg_convergence_toggle(pah_8007 *state, bool enable)
{
    DEBUG_PRINT("%s(%d, %d)\n", __func__, ENABLE_PPG_CONVERGENCE, enable);

    if (!ENABLE_PPG_CONVERGENCE)
        return PAH_RET_SUCCESS;
    state->ppg_state.is_converging = enable;
    if (ENABLE_PPG_POWER_SAVING)
    {
        //pah_8007_hal_write_clk_manual_toggle( enable);
        comm_read(ADDR_EC, &data);
        if (enable)
            PAH_SET_BIT(data, ADDR_EC_BIT_7_R_CLK_MANUAL);
        else
            PAH_CLEAR_BIT(data, ADDR_EC_BIT_7_R_CLK_MANUAL);
        comm_write(ADDR_EC, data);
    }
    return PAH_RET_SUCCESS;
}

static pah_ret interrupt_configuration(pah_8007 *state)
{
    interrupt_config interrupt_config = INTERRUPT_CONFIG_NONE;
    if (state->ppg_state.is_enabled)
    {
        interrupt_config = INTERRUPT_CONFIG_ALS;
    }
    else if (state->touch_state.is_enabled)
    {
        if (state->touch_state.approach_continuous_count < TOUCH_FLAG_TRIGGER_COUNT)
            interrupt_config = INTERRUPT_CONFIG_ALS;
        else
            interrupt_config = INTERRUPT_CONFIG_APPROACH;
    }
    else
    {
        interrupt_config = INTERRUPT_CONFIG_NONE;
    }
    DEBUG_PRINT("continuous count = %d\n", state->touch_state.approach_continuous_count);
    DEBUG_PRINT("interrupt config = %d -> %d\n", state->interrupt_config, interrupt_config);

    if (state->interrupt_config != interrupt_config)
    {
        switch (interrupt_config)
        {
        case INTERRUPT_CONFIG_NONE:
            comm_read(ADDR_11, &data);
            PAH_CLEAR_BIT(data, ADDR_11_BIT_R_LED_CONTROL_OUT_EN);
            PAH_CLEAR_BIT(data, ADDR_11_BIT_R_PS_INTERRUPT_EN);
            comm_write(ADDR_11, data);
            break;
        case INTERRUPT_CONFIG_ALS:
            comm_read(ADDR_11, &data);
            PAH_SET_BIT(data, ADDR_11_BIT_R_LED_CONTROL_OUT_EN);
            PAH_CLEAR_BIT(data, ADDR_11_BIT_R_PS_INTERRUPT_EN);
            comm_write(ADDR_11, data);
            break;
        case INTERRUPT_CONFIG_APPROACH:
            // clear PS_Interrupt
            comm_read(ADDR_09, &data);
            PAH_CLEAR_BIT(data, ADDR_09_BIT_PS_INTERRUPT);
            comm_write(ADDR_09, data);
            comm_read(ADDR_11, &data);
            PAH_CLEAR_BIT(data, ADDR_11_BIT_R_LED_CONTROL_OUT_EN);
            PAH_SET_BIT(data, ADDR_11_BIT_R_PS_INTERRUPT_EN);
            comm_write(ADDR_11, data);
            break;
        }
        state->interrupt_config = interrupt_config;
    }
    return PAH_RET_SUCCESS;
}

static pah_ret ppg_reset_exposure()
{
    DEBUG_PRINT("%s(%d)\n", __func__, PPG_INITIAL_EXPOSURE);
    if (!ENABLE_PPG_CONVERGENCE)
        return PAH_RET_SUCCESS;
    uint8_t als_exposure_low = PAH_UINT16_LOW(PPG_INITIAL_EXPOSURE);
    uint8_t als_exposure_high = PAH_UINT16_HIGH(PPG_INITIAL_EXPOSURE);
    // Lock
    comm_read(ADDR_38, &data);
    PAH_SET_BIT(data, ADDR_38_BIT_0_LOCK_EXPOTIME);
    comm_write(ADDR_38, data);
    // Write expotime
    comm_write(ADDR_34, als_exposure_low);
    comm_write(ADDR_35, als_exposure_high);
    // Unlock
    PAH_CLEAR_BIT(data, ADDR_38_BIT_0_LOCK_EXPOTIME);
    comm_write(ADDR_38, data);
    return PAH_RET_SUCCESS;
}

static pah_ret touch_process(pah_8007 *state, pah_8007_touch_data *touch_result)
{
    uint8_t ps_onoff_sync = 0;
    uint8_t ps_approach = 0;
    comm_read(ADDR_10_PS_ONOFF_SYNC, &ps_onoff_sync);
    //pread_ps_approach
    comm_read(ADDR_09, &data);
    ps_approach = PAH_CHECK_BIT(data, ADDR_09_BIT_PS_APPROACH);

    touch_result->data = ps_onoff_sync;
    touch_result->approach = (int)ps_approach;

    if (state->touch_state.approach == (int)ps_approach)
    {
        if (state->touch_state.approach_continuous_count < TOUCH_FLAG_TRIGGER_COUNT)
        {
            ++state->touch_state.approach_continuous_count;
            interrupt_configuration(state);

            if (state->touch_state.approach_continuous_count == TOUCH_FLAG_TRIGGER_COUNT)
            {
                // no-touch -> touch
                if ((state->touch_state.touch_flag_is_ready && state->touch_state.touch_flag == 0) /*is_not_touched(state)*/
                        && (ps_approach > 0))
                {
                    ppg_reset_exposure();
                    ppg_convergence_toggle(state, true);
                    _main.ppg_frame_count = 0;
                }
                state->touch_state.touch_flag_is_ready = true;
                state->touch_state.touch_flag = (int)ps_approach;
            }
        }
    }
    else
    {
        state->touch_state.approach = (int)ps_approach;

        state->touch_state.approach_continuous_count = 1;
        interrupt_configuration(state);
    }

    if (state->touch_state.touch_flag_is_ready)
    {
        touch_result->has_touch_flag = true;
        touch_result->touch_flag = state->touch_state.touch_flag;
    }

    return PAH_RET_SUCCESS;
}

static pah_ret ppg_process(pah_8007 *state, pah_8007_ppg_data *ppg_result)
{
    uint16_t ppg_value = 0;
    uint8_t ppg_overflow = 0;
    uint16_t ppg_exposure = 0;

    if (!state->ppg_state.is_enabled)
        return PAH_RET_SUCCESS;

    //read_als_lux_sync
    comm_burst_read(ADDR_0E_ALS_LUX_SYNC, (uint8_t *)&ppg_value, 2);

    // read_als_overflow_eco
    comm_read(ADDR_04, &data);
    ppg_overflow = PAH_CHECK_BIT(data, ADDR_04_BIT_ALS_OVERFLOW_ECO);

    //read_als_exposure
    comm_burst_read(ADDR_34, (uint8_t *)&ppg_exposure, 2);
    ppg_converging(state, ppg_value);

    ppg_result->has_data = true;
    ppg_result->data = ppg_value;
    ppg_result->has_overflow = (ppg_overflow > 0);
    ppg_result->exposure_time = ppg_exposure;
    return PAH_RET_SUCCESS;
}


static pah_ret clear_interrupt(pah_8007 *state, bool has_ppg_interrupt, bool has_overflow)
{
    if (state->interrupt_config == INTERRUPT_CONFIG_APPROACH)
    {
        //write_approach_interrupt_clear
        comm_read(ADDR_09, &data);
        PAH_CLEAR_BIT(data, ADDR_09_BIT_PS_INTERRUPT);
        comm_write(ADDR_09, data);
    }
    else if (state->interrupt_config == INTERRUPT_CONFIG_ALS)
    {
        if (has_ppg_interrupt)
        {
            //write_als_interrupt_clear
            comm_read(ADDR_09, &data);
            PAH_CLEAR_BIT(data, ADDR_09_BIT_ALS_INTERRUPT);
            comm_write(ADDR_09, data);
        }
        if (has_overflow)
        {
            // Clear als_overflow_eco flag
            comm_read(ADDR_38, &data);
            PAH_SET_BIT(data, ADDR_38_BIT_1_CLEAR_ALS_OVERFLOW_ECO);
            comm_write(ADDR_38, data);
            // Recover
            PAH_CLEAR_BIT(data, ADDR_38_BIT_1_CLEAR_ALS_OVERFLOW_ECO);
            comm_write(ADDR_38, data);
        }
    }
    return PAH_RET_SUCCESS;
}

static pah_ret ppg_converging(pah_8007 *state, uint16_t ppg_value)
{
    if (!ENABLE_PPG_CONVERGENCE)
        return PAH_RET_SUCCESS;

    if (ENABLE_PPG_RECONVERGENCE_IF_OUT_OF_BOUNDS
            && !state->ppg_state.is_converging
            && (state->touch_state.touch_flag_is_ready && state->touch_state.touch_flag == 1)
            && ((ppg_value < PPG_CONVERGENCE_RAW_OUTER_LOW_BOUND) || (ppg_value > PPG_CONVERGENCE_RAW_OUTER_HIGH_BOUND)))
    {
        ppg_convergence_toggle(state, true);
    }

    if (!state->ppg_state.is_converging)
        return PAH_RET_SUCCESS;

    if ((ppg_value < PPG_CONVERGENCE_RAW_INNER_HIGH_TARGET) || (ppg_value > PPG_CONVERGENCE_RAW_INNER_LOW_TARGET))
    {
        uint16_t ppg_exposure = 0;
        uint16_t new_ppg_exposure = 0;

        comm_burst_read(ADDR_34, (uint8_t *)&ppg_exposure, 2);
        new_ppg_exposure = ppg_exposure;
        if (ppg_value < PPG_CONVERGENCE_RAW_OUTER_LOW_BOUND)
            new_ppg_exposure += PPG_CONVERGENCE_T_WEIGHT_FAST_MODE;
        else if (ppg_value < PPG_CONVERGENCE_RAW_INNER_LOW_TARGET)
            new_ppg_exposure += PPG_CONVERGENCE_T_WEIGHT_NORMAL_MODE;
        else if (ppg_value > PPG_CONVERGENCE_RAW_OUTER_HIGH_BOUND)
            new_ppg_exposure -= PPG_CONVERGENCE_T_WEIGHT_FAST_MODE;
        else if (ppg_value > PPG_CONVERGENCE_RAW_INNER_HIGH_TARGET)
            new_ppg_exposure -= PPG_CONVERGENCE_T_WEIGHT_NORMAL_MODE;

        if (new_ppg_exposure > PPG_CONVERGENCE_T_MAX)
        {
            new_ppg_exposure = PPG_CONVERGENCE_T_MAX;
            ppg_convergence_toggle(state, false);
            DEBUG_PRINT("ppg converging complete: (exposure, t_max) = (%d, %d)\n", new_ppg_exposure, PPG_CONVERGENCE_T_MAX);
        }
        else if (new_ppg_exposure < PPG_CONVERGENCE_T_MIN)
        {
            new_ppg_exposure = PPG_CONVERGENCE_T_MIN;
            ppg_convergence_toggle(state, false);
            DEBUG_PRINT("ppg converging complete: (exposure, t_min) = (%d, %d)\n", new_ppg_exposure, PPG_CONVERGENCE_T_MIN);
        }
        //write_als_exposure( new_ppg_exposure);
        uint8_t als_exposure_low = PAH_UINT16_LOW(new_ppg_exposure);
        uint8_t als_exposure_high = PAH_UINT16_HIGH(new_ppg_exposure);
        // Lock
        comm_read(ADDR_38, &data);
        PAH_SET_BIT(data, ADDR_38_BIT_0_LOCK_EXPOTIME);
        comm_write(ADDR_38, data);
        // Write expotime
        comm_write(ADDR_34, als_exposure_low);
        comm_write(ADDR_35, als_exposure_high);
        // Unlock
        PAH_CLEAR_BIT(data, ADDR_38_BIT_0_LOCK_EXPOTIME);
        comm_write(ADDR_38, data);
    }
    else
    {
        ppg_convergence_toggle(state, false);
        DEBUG_PRINT("ppg converging complete: (low_target, als_lux_sync, high_target) = (%d, %d, %d)\n", PPG_CONVERGENCE_RAW_INNER_LOW_TARGET, ppg_value, PPG_CONVERGENCE_RAW_INNER_HIGH_TARGET);
    }
    return PAH_RET_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//----- verify led golden only ----- (Comment for not verify)
#define PAH_LED_GOLDEN_ONLY_MIN      110
#define PAH_LED_GOLDEN_ONLY_MAX      130

//----- verify led target sample ----- (Comment for not verify)
#define PAH_LED_TARGET_SAMPLE_MIN     60
#define PAH_LED_TARGET_SAMPLE_MAX    220
//----- verify light leak ----- (Comment for not verify)
#define PAH_LIGHT_LEAK_MAX     32

void pah_get_touch_data(uint8_t *touch_value)
{
    pah_comm_i2c_read(ADDR_10_PS_ONOFF_SYNC, touch_value, 1);
}

uint8_t factory_test_led_golden_sample(uint8_t *touch_value_out)
{
    LOG_PRINT("///----------factory_test_led_golden_sample------------///\n\n");
    uint8_t touch_value = 0;
    uint8_t test_result = 0;
    if (pah_sensor_init() == false)
    {
        return 255;
    }
    ppg_sensor_start(TOUCH_ONLY_MODE);
    pah_delay_ms(200);
    pah_get_touch_data(&touch_value);
    * touch_value_out = touch_value ;
    if ((touch_value < PAH_LED_GOLDEN_ONLY_MAX) && (touch_value > PAH_LED_GOLDEN_ONLY_MIN))
    {
        test_result = 1 ;
        LOG_PRINT(" Factory test Pass (min, value, max) =  (%d, %d, %d)\n\n", PAH_LED_GOLDEN_ONLY_MIN, touch_value, PAH_LED_GOLDEN_ONLY_MAX);
    }
    else
    {
        test_result = 0 ;
        LOG_PRINT(" Factory test Fail (min, value, max) =  (%d, %d, %d)\n\n", PAH_LED_GOLDEN_ONLY_MIN, touch_value, PAH_LED_GOLDEN_ONLY_MAX);
    }
    pah_delay_ms(100);
    pah_sensor_stop();
    return test_result ;
}

uint8_t factory_test_led_target_sample(uint8_t *touch_value_out)
{
    LOG_PRINT("///----------factory_test_led_target_sample------------///\n\n");
    uint8_t touch_value = 0;
    uint8_t test_result = 0;
    if (pah_sensor_init() == false)
    {
        return 255;
    }
    ppg_sensor_start(TOUCH_ONLY_MODE);
    pah_delay_ms(200);
    pah_get_touch_data(&touch_value);
    * touch_value_out = touch_value ;
    if ((touch_value < PAH_LED_TARGET_SAMPLE_MAX) && (touch_value > PAH_LED_TARGET_SAMPLE_MIN))
    {
        test_result = 1 ;
        LOG_PRINT(" Factory test Pass (min, value, max) =  (%d, %d, %d)\n\n", PAH_LED_TARGET_SAMPLE_MIN, touch_value, PAH_LED_TARGET_SAMPLE_MAX);
    }
    else
    {
        test_result = 0 ;
        LOG_PRINT(" Factory test Fail (min, value, max) =  (%d, %d, %d)\n\n", PAH_LED_TARGET_SAMPLE_MIN, touch_value, PAH_LED_TARGET_SAMPLE_MAX);
    }
    pah_delay_ms(100);
    pah_sensor_stop();
    return test_result ;
}

uint8_t factory_test_light_leak(uint8_t *touch_value_out)
{
    LOG_PRINT("///----------factory_test_light_leak------------///\n\n");
    uint8_t touch_value = 0;
    uint8_t test_result = 0;
    if (pah_sensor_init() == false)
    {
        return 255;
    }
    ppg_sensor_start(TOUCH_ONLY_MODE);

    pah_delay_ms(200);
    pah_get_touch_data(&touch_value);
    * touch_value_out = touch_value ;
    if (touch_value < PAH_LIGHT_LEAK_MAX)
    {
        test_result = 1 ;
        LOG_PRINT(" Factory test Pass (value, max) =  ( %d, %d)\n\n", touch_value, PAH_LIGHT_LEAK_MAX);
    }
    else
    {
        test_result = 0 ;
        LOG_PRINT(" Factory test Fail (value, max) =  ( %d, %d)\n\n", touch_value, PAH_LIGHT_LEAK_MAX);
    }
    pah_delay_ms(100);
    pah_sensor_stop();
    return test_result ;
}

static void pah_8007_SWRESET(void)
{
    comm_write(0xEE, 0xFA);
    pah_delay_ms(10);
    comm_write(0xEE, 0xFF);
    pah_delay_ms(10);
}
